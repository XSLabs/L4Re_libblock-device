/*
 * Copyright (C) 2026 Kernkonzept GmbH.
 * Author(s): Jakub Jermar <jakub.jermar@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

#pragma once

#include <l4/cxx/ref_ptr>
#include <l4/re/dataspace>
#include <l4/re/dma_space>

#include <l4/libblock-device/types.h>
#include <l4/libblock-device/debug.h>

namespace Block_device {

// DEV must be the same class used by the client factory
template <typename DEV>
struct Device_dma_map_all_impl
{
  using Device_type = DEV;

private:
  struct Dma_info : public Block_device::Dma_region_info
  {
    L4Re::Dma_space::Dma_addr addr;
    l4_size_t size;
    cxx::Ref_ptr<Device_type> device;

    Dma_info() = delete;
    explicit Dma_info(L4Re::Dma_space::Dma_addr addr, l4_size_t size,
                      cxx::Ref_ptr<Device_type> device)
    : addr(addr), size(size), device(device)
    {
    }

    virtual ~Dma_info() override
    {
      device->dma_unmap_region(this);
    }
  };

protected:
  Device_dma_map_all_impl(L4Re::Util::Shared_cap<L4Re::Dma_space> const &dma)
  : _dma(dma)
  {}

  int dma_map_all(Block_device::Mem_region *region, l4_addr_t offset,
                  l4_size_t num_sectors, L4Re::Dma_space::Direction,
                  L4Re::Dma_space::Dma_addr *dma_addr)
  {
    if (!region->dma_info)
      {
        l4_size_t size = region->size();
        L4Re::Dma_space::Dma_addr addr;
        auto ret =
          _dma->map(L4::Ipc::make_cap_rw(region->ds()), region->ds_offset(),
                    &size, L4Re::Dma_space::Attributes::None,
                    L4Re::Dma_space::Direction::Bidirectional, &addr);
        if (ret < 0 || size < num_sectors * device()->sector_size())
          {
            *dma_addr = 0;
            Dbg::info().printf(
              "Cannot resolve DMA address (ret = %d, %zu < %zu).\n", ret,
              size, num_sectors * device()->sector_size());
            return -L4_ENOMEM;
          }

        auto dev = cxx::Ref_ptr<Device_type>(device());
        auto dma_info = cxx::make_unique<Dma_info>(addr, size, dev);
        region->dma_info =
          cxx::unique_ptr<Block_device::Dma_region_info>(dma_info.release());
      }

    auto *dma_info = static_cast<Dma_info *>(region->dma_info.get());
    *dma_addr = dma_info->addr + offset - region->ds_offset();

    return L4_EOK;
  }

  void dma_unmap_region(Dma_info *dma_info)
  {
    auto ret = _dma->unmap(dma_info->addr, dma_info->size,
                           L4Re::Dma_space::Attributes::None,
                           L4Re::Dma_space::Direction::Bidirectional);
    if (ret < 0)
      Dbg::info().printf(
        "Failed to unmap (ret = %d, addr = %llx, size = %zu)\n", ret,
        dma_info->addr, dma_info->size);
  }

private:
  Device_type *device()
  { return static_cast<Device_type *>(this); }

  L4Re::Util::Shared_cap<L4Re::Dma_space> _dma;
};

} // name space
