/*
 * Copyright (C) 2018, 2026 Kernkonzept GmbH.
 * Author(s): Jakub Jermar <jakub.jermar@kernkonzept.com>
 *
 * License: see LICENSE.spdx (in this directory or the directories above)
 */

#pragma once

#include <l4/re/event_enums.h>
#include <l4/re/error_helper>
#include <l4/re/util/event>
#include <l4/vbus/vbus>
#include <l4/vbus/vbus_inhibitor.h>
#include <l4/cxx/unique_ptr>

#include <l4/libblock-device/debug.h>
#include <l4/libblock-device/types.h>

namespace Block_device {

/**
 * Abstract class for listening to and processing of PM events.
 */
class Pm : public L4::Irqep_t<Pm>
{
public:
  Pm(L4::Ipc::String<> const &reason) : _reason(reason)
  {}

  int init(L4::Cap<L4Re::Inhibitor> inhib)
  {
    auto event = cxx::make_unique<L4Re::Util::Event>();

    _inhib = inhib;
    if (!_inhib.is_valid())
      {
        Dbg::warn().printf("Invalid inhibitor capability.\n");
        return -L4_ENOENT;
      }
    int ret =
      event->init<L4::Irq>(L4::cap_reinterpret_cast<L4Re::Event>(_inhib));
    if (ret)
      {
        Dbg::warn().printf("Failed to initialize events.\n");
        return ret;
      }
    if (!event->irq().is_valid())
      {
        Dbg::warn().printf("Invalid event's irq capability.\n");
        return -L4_EINVAL;
      }
    ret = _inhib->acquire(L4VBUS_INHIBITOR_SHUTDOWN, _reason);
    if (ret)
      {
        Dbg::warn().printf("Failed to acquire shutdown inhibitor.\n");
        return ret;
      }
    ret = _inhib->acquire(L4VBUS_INHIBITOR_SUSPEND, _reason);
    if (ret)
      {
        Dbg::warn().printf("Failed to acquire suspend inhibitor.\n");
        if (_inhib->release(L4VBUS_INHIBITOR_SHUTDOWN))
          Dbg::warn().printf("Failed to release shutdown inhibitor.\n");
        return ret;
      }
    _event = cxx::move(event);
    return L4_EOK;
  }

  ~Pm()
  {
    if (!_event)
      return;

    if (_inhib.is_valid())
      {
        if (_inhib->release(L4VBUS_INHIBITOR_SHUTDOWN))
          Dbg::warn().printf("Failed to release shutdown inhibitor.\n");
        if (_inhib->release(L4VBUS_INHIBITOR_SUSPEND))
          Dbg::warn().printf("Failed to release suspend inhibitor.\n");
      }
  }

  template <typename REG>
  void register_obj(REG *registry)
  {
    static_cast<void>(
      registry->register_obj(this, L4::cap_cast<L4::Irq>(_event->irq())));
  }

  template <typename REG>
  void unregister_obj(REG *registry)
  {
    registry->unregister_obj(this);
  }

  void handle_irq()
  {
    L4Re::Util::Event_buffer::Event *e;
    while ((e = _event->buffer().next()) != NULL)
      {
        if (e->payload.type == L4RE_EV_PM)
          {
            switch (e->payload.code)
              {
              case L4VBUS_INHIBITOR_SUSPEND:
                {
                  if (_inhib->acquire(L4VBUS_INHIBITOR_WAKEUP, _reason))
                    {
                      Dbg::warn().printf("Failed to acquire wakeup inhibitor "
                                         "lock, blocking suspend.\n");
                      break;
                    }
                  suspend();
                  if (_inhib->release(L4VBUS_INHIBITOR_SUSPEND))
                    Dbg::warn().printf(
                      "Failed to release suspend inhibitor lock.\n");
                  break;
                }
              case L4VBUS_INHIBITOR_SHUTDOWN:
                {
                  shutdown();
                  if (_inhib->release(L4VBUS_INHIBITOR_SHUTDOWN))
                    Dbg::warn().printf(
                      "Failed to release shutdown inhibitor lock.\n");
                  break;
                }
              case L4VBUS_INHIBITOR_WAKEUP:
                {
                  if (_inhib->acquire(L4VBUS_INHIBITOR_SUSPEND, _reason))
                    {
                      Dbg::warn().printf("Failed to acquire suspend inhibitor "
                                         "lock, blocking wakeup.\n");
                      break;
                    }
                  wakeup();
                  if (_inhib->release(L4VBUS_INHIBITOR_WAKEUP))
                    Dbg::warn().printf(
                      "Failed to release wakeup inhibitor lock.\n");
                  break;
                }
              default:
                Dbg::warn().printf("Unexpected pm event.\n");
              }
          }
        e->free();
      }
  }

  virtual void suspend() = 0;
  virtual void shutdown() = 0;
  virtual void wakeup() = 0;

private:
  L4::Ipc::String<> _reason;
  cxx::unique_ptr<L4Re::Util::Event> _event;
  L4::Cap<L4Re::Inhibitor> _inhib;
};

/**
 * Adapter class for processing PM events via a device manager instance.
 */
template <typename DM>
class Pm_for_dm : public Pm
{
public:
  Pm_for_dm(DM &dm, L4::Ipc::String<> const &reason) : Pm(reason), _dm(dm)
  {}

  void shutdown() override
  { _dm.shutdown_event(Shutdown_type::System_shutdown); }
  void suspend() override
  { _dm.shutdown_event(Shutdown_type::System_suspend); }
  void wakeup() override
  { _dm.shutdown_event(Shutdown_type::Running); }

private:
  DM &_dm;
};

}

