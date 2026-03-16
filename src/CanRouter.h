/**
 * CanRouter - Minimal CAN router: register handlers by 11-bit ID, dispatch received frames, send via TWAI.
 * Implements shared ICanRouter interface.
 */

#ifndef CAN_ROUTER_H
#define CAN_ROUTER_H

#include "ICanRouter.h"
#include "PowerController.h"
#include "driver/twai.h"
#include <Arduino.h>

#ifndef CAN_ROUTER_MAX_HANDLERS
#define CAN_ROUTER_MAX_HANDLERS 16
#endif

struct CanRouterEntry
{
   uint32_t id;
   CanHandlerFn fn;
   void *ctx;
   bool used;
};

class CanRouter : public ICanRouter
{
public:
   CanRouter() { clear(); }

   void on(uint32_t id, CanHandlerFn fn, void *ctx) override
   {
      for (int i = 0; i < CAN_ROUTER_MAX_HANDLERS; i++)
      {
         if (!_entries[i].used)
         {
            _entries[i].id = id;
            _entries[i].fn = fn;
            _entries[i].ctx = ctx;
            _entries[i].used = true;
            return;
         }
         if (_entries[i].id == id)
         {
            _entries[i].fn = fn;
            _entries[i].ctx = ctx;
            return;
         }
      }
   }

   bool send(const twai_message_t *msg) override
   {
      twai_status_info_t status = {};
      PowerController *power = PowerController::instance();
      const unsigned guiSignalOn = (power && power->isGuiSignalOn()) ? 1u : 0u;
      const unsigned piCanAlive = (power && power->isPiCanAlive()) ? 1u : 0u;
      /*if (!isBusHealthy())
      {
         if (getStatusInfo(status))
         {
            USBSerial.printf("CAN send failed id=0x%03lX: gui_signal=%u pi_can_alive=%u state=%d tx_err=%u rx_err=%u bus_err=%u arb_lost=%u tx_failed=%u rx_missed=%u\n",
                             (unsigned long)(msg->identifier & 0x7FF),
                             guiSignalOn,
                             piCanAlive,
                             (int)status.state,
                             (unsigned)status.tx_error_counter,
                             (unsigned)status.rx_error_counter,
                             (unsigned)status.bus_error_count,
                             (unsigned)status.arb_lost_count,
                             (unsigned)status.tx_failed_count,
                             (unsigned)status.rx_missed_count);
         }
         else
         {
            USBSerial.printf("CAN send failed id=0x%03lX: gui_signal=%u pi_can_alive=%u unable to read TWAI status\n",
                             (unsigned long)(msg->identifier & 0x7FF),
                             guiSignalOn,
                             piCanAlive);
         }
         return false;
      }
      */
      if (twai_transmit(msg, 0) == ESP_OK)
         return true;

      if (getStatusInfo(status))
      {
         USBSerial.printf("CAN send failed id=0x%03lX: gui_signal=%u pi_can_alive=%u state=%d tx_err=%u rx_err=%u bus_err=%u arb_lost=%u tx_failed=%u rx_missed=%u\n",
                          (unsigned long)(msg->identifier & 0x7FF),
                          guiSignalOn,
                          piCanAlive,
                          (int)status.state,
                          (unsigned)status.tx_error_counter,
                          (unsigned)status.rx_error_counter,
                          (unsigned)status.bus_error_count,
                          (unsigned)status.arb_lost_count,
                          (unsigned)status.tx_failed_count,
                          (unsigned)status.rx_missed_count);
      }
      else
      {
         USBSerial.printf("CAN send failed id=0x%03lX: gui_signal=%u pi_can_alive=%u unable to read TWAI status\n",
                          (unsigned long)(msg->identifier & 0x7FF),
                          guiSignalOn,
                          piCanAlive);
      }
      return false;
   }

   bool getStatusInfo(twai_status_info_t &status) const
   {
      return twai_get_status_info(&status) == ESP_OK;
   }

   bool isBusHealthy() const
   {
      twai_status_info_t status = {};
      if (!getStatusInfo(status))
         return false;

      return (status.state == TWAI_STATE_RUNNING) &&
             (status.tx_error_counter == 0) &&
             (status.rx_error_counter == 0) &&
             (status.bus_error_count == 0);
   }

   /** Call from main loop when twai_receive() returns a frame; dispatches to registered handler. */
   void dispatch(const twai_message_t *msg)
   {
      uint32_t id = msg->identifier;
      for (int i = 0; i < CAN_ROUTER_MAX_HANDLERS; i++)
      {
         if (_entries[i].used && _entries[i].id == id && _entries[i].fn)
         {
            _entries[i].fn(msg, _entries[i].ctx);
            return;
         }
      }
   }

   void clear()
   {
      for (int i = 0; i < CAN_ROUTER_MAX_HANDLERS; i++)
         _entries[i].used = false;
   }

private:
   CanRouterEntry _entries[CAN_ROUTER_MAX_HANDLERS];
};

#endif // CAN_ROUTER_H
