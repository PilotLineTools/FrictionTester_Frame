/**
 * CanRouter - Minimal CAN router: register handlers by 11-bit ID, dispatch received frames, send via TWAI.
 * Implements ICanRouter from WaterBathCanAdapter.h.
 */

#ifndef CAN_ROUTER_H
#define CAN_ROUTER_H

#include "WaterBathCanAdapter.h"
#include "driver/twai.h"

#ifndef CAN_ROUTER_MAX_HANDLERS
#define CAN_ROUTER_MAX_HANDLERS 8
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
      return twai_transmit(msg, 0) == ESP_OK;
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
