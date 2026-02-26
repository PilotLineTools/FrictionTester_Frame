/**
 * ICanRouter - Abstract CAN router interface shared by CAN adapters.
 */

#ifndef I_CAN_ROUTER_H
#define I_CAN_ROUTER_H

#include "driver/twai.h"
#include <stdint.h>

typedef void (*CanHandlerFn)(const twai_message_t *msg, void *ctx);

class ICanRouter
{
public:
   /** Register handler for 11-bit CAN ID. ctx passed to handler. */
   virtual void on(uint32_t id, CanHandlerFn fn, void *ctx) = 0;
   /** Enqueue frame for TX. Non-blocking. Returns true if queued. */
   virtual bool send(const twai_message_t *msg) = 0;
   virtual ~ICanRouter() = default;
};

#endif // I_CAN_ROUTER_H
