/**
 * PowerCanAdapter - CAN (TWAI) bridge for power events/commands.
 *
 * RX: 0x081 SET_POWER command
 * TX: 0x281 POWER_STATUS telemetry/events
 */

#ifndef POWER_CAN_ADAPTER_H
#define POWER_CAN_ADAPTER_H

#include "ICanRouter.h"
#include "FrameCanAdapter.h"
#include "PowerController.h"
#include "SystemMode.h"
#include "driver/twai.h"
#include <stdint.h>

static const uint32_t CAN_ID_SET_POWER    = 0x081u;
static const uint32_t CAN_ID_POWER_STATUS = 0x281u;
static const uint32_t CAN_ID_CLEAR_FAULT = 0x011u;
static const uint32_t CAN_ID_GUI_HEARTBEAT = 0x012u;

class PowerCanAdapter
{
public:
   PowerCanAdapter(PowerController *power, ICanRouter *router, FrameCanAdapter *frameCan);

   void begin();
   void tick(uint32_t nowMs);
   void onModeChanged(SystemMode mode) { _mode = mode; }

   /** Forward PowerController notifications so GUI can react immediately. */
   void onPowerNotification(PowerController::Notification n);

private:
   PowerController *_power;
   ICanRouter *_router;
   FrameCanAdapter *_frameCan;
   SystemMode _mode = SystemMode::NORMAL;

   uint32_t _lastStatusMs = 0;
   uint32_t _statusIntervalMs = 250;
   uint8_t _eventSeq = 0;

   static void staticHandleSetPower(const twai_message_t *msg, void *ctx);
   static void staticHandleGuiHeartbeat(const twai_message_t *msg, void *ctx);
   static void staticHandleClearFault(const twai_message_t *msg, void *ctx);
   void handleSetPower(const twai_message_t *msg);
   void handleGuiHeartbeat(const twai_message_t *msg);
   void handleClearFault(const twai_message_t *msg);

   void sendPowerStatus(uint8_t eventCode);
   void sendAck(uint8_t result, uint8_t detailCode);
};

#endif // POWER_CAN_ADAPTER_H
