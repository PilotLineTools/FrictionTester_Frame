#include "PowerCanAdapter.h"
#include "Constants.h"
#include <Arduino.h>

static const uint8_t POWER_CMD_NOP = 0;
static const uint8_t POWER_CMD_REQUEST_SHUTDOWN = 1;

PowerCanAdapter::PowerCanAdapter(PowerController *power, ICanRouter *router, FrameCanAdapter *frameCan)
   : _power(power),
     _router(router),
     _frameCan(frameCan)
{
}

void PowerCanAdapter::begin()
{
   if (_router)
   {
      _router->on(CAN_ID_SET_POWER, &PowerCanAdapter::staticHandleSetPower, this);
      _router->on(CAN_ID_GUI_HEARTBEAT, &PowerCanAdapter::staticHandleGuiHeartbeat, this);
      _router->on(CAN_ID_CLEAR_FAULT, &PowerCanAdapter::staticHandleClearFault, this);

   }
}

void PowerCanAdapter::tick(uint32_t nowMs)
{
   if (!_router || !_power)
      return;

   if (nowMs - _lastStatusMs >= _statusIntervalMs)
   {
      sendPowerStatus(0);
      _lastStatusMs = nowMs;
   }
}

void PowerCanAdapter::onPowerNotification(PowerController::Notification n)
{
   uint8_t eventCode = 0;
   switch (n)
   {
   case PowerController::Notification::GUIPoweredOn:
      eventCode = 1;
      break;
   case PowerController::Notification::ShutdownInitiated:
      eventCode = 2;
      break;
   case PowerController::Notification::ShutdownRequested:
      eventCode = 3;
      break;
   case PowerController::Notification::ShutdownAborted:
      eventCode = 4;
      break;
   }

   sendPowerStatus(eventCode);
}

void PowerCanAdapter::staticHandleSetPower(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<PowerCanAdapter *>(ctx);
   if (adapter)
      adapter->handleSetPower(msg);
}

void PowerCanAdapter::staticHandleGuiHeartbeat(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<PowerCanAdapter *>(ctx);
   if (adapter)
      adapter->handleGuiHeartbeat(msg);
}

void PowerCanAdapter::staticHandleClearFault(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<PowerCanAdapter *>(ctx);
   if (adapter)
      adapter->handleClearFault(msg);
}

void PowerCanAdapter::handleSetPower(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
   {
      sendAck(1, 20);
      return;
   }

   if (_mode == SystemMode::FW_UPDATE)
   {
      sendAck(1, 21);
      return;
   }

   const uint8_t cmd = msg->data[0];
   if (msg->data[1] != 0 || msg->data[2] != 0 || msg->data[3] != 0 ||
       msg->data[4] != 0 || msg->data[5] != 0 || msg->data[6] != 0 || msg->data[7] != 0)
   {
      sendAck(1, 22);
      return;
   }

   switch (cmd)
   {
   case POWER_CMD_NOP:
      sendPowerStatus(0);
      sendAck(0, 0);
      break;

   case POWER_CMD_REQUEST_SHUTDOWN:
      if (_power)
      {
         _power->requestShutdownFromRemote();
         sendPowerStatus(3);
         sendAck(0, 0);
      }
      else
      {
         sendAck(1, 23);
      }
      break;

   default:
      sendAck(1, 24);
      break;
   }
}

void PowerCanAdapter::handleGuiHeartbeat(const twai_message_t *msg)
{
   if (!_power)
      return;
   if (msg->data_length_code != 8)
      return;

   _power->onGuiHeartbeat(millis());
}


void PowerCanAdapter::handleClearFault(const twai_message_t *msg)
{
   if (!_power)
      return;

   if (msg->data_length_code != 8) 
   { 
      sendAck(1, 26); return; 
   }

   const bool changed = _power && _power->clearFaultToActiveIfShutdown();
   sendAck(changed ? 0 : 1, changed ? 0 : 27); // 27 = not in SHUT_DOWN
   if (changed) sendPowerStatus(4);            // optional event code
}


void PowerCanAdapter::sendPowerStatus(uint8_t eventCode)
{
   if (!_router || !_power)
      return;

   twai_message_t tx = {};
   tx.identifier = CAN_ID_POWER_STATUS;
   tx.data_length_code = 8;
   tx.flags = 0;

   uint8_t flags = 0;
   if (_power->isButtonPressed())
      flags |= 0x01;
   if (_power->isGuiSignalOn())
      flags |= 0x02;
   if (digitalRead(POWER_HOLD_PIN) == HIGH)
      flags |= 0x04;

   tx.data[0] = _power->getGuiPowerStateCode();
   tx.data[1] = 0;
   tx.data[2] = 0;
   tx.data[3] = 0;
   tx.data[4] = 0;
   tx.data[5] = 0;
   tx.data[6] = 0;
   tx.data[7] = 0;
   
   USBSerial.printf("PowerCanAdapter: send state code=%u\n", (unsigned)tx.data[0]);
   USBSerial.printf("PowerCanAdapter: send flags=0x%02X (button=%u gui=%u hold=%u)\n",
                    (unsigned)flags,
                    (unsigned)((flags & 0x01) != 0),
                    (unsigned)((flags & 0x02) != 0),
                    (unsigned)((flags & 0x04) != 0));
   
   _router->send(&tx);
}


void PowerCanAdapter::sendAck(uint8_t result, uint8_t detailCode)
{
   if (_frameCan)
      _frameCan->sendAck(result, detailCode);
}
