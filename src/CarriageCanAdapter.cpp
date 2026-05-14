#include "CarriageCanAdapter.h"
#include <Arduino.h>
#include <cstring>
#include <math.h>

CarriageCanAdapter::CarriageCanAdapter(CarriageController *controller, ICanRouter *router, FrameESP_CanAdapter *espCan)
   : _controller(controller), _router(router), _espCan(espCan)
{
}

LimitSwitchState CarriageCanAdapter::readLimitState() const
{
   // Axis polling reads this snapshot through LimitSwitch(remoteSource).
   LimitSwitchState state = {};
   state.minTriggered = _remoteMinTriggered;
   state.maxTriggered = _remoteMaxTriggered;
   return state;
}

void CarriageCanAdapter::begin()
{
   if (!_router)
      return;
   _router->on(CAN_ID_CARRIAGE_SET_ACCELERATION, &CarriageCanAdapter::staticHandleSetAcceleration, this);
   _router->on(CAN_ID_CARRIAGE_LIMIT_STATUS, &CarriageCanAdapter::staticHandleLimitStatus, this);
   _router->on(CAN_ID_CARRIAGE_HOME, &CarriageCanAdapter::staticHandleHome, this);
   _router->on(CAN_ID_CARRIAGE_MOVE_REL, &CarriageCanAdapter::staticHandleMoveRel, this);
   _router->on(CAN_ID_CARRIAGE_MOVE_ABS, &CarriageCanAdapter::staticHandleMoveAbs, this);
   _router->on(CAN_ID_CARRIAGE_STOP, &CarriageCanAdapter::staticHandleStop, this);
}

void CarriageCanAdapter::requestLimitStatus()
{
   if (!_router)
      return;
   sendLimitsRequest();
}

void CarriageCanAdapter::tick()
{
   if (!_controller || !_router)
      return;

   bool moving = _controller->isMoving();
   uint32_t nowMs = millis();

   if (moving && (nowMs - _lastPositionTxMs >= POSITION_TX_INTERVAL_MS))
   {
      sendPosition(_controller->getPositionMm());
      _lastPositionTxMs = nowMs;
   }

   float limitHitPositionMm = 0.0f;
   uint8_t limitHitDirection = 0;
   if (_controller->consumeLimitHitEvent(limitHitPositionMm, limitHitDirection))
   {
      if (_controller->isHoming())
      {
         _controller->zeroPosition();
         sendHomed(1u, _controller->getPositionMm());
      }
      else
      {
         sendLimitHit(limitHitPositionMm, limitHitDirection);
      }
   }

   if (!_absMoveDonePending)
      return;

   if (!_absMoveWasMoving)
   {
      if (moving)
         _absMoveWasMoving = true;
      return;
   }

   if (!moving)
   {
      float positionMm = _controller->getPositionMm();
      sendMoveDone(positionMm);
      _absMoveDonePending = false;
      _absMoveWasMoving = false;
   }
}

void CarriageCanAdapter::staticHandleMoveRel(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<CarriageCanAdapter *>(ctx);
   if (adapter)
      adapter->handleMoveRel(msg);
}

void CarriageCanAdapter::staticHandleMoveAbs(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<CarriageCanAdapter *>(ctx);
   if (adapter)
      adapter->handleMoveAbs(msg);
}

void CarriageCanAdapter::staticHandleSetAcceleration(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<CarriageCanAdapter *>(ctx);
   if (adapter)
      adapter->handleSetAcceleration(msg);
}

void CarriageCanAdapter::staticHandleLimitStatus(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<CarriageCanAdapter *>(ctx);
   if (adapter)
      adapter->handleLimitStatus(msg);
}

void CarriageCanAdapter::staticHandleHome(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<CarriageCanAdapter *>(ctx);
   if (adapter)
      adapter->handleHome(msg);
}

void CarriageCanAdapter::staticHandleStop(const twai_message_t *msg, void *ctx)
{
   (void)msg;
   auto *adapter = static_cast<CarriageCanAdapter *>(ctx);
   if (adapter)
      adapter->handleStop();
}

void CarriageCanAdapter::handleMoveRel(const twai_message_t *msg)
{
   if (!_controller)
      return;
   if (_mode == SystemMode::FW_UPDATE)
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX blocked in FW_UPDATE\n", (unsigned long)msg->identifier);
      if (_espCan)
         _espCan->sendAck(1, 0);
      return;
   }
   if (msg->data_length_code != 8)
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX requires DLC=8\n", (unsigned long)msg->identifier);
      if (_espCan)
         _espCan->sendAck(1, 1);
      return;
   }

   float distance = 0.0f;
   if (!unpackFloatLE(&msg->data[0], distance))
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX invalid distance\n", (unsigned long)msg->identifier);
      if (_espCan)
         _espCan->sendAck(1, 2);
      return;
   }

   float speed = (float)abs(unpackI16LE(&msg->data[4])) / 100.0f;
   if (speed <= 0.0f)
      speed = _defaultSpeed;

   USBSerial.printf("CAN carriage: REL distance=%.3f speed=%.3f\n", distance, speed);
   _controller->moveRelative(distance, speed);
   if (_espCan)
      _espCan->sendAck(0, 0);
}

void CarriageCanAdapter::handleMoveAbs(const twai_message_t *msg)
{
   if (!_controller)
      return;
   if (_mode == SystemMode::FW_UPDATE)
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX blocked in FW_UPDATE\n", (unsigned long)msg->identifier);
      return;
   }
   if (msg->data_length_code != 8)
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX requires DLC=8\n", (unsigned long)msg->identifier);
      return;
   }

   float position = 0.0f;
   if (!unpackFloatLE(&msg->data[0], position))
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX invalid position\n", (unsigned long)msg->identifier);
      return;
   }

   int16_t speed_x100 = abs(unpackI16LE(&msg->data[4]));
   float speed = (float)speed_x100 / 100.0f;
   if (speed <= 0.0f)
      speed = _defaultSpeed;

   bool movingBefore = _controller->isMoving();
   USBSerial.printf("CAN carriage: ABS position=%.3f speed=%.3f\n", position, speed);
   _controller->moveAbsolute(position, speed);
   bool movingAfter = _controller->isMoving();
   _absMoveDonePending = (!movingBefore && movingAfter);
   _absMoveWasMoving = _absMoveDonePending;
}

void CarriageCanAdapter::handleSetAcceleration(const twai_message_t *msg)
{
   if (!_controller)
      return;
   if (_mode == SystemMode::FW_UPDATE)
      return;
   if (msg->data_length_code != 8)
      return;

   uint16_t accel = unpackU16LE(&msg->data[0]);
   _controller->setAccelerationMmPerS2(accel);
}

void CarriageCanAdapter::handleLimitStatus(const twai_message_t *msg)
{
   if (!_controller)
      return;

   if (msg->data_length_code < 2)
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX requires DLC>=2\n", (unsigned long)msg->identifier);
      return;
   }

   const bool minTriggered = (msg->data[0] != 0);
   const bool maxTriggered = (msg->data[1] != 0);
   // Cache latest remote limit state for Axis::checkLimitSwitches polling path.
   _remoteMinTriggered = minTriggered;
   _remoteMaxTriggered = maxTriggered;

   USBSerial.printf("CAN carriage: LIMIT_STATUS min=%u max=%u\n", minTriggered ? 1 : 0, maxTriggered ? 1 : 0);
}

void CarriageCanAdapter::handleHome(const twai_message_t *msg)
{
   if (!_controller)
      return;
   if (_mode == SystemMode::FW_UPDATE)
   {
      if (_espCan)
         _espCan->sendAck(1, 0);
      return;
   }
   if (msg->data_length_code != 8)
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX requires DLC=8\n", (unsigned long)msg->identifier);
      if (_espCan)
         _espCan->sendAck(1, 1);
      return;
   }

   int16_t velocityX100 = unpackI16LE(&msg->data[0]);
   float velocityMmS = (float)velocityX100 / 100.0f;
   USBSerial.printf("CAN carriage: HOME velocity=%.3f mm/s\n", velocityMmS);
   _controller->homeMmPerS(velocityMmS);

   // If homing was requested but no motion started (for example, hard limit
   // already active in the homing direction), treat current position as home.
   if (_controller->isHoming() && !_controller->isMoving())
   {
      _controller->zeroPosition();
      sendHomed(1u, _controller->getPositionMm());
   }
   
   if (_espCan)
      _espCan->sendAck(0, 0);
}

void CarriageCanAdapter::handleStop()
{
   if (!_controller)
      return;
   USBSerial.println("CAN carriage: STOP");
   const bool wasHoming = _controller->isHoming();
   const float positionMm = _controller->getPositionMm();
   _controller->cancelHoming();
   _controller->stop();
   _absMoveDonePending = false;
   _absMoveWasMoving = false;
   if (wasHoming)
      sendHomed(0u, positionMm);
   if (_espCan)
      _espCan->sendAck(0, 0);
}

void CarriageCanAdapter::sendHomed(uint8_t result, float positionMm)
{
   twai_message_t tx = {};
   tx.identifier = CAN_ID_CARRIAGE_HOMED;
   tx.data_length_code = 8;
   tx.flags = 0;
   tx.data[0] = result;
   std::memcpy(&tx.data[1], &positionMm, sizeof(float));
   tx.data[5] = 0;
   tx.data[6] = 0;
   tx.data[7] = 0;

   if (_router->send(&tx))
      USBSerial.printf("CAN carriage: HOMED result=%u pos=%.3f\n", (unsigned)result, positionMm);
   else
      USBSerial.println("CAN carriage: HOMED send failed");
}

void CarriageCanAdapter::sendLimitsRequest()
{
   twai_message_t tx = {};
   tx.identifier = CAN_ID_CARRIAGE_LIMITS_REQUEST;
   tx.data_length_code = 8;
   tx.flags = 0;
   tx.data[0] = 0;
   tx.data[1] = 0;
   tx.data[2] = 0;
   tx.data[3] = 0;
   tx.data[4] = 0;
   tx.data[5] = 0;
   tx.data[6] = 0;
   tx.data[7] = 0;

   if (_router->send(&tx))
      USBSerial.println("CAN carriage: LIMITS_REQUEST sent");
   else
      USBSerial.println("CAN carriage: LIMITS_REQUEST send failed");
}

void CarriageCanAdapter::sendLimitHit(float positionMm, uint8_t direction)
{
   twai_message_t tx = {};
   tx.identifier = CAN_ID_CARRIAGE_LIMIT_HIT;
   tx.data_length_code = 8;
   tx.flags = 0;
   std::memcpy(&tx.data[0], &positionMm, sizeof(float));
   tx.data[4] = direction;
   tx.data[5] = 0;
   tx.data[6] = 0;
   tx.data[7] = 0;

   if (_router->send(&tx))
      USBSerial.printf("CAN carriage: LIMIT_HIT pos=%.3f dir=%u\n", positionMm, (unsigned)direction);
   else
      USBSerial.println("CAN carriage: LIMIT_HIT send failed");
}

void CarriageCanAdapter::sendMoveDone(float positionMm)
{
   twai_message_t tx = {};
   tx.identifier = CAN_ID_CARRIAGE_MOVE_DONE;
   tx.data_length_code = 8;
   tx.flags = 0;
   // Pack position as little-endian float in first 4 bytes; remaining bytes zero
   std::memcpy(&tx.data[0], &positionMm, sizeof(float));
   tx.data[4] = 0;
   tx.data[5] = 0;
   tx.data[6] = 0;
   tx.data[7] = 0;

   if (_router->send(&tx))
      USBSerial.printf("CAN carriage: MOVE_DONE pos=%.3f\n", positionMm);
   else
      USBSerial.println("CAN carriage: MOVE_DONE send failed");
}

void CarriageCanAdapter::sendPosition(float positionMm)
{
   float velocityMmS = _controller ? _controller->getVelocityMmS() : 0.0f;
   long velocityX100Long = lroundf(velocityMmS * 100.0f);
   if (velocityX100Long > INT16_MAX)
      velocityX100Long = INT16_MAX;
   else if (velocityX100Long < INT16_MIN)
      velocityX100Long = INT16_MIN;

   twai_message_t tx = {};
   tx.identifier = CAN_ID_CARRIAGE_POSITION;
   tx.data_length_code = 8;
   tx.flags = 0;
   std::memcpy(&tx.data[0], &positionMm, sizeof(float));
   packI16LE(&tx.data[4], (int16_t)velocityX100Long);
   tx.data[6] = 0;
   tx.data[7] = 0;

   if (!_router->send(&tx))
      USBSerial.println("CAN carriage: POSITION send failed");
}
