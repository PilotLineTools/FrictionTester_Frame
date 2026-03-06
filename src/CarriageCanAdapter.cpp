#include "CarriageCanAdapter.h"
#include <Arduino.h>
#include <cstring>

CarriageCanAdapter::CarriageCanAdapter(CarriageController *controller, ICanRouter *router, FrameCanAdapter *frameCan)
   : _controller(controller), _router(router), _frameCan(frameCan)
{
}

void CarriageCanAdapter::begin()
{
   if (!_router)
      return;
   _router->on(CAN_ID_CARRIAGE_SET_ACCELERATION, &CarriageCanAdapter::staticHandleSetAcceleration, this);
   _router->on(CAN_ID_CARRIAGE_JOG, &CarriageCanAdapter::staticHandleJog, this);
   _router->on(CAN_ID_CARRIAGE_HOME, &CarriageCanAdapter::staticHandleHome, this);
   _router->on(CAN_ID_CARRIAGE_MOVE_REL, &CarriageCanAdapter::staticHandleMoveRel, this);
   _router->on(CAN_ID_CARRIAGE_MOVE_ABS, &CarriageCanAdapter::staticHandleMoveAbs, this);
   _router->on(CAN_ID_CARRIAGE_STOP, &CarriageCanAdapter::staticHandleStop, this);
}

void CarriageCanAdapter::tick()
{
   if (!_controller || !_router || !_absMoveDonePending)
      return;

   bool moving = _controller->isMoving();
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

void CarriageCanAdapter::staticHandleJog(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<CarriageCanAdapter *>(ctx);
   if (adapter)
      adapter->handleJog(msg);
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
      if (_frameCan)
         _frameCan->sendAck(1, 0);
      return;
   }
   if (msg->data_length_code != 8)
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX requires DLC=8\n", (unsigned long)msg->identifier);
      if (_frameCan)
         _frameCan->sendAck(1, 1);
      return;
   }

   float distance = 0.0f;
   if (!unpackFloatLE(&msg->data[0], distance))
   {
      USBSerial.printf("CAN carriage reject: id=0x%03lX invalid distance\n", (unsigned long)msg->identifier);
      if (_frameCan)
         _frameCan->sendAck(1, 2);
      return;
   }

   float speed = (float)abs(unpackI16LE(&msg->data[4])) / 100.0f;
   if (speed <= 0.0f)
      speed = _defaultSpeed;

   USBSerial.printf("CAN carriage: REL distance=%.3f speed=%.3f\n", distance, speed);
   _controller->moveRelative(distance, speed);
   if (_frameCan)
      _frameCan->sendAck(0, 0);
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

void CarriageCanAdapter::handleJog(const twai_message_t *msg)
{
   (void)msg;
   if (!_controller)
      return;

   // Get first 2 bytes as signed int16 speed in mm/s * 100; ignore rest of payload
   // Convert to float mm/min and jog at that speed. If speed is zero, stop jogging.
   // Note: the speed is a request, and the controller may jog at a different actual

   int16_t speed_x100 = unpackI16LE(&msg->data[0]);
   float velocityMmS = (float)speed_x100 / 100.0f;
   float velocityMmMin = velocityMmS * 60.0f;

   USBSerial.printf("CAN carriage: JOG velocity=%.2f mm/s (%.2f mm/min)\n", velocityMmS, velocityMmMin);
   _controller->jogMmPerS(velocityMmS);

   // Testing mode: ignore incoming payload and always jog at a fixed speed.
   // constexpr float kTestJogMmPerS = 2.0f; // ~1000 mm/min
   // USBSerial.printf("CAN carriage: JOG test speed=%.2f mm/s\n", kTestJogMmPerS);
   // _controller->jogMmPerS(kTestJogMmPerS);
}

void CarriageCanAdapter::handleHome(const twai_message_t *msg)
{
   if (!_controller)
      return;
   if (_mode == SystemMode::FW_UPDATE)
   {
      if (_frameCan)
         _frameCan->sendAck(1, 0);
      return;
   }
   if (msg->data_length_code != 8)
   {
      if (_frameCan)
         _frameCan->sendAck(1, 1);
      return;
   }

   int16_t vel_x100 = unpackI16LE(&msg->data[0]);
   float vel = (float)vel_x100 / 100.0f;
   _controller->homeMmPerS(vel);
   if (_frameCan)
      _frameCan->sendAck(0, 0);
}

void CarriageCanAdapter::handleStop()
{
   if (!_controller)
      return;
   USBSerial.println("CAN carriage: STOP");
   _controller->stop();
   _absMoveDonePending = false;
   _absMoveWasMoving = false;
   if (_frameCan)
      _frameCan->sendAck(0, 0);
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
