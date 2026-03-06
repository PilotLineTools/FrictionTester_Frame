#include "CarriageController.h"
#include <Arduino.h>
#include <math.h>

CarriageController::CarriageController(MotionController *motionController, Axis *carriageAxis)
   : _motionController(motionController),
     _carriageAxis(carriageAxis)
{
}

void CarriageController::moveRelative(float distance, float speed)
{
   if (_motionController)
      _motionController->moveRel(AxisId::Carriage, distance, speed);
}

void CarriageController::moveAbsolute(float position, float speed)
{
   if (_motionController)
      _motionController->moveAbs(AxisId::Carriage, position, speed);
}

void CarriageController::setAccelerationMmPerS2(uint16_t accelMmS2)
{
   if (!_carriageAxis)
      return;
   _carriageAxis->acceleration = (int32_t)((float)accelMmS2 * _carriageAxis->stepsPerUnit);
   USBSerial.printf("Carriage accel set to %u mm/s^2\n", (unsigned)accelMmS2);
}

void CarriageController::jogMmPerS(float velocityMmS)
{
   if (!_motionController)
      return;
   if (velocityMmS == 0.0f)
   {
      stop();
      return;
   }

   float speed = fabsf(velocityMmS);
   float distance = (velocityMmS > 0.0f) ? 1.0f : -1.0f;
   _motionController->moveRel(AxisId::Carriage, distance, 32000);
}

void CarriageController::homeMmPerS(float velocityMmS)
{
   if (!_motionController)
      return;
   if (_carriageAxis)
      _carriageAxis->setHomed(false);

   float v = (velocityMmS == 0.0f) ? -10.0f : velocityMmS;
   float speed = fabsf(v);
   float distance = (v > 0.0f) ? 1000000.0f : -1000000.0f;
   _motionController->moveRel(AxisId::Carriage, distance, speed);
}

void CarriageController::stop()
{
   if (_motionController)
      _motionController->stop();
}

bool CarriageController::isMoving() const
{
   if (!_motionController)
      return false;
   return _motionController->isMoving();
}

float CarriageController::getPositionMm() const
{
   if (!_motionController)
      return 0.0f;
   return _motionController->getAbsPosition(AxisId::Carriage);
}
