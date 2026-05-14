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
   _homeInProgress = false;
   if (!_motionController)
      return;

   USBSerial.printf("CarriageController: moveRelative distance=%.1fmm, speed=%.1fmm/s\n", distance, speed);
   _motionController->moveRel(AxisId::Carriage, distance, speed);
}

void CarriageController::moveAbsolute(float position, float speed)
{  
   _homeInProgress = false;
   if (!_motionController)
      return;

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
   _homeInProgress = false;
   const float positionMm = getPositionMm();
   if (velocityMmS < 0.0f && positionMm <= 0.0f)
   {
      stop();
      return;
   }

   float speed = fabsf(velocityMmS); 
   int distance_with_direction = (velocityMmS > 0.0f) ? _defaultJogDistance : -_defaultJogDistance;

   if (positionMm + distance_with_direction < 0.0f)
   {
      //distance_with_direction = -positionMm; // Adjust to not go below zero
      _motionController->moveAbs(AxisId::Carriage, 0.0f, speed); 
      return;
   }

   _motionController->moveRel(AxisId::Carriage, distance_with_direction, speed); 
}

void CarriageController::homeMmPerS(float velocityMmS)
{
   if (!_motionController)
      return;
   
   _homeInProgress = true;
   if (_carriageAxis)
      _carriageAxis->setHomed(false);

   float v = (velocityMmS == 0.0f) ? -10.0f : velocityMmS;
   float speed = fabsf(v);
   float distance = (v > 0.0f) ? 1000000.0f : -1000000.0f;
   USBSerial.printf("CarriageController: home distance=%.1fmm, speed=%.1fmm/s\n", distance, speed);
   _motionController->moveRel(AxisId::Carriage, distance, speed);
}

void CarriageController::setPositionMm(float positionMm)
{
   if (!_carriageAxis)
      return;

   _homeInProgress = false;
   int32_t positionSteps = (int32_t)lroundf(positionMm * _carriageAxis->stepsPerUnit);
   _carriageAxis->setPosition(positionSteps);
   _carriageAxis->setHomed(true);
   USBSerial.printf("Carriage position set to %.3f mm (%ld steps)\n", positionMm, (long)positionSteps);
}

void CarriageController::zeroPosition()
{
   if (!_carriageAxis)
      return;

   _homeInProgress = false;
   _carriageAxis->zero();
   USBSerial.println("CarriageController: state set to HOMED");
}

void CarriageController::cancelHoming()
{
   _homeInProgress = false;
}

void CarriageController::stop()
{
   if (_motionController)
      _motionController->stop();
}

bool CarriageController::isMoving() const
{
   if (!_carriageAxis)
      return false;
   return _carriageAxis->moving;
}

bool CarriageController::isHoming() const
{
   return _homeInProgress;
}

float CarriageController::getPositionMm() const
{
   if (!_motionController)
      return 0.0f;
   return _motionController->getAbsPosition(AxisId::Carriage);
}

float CarriageController::getVelocityMmS() const
{
   if (!_carriageAxis || !_carriageAxis->moving)
      return 0.0f;

   float speed = _carriageAxis->getSpeed();
   return _carriageAxis->getDirection() ? speed : -speed;
}

bool CarriageController::consumeLimitHitEvent(float &positionMmOut, uint8_t &directionCodeOut)
{
   if (!_carriageAxis)
      return false;

   Axis::LimitHitEvent event = {};
   if (!_carriageAxis->consumeLimitHitEvent(event))
      return false;

   positionMmOut = event.positionUnits;
   directionCodeOut = event.directionCode;
   return true;
}
