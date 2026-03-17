/**
 * CarriageController - carriage-domain wrapper over MotionController.
 */

#ifndef CARRIAGE_CONTROLLER_H
#define CARRIAGE_CONTROLLER_H

#include "MotionController.h"
#include "Axis.h"
#include "Constants.h"

class CarriageController
{
public:
   CarriageController(MotionController *motionController, Axis *carriageAxis);

   void moveRelative(float distance, float speed);
   void moveAbsolute(float position, float speed);
   void setAccelerationMmPerS2(uint16_t accelMmS2);
   void jogMmPerS(float velocityMmS);             
   void homeMmPerS(float velocityMmS);
   void stop();
   bool isMoving() const;
   float getPositionMm() const;
   float getVelocityMmS() const;

private:
   MotionController *_motionController;
   Axis *_carriageAxis;
   float _defaultSpeed = CARRIAGE_JOG_SPEED; // ~1000 mm/min
   float _defaultJogDistance = 40.0f; // mm to move when jogging (will be multiplied by direction)

};

#endif // CARRIAGE_CONTROLLER_H
