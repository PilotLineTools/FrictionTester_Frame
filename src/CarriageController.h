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
   void setPositionMm(float positionMm);
   void zeroPosition();
   void cancelHoming();
   void stop();
   bool isMoving() const;
   bool isHoming() const;
   float getPositionMm() const;
   float getVelocityMmS() const;
   bool consumeLimitHitEvent(float &positionMmOut, uint8_t &directionCodeOut);

private:
   MotionController *_motionController;
   Axis *_carriageAxis;
   float _defaultSpeed = CARRIAGE_JOG_SPEED; // ~1000 mm/min
   float _defaultJogDistance = 40.0f; // mm to move when jogging (will be multiplied by direction)
   bool _homeInProgress = false;

};

#endif // CARRIAGE_CONTROLLER_H
