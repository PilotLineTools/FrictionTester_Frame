#include "MotionController.h"
#include <Arduino.h>
#include "Constants.h"

volatile bool stallDetected_X = false;
volatile bool stallDetected_Z = false;

MotionController::MotionController() : primaryAxis(0)
{
   for (int i = 0; i < kMaxAxes; ++i)
      axisArray[i] = nullptr;
}

void MotionController::init()
{
   for (uint8_t i = 0; i < axisCount; i++)
   {
      if (axisArray[i] != nullptr)
         axisArray[i]->init();
   }
   //   startHardwareTimer3();
   // startHardwareTimer4();
}

void MotionController::addAxis(Axis &axis)
{
   for (uint8_t i = 0; i < kMaxAxes; i++)
   {
      if (axisArray[i] == nullptr)
      {
         axisArray[i] = &axis;
         if (axisCount < (i + 1))
            axisCount = i + 1;
         return;
      }
   }
   USBSerial.printf("ERROR: addAxis failed, max axes reached (%u)\n", (unsigned)kMaxAxes);
}

void MotionController::addAxis(Axis &axis, AxisId id)
{
   if (!isValidAxisId(id))
   {
      USBSerial.printf("ERROR: addAxis failed, invalid axis id: %d\n", (int)id);
      return;
   }

   uint8_t index = axisToIndex(id);
   if (index >= kMaxAxes)
   {
      USBSerial.printf("ERROR: addAxis failed, axis index %u out of range (max=%u)\n", (unsigned)index, (unsigned)(kMaxAxes - 1));
      return;
   }

   axisArray[index] = &axis;
   if (axisCount < (index + 1))
      axisCount = index + 1;
}

int32_t MotionController::calculateRamp(int _axis)
{
   float vf2 = sq((float)axisArray[primaryAxis]->nextSpeed);
   float vi2 = sq((float)axisArray[primaryAxis]->currentSpeed);

   if (axisArray[primaryAxis]->acceleration > 0)
   {
      return max((vi2 - vf2) / (2 * axisArray[primaryAxis]->acceleration), 1.0f); // don't allow zero value becuase it crashes
   }
   else
   {
      return 0;
   }
}

float MotionController::calculateAcceleration(uint32_t distance)
{
   float vf2 = sq((float)axisArray[primaryAxis]->nextSpeed);
   float vi2 = sq((float)axisArray[primaryAxis]->currentSpeed);
   float acceleration;
   if (distance == 0)
   {
      acceleration = axisArray[primaryAxis]->acceleration;
   }
   else
   {
      acceleration = (vi2 - vf2) / (2 * distance);
   }
   // USBSerial.print("Acceleration calculated = ");USBSerial.println(acceleration);
   return acceleration;
}

void MotionController::printMoveArray()
{
   for (int i = 0; i <= moveIndex; i++)
   {
      USBSerial.println("------*********--------");
      USBSerial.print("move array index = ");
      USBSerial.println(i);
      USBSerial.print("axis = ");
      USBSerial.println(moveArray[i].axis);
      USBSerial.print("direction = ");
      USBSerial.println(moveArray[i].direction);
      USBSerial.print("distance = ");
      USBSerial.println(moveArray[i].distance);
      USBSerial.print("speed = ");
      USBSerial.println(moveArray[i].speed);
      USBSerial.print("type = ");
      USBSerial.println(moveArray[i].type);
      USBSerial.println("------*********--------");
   }
}

uint32_t MotionController::loadMove()
{
   // set the axtive Axis
   primaryAxis = moveArray[moveIndex].axis; // TODO we compare positions from this move and the next assuming they're the same axis. BAD!

   // set the target speed
   axisArray[primaryAxis]->targetSpeed = moveArray[moveIndex].speed;

   // set stepsToGo and direction according to move type
   if (moveArray[moveIndex].type == relativeMove)
   {                                                                          // for relative moves
      axisArray[primaryAxis]->direction = moveArray[moveIndex].direction;     // set the primary axis direction to the direction from move array
      axisArray[primaryAxis]->stepsToGo = abs(moveArray[moveIndex].distance); // and set stepsToGo to the steps in the move array
   }
   else if (moveArray[moveIndex].type == absoluteMove)
   { // for absolute moves
      if (moveArray[moveIndex].distance > axisArray[primaryAxis]->currentPosition)
      {                                                                          // If the target position is greater than the current position ...
         moveArray[moveIndex].direction = axisArray[primaryAxis]->direction = 1; // Set the direction to 1 (increaseing steps)
      }
      else
      {                                                                          // if it's less than the current position
         moveArray[moveIndex].direction = axisArray[primaryAxis]->direction = 0; // set the direction to 0 (decreaseing steps)
      }
      axisArray[primaryAxis]->stepsToGo = abs(axisArray[primaryAxis]->currentPosition - moveArray[moveIndex].distance); // and set stepsToGo as the absolute difference in current steps and target steps
   }
   axisArray[primaryAxis]->setDirection(axisArray[primaryAxis]->direction);

   // find the next target speed
   if (moveIndex > 0 && (moveArray[moveIndex].direction == moveArray[moveIndex - 1].direction))
   {
      axisArray[primaryAxis]->nextSpeed = moveArray[moveIndex - 1].speed;
   }
   else
   {
      axisArray[primaryAxis]->nextSpeed = initialSpeed;
   }

   axisArray[primaryAxis]->moving = true;
   axisArray[primaryAxis]->enable();

   //   USBSerial.print("This is move ");USBSerial.println(moveIndex);
   //   USBSerial.print("axis stepsToGo = "); USBSerial.println(axisArray[primaryAxis]->stepsToGo);
   //   USBSerial.print("movearray distance = ");USBSerial.println(moveArray[moveIndex].distance);
   //   USBSerial.print("direction = ");USBSerial.println(axisArray[primaryAxis]->direction);
   //   USBSerial.print("target speed = ");USBSerial.println(axisArray[primaryAxis]->targetSpeed);
   //   USBSerial.print("next speed = ");USBSerial.println(axisArray[primaryAxis]->nextSpeed);
   //   USBSerial.print("-------\n\n");

   return axisArray[primaryAxis]->stepsToGo;
}

void MotionController::addRelativeMove(uint8_t axis, float distance, float targetSpeed)
{
   USBSerial.printf("#### addRelativeMove #####");
   if (axisArray[axis] == nullptr)
   {
      USBSerial.printf("ERROR: Null axis pointer in moveRel - axis=%d\n", axis);
      return;
   }
   moveArray[moveIndex].axis = axis;
   moveArray[moveIndex].distance = axisArray[axis]->stepsPerUnit * distance;
   moveArray[moveIndex].speed = axisArray[axis]->stepsPerUnit * targetSpeed; // tics/minute

   USBSerial.printf("axisArray[axis]->stepsPerUnit =%f and moveIndex = %d.\n", axisArray[axis]->stepsPerUnit, moveIndex);

   if (distance > 0)
      moveArray[moveIndex].direction = 1;
   else
      moveArray[moveIndex].direction = 0;

   moveArray[moveIndex].type = relativeMove;
   moveIndex++;
}

void MotionController::addAbsoluteMove(uint8_t axis, float position, float targetSpeed)
{
   moveArray[moveIndex].axis = axis;
   moveArray[moveIndex].distance = axisArray[axis]->stepsPerUnit * position;
   moveArray[moveIndex].speed = axisArray[axis]->stepsPerUnit * targetSpeed;
   if (position > 0)
      moveArray[moveIndex].direction = 1; // only really useful for the first direction
   else
      moveArray[moveIndex].direction = 0; // this direction will get overwritten when we loadMove()
   moveArray[moveIndex].type = absoluteMove;
   moveIndex++;
}

uint8_t MotionController::makeMoves()
{
   if (moveIndex == 0)
   {
      USBSerial.println("no moves to make");
      return 0;
   }
   else
   {
      moveIndex--;
      loadMove();
      // printMoveArray();
      // startHardwareTimer3();
      // startHardwareTimer4();
      return moveIndex;
   }
}

void MotionController::moveRel(uint8_t axis, float distance, float targetSpeed)
{
   USBSerial.printf("FUNCTION: moveRel\n");
   if (axis >= kMaxAxes)
   {
      USBSerial.printf("ERROR: Invalid axis index in moveRel maxAxes=%u\n", (unsigned)kMaxAxes);
      return;
   }

   if (axisArray[axis] == nullptr)
   {
      USBSerial.printf("ERROR: Null axis pointer in moveRel - axis=%d\n", axis);
      return;
   }
   // target speed is in units / second
   bool _direction = distance > 0 ? 1 : 0; // set the direction based on the sign of distance

   // HARD LIMITS: Physical limits (StallGuard/mechanical hard stops) - ALWAYS enforced
   if ((_direction && !axisArray[axis]->canMovePositive) || (!_direction && !axisArray[axis]->canMoveNegative))
   {
      USBSerial.printf("!!! HARD LIMIT: Movement blocked, dir=%s, canMovePos=%d, canMoveNeg=%d\n",
                       _direction ? "POSITIVE" : "NEGATIVE",
                       axisArray[axis]->canMovePositive,
                       axisArray[axis]->canMoveNegative);
      return;
   }

   if (isMoving())
   {
      USBSerial.println("MoveRel: Axis is moving, cannot move");
      return;
   }

   addRelativeMove(axis, distance, targetSpeed);
   axisArray[axis]->currentSpeed = axisArray[axis]->startFrequency;
   makeMoves();
}

void MotionController::moveAbs(uint8_t axis, float distance, float targetSpeed)
{

   // Convert target position from mm to steps for comparison
   int32_t targetPosSteps = axisArray[axis]->stepsPerUnit * distance;
   bool _direction = targetPosSteps > axisArray[axis]->currentPosition ? 1 : 0;

   USBSerial.printf("MotionController moveAbs: axis=%d, currentPos=%ld steps (%.1fmm), targetPos=%.1fmm (%ld steps), targetSpeed=%.1fmm/s, dir=%s, canMovePos=%d, canMoveNeg=%d\n",
                    axis,
                    axisArray[axis]->currentPosition,
                    axisArray[axis]->currentPosition / axisArray[axis]->stepsPerUnit,
                    distance,
                    targetPosSteps,
                    targetSpeed,
                    _direction ? "POS" : "NEG",
                    axisArray[axis]->canMovePositive,
                    axisArray[axis]->canMoveNegative);

   // HARD LIMITS: Physical limits (StallGuard/mechanical hard stops) - ALWAYS enforced
   if ((_direction && !axisArray[axis]->canMovePositive) || (!_direction && !axisArray[axis]->canMoveNegative))
   {
      USBSerial.printf("!!! HARD LIMIT: Movement blocked, dir=%s, canMovePos=%d, canMoveNeg=%d\n",
                       _direction ? "POSITIVE" : "NEGATIVE",
                       axisArray[axis]->canMovePositive,
                       axisArray[axis]->canMoveNegative);
      return;
   }

   
   if (isMoving())
   {
      USBSerial.println("MoveAbs: Axis is moving, cannot move");
      return;
   }

   addAbsoluteMove(axis, distance, targetSpeed);
   makeMoves();
}

uint32_t MotionController::updatePositions()
{
   // TODO: While testing this function cause error.
   // pollAxis();
   if (axisArray[primaryAxis]->moving)
   {
      int32_t distanceToStop = calculateRamp(primaryAxis);
      uint32_t _targetSpeed;
      float acceleration;

      if (axisArray[primaryAxis]->stepsToGo <= distanceToStop)
      {
         _targetSpeed = axisArray[primaryAxis]->nextSpeed;
      }
      else
      {
         _targetSpeed = axisArray[primaryAxis]->targetSpeed;
      }

      if (axisArray[primaryAxis]->currentSpeed < _targetSpeed)
      {
         acceleration = axisArray[primaryAxis]->acceleration;
         axisArray[primaryAxis]->currentSpeed = min((float)axisArray[primaryAxis]->currentSpeed + acceleration * timeStep, (float)_targetSpeed);
      }
      if (axisArray[primaryAxis]->currentSpeed > _targetSpeed)
      {
         acceleration = calculateAcceleration(axisArray[primaryAxis]->stepsToGo);
         axisArray[primaryAxis]->currentSpeed = max((float)axisArray[primaryAxis]->currentSpeed - acceleration * timeStep, (float)initialSpeed);
      }

      // Serial.print(distanceToStop);
      // Serial.print("\t");
      // Serial.print(moveArray[moveIndex].type);
      // Serial.print("\t");
      // Serial.print(axisArray[primaryAxis]->direction);
      // Serial.print("\t");
      // Serial.print(axisArray[primaryAxis]->currentSpeed);
      // Serial.print("\t");
      // Serial.print(axisArray[primaryAxis]->stepsToGo);
      // Serial.print("\t");
      // Serial.println(acceleration);

      compareRegisterA = (baseFrequency / 8) / axisArray[primaryAxis]->currentSpeed - 1;
   }
   else
   {
      // Motion has stopped - check if we should disable motor at zero
      axisArray[primaryAxis]->checkAndDisableAtZero();
   }
   return compareRegisterA;
}

bool MotionController::isMoving()
{
   for (uint8_t i = 0; i < axisCount; i++)
   {
      if (axisArray[i] != nullptr && axisArray[i]->moving)
      {
         return true;
      }
   }
   return false;
}

bool MotionController::isStopped()
{
   return !isMoving();
}

void MotionController::handleStepPulseStart()
{
   if (axisArray[primaryAxis]->stepsToGo > 0)
   {                                       // Ensure there are steps to perform
                                           //  USBSerial.println("Handle step pulse start: in IF condition");
      axisArray[primaryAxis]->stepHigh();  // set the step pin high (this function also increases or decreases the axis's position)
      axisArray[primaryAxis]->stepsToGo--; // decrease steps To Go

      if (axisArray[primaryAxis]->stepsToGo == 0)
      { // If we ran out of steps for this move
         if (moveIndex > 0)
         {               // and there are more moves available
            moveIndex--; //
            while (loadMove() == 0)
            {
               moveIndex--;
            }
         }
         else
         {
            axisArray[primaryAxis]->moving = false;
            axisArray[primaryAxis]->currentSpeed = 0;
         }
      }
      axisArray[primaryAxis]->stepLow(); // set the step pin high (this function also increases or decreases the axis's position)
   }
}

void MotionController::stop()
{
   if (axisArray[primaryAxis]->moving)
   {
      axisArray[primaryAxis]->targetSpeed = 0;
      axisArray[primaryAxis]->stepsToGo = calculateRamp(primaryAxis);
      USBSerial.printf("INFO: axisArray[primaryAxis]->stepsToGo = %d\n", axisArray[primaryAxis]->stepsToGo);
   }
}

void MotionController::stopFast()
{
   axisArray[primaryAxis]->targetSpeed = 0; // TODO fix this so it actually stops fast, like disable the steppers
}

bool MotionController::pollAxis()
{
   // USBSerial.printf("Current Axis count %d\n", axisCount);
   for (uint8_t i = 0; i < axisCount; i++)
   {
      if (axisArray[i] != nullptr)
      {
         if (axisArray[i]->checkLimitSwitches())
         {
            // anyLimitSwitchTripped = true;
            // limitSwitchAxis = i; // Capture the axis index that hit the limit switch
         }
      }
      else
      {
         USBSerial.printf("WARNING: Null axis pointer in pollSwitches() at index %d\n", i);
      }
   }
   return true;
}

float MotionController::getAbsPosition(uint8_t axis)
{
   return (float)axisArray[axis]->currentPosition / axisArray[axis]->stepsPerUnit;
}

uint8_t MotionController::getMoveIndex()
{
   return moveIndex;
}
