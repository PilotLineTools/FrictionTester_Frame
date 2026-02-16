#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "Axis.h"
#include <stdint.h>

class MotionController
{
public:
   MotionController();

  // Methods
  void addAxis(Axis &axis);
  uint32_t updatePositions();
  void handleStepPulseStart();
  void handleStepPulseEnd();
  void startHardwareTimer3();
  void startHardwareTimer4();
  void stopHardwareTimer4();
  void addRelativeMove(uint8_t axis, float distance, float targetSpeed);
  void addAbsoluteMove(uint8_t axis, float position, float targetSpeed);
  uint8_t makeMoves();                                           // returns zero if there are no moves to make, otherwise returns the number of moves in the move buffer
  void moveRel(uint8_t axis, float distance, float targetSpeed); // does a move by calling addRelativeMove and makeMoves all inside of itself
  void moveAbs(uint8_t axis, float distance, float targetSpeed);
  void stop();
  void stopFast();
  bool pollAxis(); // polls all the axis for their limit switches
  void init();
  bool isMoving();
  bool isStopped();
  uint32_t loadMove();                // loads a move from the move array
  float getAbsPosition(uint8_t axis); // returns the absolute position of the axis in engineering units
  uint8_t getMoveIndex();

private:

  Axis *axisArray[2]; // TODO  keep track of how many axis are added and not just set it to 3
  uint8_t primaryAxis;
  uint16_t targetSpeed;
  uint16_t compareRegisterA = 65535;
  int8_t moveIndex = 0;
  uint8_t axisCount = 0; // how many axis have been added


  enum moveTypes
  {
    relativeMove,
    absoluteMove,
  };

  struct move
  {
    uint8_t axis;
    int32_t distance;
    uint16_t speed;
    bool direction;
    moveTypes type;
  } moveArray[10];

  int32_t calculateRamp(int _axis);
  float calculateAcceleration(uint32_t distance);
  void printMoveArray();
};

#endif // MOTIONCONTROLLER_H
