#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "Axis.h"
#include "AxisId.h"
#include <stdint.h>

class MotionController
{
public:
  static constexpr uint8_t kMaxAxes = 2;
   MotionController();

  // Methods
  void addAxis(Axis &axis);
  void addAxis(Axis &axis, AxisId id);
  uint32_t updatePositions();
  void handleStepPulseStart();
  void handleStepPulseEnd();
  void startHardwareTimer3();
  void startHardwareTimer4();
  void stopHardwareTimer4();
  void addRelativeMove(uint8_t axis, float distance, float targetSpeed);
  inline void addRelativeMove(AxisId axis, float distance, float targetSpeed) { addRelativeMove(axisToIndex(axis), distance, targetSpeed); }
  void addAbsoluteMove(uint8_t axis, float position, float targetSpeed);
  inline void addAbsoluteMove(AxisId axis, float position, float targetSpeed) { addAbsoluteMove(axisToIndex(axis), position, targetSpeed); }
  uint8_t makeMoves();                                           // returns zero if there are no moves to make, otherwise returns the number of moves in the move buffer
  void moveRel(uint8_t axis, float distance, float targetSpeed); // does a move by calling addRelativeMove and makeMoves all inside of itself
  inline void moveRel(AxisId axis, float distance, float targetSpeed) { moveRel(axisToIndex(axis), distance, targetSpeed); }
  void moveAbs(uint8_t axis, float distance, float targetSpeed);
  inline void moveAbs(AxisId axis, float distance, float targetSpeed) { moveAbs(axisToIndex(axis), distance, targetSpeed); }
  void stop();
  void stopFast();
  bool pollAxis(); // polls all the axis for their limit switches
  void init();
  bool isMoving();
  bool isStopped();
  uint32_t loadMove();                // loads a move from the move array
  float getAbsPosition(uint8_t axis); // returns the absolute position of the axis in engineering units
  inline float getAbsPosition(AxisId axis) { return getAbsPosition(axisToIndex(axis)); }
  uint8_t getMoveIndex();

private:

  Axis *axisArray[kMaxAxes];
  uint8_t primaryAxis;
  uint16_t targetSpeed;
  /** Step period for ESP32 timer 4: next step in microseconds (1e6 / steps_per_sec). Written by updatePositions(), read by main loop for timerAlarmWrite(). */
  uint32_t stepPeriodUs = 10000;
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
