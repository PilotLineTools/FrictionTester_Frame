#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>
#include <stdint.h>
#include "Motor.h"
#include "TMC2209Driver.h" // For StallGuard access

#define MAX_MOTORS 2 // should this be in config?
#define MAX_AXES 4   // Maximum number of axes that can have limit switches

class Axis
{
private:
  Motor *allMotors[MAX_MOTORS]; // array of motor objects (at least pointers to them)
  uint8_t motorCount = 0;       // number of motors in the array

  int32_t stopDistance = 0; // how far to travel after a limit switch is tripped (in steps)
  bool hasMinLimitSwitch = false;
  bool hasMaxLimitSwitch = false;

  // Limit switch pins and state
  uint8_t limitPinMin = 255;
  uint8_t limitPinMax = 255;
  bool limitActiveHigh = false; // true = active HIGH, false = active LOW (with pullup)
  volatile bool limitTriggeredMin = false;
  volatile bool limitTriggeredMax = false;

  // For single limit switch: track stall and clearance
  int32_t limitClearanceDistance = 0;     // Distance (in steps) to move away before re-enabling blocked direction
  int32_t positionAtLimit = 0;            // Position where limit was triggered
  bool limitDirectionWasPositive = false; // Direction we were moving when limit triggered

  // Static tracking for all axes (for ISR routing)
  static Axis *allAxes[MAX_AXES];
  static volatile uint8_t axisCount;

  // bool direction; // direction to move the axis (only motors are reversed)

public:
  int32_t currentPosition;                                    // in steps (or pulses really, we don't care about microstepping - just pulses)
  int32_t acceleration;                                       // stored in steps/sec^2
  int32_t initialSpeed, currentSpeed, targetSpeed, nextSpeed; // in steps/sec
  bool moving = false;
  float stepsPerUnit;
  float startFrequency;
  int32_t stepsToGo;
  volatile bool canMovePositive = true, canMoveNegative = true;

  bool isHomed = false;            // True after axis has been homed (position is known)
  bool disableMotorAtZero = false; // If true, disable motor when position reaches zero

  // TODO: Review this changes.
  bool direction;                                             // direction to move the axis (only motors are reversed)
  Axis();                                                     // default constructor
  Axis(uint32_t accel, float stepsPerUnit, float startSpeed); // accel is in units/sec^2 (we'll convert to steps in the init function)

  void init();
  void addMotor(Motor &motor);
  void addSingleLimitSwitch(uint8_t limitSwitchPin, bool activeHigh, float clearanceDistanceUnits); // Single limit switch with clearance distance
  void addLimitSwitch(uint8_t limitSwitchMinPin, uint8_t limitSwitchMaxPin, bool activeHigh);       // Two separate limit switches
  void setDirection(bool _direction);
  void stepHigh(); // Set the step pin high and get out
  void stepLow();
  void enable();
  void disable();
  void zero();
  void setPosition(int32_t position); // in steps
  float getPosition();                // returns abs position units
  int32_t getPositionInSteps();       // returns abs position in steps
  float getSpeed();                   // returns units
  bool checkLimitSwitches();
  bool getDirection();                     // returns the current direction (true = positive, false = negative)
  void setStopDistance(float distance);    // this distance can be in engineerng units, we'll convert to steps in the function
  float getStopDistance();                 // returns the prescribed stopping distance in units
  int32_t getStopDistanceInSteps();        // returns the prescribed stopping distance in steps
  void setDisableMotorAtZero(bool enable); // Enable/disable motor disable at zero position
  void checkAndDisableAtZero();            // Check if at zero and disable motor if configured
  void setHomed(bool homed);               // Set homing state (true = position is known)
  bool getIsHomed();                       // Get homing state

  // Universal ISR handler - checks all axes to find which one triggered
  static void IRAM_ATTR limitISR();
};

#endif // AXIS_H
