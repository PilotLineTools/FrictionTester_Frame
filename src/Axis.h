#ifndef AXIS_H
#define AXIS_H

#include <Arduino.h>
#include <stdint.h>
#include "Motor.h"
#include "TMC2209Driver.h" // For StallGuard access
#include "LimitSwitch.h"
#include "AxisId.h"

#define MAX_MOTORS 2 // should this be in config?

class Axis
{
public:
  struct LimitState
  {
    bool minTriggered = false;
    bool maxTriggered = false;
  };
  struct LimitHitEvent
  {
    bool valid = false;
    float positionUnits = 0.0f;
    uint8_t directionCode = 0u;
  };

private:
  Motor *allMotors[MAX_MOTORS]; // array of motor objects (at least pointers to them)
  uint8_t motorCount = 0;       // number of motors in the array

  int32_t stopDistance = 0; // how far to travel after a limit switch is tripped (in steps)
  // Unified limit wiring model: one attached LimitSwitch object that can report
  // either single-direction state (min only) or dual min/max state.
  bool _hasLimitSwitch = false;
  bool _hasDualLimitSource = false;
  LimitSwitch *_limitSwitch = nullptr;

  volatile bool limitTriggeredMin = false;
  volatile bool limitTriggeredMax = false;
  bool wasTriggeredMin = false;
  bool wasTriggeredMax = false;

  // For single limit switch: track stall and clearance
  int32_t limitClearanceDistance = 0;     // Distance (in steps) to move away before re-enabling blocked direction
  int32_t positionAtLimit = 0;            // Position where limit was triggered
  bool limitDirectionWasPositive = false; // Direction we were moving when limit triggered
  bool singleLimitAppliesPositive = true;
  bool singleLimitAppliesNegative = true;

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
  // Single API for all limit sources (local/remote, single/dual).
  // If the source is single-direction, clearanceDistanceUnits controls re-enable distance.
  void addLimitSwitch(LimitSwitch &limitSwitch, float clearanceDistanceUnits = 0.0f);
  void setSingleLimitDirections(bool allowPositiveDirectionTrigger, bool allowNegativeDirectionTrigger);
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
  bool hasMinLimit() const { return _hasLimitSwitch; }
  bool hasMaxLimit() const { return _hasDualLimitSource; }
  bool isSingleLimitRawTriggered() const { return _limitSwitch && !_hasDualLimitSource && _limitSwitch->getState().minTriggered; }
  bool isSingleLimitDirectionEnabled(bool positiveDirection) const
  {
    return positiveDirection ? singleLimitAppliesPositive : singleLimitAppliesNegative;
  }
  bool isMinLimitTriggered() const { return limitTriggeredMin; }
  bool isMaxLimitTriggered() const { return limitTriggeredMax; }
  LimitState getLimitState() const { 
    LimitState state;
    state.minTriggered = limitTriggeredMin;
    state.maxTriggered = limitTriggeredMax;
    return state;
  }
  uint8_t getLimitDirectionCode() const { return limitTriggeredMax ? 1u : 0u; }
  bool consumeLimitHitEvent(LimitHitEvent &eventOut);
  void setAxisId(AxisId axisId) { _axisId = axisId; }
  AxisId getAxisId() const { return _axisId; }

private:
  void clearAttachedLimitSwitches();
  void updateSingleLimitClearance(float clearanceDistanceUnits);
  LimitHitEvent _pendingLimitHit = {};
  AxisId _axisId = AxisId::Invalid;
};

#endif // AXIS_H
