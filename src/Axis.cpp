#include "Axis.h"
#include <Arduino.h>
#include <math.h>
#include "Constants.h"
// Declare a weak stall callback the main application can override (C++)
void onAxisStall(uint8_t axis) __attribute__((weak));
void onAxisStall(uint8_t axis) {}

Axis::Axis()
    : currentPosition(0),
      acceleration(0),
      initialSpeed(0),
      currentSpeed(0),
      targetSpeed(0),
      nextSpeed(0),
      moving(false),
      stepsPerUnit(0.0f),
      startFrequency(0.0f),
      stepsToGo(0),
      direction(false)
{
  for (int i = 0; i < MAX_MOTORS; ++i)
  {
    allMotors[i] = nullptr;
  }
}

/// @brief Axis constructor
/// @param accel Acceleration in units/s^2
/// @param stepsPerUnit Steps per unit
Axis::Axis(uint32_t accel, float stepsPerUnit, float startSpeed)
    : currentPosition(0),
      acceleration(accel),
      initialSpeed(0),
      currentSpeed(0),
      targetSpeed(0),
      nextSpeed(0),
      moving(false),
      stepsPerUnit(stepsPerUnit),
      startFrequency(startSpeed),
      stepsToGo(0),
      direction(false)
{
  for (int i = 0; i < MAX_MOTORS; ++i)
  {
    allMotors[i] = nullptr; // Initialize all elements to nullptr
  }
  // disable();
}

void Axis::init()
{
  acceleration *= stepsPerUnit; // converts acceleration to steps/sec^2
  startFrequency = (startFrequency * stepsPerUnit) / 60;
  currentPosition = 0;
  currentSpeed = 0;
  targetSpeed = 0;
  nextSpeed = 0;
  stepsToGo = 0;
  direction = false;
  moving = false;
  // USBSerial.printf("Axis accelleration set to %d steps/s/s\n", acceleration);

  for (uint8_t i = 0; i < motorCount; i++)
  {
    // USBSerial.printf("initializing motor %d\n", i);
    allMotors[i]->init();
    delay(100); // small delay to allow motors to initialize (especially important for TMC drivers)
  }
}

void Axis::clearAttachedLimitSwitches()
{
  // Axis does not own externally provided limit sources; it only detaches.
  _limitSwitch = nullptr;
  _hasLimitSwitch = false;
  _hasDualLimitSource = false;
}

void Axis::updateSingleLimitClearance(float clearanceDistanceUnits)
{
  // Convert clearance distance from units to steps.
  // Keep any non-zero configured clearance at >= 1 step so we don't immediately
  // re-enable the blocked direction due to integer truncation.
  const float clearanceStepsF = fabsf(clearanceDistanceUnits * stepsPerUnit);
  if (clearanceStepsF <= 0.0f)
    limitClearanceDistance = 0;
  else
    limitClearanceDistance = max((int32_t)1, (int32_t)lroundf(clearanceStepsF));

  USBSerial.printf("Axis: Single limit clearance distance set to %.2f units (%ld steps)\n",
                   clearanceDistanceUnits, limitClearanceDistance);
}

void Axis::addMotor(Motor &motor)
{
  allMotors[motorCount] = &motor;

  // USBSerial.print("Added Motor ");
  // USBSerial.print(motorCount);
  // USBSerial.print(", and it has an address of ");
  // USBSerial.println((uintptr_t)allMotors[motorCount], HEX);
  // delay(500);
  motorCount++;
}

void Axis::addLimitSwitch(LimitSwitch &limitSwitch, float clearanceDistanceUnits)
{
  clearAttachedLimitSwitches();

  _limitSwitch = &limitSwitch;
  _hasLimitSwitch = true;
  // The LimitSwitch instance decides whether this axis behaves as single or dual.
  _hasDualLimitSource = limitSwitch.supportsMax();
  _limitSwitch->begin();

  if (_hasDualLimitSource)
  {
    limitClearanceDistance = 0;
    USBSerial.println("Axis: Attached combined min/max limit source");
  }
  else
  {
    updateSingleLimitClearance(clearanceDistanceUnits);
    USBSerial.println("Axis: Attached single-direction limit source");
  }
}

void Axis::setSingleLimitDirections(bool allowPositiveDirectionTrigger, bool allowNegativeDirectionTrigger)
{
  singleLimitAppliesPositive = allowPositiveDirectionTrigger;
  singleLimitAppliesNegative = allowNegativeDirectionTrigger;
}

// Note: initLimitSwitches() removed - initialization happens in addLimitSwitch() via interrupts

bool Axis::checkLimitSwitches()
{
  if (!_hasLimitSwitch)
    return false; // bail if there are no limit switches

  // Phase 1: Sample raw limit inputs and map them into limitTriggeredMin/Max.
  // For dual-limit setups this is direct state mapping.
  // For single-limit setups we infer which side was hit from motion direction.
  if (_hasDualLimitSource)
  {
    _limitSwitch->tick();
    const LimitSwitchState state = _limitSwitch->getState();
    limitTriggeredMin = state.minTriggered;
    limitTriggeredMax = state.maxTriggered;
  }
  else
  {
    if (_limitSwitch)
    {
      _limitSwitch->tick();
      const bool rawTriggered = _limitSwitch->getState().minTriggered;
      const bool directionAllowsTrigger = direction ? singleLimitAppliesPositive : singleLimitAppliesNegative;
      bool stallguardConfirmed = true;
      // Remote single sources already represent filtered logical limit state,
      // so StallGuard confirmation is only meaningful for local DIAG-based sources.
      const bool checkStallguard = !_limitSwitch->isRemote();
      if (checkStallguard && rawTriggered && moving && motorCount > 0 && allMotors[0] != nullptr)
      {
        DriverInterface *driver = allMotors[0]->getDriver();
        if (driver != nullptr && strcmp(driver->getDriverType(), "TMC2209") == 0)
        {
          TMC2209Driver *tmc = static_cast<TMC2209Driver *>(driver);
          const uint16_t sgValue = tmc->getSGResult();
          const uint16_t sgThreshold = tmc->getSGTHRS();
          stallguardConfirmed = (sgValue <= sgThreshold);
          if (!stallguardConfirmed)
          {
            USBSerial.printf("Axis: ignoring DIAG high without StallGuard confirm (sg=%u thrs=%u dir=%s)\n",
                             (unsigned)sgValue,
                             (unsigned)sgThreshold,
                             direction ? "POSITIVE" : "NEGATIVE");
          }
        }
      }
      const bool filteredTriggered = rawTriggered && directionAllowsTrigger && stallguardConfirmed;
      if (filteredTriggered)
      {
        const bool positiveLimitHit = moving ? direction : limitDirectionWasPositive;
        limitTriggeredMin = !positiveLimitHit;
        limitTriggeredMax = positiveLimitHit;
      }
      else
      {
        limitTriggeredMin = false;
        limitTriggeredMax = false;
      }
    }
  }

  bool limitChanged = false;
  bool capturedLimitPositionThisTick = false;

  // Phase 2: Apply policy based on wiring mode.
  // Dual-limit mode directly blocks motion toward the active side.
  if (_hasDualLimitSource)
  {
    // Combined dual-limit source: direct min/max enforcement.
    if (limitTriggeredMin != wasTriggeredMin)
    {
      limitChanged = true;
      wasTriggeredMin = limitTriggeredMin;
      if (limitTriggeredMin)
      {
        USBSerial.println("Min limit triggered");
      }
    }

    if (limitTriggeredMax != wasTriggeredMax)
    {
      limitChanged = true;
      wasTriggeredMax = limitTriggeredMax;
      if (limitTriggeredMax)
      {
        USBSerial.println("Max limit triggered");
      }
    }

    canMoveNegative = !limitTriggeredMin; // Disallow negative direction if min limit is triggered
    canMovePositive = !limitTriggeredMax; // Disallow positive direction if max limit is triggered

    // Publish limit-hit events for dual sources so higher layers (e.g. carriage CAN adapter)
    // can emit LIMIT_HIT/HOMED on the same unified event path used by single-limit mode.
    if (limitChanged && (limitTriggeredMin || limitTriggeredMax) && moving)
    {
      positionAtLimit = currentPosition;
      capturedLimitPositionThisTick = true;
      limitDirectionWasPositive = direction;
      _pendingLimitHit.valid = true;
      _pendingLimitHit.positionUnits = getPosition();
      _pendingLimitHit.directionCode = limitTriggeredMax ? 1u : 0u;
    }
  }
  else
  {
    // Single limit switch mode (e.g. DIAG/stall or one physical switch).
    // A trigger is directional: block the hit direction, allow backing away.
    if (limitTriggeredMin != wasTriggeredMin || limitTriggeredMax != wasTriggeredMax)
    {
      limitChanged = true;
      wasTriggeredMin = limitTriggeredMin;
      wasTriggeredMax = limitTriggeredMax;

      // Ignore false stalls before motion is actually underway or while decelerating.
      bool hasMotionSpeed = currentSpeed > 0;
      bool isDecelerating = currentSpeed > targetSpeed;
      if ((limitTriggeredMin || limitTriggeredMax) && moving && hasMotionSpeed && !isDecelerating)
      {
        USBSerial.printf("*** LIMIT/STALL DETECTED ***\n");
        // Read StallGuard IMMEDIATELY - don't delay, read it right now
        DriverInterface *driver = allMotors[0]->getDriver();
        if (driver != nullptr && strcmp(driver->getDriverType(), "TMC2209") == 0)
        {
          TMC2209Driver *tmc = static_cast<TMC2209Driver *>(driver);
          uint16_t sgValue = tmc->getSGResult(); // Read NOW, before any delay
          uint16_t sgThreshold = tmc->getSGTHRS();
          USBSerial.printf("  Stall guard value: %d (captured immediately)\n", sgValue);
          USBSerial.printf("  Stall guard threshold: %d\n", sgThreshold);
          USBSerial.printf("  Current speed: %ld steps/s, Target speed: %ld steps/s\n", currentSpeed, targetSpeed);
        }
        USBSerial.printf("  Position: %ld\n", currentPosition);
        USBSerial.printf("  Direction: %s\n", direction ? "POSITIVE" : "NEGATIVE");
        USBSerial.printf("  BEFORE: canMovePos=%d, canMoveNeg=%d\n", canMovePositive, canMoveNegative);

        // Record position and direction when limit was hit
        positionAtLimit = currentPosition;
        capturedLimitPositionThisTick = true;
        limitDirectionWasPositive = direction;
        _pendingLimitHit.valid = true;
        _pendingLimitHit.positionUnits = getPosition();
        _pendingLimitHit.directionCode = direction ? 1u : 0u;

        // Directional lockout: only the hit direction is blocked.
        // The opposite direction remains enabled so recovery move is possible.
        if (direction)
        {
          canMovePositive = false;
          canMoveNegative = true;
        }
        else
        {
          canMoveNegative = false;
          canMovePositive = true;
        }

        USBSerial.printf("  AFTER: canMovePos=%d, canMoveNeg=%d\n", canMovePositive, canMoveNegative);
        // Notify application of stall/limit event with the configured axis ID when available.
        onAxisStall(isValidAxisId(_axisId) ? axisToIndex(_axisId) : 0);
      }
      else if (!limitTriggeredMin && !limitTriggeredMax && !moving && (!canMovePositive || !canMoveNegative))
      {
        // If the signal already cleared, keep the axis position pinned to the
        // captured hit position until clearance logic re-enables movement.
        if (currentPosition != positionAtLimit)
        {
          USBSerial.printf(">>> Position corrected: %ld -> %ld (stopped after stall)\n",
                           currentPosition, positionAtLimit);
          currentPosition = positionAtLimit;
        }
      }
      // Note: Don't re-enable on pin clear - wait for clearance distance
    }

    // Check if we've moved far enough away from the limit to re-enable that direction
    if (!canMovePositive || !canMoveNegative)
    {
      // Single-switch StallGuard should always allow motion away from the hit side.
      if (limitDirectionWasPositive)
        canMoveNegative = true;
      else
        canMovePositive = true;

      int32_t distanceMoved = currentPosition - positionAtLimit;

      // Re-enable only after configured clearance distance from captured hit point.
      // This prevents immediate retrigger chatter near the stop.
      // If we were moving positive and hit limit, need to move negative to clear.
      if (limitDirectionWasPositive && distanceMoved <= -limitClearanceDistance)
      {
        canMovePositive = true;
        USBSerial.printf(">>> Positive direction RE-ENABLED (moved %ld steps away)\n", abs(distanceMoved));
      }
      // If we were moving negative and hit limit, need to move positive to clear
      else if (!limitDirectionWasPositive && distanceMoved >= limitClearanceDistance)
      {
        canMoveNegative = true;
        USBSerial.printf(">>> Negative direction RE-ENABLED (moved %ld steps away)\n", abs(distanceMoved));
      }
    }
  }

  const bool activeSingleLimit = limitTriggeredMin || limitTriggeredMax;

  // Phase 3: Synchronize controller state with physical stop condition.
  // On a fresh active limit transition while moving, force motion state to stopped.
  // Do not retrigger this while simply backing away from a previously latched hit.
  if (limitChanged && activeSingleLimit && moving)
  {
    // If this transition was not accepted as a fresh stall/limit capture (for example
    // noise/chatter while decelerating), never snap to a stale historical limit position.
    if (!capturedLimitPositionThisTick)
      positionAtLimit = currentPosition;
    targetSpeed = 0;
    stepsToGo = 0;
    currentSpeed = 0;
    moving = false;
    currentPosition = positionAtLimit;
    USBSerial.printf(">>> Axis motion cancelled at limit position %ld\n", positionAtLimit);
  }

  // Return true if any limit switch is currently triggered
  return limitTriggeredMin || limitTriggeredMax;
}

bool Axis::consumeLimitHitEvent(LimitHitEvent &eventOut)
{
  if (!_pendingLimitHit.valid)
    return false;
  eventOut = _pendingLimitHit;
  _pendingLimitHit.valid = false;
  return true;
}

void Axis::setDirection(bool _direction)
{
  direction = _direction;
  for (uint8_t i = 0; i < motorCount; i++)
  {
    allMotors[i]->setDirection(direction);
  }
}

void Axis::stepHigh()
{
  if (direction)
    currentPosition++; // if we're moving in the positive direction and we can move positive increase the current position
  else
    currentPosition--; // decrement the current position

  for (uint8_t i = 0; i < motorCount; i++)
  { // step all motors high
    allMotors[i]->stepHigh();
  }
}

void Axis::stepLow()
{
  for (uint8_t i = 0; i < motorCount; i++)
  { // step all motors low
    allMotors[i]->stepLow();
  }
}

void Axis::enable()
{
  for (uint8_t i = 0; i < motorCount; i++)
  { // enable all motors
    allMotors[i]->enable();
  }
}

void Axis::disable()
{
  for (uint8_t i = 0; i < motorCount; i++)
  { // disable all motors
    allMotors[i]->disable();
  }
}

void Axis::zero()
{
  currentPosition = 0;
  // Keep limit-recovery reference aligned with externally-forced position so
  // stale positionAtLimit cannot overwrite a freshly-zeroed axis.
  positionAtLimit = currentPosition;
  isHomed = true; // Mark axis as homed when zeroed
  USBSerial.println("Axis zeroed and marked as HOMED");
}

/// @brief sets the absolute position of the axis
/// @param position signed absolute position in steps
void Axis::setPosition(int32_t position)
{
  currentPosition = position;
  // Keep limit-recovery reference aligned with externally-forced position.
  positionAtLimit = currentPosition;
}

/// @brief gets the current position of the axis
/// @return absolute position in units
float Axis::getPosition()
{
  return (float)currentPosition / stepsPerUnit;
}

/// @brief gets the current position of the axis
/// @return absolute position in steps
int32_t Axis::getPositionInSteps()
{
  return currentPosition;
}

/// @brief gets the current speed of the axis
/// @return speed is in units / second
float Axis::getSpeed()
{
  return (float)currentSpeed / stepsPerUnit;
}

void Axis::setStopDistance(float distance)
{
  stopDistance = stepsPerUnit * distance;
}

float Axis::getStopDistance()
{
  return stopDistance / stepsPerUnit;
}

int32_t Axis::getStopDistanceInSteps()
{
  return stopDistance;
}

bool Axis::getDirection()
{
  return direction;
}

void Axis::setDisableMotorAtZero(bool enable)
{
  disableMotorAtZero = enable;
  USBSerial.printf("Axis: Disable motor at zero = %s\n", enable ? "true" : "false");
}

void Axis::checkAndDisableAtZero()
{
  if (allMotors[0]->isEnabled() == false)
  {
    return;
  }
  if (disableMotorAtZero && currentPosition == 0 && !moving)
  {
    // We've reached zero - disable motor to allow gravity settling
    for (uint8_t i = 0; i < motorCount; i++)
    {
      allMotors[i]->disable();
    }
    USBSerial.println("Axis: Reached zero position - motor disabled for gravity settling");
  }
}

void Axis::setHomed(bool homed)
{
  isHomed = homed;
  USBSerial.printf("Axis homed state set to: %s\n", homed ? "HOMED" : "NOT HOMED");
}

bool Axis::getIsHomed()
{
  return isHomed;
}
