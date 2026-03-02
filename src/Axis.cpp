#include "Axis.h"
#include <Arduino.h>
#include "Constants.h"
// Declare a weak stall callback the main application can override (C++)
void onAxisStall(uint8_t axis) __attribute__((weak));
void onAxisStall(uint8_t axis) {}

// Static tracking for all axes with limit switches
Axis *Axis::allAxes[MAX_AXES] = {nullptr};
volatile uint8_t Axis::axisCount = 0;

Axis::Axis() : acceleration(0), currentPosition(0), initialSpeed(0), currentSpeed(0), moving(false) {}

/// @brief Axis constructor
/// @param accel Acceleration in units/s^2
/// @param stepsPerUnit Steps per unit
Axis::Axis(uint32_t accel, float stepsPerUnit, float startSpeed)
    : acceleration(accel), stepsPerUnit(stepsPerUnit), currentPosition(0), currentSpeed(0),
      moving(false), hasMinLimitSwitch(false), hasMaxLimitSwitch(false),
      startFrequency(startSpeed)
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
  moving = false;
  // USBSerial.printf("Axis accelleration set to %d steps/s/s\n", acceleration);

  for (uint8_t i = 0; i < motorCount; i++)
  {
    // USBSerial.printf("initializing motor %d\n", i);
    allMotors[i]->init();
  }
}

// Universal ISR - checks all axes to find which limit switch triggered
void IRAM_ATTR Axis::limitISR()
{
  // Safety check - don't access array if empty
  if (axisCount == 0)
    return;

  // Check all registered axes
  for (uint8_t i = 0; i < axisCount && i < MAX_AXES; i++)
  {
    Axis *axis = allAxes[i];
    if (axis == nullptr)
      continue;

    // Check min limit if configured
    if (axis->limitPinMin != 255)
    {
      bool pinState = digitalRead(axis->limitPinMin);
      axis->limitTriggeredMin = (pinState == axis->limitActiveHigh);
    }

    // Check max limit if configured
    if (axis->limitPinMax != 255)
    {
      bool pinState = digitalRead(axis->limitPinMax);
      axis->limitTriggeredMax = (pinState == axis->limitActiveHigh);
    }
  }
}

void Axis::addSingleLimitSwitch(uint8_t limitSwitchPin, bool activeHigh, float clearanceDistanceUnits)
{
  // Single limit switch configuration (for both directions)
  // This can be used for regular limit switches or TMC StallGuard DIAG pin
  limitPinMin = limitSwitchPin;
  limitActiveHigh = activeHigh;
  hasMinLimitSwitch = true;
  hasMaxLimitSwitch = false;

  // Convert clearance distance from units to steps
  limitClearanceDistance = (int32_t)(clearanceDistanceUnits * stepsPerUnit);

  USBSerial.printf("Axis: Single limit clearance distance set to %.2f units (%ld steps)\n",
                   clearanceDistanceUnits, limitClearanceDistance);

  // Configure pin mode - use pullup for active LOW, regular INPUT for active HIGH
  pinMode(limitPinMin, activeHigh ? INPUT : INPUT_PULLUP);

  // Register this axis if there's room
  if (axisCount < MAX_AXES)
  {
    uint8_t myIndex = axisCount;
    allAxes[myIndex] = this;
    axisCount++; // Increment after assignment

    // Attach universal ISR to this pin
    attachInterrupt(digitalPinToInterrupt(limitPinMin), limitISR, CHANGE);
    USBSerial.printf("Axis: Attached limit switch on pin %d (axis %d/%d)\n", limitPinMin, myIndex + 1, MAX_AXES);
  }
  else
  {
    USBSerial.printf("ERROR: Maximum %d axes with limit switches supported! Increase MAX_AXES.\n", MAX_AXES);
  }
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

void Axis::addLimitSwitch(uint8_t limitSwitchMinPin, uint8_t limitSwitchMaxPin, bool activeHigh)
{
  // Separate limit switches for min and max directions
  limitPinMin = limitSwitchMinPin;
  limitPinMax = limitSwitchMaxPin;
  limitActiveHigh = activeHigh;
  hasMinLimitSwitch = true;
  hasMaxLimitSwitch = true;

  // Configure pin modes
  pinMode(limitPinMin, activeHigh ? INPUT : INPUT_PULLUP);
  pinMode(limitPinMax, activeHigh ? INPUT : INPUT_PULLUP);

  // Register this axis if there's room
  if (axisCount < MAX_AXES)
  {
    uint8_t myIndex = axisCount;
    allAxes[myIndex] = this;
    axisCount++; // Increment after assignment

    // Attach universal ISR to both pins
    attachInterrupt(digitalPinToInterrupt(limitPinMin), limitISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(limitPinMax), limitISR, CHANGE);
    USBSerial.printf("Axis: Attached min/max limits on pins %d/%d (axis %d/%d)\n",
                     limitPinMin, limitPinMax, myIndex + 1, MAX_AXES);
  }
  else
  {
    USBSerial.printf("ERROR: Maximum %d axes with limit switches supported! Increase MAX_AXES.\n", MAX_AXES);
  }
}

// Note: initLimitSwitches() removed - initialization happens in addLimitSwitch() via interrupts

bool Axis::checkLimitSwitches()
{
  if (!hasMinLimitSwitch && !hasMaxLimitSwitch)
    return false; // bail if there are no limit switches

  static bool wasTriggeredMin = false;
  static bool wasTriggeredMax = false;
  bool limitChanged = false;

  if (hasMaxLimitSwitch)
  {
    // Two separate limit switches (min and max)
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
  }
  else if (hasMinLimitSwitch)
  {
    // Single limit switch (for both directions) - typically DIAG pin or single physical limit
    if (limitTriggeredMin != wasTriggeredMin)
    {
      limitChanged = true;
      wasTriggeredMin = limitTriggeredMin;

      // Check if we're decelerating (ignore false stalls during decel)
      bool isDecelerating = currentSpeed > targetSpeed;
      // if (limitTriggeredMin && moving) // Only trigger when actively moving
      if (limitTriggeredMin && moving && !isDecelerating) // Only trigger when moving AND not decelerating
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
        limitDirectionWasPositive = direction;

        // Stop movement in current direction ONLY
        if (direction)
          canMovePositive = false;
        else
          canMoveNegative = false;

        USBSerial.printf("  AFTER: canMovePos=%d, canMoveNeg=%d\n", canMovePositive, canMoveNegative);
        // Notify application of stall/limit event
        onAxisStall((uint8_t)(this == Axis::allAxes[0] ? 0 : 1));
      }
      else if (!limitTriggeredMin && !moving && (!canMovePositive || !canMoveNegative))
      {
        // Motion has stopped after hitting limit - correct position to where we actually hit it
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
      int32_t distanceMoved = currentPosition - positionAtLimit;

      // If we were moving positive and hit limit, need to move negative to clear
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

  // If any limit switch was triggered, stop the axis in the prescribed "stop distance"
  if (limitChanged && (!canMovePositive || !canMoveNegative))
  {
    targetSpeed = 0;
    stepsToGo = stopDistance;
  }

  // Return true if any limit switch is currently triggered
  return limitTriggeredMin || limitTriggeredMax;
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
  isHomed = true; // Mark axis as homed when zeroed
  USBSerial.println("Axis zeroed and marked as HOMED");
}

/// @brief sets the absolute position of the axis
/// @param position signed absolute position in steps
void Axis::setPosition(int32_t position)
{
  currentPosition = position;
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