#include "Motor.h"

Motor::Motor(uint8_t stepPin,
             uint8_t directionPin,
             uint8_t enablePin,
             bool invertEnable,
             bool invertDirection)
    : _direction_pin(directionPin), _step_pin(stepPin), _enable_pin(enablePin)
{
  _step_port = nullptr;
  _portIndex = digitalPinToPort(stepPin);
  _step_port = portOutputRegister(_portIndex);
  _step_bitmask = digitalPinToBitMask(stepPin);

  // Set up direction pin for direct port manipulation
  _direction_portIndex = digitalPinToPort(directionPin);
  _direction_port = portOutputRegister(_direction_portIndex);
  _direction_bitmask = digitalPinToBitMask(directionPin);

  // Set up enable pin for direct port manipulation
  _enable_portIndex = digitalPinToPort(enablePin);
  _enable_port = portOutputRegister(_enable_portIndex);
  _enable_bitmask = digitalPinToBitMask(enablePin);

  // _step_pin = stepPin;
  // _direction_pin = directionPin;
  // _enable_pin = enablePin;
  _invertEnable = invertEnable;
  _invertDirection = invertDirection;
  _direction = false;
}

uint8_t Motor::getDirectionPin()
{
  return _direction_pin;
}

void Motor::init()
{
  pinMode(_step_pin, OUTPUT);
  pinMode(_direction_pin, OUTPUT);
  pinMode(_enable_pin, OUTPUT);

  // Initialize driver if one is attached (initDriver handles nullptr check)
  initDriver(false); // false = step/dir mode
}

void Motor::attachDriver(DriverInterface *driver)
{
  _driver = driver;
}

void Motor::initDriver(bool use_velocity_mode)
{
  if (_driver != nullptr)
  {
    _driver->init(use_velocity_mode);
  }
}

DriverInterface *Motor::getDriver() const
{
  return _driver;
}

void Motor::stepHigh()
{
  *_step_port |= _step_bitmask; // set the step pin high (take a step)
}

void Motor::stepLow()
{
  *_step_port &= ~_step_bitmask; // set the step pin low
}

bool Motor::setDirection(bool dir)
{
  _direction = dir;
  bool _dir = dir ^ _invertDirection;

  // Use direct port manipulation instead of digitalWrite() for ISR safety
  if (_dir)
  {
    *_direction_port |= _direction_bitmask; // Set direction pin high
  }
  else
  {
    *_direction_port &= ~_direction_bitmask; // Set direction pin low
  }

  return _dir;
}

bool Motor::enable()
{
  bool enableState = 1 ^ _invertEnable;

  // Use direct port manipulation instead of digitalWrite() for ISR safety
  if (enableState)
  {
    *_enable_port |= _enable_bitmask; // Set enable pin high
  }
  else
  {
    *_enable_port &= ~_enable_bitmask; // Set enable pin low
  }

  _isEnabled = true;
  return enableState;
}

bool Motor::disable()
{
  bool disableState = 0 ^ _invertEnable;

  // Use direct port manipulation instead of digitalWrite() for ISR safety
  if (disableState)
  {
    *_enable_port |= _enable_bitmask; // Set enable pin high
  }
  else
  {
    *_enable_port &= ~_enable_bitmask; // Set enable pin low
  }

  _isEnabled = false;
  return disableState;
}

bool Motor::isEnabled() const
{
  return _isEnabled;
}