#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "DriverInterface.h"

/**
 * Motor - Generic stepper motor control class
 *
 * Handles basic step/direction/enable control for stepper motors.
 * Works with any stepper driver through the DriverInterface abstraction.
 *
 * Usage:
 *   Motor* motor = new Motor(STEP_PIN, DIR_PIN, EN_PIN, invertEnable, invertDir);
 *   motor->attachDriver(myDriver);  // Attach driver (TMC2209Driver, A4988Driver, etc.)
 *   motor->init();
 */

class Motor
{
private:
  uint32_t _portIndex; // used to find port
  volatile uint32_t *_step_port = nullptr;
  uint32_t _step_bitmask;
  uint8_t _step_pin;
  uint8_t _direction_pin;
  uint8_t _enable_pin;

  // Direction pin direct port manipulation
  uint32_t _direction_portIndex;
  volatile uint32_t *_direction_port = nullptr;
  uint32_t _direction_bitmask;

  // Enable pin direct port manipulation
  uint32_t _enable_portIndex;
  volatile uint32_t *_enable_port = nullptr;
  uint32_t _enable_bitmask;

  bool _invertEnable;
  bool _invertDirection;

  bool _direction;
  bool _isEnabled = false; // Track enabled state

  // Generic driver interface
  DriverInterface *_driver = nullptr;

public:
  Motor() {} // default constructor
  Motor(uint8_t stepPin,
        uint8_t directionPin,
        uint8_t enablePin,
        bool invertEnable,
        bool invertDirection);

  void init();
  uint8_t getDirectionPin();
  void stepHigh();
  void stepLow();
  bool setDirection(bool dir);
  bool enable();
  bool disable();
  bool isEnabled() const;

  // Driver management
  void attachDriver(DriverInterface *driver);
  void initDriver(bool use_velocity_mode = false);
  DriverInterface *getDriver() const;
};

#endif
