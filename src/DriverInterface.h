/**
 * DriverInterface - Abstract base class for stepper motor drivers
 *
 * This interface allows Motor class to work with any stepper driver
 * (A4988, DRV8825, TMC2209, TMC5160, etc.) without tight coupling.
 *
 * Each driver implementation handles its own configuration details
 * while Motor class handles generic step/dir/enable control.
 */

#ifndef DRIVER_INTERFACE_H
#define DRIVER_INTERFACE_H

#include <Arduino.h>

class DriverInterface
{
public:
    virtual ~DriverInterface() {}

    /**
     * Initialize the driver hardware
     * @param use_velocity_mode True for velocity control mode (VACTUAL), false for step/dir mode
     */
    virtual void init(bool use_velocity_mode = false) = 0;

    /**
     * Get the driver type name for logging
     */
    virtual const char *getDriverType() const = 0;

    /**
     * Get the driver name/label (e.g., "X", "Z", "Spin")
     */
    virtual const char *getName() const = 0;
};

#endif // DRIVER_INTERFACE_H
