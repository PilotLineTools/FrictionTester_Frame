/**
 * TMC2209Driver - Trinamic TMC2209 stepper driver implementation
 *
 * Handles all TMC2209-specific functionality including:
 * - Physics-based current configuration
 * - StallGuard configuration
 * - Microstep settings
 * - UART communication
 *
 * Usage:
 *   TMC2209Driver* driver = new TMC2209Driver(&XStepper, "X");
 *   driver->configure(800, 16, 100, 0.100f);  // 800mA, 16 microsteps, SGTHRS=100, R_SENSE=0.1Ω
 *   driver->init();
 */

#ifndef TMC2209_DRIVER_H
#define TMC2209_DRIVER_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "DriverInterface.h"

/**
 * TMC2209 Physics-Based Current Configuration Result
 *
 * Formula: CS = round(32 * I_rms * R_SENSE * √2 / VREF - 1)
 *
 * Assumptions:
 * - Internal current reference (VREF = 0.325V for TMC2209)
 * - VSENSE = 0 (high sensitivity, 0.325V reference path)
 * - IHOLD = IRUN / 2 for power savings
 */
struct TMC2209CurrentConfig
{
    uint8_t cs_irun;        // Current scale for IRUN (0-31)
    uint8_t cs_ihold;       // Current scale for IHOLD (0-31)
    float irms_requested_A; // Requested RMS current in Amps
    float irms_achieved_A;  // Actual achieved RMS current in Amps
    float vref_used_V;      // VREF voltage used (internal reference)
    float rsense_ohm;       // Sense resistor value in Ohms
    bool clamped;           // True if CS was clamped to 0 or 31
};

struct TMC2209Config
{
    uint16_t tpwmthrs;   // StealthChop threshold (0=always StealthChop, higher=SpreadCycle)
    uint16_t tcoolthrs;  // CoolStep threshold (higher=more aggressive current reduction)
    uint8_t toff;        // Chopper off time (typical: 4)
    uint8_t tbl;         // Blank time (typical: 1, increase for lower EMI at low speeds)
    bool en_spreadcycle; // Force SpreadCycle mode (true) or StealthChop (false)

    TMC2209Config() : tpwmthrs(200), tcoolthrs(500), toff(4), tbl(1), en_spreadcycle(false) {}
};

class TMC2209Driver : public DriverInterface
{
public:
    /**
     * Clamp to TMC2209 allowed microsteps per datasheet: 1, 2, 4, 8, 16, 32, 64, 128, 256 (MRES 8..0).
     */
    static uint16_t clampMicrostepsToValid(uint16_t value);

private:
    TMC2209Stepper *_stepper;
    const char *_name;
    uint16_t _rmsCurrent; // mA
    uint16_t _microsteps; // 1, 2, 4, 8, 16, 32, 64, 128, 256
    uint8_t _sgthrs;
    float _rsense;
    TMC2209Config _config;
    const unsigned long _fclk_hz = 12000000; // 12 MHz internal clock for TMC2209
    const uint8_t _scale_exp = 24; // VACTUAL scaling exponent (2^24)

public:
    /**
     * Constructor
     * @param stepper Pointer to TMC2209Stepper object
     * @param name Driver name for logging (e.g., "X", "Z", "Spin")
     */
    TMC2209Driver(TMC2209Stepper *stepper, const char *name);

    /**
     * Configure TMC2209 parameters (call before init)
     * @param rmsCurrent Target RMS current in milliamps
     * @param microsteps Microstepping setting (1, 2, 4, 8, 16, 32, 64, 128, 256)
     * @param sgthrs StallGuard threshold (0-255)
     * @param rsense Sense resistor value in Ohms (e.g., 0.100)
     * @param config Optional configuration struct for StealthChop, CoolStep, etc.
     */
    void configure(uint16_t rmsCurrent, uint16_t microsteps, uint8_t sgthrs, float rsense, const TMC2209Config &config);

    /**
     * Initialize the TMC2209 driver
     * @param use_velocity_mode True for VACTUAL mode (velocity control), false for step/dir
     */
    void init(bool use_velocity_mode = false) override;

    /**
     * Update microstepping and push to driver (call after changing microsteps at runtime)
     * @param microsteps 1, 2, 4, 8, 16, 32, 64, 128, or 256 (datasheet MRES)
     */
    void setMicrosteps(uint16_t microsteps);

    /**
     * Read back microstep setting from driver CHOPCONF (MRES). Confirms register was written correctly.
     * @return Microsteps (1, 2, 4, 8, 16, 32, 64, 128, or 256)
     */
    uint16_t getMicrosteps() const;

    /**
     * Update RMS current and push to driver (call after changing current at runtime)
     * @param rmsCurrent Target RMS current in milliamps
     */
    void setRmsCurrent(uint16_t rmsCurrent);

    /**
     * Configure current using physics-based calculation
     * @param target_irms_mA Target RMS current in milliamps
     * @param rsense_ohm Sense resistor in Ohms
     * @param vref_volts Internal reference voltage (default 0.325V)
     * @return Configuration struct with computed values
     */
    TMC2209CurrentConfig configureCurrent(float target_irms_mA, float rsense_ohm, float vref_volts = 0.325f);

    /**
     * Set velocity in VACTUAL mode (for spin motor). Caller must pass the raw register value:
     *   VACTUAL = desired_µsteps_per_second × (2²⁴ / fCLK), signed for direction; 0 = STEP/DIR control.
     * @param vactualRegisterValue Signed VACTUAL register value (µsteps/tick of driver timing base)
     */
    void setVactual(int32_t vactualRegisterValue);

    /**
    * Set velocity in RPM (for spin motor). Converts to VACTUAL internally.
    * @param rpm Target speed in RPM (positive or negative for direction)
    */ void setRpmActual(float rpm);

    /**
     * Get StallGuard result register value
     */
    uint16_t getSGResult();

    /**
     * Get configured StallGuard threshold
     */
    uint8_t getSGTHRS() const;

    /**
     * Get TSTEP register (0x12): time between two 1/256 microsteps in clock cycles (12 MHz).
     * 0 when motor stopped; lower value = higher speed.
     */
    uint32_t getTstep() const;

    // DriverInterface implementation
    const char *getDriverType() const override { return "TMC2209"; }
    const char *getName() const override { return _name; }
};

#endif // TMC2209_DRIVER_H
