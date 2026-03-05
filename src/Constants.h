#ifndef CONSTANTS_H
#define CONSTANTS_H

// ============================================================================
// Stepper Motor Pin Configuration
// ============================================================================

// Stepper Motor Configuration (TMC2209)
#define CARRIAGE_STEP_PIN (GPIO_NUM_16)   // IO16  Step
#define CARRIAGE_DIR_PIN (GPIO_NUM_15)   // IO15 Direction
#define CARRIAGE_ENABLE_PIN (GPIO_NUM_10) // IO10 Enable
#define CARRIAGE_DIAG_PIN (GPIO_NUM_3)  // IO3 Diagnostic / StallGuard
#define CARRIAGE_INVERT_DIRECTION (1)    // Inverts direction
#define CARRIAGE_ACCEL (15)             // mm/s^2
// CARRIAGE_STEPS_PER_UNIT = MOTOR_STEPS_PER_REV × CARRIAGE_TMC_MICROSTEPS (defined in TMC section below)
#define CARRIAGE_START_SPEED (5)       // mm/min
#define CARRIAGE_JOG_SPEED (3000)        // mm/min

#define BATH_STEP_PIN (GPIO_NUM_36)   // IO36  Step
#define BATH_DIR_PIN (GPIO_NUM_35)   // IO35 Direction
#define BATH_ENABLE_PIN (GPIO_NUM_38) // IO38 Enable
#define BATH_DIAG_PIN (GPIO_NUM_37)  // IO37 Diagnostic / StallGuard
#define BATH_INVERT_DIRECTION (1)    // Inverts direction
#define BATH_ACCEL (1500)             // mm/s^2
#define BATH_STEPS_PER_UNIT (200)  // Steps per revolution (200 steps/rev for 1.8° motors) divided by mm per revolution (e.g. 4 mm/rev leads to 50 steps/mm)
#define BATH_START_SPEED (500)       // mm/min
#define BATH_JOG_SPEED (3000)        // mm/min


// ============================================================================
// Output Pins
// ============================================================================
#define LED_BUILTIN_PIN GPIO_NUM_14

// ============================================================================
// User Inputs (digital switches, MCU reads)
// ============================================================================
//#define LEFT_RIGHT_SWITCH_PIN (GPIO_NUM_2)  // User left/right toggle (digital 0/1)
#define POWER_BUTTON_SIGNAL (GPIO_NUM_42)  // Start button (digital 0/1)

// ============================================================================
// Analog Inputs (ESP32-S3: only GPIO 1–10 are ADC-capable)
// ============================================================================
//#define POT_PIN GPIO_NUM_1  // Potentiometer ADC
//#define OLD_POT_PIN GPIO_NUM_39  // Old potentiometer ADC

// Pot ADC filtering: oversample each update, then rolling average (~2–3 s response)
//#define POT_ADC_OVERSAMPLE (12)   // Number of ADC reads to average per update (reduces single-sample noise)
//#define POT_ADC_FILTER_SIZE (40)  // Rolling window size; at ~10 ms/update → ~0.5 s smoothing

// Pot calibration: dial scale 1–9 (0–0.8 is mechanical dead zone). Stored in MachineParameters (serial 1/9 to set).
//#define ADC_MAX (4095)
// RPM = (speedDial/9) * maxRpm; speedDial 0 = 0 RPM, 1–9 = linear in current speed mode.
#define MOTOR_STEPS_PER_REV (200)   // Full steps per motor shaft revolution

// Speed mode RPM ranges (low / high / ultra)
//#define MOTOR_LOW_MIN   (0)
//#define MOTOR_LOW_MAX   (90)
//#define MOTOR_HIGH_MIN  (0)
//#define MOTOR_HIGH_MAX  (900)
//#define MOTOR_ULTRA_MIN (0)
//#define MOTOR_ULTRA_MAX (9000)

// Ramp rate (RPM/s) in MachineParameter accelRpmPerSec. Serial "A" to set. Default 200 (was 400).

// Speed mode selection: hold START while idle for this long to enter mode pick (then 4 s slow / 4 s fast, repeat)
//#define MODE_SELECT_HOLD_MS (1000)
//#define MODE_SELECT_INTERVAL_MS (4000)

// ============================================================================
// Power saving (reduces idle current draw from ~100 mA)
// ============================================================================
#define CPU_FREQ_MHZ_IDLE (80)   // 80 or 160 MHz (vs 240). Saves ~30–50 mA. Set 240 to keep max speed.
#define HEARTBEAT_LED_EN (1)     // 1 = blink LED in loop (heartbeat); 0 = off to save a few mA
#define POWER_SAVE_WIFI_OFF (1)  // 1 = turn WiFi off in setup (saves ~20–40 mA if radio was on)

// ============================================================================
// Motion Timing
// ============================================================================
#define DISPLAY_SLOWER (10) // Display update delay in accel intervals
#define ACCEL_INTERVAL (10) // Acceleration interval in ms

const float timeStep = 0.01;            // 10 ms time step
const double baseFrequency = 8000000.0; // MCU clock frequency (Hz)
const double initialSpeed = 100;        // Initial speed in steps/sec

// ============================================================================
// TMC2209 UART Configuration
// ============================================================================
#define TMC_UART_NUM UART_NUM_2
#define TMC_UART_TX GPIO_NUM_17
#define TMC_UART_RX GPIO_NUM_18
#define TMC_BAUD_RATE (115200)

// ============================================================================
// TMC2209 Axis-Specific Configuration
// ============================================================================

// TMC2209 Configuration
#define CARRIAGE_TMC_ADDRESS (0)  // UART address 0
#define CARRIAGE_TMC_RSENSE (0.100f)
#define CARRIAGE_TMC_RMS_CURRENT_MA (400)
#define CARRIAGE_TMC_MICROSTEPS (8)
// Carriage steps per unit (e.g. per rev): physical steps/rev × microsteps. Runtime microsteps: main.cpp carriageMicroSteps (MachineParameter).
#define CARRIAGE_STEPS_PER_UNIT (MOTOR_STEPS_PER_REV * CARRIAGE_TMC_MICROSTEPS)

// TMC2209 Configuration
#define BATH_TMC_ADDRESS (1)  // UART address 1
#define BATH_TMC_RSENSE (0.100f)
#define BATH_TMC_RMS_CURRENT_MA (400)
#define BATH_TMC_MICROSTEPS (16)

// TMC2209 CHOPCONF MRES (datasheet): MRES 0 = 256 native; MRES 1..8 = 128,64,32,16,8,4,2,1 (2^MRES microsteps per step)
#define TMC2209_MICROSTEPS_MIN (1)   // FULLSTEP
#define TMC2209_MICROSTEPS_MAX (256) // Native 256
// VACTUAL: signed 24-bit velocity. µsteps/s = VACTUAL × (fCLK / 2²⁴) ⇒ VACTUAL = usteps_per_sec × (2²⁴ / fCLK).
// Use configured MRES microsteps (not fixed 256) so commanded RPM is invariant when changing microstep resolution.
#define TMC2209_FCLK_HZ (12000000UL)
#define TMC2209_VACTUAL_SCALE_EXP (24)
#define TMC_STALL_GUARD_THRESHOLD (80)

#define BATH_TMC_TPWMTHRS (200) // Upper limit for stealthchop mode. Units are in TSTEP (number of clock cycles (12mHhz) between two internal microsteps) (140 = ~100 RPM)
#define BATH_TMC_TCOOLTHRS (350)
#define BATH_TMC_TOFF (4)
#define BATH_TMC_TBL (1)
#define BATH_TMC_EN_SPREADCYCLE (false) // do not force SpreadCycle (switch based on TPWMTHRS)
// StallGuard threshold (shared)
#define BATH_TMC_SGTHRS (20)


#define CARRIAGE_TMC_TPWMTHRS (200) // Upper limit for stealthchop mode. Units are in TSTEP (number of clock cycles (12mHhz) between two internal microsteps) (140 = ~100 RPM)
#define CARRIAGE_TMC_TCOOLTHRS (350)
#define CARRIAGE_TMC_TOFF (4)
#define CARRIAGE_TMC_TBL (1)
#define CARRIAGE_TMC_EN_SPREADCYCLE (false) // do not force SpreadCycle (switch based on TPWMTHRS)
// StallGuard threshold (shared)
#define CARRIAGE_TMC_SGTHRS (20)

#define HEATER_FET_PIN GPIO_NUM_1       // FET gate for heater (PWM)
#define BATH_TEMP_DQ_PIN GPIO_NUM_9    // 1-Wire data for bath temperature sensor (DS18B20)
#define HEATER_BLOCK_THERMISTOR_PIN GPIO_NUM_13  // ADC-capable pin for heater block NTC thermistor

// Water bath controller limits (tunable; heater shuts down and reports error if exceeded)
#define WATER_BATH_HEATER_CURRENT_MIN_A (6.0f)   // Below this = fault (e.g. open circuit)
#define WATER_BATH_HEATER_CURRENT_MAX_A (7.0f)   // Above this = fault (e.g. short)
#define WATER_BATH_BLOCK_TEMP_MAX_C (45.0f)      // Heater block overtemp limit (°C)

#define CAN_TX_PIN GPIO_NUM_4  // CAN bus TX
#define CAN_RX_PIN GPIO_NUM_5  // CAN bus RX

#define GUI_SHUTDOWN_PIN GPIO_NUM_7 // Input pin for GUI shutdown signal (digital 0/1)
#define POWER_HOLD_PIN GPIO_NUM_41 // GPIO pin used to hold power on (active HIGH). Should be connected to a P-channel MOSFET gate that controls the main power line to the system. When HIGH, the MOSFET is on and power is supplied; when LOW, the MOSFET turns off and cuts power to the system. This allows the microcontroller to safely shut down by setting this pin LOW before going to sleep or resetting, ensuring that power is cut and preventing brownouts or undefined behavior on restart.

#define CURRENT_SCA_PIN GPIO_NUM_39 // Current Sensor Data (I^2C)
#define CURRENT_SCL_PIN GPIO_NUM_40 // Current Sensor Clock (I^2C)

#endif // CONSTANTS_H
