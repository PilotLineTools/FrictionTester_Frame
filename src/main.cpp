/*  Pilot Line Dip Coating Machine
 *  Uses MKS GEN L V2.1 board
 *  Uses DPC_3 Nextion project
 *  Serial Menu for machine parameters
 *  System eeprom varialbe space 0-100
 *  Corrected speed calculation errors (was expecting float result from dividing int32_t)
 *  Homing sequence goes up to upper limit if carriage is not in loading position
 *  Added odometers for total run time, uv time, and # of cycles
 *  Added QR code for replacement parts (in nextion side)
 *
 */

#include "Constants.h"
#include "MotionController.h"
#include "Motor.h"
#include "TMC2209Driver.h"
#include "Axis.h"
#include "Machine_Parameter.h"
#include "version.h"
#include <TMCStepper.h>
#include <HardwareSerial.h>
#include <esp32-hal-cpu.h>
#if POWER_SAVE_WIFI_OFF
#include <WiFi.h>
#endif

// ---------------------------------------------------------------------------
// Forward declarations (implementations below)
// ---------------------------------------------------------------------------
void serialEvent(void);
int updatePotFilter(void);
void clearNVS(void);

// ---------------------------------------------------------------------------
// LED pattern: table-driven (level, duration_ms). Easy to add modes later.
// ---------------------------------------------------------------------------
typedef struct
{
   uint8_t level;    // 0 = off, 1 = on
   uint16_t dur_ms;
} LedPhase;

enum LedMode
{
   LED_MODE_OFF,            // LED off (e.g. while holding START when idle)
   LED_MODE_STANDBY,        // one blip every 3 s (idle, low)
   LED_MODE_STANDBY_DOUBLE, // two blips every 3 s (idle, high)
   LED_MODE_STANDBY_TRIPLE, // three blips every 3 s (idle, ultra)
   LED_MODE_RUNNING,        // one pulse per second (motor running, low)
   LED_MODE_RUNNING_DOUBLE, // two blips per second (motor running, high)
   LED_MODE_RUNNING_TRIPLE  // three blips per second (motor running or mode select, ultra)
};

// Speed range: low 0–90, high 0–900, ultra 100–5000 RPM. Hold START when idle to cycle.
enum SpeedMode
{
   SPEED_MODE_LOW,   // 0–90 RPM
   SPEED_MODE_HIGH,  // 0–900 RPM
   SPEED_MODE_ULTRA  // pot 1=100 RPM, pot 9=5000 RPM
};

static const LedPhase pattern_off[] = {
   { 0, 60000 }
};
static const LedPhase pattern_standby[] = {
   { 1, 100 },
   { 0, 2900 }
};
static const LedPhase pattern_standby_double[] = {
   { 1, 100 },
   { 0, 100 },
   { 1, 100 },
   { 0, 2700 }
};
static const LedPhase pattern_running[] = {
   { 1, 100 },
   { 0, 900 }
};
static const LedPhase pattern_double[] = {
   { 1, 100 },
   { 0, 100 },
   { 1, 100 },
   { 0, 700 }
};
static const LedPhase pattern_standby_triple[] = {
   { 1, 100 }, { 0, 100 }, { 1, 100 }, { 0, 100 }, { 1, 100 }, { 0, 2500 }
};
static const LedPhase pattern_triple[] = {
   { 1, 100 }, { 0, 100 }, { 1, 100 }, { 0, 100 }, { 1, 100 }, { 0, 500 }
};

static const LedPhase *const led_patterns[] = { pattern_off, pattern_standby, pattern_standby_double, pattern_standby_triple, pattern_running, pattern_double, pattern_triple };
static const uint8_t led_pattern_len[] = { 1, 2, 4, 6, 2, 4, 6 };

static void updateLedPattern(LedMode mode)
{
   static uint8_t phase = 0;
   static uint32_t phase_start_ms = 0;
   static LedMode last_mode = LED_MODE_STANDBY;
   if (mode != last_mode)
   {
      last_mode = mode;
      phase = 0;
      phase_start_ms = millis();
   }
   const LedPhase *p = led_patterns[mode];
   uint8_t n = led_pattern_len[mode];
   uint32_t now = millis();
   if ((uint32_t)(now - phase_start_ms) >= (uint32_t)p[phase].dur_ms)
   {
      phase_start_ms = now;
      phase = (phase + 1) % n;
   }
   digitalWrite(LED_BUILTIN_PIN, p[phase].level ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// State and globals
// ---------------------------------------------------------------------------
char swVer[8];
static int potFiltered = 0;       // Latest filtered ADC; updated in timer3Didtic
static bool motorOn = false;      // Start button toggles this (motor run when true)
static float actualRPM = 0.0f;   // Ramped RPM toward target, updated each timer3 tick
static SpeedMode speedMode = SPEED_MODE_HIGH;  // Low / high / ultra (hold START when idle to cycle)
volatile bool timer3Didtic = false;
static const uint32_t TIMER4_IDLE_INTERVAL_US = 10000;  // Idle step interval (µs); must be non-zero or ISR watchdog
volatile uint32_t timer4value = TIMER4_IDLE_INTERVAL_US;
static bool togglePrintTstep = false;
static bool togglePrintRpm = false;

// ---------------------------------------------------------------------------
// Machine parameters (NVS-backed; serial menu to set)
// ---------------------------------------------------------------------------
MachineParameter<uint16_t> *allParameters[10];
// TMC2209 CHOPCONF MRES: 1,2,4,8,16,32,64,128,256 (datasheet)
MachineParameter<uint16_t> carriageMicroSteps("CarriageMicroSteps", 16, TMC2209_MICROSTEPS_MIN, TMC2209_MICROSTEPS_MAX);
MachineParameter<uint8_t> carriageStallGuardThreshold("CarriageStallGuard", (uint8_t)CARRIAGE_TMC_SGTHRS, 0, 255);
MachineParameter<uint16_t> carriageRmsCurrentMa("CarriageRmsCurrentMA", CARRIAGE_TMC_RMS_CURRENT_MA, 50, 2000);  // RMS current in mA. Serial "C" to set.
MachineParameter<uint16_t> carriageTpwmThrs("CarriageTPWMTHRS", CARRIAGE_TMC_TPWMTHRS, 0, 65535);
MachineParameter<float> carriageAccelRpmPerSec("CarriageAccelRPM", 200.0f, 10.0f, 20000.0f);  // Ramp rate (RPM/s). Serial "A" to set.
// Pot calibration: ADC at dial 1–9 (serial "1".."9"). Sub-dial (0–1) extrapolated from slope between 1 and 2.

// Bath TMC2209 Configuration
MachineParameter<uint16_t> bathMicroSteps("BathMicroSteps", 16, TMC2209_MICROSTEPS_MIN, TMC2209_MICROSTEPS_MAX);
MachineParameter<uint8_t> bathStallGuardThreshold("BathStallGuard", (uint8_t)BATH_TMC_SGTHRS, 0, 255);
MachineParameter<uint16_t> bathRmsCurrentMa("BathRmsCurrentMA", BATH_TMC_RMS_CURRENT_MA, 50, 2000);  // RMS current in mA. Serial "C" to set.
MachineParameter<uint16_t> bathTpwmThrs("BathTPWMTHRS", BATH_TMC_TPWMTHRS, 0, 65535);
MachineParameter<float> bathAccelRpmPerSec("BathAccelRPM", 200.0f, 10.0f, 20000.0f);  // Ramp rate (RPM/s). Serial "A" to set.
// Pot calibration: ADC at dial 1–9 (serial "1".."9"). Sub-dial (0–1) extrapolated from slope between 1 and 2.

// ---------------------------------------------------------------------------
// Hardware and driver instances
// ---------------------------------------------------------------------------
hw_timer_t *timer4 = NULL;
hw_timer_t *timer3 = NULL;
Axis *carriageAxis = nullptr;
Motor *carriageMotor = nullptr;
Motor *bathMotor = nullptr;
MotionController *motionController = nullptr;
MotionController *globalMC = nullptr;
HardwareSerial TMCSerial(1);
TMC2209Stepper TMC2209_carriage(&TMCSerial, CARRIAGE_TMC_RSENSE, CARRIAGE_TMC_ADDRESS);
TMC2209Driver *TMC2209_carriageDriver = nullptr;

TMC2209Stepper TMC2209_bath(&TMCSerial, BATH_TMC_RSENSE, BATH_TMC_ADDRESS);
TMC2209Driver *TMC2209_bathDriver = nullptr;


// ---------------------------------------------------------------------------
// Timer ISRs
// ---------------------------------------------------------------------------
void IRAM_ATTR onTimer4A()
{
   motionController->handleStepPulseStart();
   // Never set alarm to 0 or very small or timer retriggers immediately → interrupt wdt timeout
   uint32_t nextUs = (timer4value >= 100) ? timer4value : TIMER4_IDLE_INTERVAL_US;
   timerAlarmWrite(timer4, nextUs, true);
   timerAlarmEnable(timer4);
}

void IRAM_ATTR onTimer3()
{
   timer3Didtic = true;
}

void setup()
{
   // Power saving: lower CPU frequency (saves ~30–50 mA)
   if (CPU_FREQ_MHZ_IDLE == 80 || CPU_FREQ_MHZ_IDLE == 160)
      setCpuFrequencyMhz(CPU_FREQ_MHZ_IDLE);
#if POWER_SAVE_WIFI_OFF
   WiFi.mode(WIFI_OFF);
#endif

   USBSerial.begin(115200);
   delay(50);
   USBSerial.setTimeout(2000);

   strcpy(swVer, VERSION_STRING);
   USBSerial.print("Version: ");
   USBSerial.println(swVer);
   USBSerial.print("Build Date: ");
   USBSerial.println(BUILD_DATE);
   USBSerial.print("Build Time: ");
   USBSerial.println(BUILD_TIME);

   // Initialize machine parameters (load from NVS)
   carriageMicroSteps.init();
   carriageStallGuardThreshold.init();
   carriageRmsCurrentMa.init();
   carriageTpwmThrs.init();
   carriageAccelRpmPerSec.init();
   
   bathMicroSteps.init();
   bathStallGuardThreshold.init();
   bathRmsCurrentMa.init();
   bathTpwmThrs.init();
   bathAccelRpmPerSec.init();
   
   pinMode(POWER_BUTTON_SIGNAL, INPUT);       
   pinMode(GUI_SHUTDOWN_PIN, INPUT_PULLUP); 

   pinMode(POWER_HOLD_PIN, OUTPUT);
   pinMode(HEATER_FET_PIN, OUTPUT);
   pinMode(LED_BUILTIN_PIN, OUTPUT);

   motionController = new MotionController();
   globalMC = motionController;

   // Create axis objects
   carriageAxis = new Axis(carriageAccelRpmPerSec.value, CARRIAGE_STEPS_PER_UNIT, CARRIAGE_START_SPEED);

   // Create motor object
   carriageMotor = new Motor(CARRIAGE_STEP_PIN, CARRIAGE_DIR_PIN, CARRIAGE_ENABLE_PIN, 1, CARRIAGE_INVERT_DIRECTION);
   bathMotor = new Motor(BATH_STEP_PIN, BATH_DIR_PIN, BATH_ENABLE_PIN, 1, BATH_INVERT_DIRECTION);

   // Create TMC2209 driver instance
   TMC2209_carriageDriver = new TMC2209Driver(&TMC2209_carriage, "Carriage");
   TMC2209_bathDriver = new TMC2209Driver(&TMC2209_bath, "Bath");

   // Carriage Stepper configuration
   TMC2209Config CarriageConfig;
   CarriageConfig.tpwmthrs = carriageTpwmThrs.value;
   CarriageConfig.tcoolthrs = CARRIAGE_TMC_TCOOLTHRS;
   CarriageConfig.toff = CARRIAGE_TMC_TOFF;
   CarriageConfig.tbl = CARRIAGE_TMC_TBL;
   CarriageConfig.en_spreadcycle = CARRIAGE_TMC_EN_SPREADCYCLE;

   // Bath Stepper configuration
   TMC2209Config BathConfig;
   BathConfig.tpwmthrs = bathTpwmThrs.value;
   BathConfig.tcoolthrs = BATH_TMC_TCOOLTHRS;
   BathConfig.toff = BATH_TMC_TOFF;
   BathConfig.tbl = BATH_TMC_TBL;
   BathConfig.en_spreadcycle = BATH_TMC_EN_SPREADCYCLE;
   

   TMC2209_carriageDriver->configure(carriageRmsCurrentMa.value, carriageMicroSteps.value, CARRIAGE_TMC_SGTHRS, CARRIAGE_TMC_RSENSE, CarriageConfig);
   TMC2209_bathDriver->configure(bathRmsCurrentMa.value, bathMicroSteps.value, BATH_TMC_SGTHRS, BATH_TMC_RSENSE, BathConfig);

   // Attach driver to motor
   carriageMotor->attachDriver(TMC2209_carriageDriver);
   bathMotor->attachDriver(TMC2209_bathDriver);

   carriageAxis->addMotor(*carriageMotor);

   motionController->addAxis(*carriageAxis); // this is the 0th axis, Stepper axis

   TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);

   motionController->init();

   // Initialize pilotLine specific hardware (NOT USING TMC)
   carriageMotor->initDriver(true); // true = use velocity mode
   carriageMotor->disable();        // Keep motor disabled until needed

   bathMotor->initDriver(true); // true = use velocity mode
   bathMotor->disable();        // Keep motor disabled until needed

   // === Start Timer 4 (Step Pulse Timer) ===
   timer4 = timerBegin(0, 80, true); // 1µs per tick
   timerAttachInterrupt(timer4, &onTimer4A, false); // false = LEVEL (EDGE not supported on ESP32-S3)

   int interval_us = (baseFrequency / 8) / 100; // Convert Hz to µs period
   USBSerial.printf("INFO: for timer 4 interval = %d\n", interval_us);
   timerAlarmWrite(timer4, interval_us, true); // auto-reload
   timerAlarmEnable(timer4);

   timer3 = timerBegin(1, 80, true); // 1µs per tick (80MHz / 80)
   timerAttachInterrupt(timer3, &onTimer3, false); // false = LEVEL

   interval_us = (baseFrequency / 8) * timeStep; // timeStep in seconds
                                                 // int interval_us = 1000000; // timeStep in seconds
   timerAlarmWrite(timer3, interval_us, true);   // auto-reload
   timerAlarmEnable(timer3);

   USBSerial.print("\nbooted up in ");
   USBSerial.print(millis());
   USBSerial.print(" ms\n\n");
}


void loop()
{
   static uint8_t displayCounter;

   // --- Power button logic with ON/OFF toggle ---
   static bool powerLatched = false;
   static bool lastButtonState = HIGH;
   bool buttonState = digitalRead(POWER_BUTTON_SIGNAL);

   // Detect button press (falling edge)
   if (lastButtonState == HIGH && buttonState == LOW)
   {
      powerLatched = !powerLatched;
      digitalWrite(POWER_HOLD_PIN, powerLatched ? HIGH : LOW);
      USBSerial.printf("Power %s\n", powerLatched ? "latched ON" : "unlatched OFF");
      if (powerLatched)
      {
         digitalWrite(LED_BUILTIN_PIN, HIGH); // LED on when power on
      }
      else
      {
         digitalWrite(LED_BUILTIN_PIN, LOW); // LED off when power off
      }

   }
   lastButtonState = buttonState;

   // Call serialEvent() if there's data available (like Arduino convention)
   if (USBSerial.available())
   {
      serialEvent();
   }

   if (timer3Didtic)
   {
      timer3Didtic = false;
      motionController->pollAxis();
      timer4value = motionController->updatePositions();
      displayCounter++;
   }

   
}