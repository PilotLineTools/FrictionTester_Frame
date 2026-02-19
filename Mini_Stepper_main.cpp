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
MachineParameter<uint16_t> microSteps("MicroSteps", 16, TMC2209_MICROSTEPS_MIN, TMC2209_MICROSTEPS_MAX);
MachineParameter<uint8_t> zStallGuardThreshold("StallGuard", (uint8_t)TMC_SGTHRS, 0, 255);
MachineParameter<uint16_t> rmsCurrentMa("RmsCurrentMA", TMC_RMS_CURRENT_MA, 50, 2000);  // RMS current in mA. Serial "C" to set.
MachineParameter<uint16_t> tpwmthrs("TPWMTHRS", TMC_TPWMTHRS, 0, 65535);
MachineParameter<uint8_t> speedModeParam("SpeedMode", (uint8_t)SPEED_MODE_HIGH, (uint8_t)SPEED_MODE_LOW, (uint8_t)SPEED_MODE_ULTRA);  // 0=low, 1=high, 2=ultra 100–5000
MachineParameter<float> accelRpmPerSec("AccelRPM", 200.0f, 10.0f, 20000.0f);  // Ramp rate (RPM/s). Serial "A" to set.
// Pot calibration: ADC at dial 1–9 (serial "1".."9"). Sub-dial (0–1) extrapolated from slope between 1 and 2.
MachineParameter<uint16_t> adcAtDial1("PotADCAt1", 3600, 0, ADC_MAX);
MachineParameter<uint16_t> adcAtDial2("PotADCAt2", 3200, 0, ADC_MAX);
MachineParameter<uint16_t> adcAtDial3("PotADCAt3", 2800, 0, ADC_MAX);
MachineParameter<uint16_t> adcAtDial4("PotADCAt4", 2400, 0, ADC_MAX);
MachineParameter<uint16_t> adcAtDial5("PotADCAt5", 2000, 0, ADC_MAX);
MachineParameter<uint16_t> adcAtDial6("PotADCAt6", 1600, 0, ADC_MAX);
MachineParameter<uint16_t> adcAtDial7("PotADCAt7", 1200, 0, ADC_MAX);
MachineParameter<uint16_t> adcAtDial8("PotADCAt8", 800, 0, ADC_MAX);
MachineParameter<uint16_t> adcAtDial9("PotADCAt9", 400, 0, ADC_MAX);

// ---------------------------------------------------------------------------
// Hardware and driver instances
// ---------------------------------------------------------------------------
hw_timer_t *timer4 = NULL;
hw_timer_t *timer3 = NULL;
Axis *axisStepper = nullptr;
Motor *stepperMotor = nullptr;
MotionController *motionController = nullptr;
MotionController *globalMC = nullptr;
HardwareSerial TMCSerial(1);
TMC2209Stepper TMC2209_stepper(&TMCSerial, TMC_RSENSE, TMC_ADDRESS);
TMC2209Driver *motorStepperDriver = nullptr;

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
   microSteps.init();
   zStallGuardThreshold.init();
   rmsCurrentMa.init();
   tpwmthrs.init();
   speedModeParam.init();
   speedMode = (SpeedMode)speedModeParam.value;  // restore persisted speed mode (0=low, 1=high, 2=ultra)
   accelRpmPerSec.init();
   adcAtDial1.init();
   adcAtDial2.init();
   adcAtDial3.init();
   adcAtDial4.init();
   adcAtDial5.init();
   adcAtDial6.init();
   adcAtDial7.init();
   adcAtDial8.init();
   adcAtDial9.init();

   pinMode(LED_BUILTIN_PIN, OUTPUT);
   digitalWrite(LED_BUILTIN_PIN, LOW);
   pinMode(POT_PIN, INPUT);                        // Potentiometer ADC
   pinMode(OLD_POT_PIN, INPUT);                    // Old potentiometer ADC
   pinMode(LEFT_RIGHT_SWITCH_PIN, INPUT_PULLUP);   // User left/right toggle
   pinMode(START_BUTTON_PIN, INPUT_PULLUP);       // Start button

   motionController = new MotionController();
   globalMC = motionController;

   // Create axis objects
   axisStepper = new Axis(ACCEL, STEPS_PER_UNIT, START_SPEED);

   // Create motor object
   stepperMotor = new Motor(STEP_PIN, DIR_PIN, ENABLE_PIN, 1, INVERT_DIRECTION);

   // Create TMC2209 driver instance
   motorStepperDriver = new TMC2209Driver(&TMC2209_stepper, "Spin");

   TMC2209Config stepperConfig;
   stepperConfig.tpwmthrs = tpwmthrs.value;
   stepperConfig.tcoolthrs = TMC_TCOOLTHRS;
   stepperConfig.toff = TMC_TOFF;
   stepperConfig.tbl = TMC_TBL;
   stepperConfig.en_spreadcycle = TMC_EN_SPREADCYCLE;

   motorStepperDriver->configure(rmsCurrentMa.value, microSteps.value, TMC_SGTHRS, TMC_RSENSE, stepperConfig);

   // Attach driver to motor
   stepperMotor->attachDriver(motorStepperDriver);

   axisStepper->addMotor(*stepperMotor);

   motionController->addAxis(*axisStepper); // this is the 0th axis, Stepper axis

   TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);

   motionController->init();

   // Stepper uses velocity mode (VACTUAL register for RPM control) when needed
   stepperMotor->initDriver(true); // true = use velocity mode
   stepperMotor->disable();        // Keep motor disabled until needed

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
      potFiltered = updatePotFilter();  // ADC oversample + ring buffer, every ~10 ms

      // Start/Stop and speed-mode selection (INPUT_PULLUP → pressed = LOW)
      // Short press when idle: toggle motor. Hold 1 s when idle: enter mode selection (1/2/3 blips per s = low/high/ultra); release to select.
      static bool lastStartPressed = false;
      static uint32_t startPressMs = 0;
      static bool inModeSelection = false;
      static uint32_t modeSelectionStartMs = 0;
      static uint32_t ledQuietUntilMs = 0;  // after releasing from mode selection, keep LED off briefly
      bool startPressed = (digitalRead(START_BUTTON_PIN) == LOW);

      if (startPressed && !motorOn)
      {
         if (!lastStartPressed)
            startPressMs = millis();
         if (!inModeSelection && (uint32_t)(millis() - startPressMs) >= MODE_SELECT_HOLD_MS)
         {
            inModeSelection = true;
            modeSelectionStartMs = millis();
         }
      }
      if (!startPressed)
      {
         if (lastStartPressed)
         {
            if (inModeSelection)
            {
               uint32_t elapsed = (uint32_t)(millis() - modeSelectionStartMs);
               uint8_t phase = (elapsed / MODE_SELECT_INTERVAL_MS) % 3;
               speedMode = (phase == 0) ? SPEED_MODE_LOW : (phase == 1) ? SPEED_MODE_HIGH : SPEED_MODE_ULTRA;
               speedModeParam.setValue((uint8_t)speedMode);  // persist to NVS
               inModeSelection = false;
               ledQuietUntilMs = millis() + 400;  // no LED flash right after release
            }
            else
               motorOn = !motorOn;
         }
      }
      lastStartPressed = startPressed;

      // Pot → speedDial (0.0–9.0): extrapolate below 1 from slope between dial 1 and 2; then 9-point 1–9
      uint16_t adc[9] = { adcAtDial1.value, adcAtDial2.value, adcAtDial3.value, adcAtDial4.value,
                          adcAtDial5.value, adcAtDial6.value, adcAtDial7.value, adcAtDial8.value, adcAtDial9.value };
      float speedDial = 1.0f;
      if (potFiltered >= adc[0])
      {
         // Extrapolate below 1 using slope between dial 1 and 2 (no dial 0 calibrator)
         if (adc[0] != adc[1])
         {
            speedDial = 1.0f + (float)(adc[0] - potFiltered) / (float)(adc[0] - adc[1]);
            if (speedDial < 0.0f)
               speedDial = 0.0f;
         }
      }
      else if (potFiltered <= adc[8])
         speedDial = 9.0f;
      else
      {
         for (int i = 0; i < 8; i++)
         {
            uint16_t a = adc[i], b = adc[i + 1];
            if (a == b)
               continue;
            int lo = (a < b) ? a : b;
            int hi = (a > b) ? a : b;
            if (potFiltered >= lo && potFiltered <= hi)
            {
               float frac = (float)(potFiltered - lo) / (float)(hi - lo);
               speedDial = (a > b) ? (float)(i + 2) - frac : (float)(i + 1) + frac;
               break;
            }
         }
      }

      // RPM range for current speed mode; dial 0 → rpmMin, dial 9 → rpmMax (all modes)
      int rpmMin, rpmMax;
      switch (speedMode)
      {
         case SPEED_MODE_LOW:   rpmMin = MOTOR_LOW_MIN;   rpmMax = MOTOR_LOW_MAX;   break;
         case SPEED_MODE_HIGH:  rpmMin = MOTOR_HIGH_MIN;  rpmMax = MOTOR_HIGH_MAX;  break;
         case SPEED_MODE_ULTRA: rpmMin = MOTOR_ULTRA_MIN; rpmMax = MOTOR_ULTRA_MAX; break;
      }
      int potRPM = (int)(rpmMin + (speedDial / 9.0f) * (float)(rpmMax - rpmMin));
      if (potRPM < rpmMin)
         potRPM = rpmMin;
      if (potRPM > rpmMax)
         potRPM = rpmMax;

      int dir = (digitalRead(LEFT_RIGHT_SWITCH_PIN) == HIGH) ? 1 : -1;
      float targetRPM = motorOn ? (float)(dir * potRPM) : 0.0f;

      // Ramp actualRPM toward target once per tick (accel/decel)
      float dRPM = accelRpmPerSec.value * (float)timeStep;
      if (actualRPM < targetRPM)
      {
         actualRPM += dRPM;
         if (actualRPM > targetRPM)
            actualRPM = targetRPM;
      }
      else if (actualRPM > targetRPM)
      {
         actualRPM -= dRPM;
         if (actualRPM < targetRPM)
            actualRPM = targetRPM;
      }

      // VACTUAL is in configured microsteps (MRES), not fixed 256. Use microSteps.value so RPM is invariant when we change microsteps.
      // usteps_per_rev = full_steps_per_rev × microSteps; usteps_per_sec = (|RPM|/60) × usteps_per_rev
      // VACTUAL = round(usteps_per_sec × (2²⁴ / fCLK)); sign = direction; 0 = STEP/DIR.
      float ustepsPerRev = (float)(MOTOR_STEPS_PER_REV * microSteps.value);
      float ustepsPerSec = (fabsf(actualRPM) / 60.0f) * ustepsPerRev;
      float vactualScale = (float)(1UL << TMC2209_VACTUAL_SCALE_EXP) / (float)TMC2209_FCLK_HZ;
      int32_t vactual = (int32_t)(ustepsPerSec * vactualScale + 0.5f);
      if (actualRPM < 0.0f)
         vactual = -vactual;

      if (motorStepperDriver != nullptr)
      {
         static int32_t lastVactual = 0;
         if (vactual != lastVactual)
         {
            motorStepperDriver->setVactual(vactual);
            lastVactual = vactual;
         }
         if (vactual != 0)
            stepperMotor->enable();
         else
            stepperMotor->disable();
      }
      displayCounter++;
#if HEARTBEAT_LED_EN
      LedMode ledMode;
      if (ledQuietUntilMs != 0 && millis() >= ledQuietUntilMs)
         ledQuietUntilMs = 0;  // quiet period over
      if (!motorOn && ledQuietUntilMs != 0 && millis() < ledQuietUntilMs)
         ledMode = LED_MODE_OFF;  // no flash right after releasing from mode selection
      else if (inModeSelection)
      {
         uint32_t elapsed = (uint32_t)(millis() - modeSelectionStartMs);
         uint8_t phase = (elapsed / MODE_SELECT_INTERVAL_MS) % 3;
         ledMode = (phase == 0) ? LED_MODE_RUNNING : (phase == 1) ? LED_MODE_RUNNING_DOUBLE : LED_MODE_RUNNING_TRIPLE;
      }
      else if (motorOn)
         ledMode = (speedMode == SPEED_MODE_LOW) ? LED_MODE_RUNNING : (speedMode == SPEED_MODE_HIGH) ? LED_MODE_RUNNING_DOUBLE : LED_MODE_RUNNING_TRIPLE;
      else if (startPressed)
         ledMode = LED_MODE_OFF;  // no standby blip while holding START (avoid confusion before mode selection)
      else
         ledMode = (speedMode == SPEED_MODE_LOW) ? LED_MODE_STANDBY : (speedMode == SPEED_MODE_HIGH) ? LED_MODE_STANDBY_DOUBLE : LED_MODE_STANDBY_TRIPLE;
      updateLedPattern(ledMode);
#endif
   }

   if (displayCounter >= DISPLAY_SLOWER)
   {
      displayCounter = 0;
      if (motorStepperDriver != nullptr && togglePrintTstep)
         USBSerial.printf("TSTEP(0x12): %lu\n", (unsigned long)motorStepperDriver->getTstep());
      if (togglePrintRpm)
         USBSerial.printf("RPM: %.1f\n", actualRPM);
   }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------
int updatePotFilter(void)
{
   // Oversample: average several ADC reads to reduce noise
   uint32_t sum = 0;
   for (int i = 0; i < POT_ADC_OVERSAMPLE; i++)
      sum += analogRead(POT_PIN);
   int sample = (int)(sum / POT_ADC_OVERSAMPLE);

   // Rolling average for ~2–3 s smooth response (no fluctuations)
   static int buf[POT_ADC_FILTER_SIZE];
   static int idx = 0;
   static int nfilled = 0;
   static int32_t runSum = 0;

   if (nfilled < POT_ADC_FILTER_SIZE)
   {
      buf[nfilled] = sample;
      runSum += sample;
      nfilled++;
   }
   else
   {
      runSum -= buf[idx];
      buf[idx] = sample;
      runSum += sample;
      idx = (idx + 1) % POT_ADC_FILTER_SIZE;
   }
   return (int)(runSum / nfilled);
}

void clearNVS(void)
{
   Preferences prefs;
   prefs.begin("params", false);
   prefs.clear(); // Clears everything in "params" namespace
   prefs.end();
   USBSerial.println("NVS cleared!");
}

void serialEvent(void)
{
   // Process one character at a time (called from loop when data is available)
   // Don't use while loop - setFromSerial() blocks and we don't want to process
   // additional buffered characters after it returns
   if (!USBSerial.available())
      return;
   int c = USBSerial.read();

   static bool inCalibrationMode = false;
   if (c == 'K' || c == 'k')
   {
      inCalibrationMode = !inCalibrationMode;
      if (inCalibrationMode)
         USBSerial.println(F("Calibration mode ON. Set pot to position 1-9, send 1-9 to store current ADC. Send K again to exit."));
      else
         USBSerial.println(F("Calibration mode OFF."));
      return;
   }
   if (inCalibrationMode && c >= '1' && c <= '9')
   {
      int n = c - '0';
      MachineParameter<uint16_t> *adcParams[] = { &adcAtDial1, &adcAtDial2, &adcAtDial3, &adcAtDial4, &adcAtDial5, &adcAtDial6, &adcAtDial7, &adcAtDial8, &adcAtDial9 };
      adcParams[n - 1]->setValue((uint16_t)potFiltered);
      USBSerial.printf("Pot ADC at dial %d set to %d\n", n, potFiltered);
      return;
   }

   switch (c)
   {
      case '?':
         USBSerial.println(F("\n\tHere is a list of available Commands\n"));
         USBSerial.println(F(" key\tName\t\t\tAction"));
         USBSerial.println(F("============================================================================"));
         USBSerial.println(F("  u \tmicroSteps\t\t1,2,4,8,16,32,64,128,256"));
         USBSerial.println(F("  A \tAccel RPM/s\t\tRamp rate when accelerating/decelerating (RPM/s)"));
         USBSerial.println(F("  C \tRMS current (mA)\tStepper coil RMS current in milliamps"));
         USBSerial.println(F("  Y \tStallGuard Threshold\tStepper StallGuard threshold (0-255)"));
         USBSerial.println(F("  T \tTPWMTHRS\t\tStealthChop threshold in TSTEP units"));
         USBSerial.println(F("  P \tPrintTSTEP\t\tPrint TSTEP(0x12) register value"));
         USBSerial.println(F("  R \tPrintRPM\t\tPrint ramped RPM value"));
         USBSerial.println(F("  K \tCalibrate dial\t\tEnter calibration mode; then send 1-9 to store ADC. K again to exit."));
         USBSerial.println(F("  X \tClear NVS\t\tErase all stored parameters"));
         USBSerial.println(F("  i \tInfo\t\t\tPrint information about the system"));
         break;

      case 'u':
         microSteps.setFromSerial();
         {
            uint16_t ms = TMC2209Driver::clampMicrostepsToValid(microSteps.value);
            if (ms != microSteps.value)
            {
               microSteps.setValue(ms);
               USBSerial.printf("Microsteps rounded to TMC2209 allowed value: %u\n", (unsigned)ms);
            }
            if (motorStepperDriver != nullptr)
            {
               motorStepperDriver->setMicrosteps(ms);
               uint16_t readback = motorStepperDriver->getMicrosteps();
               USBSerial.printf("Microsteps set to %u (RPM uses internal 256 for VACTUAL). Allowed: 1,2,4,8,16,32,64,128,256.\n", (unsigned)ms);
               USBSerial.printf("CHOPCONF readback: %u microsteps %s\n", (unsigned)readback, (readback == ms) ? "[OK]" : "[MISMATCH]");
            }
         }
         break;

      case 'A':
         accelRpmPerSec.setFromSerial();
         USBSerial.printf("Accel RPM/s set to %.1f\n", (double)accelRpmPerSec.value);
         break;

      case 'C':
         rmsCurrentMa.setFromSerial();
         if (motorStepperDriver != nullptr)
         {
            motorStepperDriver->setRmsCurrent(rmsCurrentMa.value);
            USBSerial.printf("RMS current set to %u mA\n", (unsigned)rmsCurrentMa.value);
         }
         break;

      case 'Y':
         zStallGuardThreshold.setFromSerial();
         if (motorStepperDriver != nullptr)
         {
            TMC2209Config stepperConfig;
            stepperConfig.tpwmthrs = tpwmthrs.value;
            stepperConfig.tcoolthrs = TMC_TCOOLTHRS;
            stepperConfig.toff = TMC_TOFF;
            stepperConfig.tbl = TMC_TBL;
            stepperConfig.en_spreadcycle = TMC_EN_SPREADCYCLE;
            motorStepperDriver->configure(rmsCurrentMa.value, TMC_MICROSTEPS, zStallGuardThreshold.value, TMC_RSENSE, stepperConfig);
            USBSerial.printf("Stepper StallGuard threshold updated to %d\n", (int)zStallGuardThreshold.value);
         }
         break;

      case 'T':
         tpwmthrs.setFromSerial();
         if (motorStepperDriver != nullptr)
         {
            TMC2209Config stepperConfig;
            stepperConfig.tpwmthrs = tpwmthrs.value;
            stepperConfig.tcoolthrs = TMC_TCOOLTHRS;
            stepperConfig.toff = TMC_TOFF;
            stepperConfig.tbl = TMC_TBL;
            stepperConfig.en_spreadcycle = TMC_EN_SPREADCYCLE;
            motorStepperDriver->configure(rmsCurrentMa.value, microSteps.value, zStallGuardThreshold.value, TMC_RSENSE, stepperConfig);
            USBSerial.printf("Stepper TPWMTHRS updated to %d\n", (int)tpwmthrs.value);
         }
         break;

      case 'P':
         togglePrintTstep = !togglePrintTstep;
         USBSerial.printf("PrintTSTEP toggled to %d\n", (int)togglePrintTstep);
         break;

      case 'R':
         togglePrintRpm = !togglePrintRpm;
         USBSerial.printf("PrintRPM toggled to %d\n", (int)togglePrintRpm);
         break;

      case 'X':
         clearNVS();
         break;

      case 'i':
         USBSerial.println(F("\n\tSystem Information:\n============================================"));
         USBSerial.printf("Version: %s\n", VERSION_STRING);
         USBSerial.printf("Build Date: %s\n", BUILD_DATE);
         USBSerial.printf("Build Time: %s\n\n", BUILD_TIME);
         USBSerial.printf("Microsteps: %u\n", (unsigned)microSteps.value);
         USBSerial.printf("Acceleration RPM/s: %f\n", (double)accelRpmPerSec.value);
         USBSerial.printf("RMS current: %u mA\n", (unsigned)rmsCurrentMa.value);

         break;

      case '\n': // newline?
         break;

      case '\r': // carriage return?
         break;

      default:
         USBSerial.println("Did not recognize command. Send \"?\" for a list of commands");
         break;
   }
}
