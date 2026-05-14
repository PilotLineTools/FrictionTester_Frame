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
#include <Wire.h>
#include "INA219.h"
#include <esp32-hal-cpu.h>
#if POWER_SAVE_WIFI_OFF
#include <WiFi.h>
#endif
#include <OneWire.h>
#include <DallasTemperature.h>
#include "driver/twai.h"
#include "WaterBathController.h"
#include "WaterBathCanAdapter.h"
#include "FrameESP_CanAdapter.h"
#include "CarriageController.h"
#include "CarriageCanAdapter.h"
#include "CanRouter.h"
#include "PowerController.h"
#include "PowerCanAdapter.h"
#include "HealthLed.h"

// ---------------------------------------------------------------------------
// Forward declarations (implementations below)
// ---------------------------------------------------------------------------
extern MotionController *motionController;
extern MachineParameter<uint16_t> carriageMicroSteps;
extern TMC2209Driver *TMC2209_carriageDriver;
extern Axis *carriageAxis;
extern WaterBathController waterBathController;
extern WaterBathCanAdapter waterBathCanAdapter;

void serialEvent(void)
{
   while (USBSerial.available())
   {
      char c = (char)USBSerial.read();
      if (c == 'h' || c == 'H')
      {
         // Carriage: 1 rev at 60 RPM (1 rev/s when unit = rev; 60 RPM = 1 rev/s)
         if (motionController)
         {
            motionController->moveRel(AxisId::Carriage, 50.0f, 15.0f);
            USBSerial.println("Carriage: 1 rev at 60 RPM (1 rev/s)");
         }
      }
      else if (c == 'u' || c == 'U')
      {
         // Carriage microsteps: set from serial, clamp to TMC2209 allowed, push to driver and axis
         carriageMicroSteps.setFromSerial();
         uint16_t ms = TMC2209Driver::clampMicrostepsToValid(carriageMicroSteps.value);
         if (ms != carriageMicroSteps.value)
         {
            carriageMicroSteps.setValue(ms);
            USBSerial.printf("Microsteps rounded to TMC2209 allowed value: %u\n", (unsigned)ms);
         }
         if (TMC2209_carriageDriver != nullptr)
         {
            TMC2209_carriageDriver->setMicrosteps(ms);
            uint16_t readback = TMC2209_carriageDriver->getMicrosteps();
            USBSerial.printf("Carriage microsteps set to %u. Allowed: 1,2,4,8,16,32,64,128,256.\n", (unsigned)ms);
            USBSerial.printf("CHOPCONF readback: %u %s\n", (unsigned)readback, (readback == ms) ? "[OK]" : "[MISMATCH]");
         }
         if (carriageAxis != nullptr)
         {
            carriageAxis->stepsPerUnit = (float)(MOTOR_STEPS_PER_REV * ms);
            USBSerial.printf("Carriage stepsPerUnit = %u (200 × %u)\n", (unsigned)(MOTOR_STEPS_PER_REV * ms), (unsigned)ms);
         }
      }
      else if (c == 'w' || c == 'W')
      {
         // Test command: set bath target and circulator RPM from serial.
         // Use 700 RPM here; applySetWaterBath uses int16_t so -1000..1000 is supported.
         waterBathCanAdapter.applySetWaterBath(1, 37.0f, 250);
         USBSerial.println("Water bath: target 37 C, heater enabled (PID), circulator 250 RPM");
         USBSerial.println("P = .15, I = 0.0003, D = 40, tau = 120s, integral max = .30");
         USBSerial.println("Time(ms), Temp(C), pTerm, I, slope, D, output");
      }
   }
}

// ---------------------------------------------------------------------------
// State and globals
// ---------------------------------------------------------------------------
char swVer[8];
static int potFiltered = 0;       // Latest filtered ADC; updated in timer3Didtic
static bool motorOn = false;      // Start button toggles this (motor run when true)
volatile bool timer3Didtic = false;
static const uint32_t TIMER4_IDLE_INTERVAL_US = 10000;  // Idle step interval (µs); must be non-zero or ISR watchdog
volatile uint32_t timer4value = TIMER4_IDLE_INTERVAL_US;
static bool togglePrintTstep = false;
static bool togglePrintRpm = false;
static HealthLed pcbHealthLed(LED_BUILTIN_PIN);
static const uint32_t HEARTBEAT_TX_INTERVAL_MS = 100;
static bool lastPiCanAliveState = false;
static bool piCanAliveStateInit = false;
static bool piHeartbeatSafetyLockout = false;

// ---------------------------------------------------------------------------
// Machine parameters (NVS-backed; serial menu to set)
// ---------------------------------------------------------------------------
MachineParameter<uint16_t> *allParameters[10];
// TMC2209 CHOPCONF MRES: 1,2,4,8,16,32,64,128,256 (datasheet)
MachineParameter<uint16_t> carriageMicroSteps("CarriageMicroSteps", CARRIAGE_TMC_MICROSTEPS, TMC2209_MICROSTEPS_MIN, TMC2209_MICROSTEPS_MAX);
MachineParameter<uint8_t> carriageStallGuardThreshold("CarriageStallGuard", (uint8_t)CARRIAGE_TMC_SGTHRS, 0, 255);
MachineParameter<uint16_t> carriageRmsCurrentMa("CarriageRmsCurrentMA", CARRIAGE_TMC_RMS_CURRENT_MA, 50, 2000);  // RMS current in mA. Serial "C" to set.
MachineParameter<uint16_t> carriageTpwmThrs("CarriageTPWMTHRS", CARRIAGE_TMC_TPWMTHRS, 0, 65535);
MachineParameter<float> carriageAccelRpmPerSec("CarriageAccelRPM", 200.0f, 10.0f, 20000.0f);  // Ramp rate (RPM/s). Serial "A" to set.
// Pot calibration: ADC at dial 1–9 (serial "1".."9"). Sub-dial (0–1) extrapolated from slope between 1 and 2.

// Bath TMC2209 Configuration
MachineParameter<uint16_t> bathMicroSteps("BathMicroSteps", BATH_TMC_MICROSTEPS, TMC2209_MICROSTEPS_MIN, TMC2209_MICROSTEPS_MAX);
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
LimitSwitch *carriageLimitSwitch = nullptr;
MotionController *motionController = nullptr;
MotionController *globalMC = nullptr;
HardwareSerial TMCSerial(1);
TMC2209Stepper TMC2209_carriage(&TMCSerial, CARRIAGE_TMC_RSENSE, CARRIAGE_TMC_ADDRESS);
TMC2209Driver *TMC2209_carriageDriver = nullptr;

TMC2209Stepper TMC2209_bath(&TMCSerial, BATH_TMC_RSENSE, BATH_TMC_ADDRESS);
TMC2209Driver *TMC2209_bathDriver = nullptr;

// Temporary bring-up override: force bath circulator motor on continuously.
static const bool FORCE_BATH_CIRCULATOR_ON = false;
static const float FORCE_BATH_CIRCULATOR_RPM = 300.0f;


// ---------------------------------------------------------------------------
// Timer ISRs
// ---------------------------------------------------------------------------
void IRAM_ATTR onTimer4A()
{
   motionController->handleStepPulseStart();
   // When not moving use idle interval to avoid ISR storm. When moving use timer4value with min 10 µs (max ~100k steps/s).
   const uint32_t MIN_STEP_PERIOD_US = 20; // good for 900 RPM at 16 microsteps
   uint32_t nextUs =(timer4value <= MIN_STEP_PERIOD_US ? MIN_STEP_PERIOD_US : timer4value);
   timerAlarmWrite(timer4, nextUs, true);
   timerAlarmEnable(timer4);
}

void IRAM_ATTR onTimer3()
{
   timer3Didtic = true;
}

INA219 ina219(0x45, &Wire);
OneWire oneWireBath(BATH_TEMP_DQ_PIN);
DallasTemperature bathTempSensor(&oneWireBath);
WaterBathController waterBathController(HEATER_FET_PIN, &bathTempSensor, &ina219, HEATER_BLOCK_THERMISTOR_PIN);
CanRouter canRouter;
FrameESP_CanAdapter espCanAdapter(&canRouter);
WaterBathCanAdapter waterBathCanAdapter(&waterBathController, &canRouter, &espCanAdapter);
CarriageController *carriageController = nullptr;
CarriageCanAdapter *carriageCanAdapter = nullptr;
PowerController *powerController = nullptr;
PowerCanAdapter *powerCanAdapter = nullptr;

static void handlePowerNotification(PowerController::Notification n)
{
   if (powerCanAdapter)
      powerCanAdapter->onPowerNotification(n);

   switch (n)
   {
   case PowerController::Notification::GUIPoweredOn:
      USBSerial.println("PowerController: GUI powered on");
      break;
   case PowerController::Notification::ShutdownRequested:
      USBSerial.println("PowerController: shutdown requested");
      //TODO: Send a shutdown request to the GUI via CAN. Tell linux to shutdown.
      break;
   case PowerController::Notification::ShutdownInitiated:
      USBSerial.println("PowerController: shutdown initiated (hold to power off)");
      //TODO: Send a shutdown initiated message to the GUI via CAN. The GUI can tell the user to hold the power button for 2 seconds to shutdown the system.
      break;
   case PowerController::Notification::ShutdownAborted:
      USBSerial.println("PowerController: shutdown aborted (short press / release)");
      //TODO: Send a shutdown aborted request to the GUI via CAN. The GUI can remove the shutdown initiated message from the screen.
      break;
   }
}

static HealthLedState computePcbHealthState()
{
   if (!powerController || !powerCanAdapter || !carriageCanAdapter)
   {
      if(!powerController)
         USBSerial.println("Health check: PowerController not initialized");
      if(!powerCanAdapter)
         USBSerial.println("Health check: PowerCanAdapter not initialized");
      if(!carriageCanAdapter)
         USBSerial.println("Health check: CarriageCanAdapter not initialized");  
      return HealthLedState::Fault;
   }

   if (!canRouter.isBusHealthy())
   {
      USBSerial.println("Health check: CAN bus not healthy");
      return HealthLedState::Fault;
   }

   // 1) Hardware-level presence/power indication from the GUI/Pi.
   if (!powerController->isGuiSignalOn())
   {
      USBSerial.println("Health check: GUI signal not on");
      return HealthLedState::Fault;
   }
   // 2) Communication-level liveness (recent GUI heartbeat over CAN).
   if (!powerController->isPiCanAlive())
   {
      USBSerial.println("Health check: Pi CAN not alive");
      return HealthLedState::Fault;
   }

   // 3) Application-level state reported by GUI (must be in ACTIVE mode).
   const uint8_t stateCode = powerController->getGuiPowerStateCode();
   if (stateCode != static_cast<uint8_t>(PowerController::GuiPowerState::ACTIVE))
   {
      USBSerial.println("Health check: GUI power state not active; Current state code: " + String(stateCode));
      return HealthLedState::Fault;
   }


   return HealthLedState::Healthy;
}

static void logCanRxFrame(const twai_message_t &msg)
{
   const uint32_t id = msg.identifier & 0x7FF;

   // Only log IDs that this firmware actually handles via the adapters/router.
   if (!canRouter.handles(id))
      return;
   
   if (id == CAN_ID_SET_POWER || id == CAN_ID_GUI_HEARTBEAT || id == CAN_ID_CLEAR_FAULT || id == CAN_ID_SHUTDOWN_REQUEST )
   {
      // These are expected to be sent frequently by the GUI; skip logging to avoid spamming the console.
      return;
   }

   USBSerial.printf("CAN RX id=0x%03lX dlc=%u data=", (unsigned long)id, (unsigned)msg.data_length_code);
   for (uint8_t i = 0; i < msg.data_length_code; i++)
      USBSerial.printf("%02X%s", msg.data[i], (i + 1 < msg.data_length_code) ? " " : "");
   USBSerial.println();
}

static void initBoardAndDiagnostics()
{
   // Power saving: lower CPU frequency (saves ~30-50 mA)
   if (CPU_FREQ_MHZ_IDLE == 80 || CPU_FREQ_MHZ_IDLE == 160)
      setCpuFrequencyMhz(CPU_FREQ_MHZ_IDLE);
#if POWER_SAVE_WIFI_OFF
   WiFi.mode(WIFI_OFF);
#endif

   // Latch frame power as early as possible so the user
   // does not need to hold the power button through full boot.
   pinMode(POWER_HOLD_PIN, OUTPUT);
   //digitalWrite(POWER_HOLD_PIN, HIGH);

   // Basic safety on critical outputs before heavier init.
   pinMode(HEATER_FET_PIN, OUTPUT);
   digitalWrite(HEATER_FET_PIN, LOW); // startup failsafe: keep heater off

   USBSerial.begin(115200);
   TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
   pcbHealthLed.begin();
   pcbHealthLed.setState(HealthLedState::Fault);

   strcpy(swVer, VERSION_STRING);
   delay(1000);
   USBSerial.print("Version: ");
   USBSerial.println(swVer);
   USBSerial.print("Build Date: ");
   USBSerial.println(BUILD_DATE);
   USBSerial.print("Build Time: ");
   USBSerial.println(BUILD_TIME);
}

static void initMachineParameters()
{
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
}

static void initMotionCore()
{
   motionController = new MotionController();
   globalMC = motionController;

   // Create axis objects; carriage stepsPerUnit = 200 x carriageMicroSteps (from NVS/default)
   carriageAxis = new Axis(CARRIAGE_ACCEL, (float)(MOTOR_STEPS_PER_REV * carriageMicroSteps.value / CARRIAGE_MM_PER_REV), CARRIAGE_START_SPEED);

   // Create motor objects
   carriageMotor = new Motor(CARRIAGE_STEP_PIN, CARRIAGE_DIR_PIN, CARRIAGE_ENABLE_PIN, 1, CARRIAGE_INVERT_DIRECTION);
   bathMotor = new Motor(BATH_STEP_PIN, BATH_DIR_PIN, BATH_ENABLE_PIN, 1, BATH_INVERT_DIRECTION);

   // Create TMC2209 driver instances
   TMC2209_carriageDriver = new TMC2209Driver(&TMC2209_carriage, "Carriage");
   TMC2209_bathDriver = new TMC2209Driver(&TMC2209_bath, "Bath");

   // Carriage stepper configuration
   TMC2209Config carriageConfig;
   carriageConfig.tpwmthrs = carriageTpwmThrs.value;
   carriageConfig.tcoolthrs = CARRIAGE_TMC_TCOOLTHRS;
   carriageConfig.toff = CARRIAGE_TMC_TOFF;
   carriageConfig.tbl = CARRIAGE_TMC_TBL;
   carriageConfig.en_spreadcycle = CARRIAGE_TMC_EN_SPREADCYCLE;

   // Bath stepper configuration
   TMC2209Config bathConfig;
   bathConfig.tpwmthrs = bathTpwmThrs.value;
   bathConfig.tcoolthrs = BATH_TMC_TCOOLTHRS;
   bathConfig.toff = BATH_TMC_TOFF;
   bathConfig.tbl = BATH_TMC_TBL;
   bathConfig.en_spreadcycle = BATH_TMC_EN_SPREADCYCLE;

   TMC2209_carriageDriver->configure(carriageRmsCurrentMa.value, carriageMicroSteps.value, CARRIAGE_TMC_SGTHRS, CARRIAGE_TMC_RSENSE, carriageConfig);
   TMC2209_bathDriver->configure(bathRmsCurrentMa.value, bathMicroSteps.value, BATH_TMC_SGTHRS, BATH_TMC_RSENSE, bathConfig);

   // Quick sanity check: bath microsteps configured vs. read back from driver
   {
      uint16_t bathCfgMs = bathMicroSteps.value;
      uint16_t bathReadMs = TMC2209_bathDriver->getMicrosteps();
      USBSerial.printf("**************Bath microsteps cfg=%u readback=%u\n",
                       (unsigned)bathCfgMs, (unsigned)bathReadMs);
   }

   // Attach drivers to motors
   carriageMotor->attachDriver(TMC2209_carriageDriver);
   bathMotor->attachDriver(TMC2209_bathDriver);

   carriageAxis->addMotor(*carriageMotor);

   carriageAxis->setStopDistance(0.001);

   motionController->addAxis(*carriageAxis, AxisId::Carriage);
   motionController->init();

   // Initialize pilot line specific hardware (NOT USING TMC)
   carriageMotor->initDriver(true); // true = use velocity mode
   carriageMotor->disable();        // Keep motor disabled until needed
   bathMotor->initDriver(true);     // true = use velocity mode
   bathMotor->disable();            // Keep motor disabled until needed
}

static void initMotionTimers()
{
   // Timer 4: step pulse generation scheduler
   timer4 = timerBegin(0, 80, true); // 1us per tick
   timerAttachInterrupt(timer4, &onTimer4A, false); // false = LEVEL (EDGE not supported on ESP32-S3)

   int interval_us = (baseFrequency / 8) / 100; // Convert Hz to us period
   USBSerial.printf("INFO: for timer 4 interval = %d\n", interval_us);
   timerAlarmWrite(timer4, interval_us, true); // auto-reload
   timerAlarmEnable(timer4);

   // Timer 3: periodic control loop tick
   timer3 = timerBegin(1, 80, true); // 1us per tick (80MHz / 80)
   timerAttachInterrupt(timer3, &onTimer3, false); // false = LEVEL
   interval_us = (baseFrequency / 8) * timeStep; // timeStep in seconds
   timerAlarmWrite(timer3, interval_us, true);   // auto-reload
   timerAlarmEnable(timer3);

   USBSerial.print("\nbooted up in ");
   USBSerial.print(millis());
   USBSerial.print(" ms\n\n");
}

static void initPeripheralsAndSensors()
{
   // Initialize I2C for INA219 on custom pins
   Wire.begin(CURRENT_SCA_PIN, CURRENT_SCL_PIN);

   // Initialize DS18B20 bath temperature sensor (OneWire on BATH_TEMP_DQ_PIN)
   bathTempSensor.begin();
   bathTempSensor.setWaitForConversion(false); // non-blocking conversions
   bathTempSensor.setResolution(11);
   bathTempSensor.requestTemperatures(); // kick off first conversion

   // Check INA219 connection
   if (!ina219.begin()) {
      USBSerial.println("INA219 not found!");
   } else {
      USBSerial.println("INA219 connected!");
      ina219.setBusVoltageRange(32);
      ina219.setGain(2);
      ina219.setMaxCurrentShunt(8.0, 0.004);
   }

   // Water bath controller: on/off bath temp control with safety limits
   waterBathController.init();
   waterBathController.setCirculatorHardware(bathMotor, TMC2209_bathDriver);
   waterBathController.setTargetTemp(30.0f);   // default target C
   waterBathController.setHeaterEnableRequest(false);
   waterBathController.setCirculatorTargetRpm(0.0f);
   waterBathController.disable();              // both outputs off at boot -> controller disabled
}

static void initCanControllers()
{
   espCanAdapter.begin();
   waterBathCanAdapter.begin();

   powerController = PowerController::setup(handlePowerNotification);
   powerCanAdapter = new PowerCanAdapter(powerController, &canRouter, &espCanAdapter);
   powerCanAdapter->begin();

   carriageController = new CarriageController(motionController, carriageAxis);
   carriageCanAdapter = new CarriageCanAdapter(carriageController, &canRouter, &espCanAdapter);
   // Remote limit switch: state is sourced from CarriageCanAdapter (CAN LIMIT_STATUS cache).
   carriageLimitSwitch = new LimitSwitch(carriageCanAdapter, true);
   carriageAxis->addLimitSwitch(*carriageLimitSwitch);
   carriageCanAdapter->begin();
}

static void initTwai()
{
   twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
   twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 kbps
   twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

   if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
      USBSerial.println("TWAI driver installed");
   else
      USBSerial.println("TWAI driver install failed");

   bool twaiStarted = (twai_start() == ESP_OK);
   if (twaiStarted)
      USBSerial.println("TWAI driver started");
   else
      USBSerial.println("TWAI driver start failed");

   if (twaiStarted && carriageCanAdapter)
   {
      // Request current remote limit status on boot so carriage canMove flags
      // are initialized even when a switch is already active at startup.
      carriageCanAdapter->requestLimitStatus();
   }
}

static void tickHealthLed(uint32_t nowMs)
{
#if HEARTBEAT_LED_EN
   pcbHealthLed.tick(nowMs);
#else
   (void)nowMs;
#endif
}

static void serviceSerialInput()
{
   if (USBSerial.available())
      serialEvent();
}

static void processControlTicks(uint32_t nowMs, uint8_t &displayCounter, uint8_t &tempCounter, bool &needWaterBathUpdate)
{
   while (timer3Didtic)
   {
      timer3Didtic = false;
      motionController->pollAxis();
      timer4value = motionController->updatePositions();
      displayCounter++;
      tempCounter++;

      if (powerController)
         powerController->poll10ms();
      if (powerCanAdapter)
         powerCanAdapter->tick(nowMs);

      if (displayCounter >= DISPLAY_SLOWER)
      {
         displayCounter = 0;
      }

      if (tempCounter >= 50)
      {
         tempCounter = 0;
         needWaterBathUpdate = true;
      }
   }
}

static void processWaterBathUpdate(uint32_t nowMs, bool &needWaterBathUpdate)
{
   (void)nowMs;
   if (!needWaterBathUpdate)
      return;

   needWaterBathUpdate = false;
   waterBathController.setCirculatorAccelRpmPerSec(bathAccelRpmPerSec.value);
   waterBathController.update();
}

static void tickHeartbeatTx(uint32_t nowMs)
{
   static uint32_t lastHeartbeatMs = 0;
   if (nowMs - lastHeartbeatMs < HEARTBEAT_TX_INTERVAL_MS)
      return;

   lastHeartbeatMs = nowMs;
   espCanAdapter.sendHeartbeat(
      waterBathController.getBathTempC(),
      waterBathController.getBlockTempC(),
      waterBathController.getHeaterCurrentA(),
      waterBathController.isHeaterOn(),
      waterBathController.isEnabled());
}

static void logDiagnostics(uint32_t nowMs)
{
   static uint32_t lastScaleLogMs = 0;
   if (nowMs - lastScaleLogMs < 1000)
      return;

   lastScaleLogMs = nowMs;

   const unsigned loadSg = TMC2209_carriageDriver ? (unsigned)TMC2209_carriageDriver->getSGResult() : 0u;
   const unsigned loadSgthrs = TMC2209_carriageDriver ? (unsigned)TMC2209_carriageDriver->getSGTHRS() : 0u;
   const unsigned long loadTstep = TMC2209_carriageDriver ? (unsigned long)TMC2209_carriageDriver->getTstep() : 0ul;
   const unsigned loadDiag = (unsigned)digitalRead(CARRIAGE_DIAG_PIN);
   

   // Log stall guard and diag values for both axes; SG values are meaningful while moving.
   USBSerial.printf("Diag=%u Carriage: SG=%u SGTHRS=%u Tstep=%lu\n",
                    loadDiag,
                    loadSg,
                    loadSgthrs,
                    loadTstep);


}


static void dispatchCanRx()
{
   twai_message_t rx_message;
   if (twai_receive(&rx_message, 0) == ESP_OK)
   {
      logCanRxFrame(rx_message);
      canRouter.dispatch(&rx_message);
   }
}

static void tickCanAdapters()
{
   if (carriageCanAdapter)
      carriageCanAdapter->tick();
}

static void applyModeChanges()
{
   SystemMode mode;
   if (espCanAdapter.consumeModeChange(mode))
   {
      waterBathCanAdapter.onModeChanged(mode);
      if (powerCanAdapter)
         powerCanAdapter->onModeChanged(mode);
      if (carriageCanAdapter)
         carriageCanAdapter->onModeChanged(mode);
   }
}

static void refreshHealthLedState()
{
   if (powerController)
   {
      const bool piAliveNow = powerController->isPiCanAlive();
      if (!piCanAliveStateInit)
      {
         lastPiCanAliveState = piAliveNow;
         piCanAliveStateInit = true;
      }
      else if (piAliveNow != lastPiCanAliveState)
      {
         lastPiCanAliveState = piAliveNow;
         USBSerial.printf("PowerController: Pi CAN heartbeat %s (timeout=%lu ms)\n",
                          piAliveNow ? "RESTORED" : "LOST",
                          (unsigned long)3000UL);
      }
   }

#if HEARTBEAT_LED_EN
   pcbHealthLed.setState(computePcbHealthState());
#endif
}

static void enforcePiHeartbeatSafety()
{
   if (!powerController)
      return;

   const bool piAliveNow = powerController->isPiCanAlive();
   if (!piAliveNow)
   {
      if (!piHeartbeatSafetyLockout)
      {
         piHeartbeatSafetyLockout = true;
         USBSerial.println("SAFETY: Pi CAN heartbeat lost -> stopping carriage, bath motor, and heater");
         if (carriageController)
            carriageController->stop();
         waterBathController.setHeaterEnableRequest(false);
         waterBathController.setCirculatorTargetRpm(0.0f);
         waterBathController.disable();
      }

      // Keep outputs forced off while heartbeat is missing.
      if (carriageMotor)
         carriageMotor->disable();
      if (bathMotor)
         bathMotor->disable();
      digitalWrite(HEATER_FET_PIN, LOW);
      return;
   }

   if (piHeartbeatSafetyLockout)
   {
      piHeartbeatSafetyLockout = false;
      USBSerial.println("SAFETY: Pi CAN heartbeat restored -> controls can be re-enabled by commands");
   }
}

void setup()
{
   // 1) Early board bring-up and boot diagnostics
   initBoardAndDiagnostics();

   // 2) Persistent machine parameters (NVS-backed)
   initMachineParameters();

   // 3) Motion core objects (controller, axis, motors, drivers)
   initMotionCore();

   // 4) Motion timers (step pulse + control tick)
   initMotionTimers();

   // 5) Peripherals and sensors
   initPeripheralsAndSensors();

   // 6) CAN-facing adapters/controllers
   initCanControllers();

   // 7) TWAI bus setup
   initTwai();

   // 8) Report health status
   refreshHealthLedState();
}

void loop()
{
   static uint8_t displayCounter = 0;  // 0..(DISPLAY_SLOWER-1), 10 Hz
   static uint8_t tempCounter = 0;     // 0..49, 500 ms water bath update
   static bool needWaterBathUpdate = false;
   const uint32_t nowMs = millis();

   // 1) Heartbeat/health LED tick.
   tickHealthLed(nowMs);

   // 2) Service local serial commands.
   serviceSerialInput();

   // 3) Enforce heartbeat safety interlock.
   enforcePiHeartbeatSafety();

   // 4) Drain control ticks so motion timing never falls behind.
   processControlTicks(nowMs, displayCounter, tempCounter, needWaterBathUpdate);

   // 5) Water-bath periodic update + telemetry.
   processWaterBathUpdate(nowMs, needWaterBathUpdate);

   // 6) Frame heartbeat TX.
   tickHeartbeatTx(nowMs);

   // 7) Dispatch inbound CAN frames.
   dispatchCanRx();

   // 8) Adapter periodic work.
   tickCanAdapters();

   // 9) Apply system mode changes from frame-level CAN adapter.
   applyModeChanges();

   // 10) Refresh aggregate health state.
   refreshHealthLedState();

   //logDiagnostics(nowMs);
}
