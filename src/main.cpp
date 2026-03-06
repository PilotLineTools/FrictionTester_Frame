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
#include "FrameCanAdapter.h"
#include "CarriageController.h"
#include "CarriageCanAdapter.h"
#include "CanRouter.h"
#include "PowerController.h"
#include "PowerCanAdapter.h"

// ---------------------------------------------------------------------------
// Forward declarations (implementations below)
// ---------------------------------------------------------------------------
extern MotionController *motionController;
extern MachineParameter<uint16_t> carriageMicroSteps;
extern TMC2209Driver *TMC2209_carriageDriver;
extern Axis *carriageAxis;

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
FrameCanAdapter frameCanAdapter(&canRouter);
WaterBathCanAdapter waterBathCanAdapter(&waterBathController, &canRouter, &frameCanAdapter);
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

static void logCanRxFrame(const twai_message_t &msg)
{
   if ((msg.identifier & 0x7FF) == CAN_ID_GUI_HEARTBEAT)
      return;

   USBSerial.printf("CAN RX id=0x%03lX dlc=%u data=", (unsigned long)(msg.identifier & 0x7FF), (unsigned)msg.data_length_code);
   for (uint8_t i = 0; i < msg.data_length_code; i++)
      USBSerial.printf("%02X%s", msg.data[i], (i + 1 < msg.data_length_code) ? " " : "");
   USBSerial.println();
}

void setup()
{
   // Power saving: lower CPU frequency (saves ~30–50 mA)
   if (CPU_FREQ_MHZ_IDLE == 80 || CPU_FREQ_MHZ_IDLE == 160)
      setCpuFrequencyMhz(CPU_FREQ_MHZ_IDLE);
#if POWER_SAVE_WIFI_OFF
   WiFi.mode(WIFI_OFF);
#endif

   // Latch frame power as early as possible so the user
   // does not need to hold the power button through full boot.
   pinMode(POWER_HOLD_PIN, OUTPUT);
   digitalWrite(POWER_HOLD_PIN, HIGH);

   // Basic safety on critical outputs before heavier init.
   pinMode(HEATER_FET_PIN, OUTPUT);
   digitalWrite(HEATER_FET_PIN, LOW); // startup failsafe: keep heater off

   USBSerial.begin(115200);

   strcpy(swVer, VERSION_STRING);
   delay(100);
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
   


   motionController = new MotionController();
   globalMC = motionController;

   // Create axis objects; carriage stepsPerUnit = 200 × carriageMicroSteps (from NVS/default)
   carriageAxis = new Axis(CARRIAGE_ACCEL, (float)(MOTOR_STEPS_PER_REV * carriageMicroSteps.value / CARRIAGE_LEADSCREW_PITCH), CARRIAGE_START_SPEED);

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

   motionController->addAxis(*carriageAxis, AxisId::Carriage);

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

   // Test Move the carriage motor by 1 mm at 500 mm/min (8.33 mm/s)
   // motionController->moveRel(AxisId::Carriage, 1, 500);

   // Initialize I2C for INA219 on custom pins
   Wire.begin(CURRENT_SCA_PIN, CURRENT_SCL_PIN);

   // Initialize DS18B20 bath temperature sensor (OneWire on BATH_TEMP_DQ_PIN)
   bathTempSensor.begin();
   // Use non-blocking conversions; we manage timing in loop()
   bathTempSensor.setWaitForConversion(false);
   // Optional: set resolution (9–12 bits). 11-bit ≈ 0.125°C, ~375 ms conversion.
   bathTempSensor.setResolution(11);
   // Kick off first conversion so the first read has valid data
   bathTempSensor.requestTemperatures();

   // Check INA219 connection
   if (!ina219.begin()) {
      USBSerial.println("INA219 not found!");
   } else {
      USBSerial.println("INA219 connected!");   
      ina219.setBusVoltageRange(32); // 32V range
      ina219.setGain(2); // Gain 2 = 320mV range on shunt (default and good for up to ~3.2A with 0.1Ω shunt)
      ina219.setMaxCurrentShunt(8.0, 0.004); // Max current 8A, shunt 4mΩ (for testing with higher current; adjust for production)
   }

   // Water bath controller: on/off bath temp control with safety limits
   waterBathController.init();
   waterBathController.setCirculatorHardware(bathMotor, TMC2209_bathDriver);
   waterBathController.setTargetTemp(30.0f);   // default target °C
   waterBathController.setHeaterEnableRequest(false);
   waterBathController.setCirculatorTargetRpm(0.0f);
   waterBathController.disable();              // both outputs off at boot -> controller disabled

   frameCanAdapter.begin();
   waterBathCanAdapter.begin();

   powerController = PowerController::setup(handlePowerNotification);
   powerCanAdapter = new PowerCanAdapter(powerController, &canRouter, &frameCanAdapter);
   powerCanAdapter->begin();
   carriageController = new CarriageController(motionController, carriageAxis);
   carriageCanAdapter = new CarriageCanAdapter(carriageController, &canRouter, &frameCanAdapter);
   carriageCanAdapter->begin();

   // --- TWAI (CAN) Configuration ---
   twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
   twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500 kbps
   twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

   // Install TWAI driver
   if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      USBSerial.println("TWAI driver installed");
   } else {
      USBSerial.println("TWAI driver install failed");
   }

   // Start TWAI driver
   if (twai_start() == ESP_OK) {
      USBSerial.println("TWAI driver started");
   } else {
      USBSerial.println("TWAI driver start failed");
   }
   
}


void loop()
{
   static uint8_t displayCounter = 0;  // 0..(DISPLAY_SLOWER-1), 10 Hz
   static uint8_t tempCounter = 0;     // 0..49, 500 ms water bath update
   static bool needWaterBathUpdate = false;

   // Call serialEvent() if there's data available (like Arduino convention)
   if (USBSerial.available())
   {
      serialEvent();
   }

   // Process every 10 ms tick; drain so we never miss motion updates (avoids glitch when 500 ms work ran)
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
         powerCanAdapter->tick(millis());

      if (displayCounter >= DISPLAY_SLOWER) // 10 Hz
      {
         displayCounter = 0;
         // Add other 10 Hz display / INA219 / UI updates here
      }

      if (tempCounter >= 50) // every 500 ms - set flag only; do work outside so this block stays short
      {
         tempCounter = 0;
         needWaterBathUpdate = true;
      }
   }

   // 500 ms work outside timer3 block so it doesn't delay the next updatePositions()
   if (needWaterBathUpdate)
   {
      needWaterBathUpdate = false;
      waterBathController.update();
      waterBathCanAdapter.tick(millis());  // emit BATH_STATUS (0x280) every half second

      WaterBathError err = waterBathController.getErrorCode();
      if (err == WaterBathError::BathSensorDisconnected)
         USBSerial.print("WaterBath: bath sensor disconnected\t");
      else if (err != WaterBathError::None)
         USBSerial.printf("WaterBath FAULT: %s\t", WaterBathController::errorToString(err));
      else
         USBSerial.printf("Bath: %.1f C  Block: %.1f C  Heater: %s  Current: %.2f A\n",
            waterBathController.getBathTempC(), waterBathController.getBlockTempC(),
            waterBathController.isHeaterOn() ? "On" : "Off", waterBathController.getHeaterCurrentA());
   }

   twai_message_t rx_message;
   if (twai_receive(&rx_message, 0) == ESP_OK)
   {
      logCanRxFrame(rx_message);
      canRouter.dispatch(&rx_message);
   }

   if (carriageCanAdapter)
      carriageCanAdapter->tick();

   SystemMode mode;
   if (frameCanAdapter.consumeModeChange(mode))
   {
      waterBathCanAdapter.onModeChanged(mode);
      if (powerCanAdapter)
         powerCanAdapter->onModeChanged(mode);
      if (carriageCanAdapter)
         carriageCanAdapter->onModeChanged(mode);
   }
}
