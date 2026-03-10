/**
 * WaterBathController - PID temperature control for water bath with safety limits.
 *
 * - PID output (0–1) thresholded to on/off each 500 ms cycle (relay-friendly, no fast cycling).
 * - Heater driven via digital on/off on HEATER_FET_PIN, updated every ~500 ms.
 * - INA219 monitors heater current; shuts down + error if out of bounds.
 * - Heater block thermistor: shutdown + fault if over limit.
 * - Circulator (TMC-driven) is controlled directly by WaterBathController via configured motor/driver pointers.
 *
 * Call update(bathTempC) every ~500 ms from main (non-blocking). No blocking calls.
 */

#ifndef WATER_BATH_CONTROLLER_H
#define WATER_BATH_CONTROLLER_H

#include <Arduino.h>
#include <DallasTemperature.h>
#include "INA219.h"
#include "Constants.h"

// Error/fault codes reported when heater is shut down
enum class WaterBathError : uint8_t
{
   None = 0,
   HeaterCurrentHigh,
   HeaterCurrentLow,
   BlockOvertemp,
   BathSensorDisconnected
};

class Motor;
class TMC2209Driver;

class WaterBathController
{
public:
   /**
    * @param heaterPin GPIO for heater MOSFET (on/off).
    * @param bathSensor DS18B20 sensor for water bath temperature.
    * @param currentSensor INA219 on heater circuit.
    * @param blockThermistorPin ADC-capable GPIO for heater block NTC thermistor.
    */
   WaterBathController(int heaterPin,
                       DallasTemperature *bathSensor,
                       INA219 *currentSensor,
                       int blockThermistorPin);

   /** Initialize GPIO for heater (output, off). Call once from setup(). */
   void init();

   /** Enable controller (heater control active, circulator can run). Disable turns heater off and stops circulator. */
   void enable();
   void disable();
   bool isEnabled() const { return _enabled; }

   /** Target bath temperature (°C). */
   void setTargetTemp(float tempC);
   float getTargetTemp() const { return _targetTempC; }

   /** Heater current limits (A). Outside range triggers shutdown and error. */
   void setHeaterCurrentLimits(float minA, float maxA);
   /** Heater block overtemp limit (°C). Above triggers shutdown and fault. */
   void setBlockTempLimit(float tempC);
   float getBlockTempLimit() const { return _blockTempMaxC; }

   /** PID gains. Defaults: Kp=5, Ki=0.2, Kd=0.5. */
   void setPid(float kp, float ki, float kd);
   void getPid(float &kp, float &ki, float &kd) const { kp = _kp; ki = _ki; kd = _kd; }

   /** Integral term (for tuning/debug). Reset on fault or disable. */
   float getIntegral() const { return _integral; }

   /**
    * Call every ~500 ms. Reads bath temp (DS18B20), block thermistor, and heater current;
    * runs limits check, then sets heater on/off. Requests next DS18B20 conversion for next call.
    * Non-blocking.
    */
   void update();

   /** Last reported error (cleared when fault condition clears and controller is updated). */
   WaterBathError getErrorCode() const { return _errorCode; }
   /** Human-readable error string for logging/GUI. */
   static const char *errorToString(WaterBathError code);

   /** Last read bath temperature from DS18B20 (°C). */
   float getBathTempC() const { return _bathTempC; }
   /** Last read block temperature (°C). */
   float getBlockTempC() const { return _blockTempC; }
   /** Last read heater current (A). */
   float getHeaterCurrentA() const { return _heaterCurrentA; }
   /** Last PID output 0–1 (before threshold). For display/tuning. */
   float getHeaterDuty() const { return _pidOutput; }
   bool isHeaterOn() const { return _heaterOn; }

   /** Configure circulator hardware controlled by this controller. */
   void setCirculatorHardware(Motor *motor, TMC2209Driver *driver);
   /** Request heater permission from command layer (independent of controller master enabled). */
   void setHeaterEnableRequest(bool enable);
   bool isHeaterEnableRequested() const { return _heaterRequestEnabled; }
   /** Requested circulator RPM from command layer. Applied only when controller is enabled and fault-free. */
   void setCirculatorTargetRpm(float rpm);
   float getCirculatorTargetRpm() const { return _circulatorTargetRpm; }

private:
   int _heaterPin;
   DallasTemperature *_bathSensor;
   INA219 *_currentSensor;
   int _blockThermistorPin;

   bool _enabled = false;
   float _targetTempC = 50.0f;
   float _heaterCurrentMinA = WATER_BATH_HEATER_CURRENT_MIN_A;
   float _heaterCurrentMaxA = WATER_BATH_HEATER_CURRENT_MAX_A;
   float _blockTempMaxC = WATER_BATH_BLOCK_TEMP_MAX_C;

   float _kp = .15f;
   float _ki = 0.002f;
   float _kd = 60.0f;
   float _integral = 0.0f;
   float _lastError = 0.0f;
   float _pidOutput = 0.0f;
   float _dutyAccum = 0.0f;   // Bresenham-style accumulator for duty-cycle heater (0–1 per 500 ms cycle)
   // Derivative-on-measurement state: filtered bath temperature slope (°C/s) over several minutes.
   float _tempSlopeFiltered = 0.0f;
   float _lastBathTempForSlope = 0.0f;
   bool  _slopeInit = false;
   uint32_t _lastDebugMs = 0; // throttled PID debug print timestamp
   static constexpr float _dt = 0.5f;        // update interval (s)
   static constexpr float _integralMax = .40f;   // anti-windup
   static constexpr float _derivTauSec = 120.0f; // derivative filter time constant (s) 2 minutes

   float _bathTempC = 0.0f;
   float _blockTempC = 0.0f;
   float _heaterCurrentA = 0.0f;
   bool _heaterOn = false;
   bool _heaterRequestEnabled = false;

   WaterBathError _errorCode = WaterBathError::None;
   Motor *_circulatorMotor = nullptr;
   TMC2209Driver *_circulatorDriver = nullptr;
   float _circulatorTargetRpm = 0.0f;

   void setHeaterOn(bool on);
   void enableHeater();
   void disableHeater();
   void enableCirculator();
   void disableCirculator();
   float readBlockTempC();
   void applyCirculator();
};

#endif // WATER_BATH_CONTROLLER_H
