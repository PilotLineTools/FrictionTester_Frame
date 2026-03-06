/**
 * WaterBathController - On/off temperature control for water bath with safety limits.
 *
 * - On/off (bang-bang) regulation: heater on when bath below target, off when above (with deadband).
 * - Heater driven via MOSFET (digital on/off on HEATER_FET_PIN), updated every ~500 ms.
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

   /** Deadband around target (°C). Heater on when bath < target - deadband, off when > target + deadband. Default 0.5. */
   void setDeadband(float deadbandC) { _deadbandC = deadbandC; }
   float getDeadband() const { return _deadbandC; }

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
   /** Heater state: 1 = on, 0 = off (for display/compatibility). */
   float getHeaterDuty() const { return _heaterOn ? 1.0f : 0.0f; }
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
   float _deadbandC = 0.5f;

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
