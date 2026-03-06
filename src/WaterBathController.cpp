/**
 * WaterBathController - Implementation.
 * PID bath temperature control with heater current and block overtemp protection.
 */

#include "WaterBathController.h"
#include "Motor.h"
#include "TMC2209Driver.h"

// NTC thermistor: 10k @ 25C, Beta(25/85)=3977, series resistor 4.7k to 3.3V, NTC to GND.
#define NTC_T0 298.15f   // 25°C in K
#define ADC_MAX 4095.0f

WaterBathController::WaterBathController(int heaterPin,
                                         DallasTemperature *bathSensor,
                                         INA219 *currentSensor,
                                         int blockThermistorPin)
   : _heaterPin(heaterPin),
     _bathSensor(bathSensor),
     _currentSensor(currentSensor),
     _blockThermistorPin(blockThermistorPin)
{
}

void WaterBathController::init()
{
   pinMode(_heaterPin, OUTPUT);
   digitalWrite(_heaterPin, LOW);

   pinMode(_blockThermistorPin, INPUT);
}

void WaterBathController::enable()
{
   _enabled = true;
   _errorCode = WaterBathError::None;
   enableCirculator();
   enableHeater();
   USBSerial.println("WB FLOW ctrl enable()");
}

void WaterBathController::disable()
{
   _enabled = false;
   disableHeater();
   disableCirculator();
   USBSerial.println("WB FLOW ctrl disable()");
}

void WaterBathController::setTargetTemp(float tempC)
{
   _targetTempC = tempC;
}

void WaterBathController::setHeaterCurrentLimits(float minA, float maxA)
{
   _heaterCurrentMinA = minA;
   _heaterCurrentMaxA = maxA;
}

void WaterBathController::setBlockTempLimit(float tempC)
{
   _blockTempMaxC = tempC;
}

void WaterBathController::setCirculatorHardware(Motor *motor, TMC2209Driver *driver)
{
   _circulatorMotor = motor;
   _circulatorDriver = driver;
}

void WaterBathController::setHeaterEnableRequest(bool enable)
{
   _heaterRequestEnabled = enable;
   if (!enable)
      setHeaterOn(false);
}

void WaterBathController::setCirculatorTargetRpm(float rpm)
{
   if (rpm < 0.0f)
      rpm = 0.0f;
   _circulatorTargetRpm = rpm;
   applyCirculator();
}

void WaterBathController::setPid(float kp, float ki, float kd)
{
   _kp = kp;
   _ki = ki;
   _kd = kd;
}

void WaterBathController::setHeaterOn(bool on)
{
   if (!_enabled || !_heaterRequestEnabled || _errorCode != WaterBathError::None)
      on = false;

   _heaterOn = on;
   digitalWrite(_heaterPin, on ? HIGH : LOW);
}

void WaterBathController::enableHeater()
{
   setHeaterOn(_heaterOn);
}

void WaterBathController::disableHeater()
{
   _integral = 0.0f;
   _lastError = 0.0f;
   setHeaterOn(false);
}

float WaterBathController::readBlockTempC()
{
   uint32_t rawSum = 0;
   for (int i = 0; i < THERM_NUM_SAMPLES; ++i)
      rawSum += (uint32_t)analogRead(_blockThermistorPin);

   float raw = (float)rawSum / (float)THERM_NUM_SAMPLES;
   if (raw <= 0.0f || raw >= ADC_MAX)
   {
#if THERMISTOR_TEST_LOG
      //USBSerial.printf("THERM TEST invalid raw=%.1f pin=%d\n", raw, _blockThermistorPin);
#endif
      return -100.0f; // invalid
   }

   float v = raw / ADC_MAX * THERM_ADC_REFERENCE_V;
   float r_ntc = (THERM_ADC_REFERENCE_V - v) > 0.01f
                    ? (v * THERM_SERIES_RESISTOR_OHM / (THERM_ADC_REFERENCE_V - v))
                    : THERM_R0_OHM;

   float ln_r = logf(r_ntc / THERM_R0_OHM);
   float t_k = 1.0f / (1.0f / NTC_T0 + ln_r / THERM_BETA);
   float tempC = t_k - 273.15f;

#if THERMISTOR_TEST_LOG
   //USBSerial.printf("THERM TEST raw=%.1f v=%.3fV r=%.1fOhm temp=%.2fC pin=%d\n",
   //                 raw, v, r_ntc, tempC, _blockThermistorPin);
#endif

   return tempC;
}

void WaterBathController::applyCirculator()
{
   // Circulator follows commanded RPM whenever controller is enabled.
   // This is intentionally independent from heater request/errors.
   float rpm = _enabled ? _circulatorTargetRpm : 0.0f;

   if (_circulatorDriver != nullptr)
      _circulatorDriver->setRpmActual(rpm);

   if (_circulatorMotor != nullptr)
      (rpm > 0.0f) ? _circulatorMotor->enable() : _circulatorMotor->disable();
}

void WaterBathController::enableCirculator()
{
   applyCirculator();
}

void WaterBathController::disableCirculator()
{
   if (_circulatorDriver != nullptr)
      _circulatorDriver->setRpmActual(0.0f);

   if (_circulatorMotor != nullptr)
      _circulatorMotor->disable();
}

void WaterBathController::update()
{
   _bathTempC = _bathSensor->getTempCByIndex(0);
   _blockTempC = readBlockTempC();
   _heaterCurrentA = _currentSensor->getCurrent();

   // Check error conditions. If any fault is active, turn off heater and set error code.

   // Case 1: Bath sensor disconnected or out of range
   if (_bathTempC <= -127.0f || _bathTempC >= 85.0f)
   {
      _errorCode = WaterBathError::BathSensorDisconnected;
      _integral = 0.0f;
      _lastError = 0.0f;
      _pidOutput = 0.0f;
      setHeaterOn(false);
   }

   // Case 2: Heater current out of limits
   else if (_heaterCurrentA > _heaterCurrentMaxA)
   {
      _errorCode = WaterBathError::HeaterCurrentHigh;
      _integral = 0.0f;
      _lastError = 0.0f;
      _pidOutput = 0.0f;
      setHeaterOn(false);
   }

   // Case 3: Heater current too low (possible open circuit)
   /*
   else if (_heaterCurrentA < _heaterCurrentMinA && _heaterOn)
   {
      _errorCode = WaterBathError::HeaterCurrentLow;
      _integral = 0.0f;
      _lastError = 0.0f;
      _pidOutput = 0.0f;
      setHeaterOn(false);
   }
   

   // Case 4: Block over-temperature
   else if (_blockTempC > _blockTempMaxC)
   {
      _errorCode = WaterBathError::BlockOvertemp;
      _integral = 0.0f;
      _lastError = 0.0f;
      _pidOutput = 0.0f;
      setHeaterOn(false);
   }
   */

   // No faults, apply PID: output 0–1, threshold at 0.5 for on/off (one decision per 500 ms cycle)
   else
   {
      _errorCode = WaterBathError::None;
      if (!_heaterRequestEnabled)
      {
         _heaterOn = false;
      }
      else
      {
         // Heater control logic with deadband
         // Heater on if bath temp is below target - deadband, off if above target + deadband
         if (_bathTempC < _targetTempC - _deadbandC)
            _heaterOn = true;
         else if (_bathTempC > _targetTempC + _deadbandC)
            _heaterOn = false;
      }
      setHeaterOn(_heaterOn);
   }

   _bathSensor->requestTemperatures();  // for next update()
}

const char *WaterBathController::errorToString(WaterBathError code)
{
   switch (code)
   {
      case WaterBathError::None:                  return "None";
      case WaterBathError::HeaterCurrentHigh:     return "HeaterCurrentHigh";
      case WaterBathError::HeaterCurrentLow:     return "HeaterCurrentLow";
      case WaterBathError::BlockOvertemp:         return "BlockOvertemp";
      case WaterBathError::BathSensorDisconnected: return "BathSensorDisconnected";
      default:                                   return "Unknown";
   }
}
