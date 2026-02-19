/**
 * WaterBathController - Implementation.
 * On/off bath temperature control with heater current and block overtemp protection.
 */

#include "WaterBathController.h"

// NTC thermistor: 10k @ 25°C, B = 3950, series resistor 10k to 3.3V, NTC to GND
#define NTC_R0 10000.0f
#define NTC_T0 298.15f   // 25°C in K
#define NTC_B 3950.0f
#define NTC_SERIES_R 10000.0f
#define ADC_MAX 4095.0f
#define ADC_VREF 3.3f

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
   applyCirculator(true);
}

void WaterBathController::disable()
{
   _enabled = false;
   setHeaterOn(false);
   applyCirculator(false);
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

void WaterBathController::setHeaterOn(bool on)
{
   if (!_enabled || _errorCode != WaterBathError::None)
      on = false;

   _heaterOn = on;
   digitalWrite(_heaterPin, on ? HIGH : LOW);
}

float WaterBathController::readBlockTempC()
{
   int raw = analogRead(_blockThermistorPin);
   if (raw <= 0 || raw >= ADC_MAX)
      return -100.0f; // invalid

   float v = (float)raw / ADC_MAX * ADC_VREF;
   float r_ntc = (ADC_VREF - v) > 0.01f ? (v * NTC_SERIES_R / (ADC_VREF - v)) : NTC_R0;

   float ln_r = logf(r_ntc / NTC_R0);
   float t_k = 1.0f / (1.0f / NTC_T0 + ln_r / NTC_B);
   return t_k - 273.15f;
}

void WaterBathController::applyCirculator(bool on)
{
   if (_setCirculatorRpm)
      _setCirculatorRpm(on ? 100.0f : 0.0f);
}

void WaterBathController::update()
{
   _bathTempC = _bathSensor->getTempCByIndex(0);
   _blockTempC = readBlockTempC();
   _heaterCurrentA = _currentSensor->getCurrent();

   if (_bathTempC <= -127.0f || _bathTempC >= 85.0f)
   {
      _errorCode = WaterBathError::BathSensorDisconnected;
      setHeaterOn(false);
   }
   else if (_heaterCurrentA > _heaterCurrentMaxA)
   {
      _errorCode = WaterBathError::HeaterCurrentHigh;
      setHeaterOn(false);
   }
   else if (_heaterCurrentA < _heaterCurrentMinA && _heaterOn)
   {
      _errorCode = WaterBathError::HeaterCurrentLow;
      setHeaterOn(false);
   }
   else if (_blockTempC > _blockTempMaxC)
   {
      _errorCode = WaterBathError::BlockOvertemp;
      setHeaterOn(false);
   }
   else
   {
      _errorCode = WaterBathError::None;
      if (_bathTempC < _targetTempC - _deadbandC)
         _heaterOn = true;
      else if (_bathTempC > _targetTempC + _deadbandC)
         _heaterOn = false;
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
