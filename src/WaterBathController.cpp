/**
 * WaterBathController - Implementation.
 * On/off bath temperature control with heater current and block overtemp protection.
 */

#include "WaterBathController.h"
#include "Motor.h"
#include "TMC2209Driver.h"

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

void WaterBathController::setCirculatorTargetRpm(float rpm)
{
   if (rpm < 0.0f)
      rpm = 0.0f;
   _circulatorTargetRpm = rpm;
   applyCirculator();
}

void WaterBathController::setHeaterOn(bool on)
{
   const bool requestedOn = on;
   if (!_enabled || _errorCode != WaterBathError::None)
      on = false;
   
   _heaterOn = on;
   uint8_t pinLevel = on ? HIGH : LOW;
   digitalWrite(_heaterPin, pinLevel);
   USBSerial.printf("WB FLOW heater apply requested=%u applied=%u enabled=%u err=%u pin=%u\n",
                    (unsigned)requestedOn,
                    (unsigned)on,
                    (unsigned)_enabled,
                    (unsigned)_errorCode,
                    (unsigned)pinLevel);
}

void WaterBathController::enableHeater()
{
   // Do not force ON here; apply the currently requested heater state.
   setHeaterOn(_heaterOn);
}

void WaterBathController::disableHeater()
{
   setHeaterOn(false);
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

void WaterBathController::applyCirculator()
{
   float rpm = (_enabled && _errorCode == WaterBathError::None) ? _circulatorTargetRpm : 0.0f;

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
      setHeaterOn(false);
   }

   /**/

   // Case 2: Heater current out of limits
   else if (_heaterCurrentA > _heaterCurrentMaxA)
   {
      _errorCode = WaterBathError::HeaterCurrentHigh;
      setHeaterOn(false);
   }

   // Case 3: Heater current too low (possible open circuit)
   else if (_heaterCurrentA < _heaterCurrentMinA && _heaterOn)
   {
      _errorCode = WaterBathError::HeaterCurrentLow;
      setHeaterOn(false);
   }

   // Case 4: Block over-temperature
   else if (_blockTempC > _blockTempMaxC)
   {
      _errorCode = WaterBathError::BlockOvertemp;
      setHeaterOn(false);
   }

   // No faults, apply normal control logic
   else
   {
      _errorCode = WaterBathError::None;
      // Heater control logic with deadband
      // Heater on if bath temp is below target - deadband, off if above target + deadband
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
