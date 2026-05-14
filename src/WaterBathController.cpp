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

void WaterBathController::setCirculatorAccelRpmPerSec(float accelRpmPerSec)
{
   if (accelRpmPerSec < 1.0f)
      accelRpmPerSec = 1.0f;
   _circulatorAccelRpmPerSec = accelRpmPerSec;
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
   const float targetRpm = _enabled ? _circulatorTargetRpm : 0.0f;
   const float maxStep = _circulatorAccelRpmPerSec * _dt;
   const float delta = targetRpm - _circulatorAppliedRpm;
   if (fabsf(delta) <= maxStep)
      _circulatorAppliedRpm = targetRpm;
   else
      _circulatorAppliedRpm += (delta > 0.0f) ? maxStep : -maxStep;

   if (_circulatorDriver != nullptr)
      _circulatorDriver->setRpmActual(_circulatorAppliedRpm);

   if (_circulatorMotor != nullptr)
      (_circulatorAppliedRpm > 0.0f) ? _circulatorMotor->enable() : _circulatorMotor->disable();
}

void WaterBathController::enableCirculator()
{
   applyCirculator();
}

void WaterBathController::disableCirculator()
{
   _circulatorAppliedRpm = 0.0f;
   if (_circulatorDriver != nullptr)
      _circulatorDriver->setRpmActual(0.0f);

   if (_circulatorMotor != nullptr)
      _circulatorMotor->disable();
}

void WaterBathController::update()
{
   applyCirculator();
   _bathTempC = _bathSensor->getTempCByIndex(0);
   _blockTempC = readBlockTempC();
   _heaterCurrentA = _currentSensor->getCurrent();

   // Update filtered bath temperature slope for D term (derivative on measurement).
   // Target window ~3–4 minutes: tau ≈ _derivTauSec; alpha ≈ dt / tau.
   if (!_slopeInit)
   {
      _lastBathTempForSlope = _bathTempC;
      _tempSlopeFiltered = 0.0f;
      _slopeInit = true;
   }
   float instSlope = (_bathTempC - _lastBathTempForSlope) / _dt;  // °C/s over last 0.5 s
   _lastBathTempForSlope = _bathTempC;
   const float alphaD = _dt / _derivTauSec;
   _tempSlopeFiltered = (1.0f - alphaD) * _tempSlopeFiltered + alphaD * instSlope;

   // Check error conditions. If any fault is active, turn off heater and set error code.

   // Case 1: Bath sensor disconnected or out of range
   if (_bathTempC <= -127.0f || _bathTempC >= 85.0f)
   {
      _errorCode = WaterBathError::BathSensorDisconnected;
      _integral = 0.0f;
      _lastError = 0.0f;
      _pidOutput = 0.0f;
      _dutyAccum = 0.0f;
      _tempSlopeFiltered = 0.0f;
      _slopeInit = false;
      setHeaterOn(false);
   }

   // Case 2: Heater current out of limits
   else if (_heaterCurrentA > _heaterCurrentMaxA)
   {
      _errorCode = WaterBathError::HeaterCurrentHigh;
      _integral = 0.0f;
      _lastError = 0.0f;
      _pidOutput = 0.0f;
      _dutyAccum = 0.0f;
      _tempSlopeFiltered = 0.0f;
      _slopeInit = false;
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
         _pidOutput = 0.0f;
         _dutyAccum = 0.0f;
         _integral = 0.0f;
         _lastError = 0.0f;
         _tempSlopeFiltered = 0.0f;
         _slopeInit = false;
      }
      else
      {
         // PID: error positive = below target, need heating
         float error = _targetTempC - _bathTempC;
         float pTerm = _kp * error;
         _integral += _ki * error * _dt;
         if (_integral > _integralMax)
            _integral = _integralMax;
         else if (_integral < 0)
            _integral = 0;
         // Derivative on measurement: when temperature is rising (positive slope),
         // D term reduces output; when falling, it increases output.
         float dTerm = -_kd * _tempSlopeFiltered;
         _lastError = error;

         _pidOutput = pTerm + _integral + dTerm;
         if (_pidOutput > 1.0f)
            _pidOutput = 1.0f;
         else if (_pidOutput < 0.0f)
            _pidOutput = 0.0f;

         // Duty-cycle heater: Bresenham-style, one on/off decision per 500 ms cycle.
         // e.g. 33% duty → heater on 1 in 3 cycles; 50% → every other cycle.
         _dutyAccum += _pidOutput;
         if (_dutyAccum >= 1.0f)
         {
            _heaterOn = true;
            _dutyAccum -= 1.0f;
         }
         else
            _heaterOn = false;

         // Throttled debug print: once every ~10 seconds
         uint32_t nowMs = millis();
         if (nowMs - _lastDebugMs >= 10000u)
         {
            _lastDebugMs = nowMs;
            USBSerial.printf("%lu, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                             (unsigned long)nowMs, _bathTempC, pTerm, _integral, _tempSlopeFiltered*1000.0f, dTerm, _pidOutput);
         }
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
