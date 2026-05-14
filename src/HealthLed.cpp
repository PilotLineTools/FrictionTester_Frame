#include "HealthLed.h"

namespace
{
constexpr uint32_t kHealthyCycleMs = 2000;
constexpr uint32_t kHealthyPulseMs = 120;
constexpr uint32_t kFaultCycleMs = 160;
constexpr uint32_t kFaultPulseMs = 80;
}

HealthLed::HealthLed(uint8_t pin, bool activeHigh)
   : _pin(pin), _activeHigh(activeHigh)
{
}

void HealthLed::begin()
{
   pinMode(_pin, OUTPUT);
   _initialized = true;
   _stateStartMs = millis();
   writeOutput(false);
}

void HealthLed::setState(HealthLedState state)
{
   if (_state == state)
      return;

   _state = state;
   _stateStartMs = millis();
}

void HealthLed::tick(uint32_t nowMs)
{
   if (!_initialized)
      return;

   const uint32_t phaseMs = nowMs - _stateStartMs;
   writeOutput(computeOutput(phaseMs));
}

bool HealthLed::computeOutput(uint32_t phaseMs) const
{
   switch (_state)
   {
   case HealthLedState::Off:
      return false;
   case HealthLedState::Healthy: // brief flash every 2 seconds: on for 120 ms, off for 1880 ms
      return (phaseMs % kHealthyCycleMs) < kHealthyPulseMs;
   case HealthLedState::Fault:   // faster flashing: on for 80 ms, off for 80 ms
      return (phaseMs % kFaultCycleMs) < kFaultPulseMs;
   }

   return false;
}

void HealthLed::writeOutput(bool isOn)
{
   if (_hasWrittenOutput && _lastOutput == isOn)
      return;

   digitalWrite(_pin, (isOn == _activeHigh) ? HIGH : LOW);
   _lastOutput = isOn;
   _hasWrittenOutput = true;
}
