#ifndef HEALTH_LED_H
#define HEALTH_LED_H

#include <Arduino.h>

enum class HealthLedState : uint8_t
{
   Off = 0,
   Healthy,
   Fault,
};

class HealthLed
{
public:
   HealthLed(uint8_t pin, bool activeHigh = true);

   void begin();
   void setState(HealthLedState state);
   HealthLedState getState() const { return _state; }
   void tick(uint32_t nowMs);

private:
   bool computeOutput(uint32_t phaseMs) const;
   void writeOutput(bool isOn);

   uint8_t _pin;
   bool _activeHigh;
   bool _initialized = false;
   bool _lastOutput = false;
   bool _hasWrittenOutput = false;
   HealthLedState _state = HealthLedState::Off;
   uint32_t _stateStartMs = 0;
};

#endif // HEALTH_LED_H
