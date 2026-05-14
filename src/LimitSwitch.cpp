#include "LimitSwitch.h"
#include <Arduino.h>

LimitSwitch::LimitSwitch(uint8_t pin, bool activeHigh)
   : _minPin(pin), _maxPin(255), _activeHigh(activeHigh), _mode(Mode::LocalSingle), _hasMax(false)
{
}

LimitSwitch::LimitSwitch(uint8_t minPin, uint8_t maxPin, bool activeHigh)
   : _minPin(minPin), _maxPin(maxPin), _activeHigh(activeHigh), _mode(Mode::LocalDual), _hasMax(maxPin != 255)
{
}

LimitSwitch::LimitSwitch(ILimitSource *remoteSource, bool hasMax)
   : _minPin(255), _maxPin(255), _activeHigh(true), _mode(Mode::Remote), _remoteSource(remoteSource), _hasMax(hasMax)
{
}

void LimitSwitch::setTriggerMode(TriggerMode mode)
{
   _triggerMode = mode;
}

void LimitSwitch::setIsrNotifyCallback(ChangeCallback cb, void *ctx)
{
   _isrNotifyCallback = cb;
   _isrNotifyCallbackCtx = ctx;
}

void LimitSwitch::begin()
{
   // Remote mode has no GPIO setup; we only validate the source and start tracking.
   if (_mode == Mode::Remote)
   {
      if (_remoteSource == nullptr)
      {
         USBSerial.println("LimitSwitch: remote source not configured");
         return;
      }
      _configured = true;
      _lastState = _remoteSource->readLimitState();
      _isrStateDirtyFlag = true;
      _hasPublishedState = false;
      return;
   }

   if (_minPin == 255 && _maxPin == 255)
   {
      USBSerial.println("LimitSwitch: no limit pins configured");
      return;
   }

   if (_minPin != 255)
      pinMode(_minPin, _activeHigh ? INPUT : INPUT_PULLUP);
   if (_maxPin != 255)
      pinMode(_maxPin, _activeHigh ? INPUT : INPUT_PULLUP);

   _configured = true;
   _lastState = getState();
   _isrStateDirtyFlag = true;
   _hasPublishedState = false;

   if (_triggerMode == TriggerMode::InterruptLatched)
   {
      // Capture edges immediately and defer heavier work to task context.
      if (_minPin != 255)
         attachInterruptArg(_minPin, &LimitSwitch::staticHandleInterruptISR, this, CHANGE);
      if (_maxPin != 255)
         attachInterruptArg(_maxPin, &LimitSwitch::staticHandleInterruptISR, this, CHANGE);
   }
}

void LimitSwitch::tick()
{
   if (!_configured)
      return;

   if (_triggerMode == TriggerMode::InterruptLatched && _mode != Mode::Remote)
   {
      // In interrupt-latched mode, only sample when an edge was observed.
      noInterrupts();
      const bool edgeLatched = _isrEdgeLatched;
      _isrEdgeLatched = false;
      interrupts();
      if (!edgeLatched)
         return;
   }

   LimitSwitchState state = getState();
   if (state.minTriggered != _lastState.minTriggered ||
       state.maxTriggered != _lastState.maxTriggered)
   {
      _lastState = state;
      _isrStateDirtyFlag = true;
      USBSerial.printf("LimitSwitch: min=%d max=%d\n",
                       state.minTriggered,
                       state.maxTriggered);
   }
}

LimitSwitchState LimitSwitch::getState() const
{
   LimitSwitchState state = {};
   if (!_configured)
      return state;

   // Remote switches read from the injected source (e.g. latest CAN status cache).
   if (_mode == Mode::Remote)
   {
      if (_remoteSource != nullptr)
         return _remoteSource->readLimitState();
      return state;
   }

   if (_minPin != 255)
      state.minTriggered = (digitalRead(_minPin) == _activeHigh);
   if (_maxPin != 255)
      state.maxTriggered = (digitalRead(_maxPin) == _activeHigh);
   return state;
}

bool LimitSwitch::consumeStateChange(LimitSwitchState &stateOut)
{
   if (!_configured)
      return false;

   // Snapshot and clear the ISR-set dirty flag atomically.
   noInterrupts();
   const bool dirty = _isrStateDirtyFlag;
   _isrStateDirtyFlag = false;
   interrupts();
   if (!dirty)
      return false;

   const LimitSwitchState state = getState();
   const bool changed = (!_hasPublishedState) ||
                        (state.minTriggered != _lastPublishedState.minTriggered) ||
                        (state.maxTriggered != _lastPublishedState.maxTriggered);
   if (!changed)
      return false;

   _lastState = state;
   _lastPublishedState = state;
   _hasPublishedState = true;
   stateOut = state;
   return true;
}

void IRAM_ATTR LimitSwitch::staticHandleInterruptISR(void *ctx)
{
   auto *instance = static_cast<LimitSwitch *>(ctx);
   if (instance)
      instance->handleInterruptISR();
}

void IRAM_ATTR LimitSwitch::handleInterruptISR()
{
   // ISR must stay lightweight: just mark dirty and notify listener.
   _isrEdgeLatched = true;
   _isrStateDirtyFlag = true;
   if (_isrNotifyCallback)
      _isrNotifyCallback(_isrNotifyCallbackCtx);
}
