/**
 * LimitSwitch - owns standalone GPIO limit switch monitoring.
 */

#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stdint.h>
#include "ILimitSource.h"

class LimitSwitch
{
public:
   // Trigger mode controls whether we simply poll the switch state in the main loop, 
   // or if we also track edge-triggered changes from an ISR for more immediate response 
   // and to avoid missing short triggers.
   enum class TriggerMode : uint8_t
   {
      PollOnly,
      InterruptLatched
   };

   using ChangeCallback = void (*)(void *ctx);

   // Constructors for different wiring configurations:
   // Single local switch (min only)
   LimitSwitch(uint8_t pin, bool activeHigh);
   // Dual local switches (min and max)
   LimitSwitch(uint8_t minPin, uint8_t maxPin, bool activeHigh);
   // Remote switch(es) read from an injected source (e.g. latest CAN status cache).
   LimitSwitch(ILimitSource *remoteSource, bool hasMax = true);

   void begin();
   void tick();
   void setTriggerMode(TriggerMode mode);
   TriggerMode getTriggerMode() const { return _triggerMode; }
   void setIsrNotifyCallback(ChangeCallback cb, void *ctx);
   void setChangeCallback(ChangeCallback cb, void *ctx) { setIsrNotifyCallback(cb, ctx); } // Backward-compatible alias.

   LimitSwitchState getState() const;
   bool consumeStateChange(LimitSwitchState &stateOut);
   bool isConfigured() const { return _configured; }
   uint8_t getMinPin() const { return _minPin; }
   uint8_t getMaxPin() const { return _maxPin; }
   bool isActiveHigh() const { return _activeHigh; }
   bool supportsMin() const { return true; }
   bool supportsMax() const { return _hasMax; }
   bool isRemote() const { return _remoteSource != nullptr; }

private:
   enum class Mode : uint8_t
   {
      LocalSingle,
      LocalDual,
      Remote
   };

   uint8_t _minPin;
   uint8_t _maxPin;
   bool _activeHigh;
   Mode _mode = Mode::LocalSingle;
   ILimitSource *_remoteSource = nullptr; // For remote mode, the injected source of limit state (e.g. latest CAN status cache).
   bool _hasMax = false;
   bool _configured = false;
   volatile bool _isrStateDirtyFlag = false;
   volatile bool _isrEdgeLatched = false;
   bool _hasPublishedState = false;
   TriggerMode _triggerMode = TriggerMode::PollOnly;
   ChangeCallback _isrNotifyCallback = nullptr;
   void *_isrNotifyCallbackCtx = nullptr;
   LimitSwitchState _lastState = {};
   LimitSwitchState _lastPublishedState = {};

   static void staticHandleInterruptISR(void *ctx);
   void handleInterruptISR();
};

#endif // LIMIT_SWITCH_H
