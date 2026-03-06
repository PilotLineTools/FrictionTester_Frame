/**
 * PowerController - manages frame power / GUI shutdown sequencing.
 *
 * - Latches power via POWER_HOLD_PIN.
 * - Interprets POWER_BUTTON_SIGNAL (short vs long press) for shutdown.
 * - Tracks remote GUI state via CAN heartbeat (0x012) timeout.
 * - Drives LED_BUILTIN_PIN to indicate state.
 *
 * poll10ms() is called every 10 ms from the 100 Hz timer3 loop.
 */

#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"

class PowerController
{
public:
    enum class Notification
    {
        GUIPoweredOn,
        ShutdownRequested,
        ShutdownInitiated,
        ShutdownAborted,
    };

    using NotificationCallback = void (*)(Notification notification);

    // Singleton setup; call once from setup() to initialize and register callback.
    static PowerController *setup(NotificationCallback cb)
    {
        PowerController *instance = getInstance();
        instance->_notificationCallback = cb;
        return instance;
    }

    // Called every 10 ms from timer3Didtic block.
    void poll10ms();
    void requestShutdownFromRemote();
    void onGuiHeartbeat(uint32_t nowMs);

    bool isButtonPressed() const { return _powerButtonIsPushed != 0; }
    bool isGuiSignalOn() const;
    bool clearFaultToActiveIfShutdown();

    uint8_t getGuiPowerStateCode() const { return static_cast<uint8_t>(_guiPowerState); }

private:
    NotificationCallback _notificationCallback = nullptr;

    // GUI power state inferred from heartbeat activity, used for LED and shutdown sequencing.
    enum class GuiPowerState : uint8_t
    {
        OFF,
        BOOTING_UP,
        ACTIVE,
        SHUTTING_DOWN,
        SHUT_DOWN
    };

    GuiPowerState _guiPowerState = GuiPowerState::OFF;

    // Timers and state (in milliseconds where noted)
    uint8_t _powerLED = 1;
    uint16_t _powerLEDTimerMs = 0;

    uint8_t _powerButtonIsPushed = 0;
    uint8_t _previousPowerButtonIsPushed = 0;

    uint16_t _powerButtonPressTimerMs = 0;
    uint16_t _powerButtonPressedCounter = 0; // how many short presses
    uint16_t _powerButtonPowerOffTimerMs = 0;
    bool _longPressHandled = false;
    uint32_t _lastGuiHeartbeatMs = 0;
    bool _guiHeartbeatSeen = false;

    PowerController();
    ~PowerController() = default;

    PowerController(const PowerController &) = delete;
    PowerController &operator=(const PowerController &) = delete;

    static PowerController *getInstance()
    {
        static PowerController instance;
        return &instance;
    }

    void emit(Notification n)
    {
        if (_notificationCallback)
            _notificationCallback(n);
    }

    void checkStartupState(bool guiOn);
    bool isGuiAliveNow() const;
};

#endif // POWER_CONTROLLER_H
