/**
 * PowerController - Implementation for friction tester.
 *
 * Uses:
 *  - POWER_BUTTON_SIGNAL: user power button (active when LOW).
 *  - CAN heartbeat (0x012): remote GUI alive indication.
 *  - POWER_HOLD_PIN: latch power (HIGH to hold on, LOW to cut).
 *  - LED_BUILTIN_PIN: power status LED.
 *
 * poll10ms() must be called every 10 ms from the 100 Hz timer3 loop.
 */

#include "PowerController.h"

// Button timing in ms
static constexpr uint16_t POWER_BUTTON_PRESS_TIMEOUT_MS = 100;   // timeout between short presses
static constexpr uint16_t POWER_BUTTON_HOLD_OFF_TIME_MS = 2000; // hold to request shutdown
static constexpr uint32_t GUI_HEARTBEAT_TIMEOUT_MS = 3000;       // 1 s heartbeat + jitter margin

// LED blink intervals in ms
static constexpr uint16_t LED_BLINK_FAST_MS = 200;
static constexpr uint16_t LED_BLINK_SLOW_MS = 1000;
static constexpr uint16_t LED_BLINK_SHUTTING_MS = 500;

PowerController::PowerController()
{
    pinMode(POWER_HOLD_PIN, OUTPUT);
    digitalWrite(POWER_HOLD_PIN, HIGH); // latch power on at boot

    pinMode(POWER_BUTTON_SIGNAL, INPUT);
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, _powerLED);
}

void PowerController::poll10ms()
{
    // 10 ms per call
    _powerLEDTimerMs += 10;
    _powerButtonPressTimerMs += 10;

    // Button assumed active HIGH (pressed when reading HIGH)
    _powerButtonIsPushed = (digitalRead(POWER_BUTTON_SIGNAL) == HIGH) ? 1 : 0;
    bool guiOn = isGuiSignalOn();

    // Button edge/hold debug (runs regardless of GUI power state)
    const bool justPressed  = (_powerButtonIsPushed && !_previousPowerButtonIsPushed);
    const bool justReleased = (!_powerButtonIsPushed && _previousPowerButtonIsPushed);

    // Heartbeat timeout means GUI is no longer alive; reflect that in state.
    if (!guiOn &&
        (_guiPowerState == GuiPowerState::BOOTING_UP || _guiPowerState == GuiPowerState::ACTIVE))
    {
        _guiPowerState = GuiPowerState::OFF;
    }

    if (justPressed)
    {
        _powerButtonPressedCounter++;
        _powerButtonPressTimerMs = 0;
    }

    if (_powerButtonIsPushed)
    {
        _powerButtonPowerOffTimerMs += 10;

        if (!_longPressHandled &&
            _powerButtonPowerOffTimerMs >= POWER_BUTTON_HOLD_OFF_TIME_MS)
        {
            if (!guiOn)
            {
                // GUI is off (never powered or already shut down): hard power-off is safe.
                emit(Notification::ShutdownRequested);
                _guiPowerState = GuiPowerState::SHUT_DOWN;
            }
            else
            {
                // GUI currently on: request graceful shutdown and wait for GUI_SHUTDOWN_PIN to go HIGH.
                if (_guiPowerState != GuiPowerState::SHUTTING_DOWN)
                {
                    _guiPowerState = GuiPowerState::SHUTTING_DOWN;
                    emit(Notification::ShutdownRequested);
                }
            }

            _longPressHandled = true; // do not retrigger for this hold
        }
    }
    else if (justReleased)
    {
        _powerButtonPowerOffTimerMs = 0;
        _longPressHandled = false;
    }

    switch (_guiPowerState)
    {
    case GuiPowerState::OFF:
    {
        // GUI has no power yet (GUI_SHUTDOWN_PIN HIGH): fast blink, wait for first LOW.
        uint16_t interval = LED_BLINK_FAST_MS;
        if (_powerLEDTimerMs >= interval)
        {
            _powerLEDTimerMs = 0;
            _powerLED = _powerLED ? 0 : 1;
        }

        if (guiOn)
        {
            _guiPowerState = GuiPowerState::BOOTING_UP;
            USBSerial.println("GUI power detected (pin LOW), entering BOOTING_UP");
        }
    }
    break;

    case GuiPowerState::BOOTING_UP:
    {
        // GUI has power (24 V on, shutdown pin LOW); slow blink until we see a CAN heartbeat.
        uint16_t interval = LED_BLINK_SLOW_MS;
        if (_powerLEDTimerMs >= interval)
        {
            _powerLEDTimerMs = 0;
            _powerLED = _powerLED ? 0 : 1;
        }
        // Transition to ACTIVE happens in onGuiHeartbeat() when first heartbeat arrives.
    }
    break;

    case GuiPowerState::ACTIVE:
        _powerLED = 1;
        _powerLEDTimerMs = 0;

        // Count short presses (button down/up cycles) within timeout window
        if (_powerButtonPressTimerMs > POWER_BUTTON_PRESS_TIMEOUT_MS && _powerButtonPressedCounter > 0)
        {
            emit(Notification::ShutdownInitiated);
            _powerButtonPressedCounter = 0;
            _powerButtonPressTimerMs = 0;
        }

        // Long-press behavior is handled in the generic section above.
        // Here we only treat sub-threshold releases as an aborted shutdown.
        if (! _powerButtonIsPushed &&
            _powerButtonPowerOffTimerMs < POWER_BUTTON_HOLD_OFF_TIME_MS &&
            _powerButtonPowerOffTimerMs > 0)
        {
            emit(Notification::ShutdownAborted);
        }
        break;

    case GuiPowerState::SHUTTING_DOWN:
        if (_powerLEDTimerMs >= LED_BLINK_SHUTTING_MS)
        {
            _powerLEDTimerMs = 0;
            _powerLED = _powerLED ? 0 : 1;
        }

        // Wait for GUI to power off (GUI_SHUTDOWN_PIN HIGH) before cutting frame power
        if (!guiOn)
        {
            USBSerial.println("GUI off. Cutting main power.");
            _guiPowerState = GuiPowerState::SHUT_DOWN;
        }
        break;

    case GuiPowerState::SHUT_DOWN:
        _powerLED = 0;
        digitalWrite(POWER_HOLD_PIN, LOW); // cut power
        if (!guiOn)
            _guiPowerState = GuiPowerState::OFF;
        break;
    }

    _previousPowerButtonIsPushed = _powerButtonIsPushed;
    digitalWrite(LED_BUILTIN_PIN, _powerLED);
}

void PowerController::requestShutdownFromRemote()
{
    bool guiOn = isGuiAliveNow();
    if (!guiOn)
    {
        emit(Notification::ShutdownRequested);
        _guiPowerState = GuiPowerState::SHUT_DOWN;
        return;
    }

    if (_guiPowerState != GuiPowerState::SHUTTING_DOWN &&
        _guiPowerState != GuiPowerState::SHUT_DOWN)
    {
        _guiPowerState = GuiPowerState::SHUTTING_DOWN;
        emit(Notification::ShutdownRequested);
    }
}

void PowerController::onGuiHeartbeat(uint32_t nowMs)
{
    _lastGuiHeartbeatMs = nowMs;
    _guiHeartbeatSeen = true;

    // First heartbeat after power-on: GUI OS + app are up, enter ACTIVE.
    if (_guiPowerState == GuiPowerState::BOOTING_UP)
    {
        _guiPowerState = GuiPowerState::ACTIVE;
        _powerLED = 1;
        _powerLEDTimerMs = 0;
        emit(Notification::GUIPoweredOn);
    }
}

bool PowerController::isGuiSignalOn() const
{
    return digitalRead(GUI_SHUTDOWN_PIN) == LOW;
}

bool PowerController::isGuiAliveNow() const
{
    if (!_guiHeartbeatSeen)
        return false;
    return (millis() - _lastGuiHeartbeatMs) <= GUI_HEARTBEAT_TIMEOUT_MS;
}

bool PowerController::clearFaultToActiveIfShuttingDown()
{

    if (_guiPowerState == GuiPowerState::SHUTTING_DOWN)
    {
        setGuiPowerStateCode(static_cast<uint8_t>(GuiPowerState::ACTIVE));
        emit(Notification::ShutdownAborted); // optional
        return true;
    }
    return false;
}

bool PowerController::setGuiPowerStateCode(uint8_t code)
{
    switch (static_cast<GuiPowerState>(code))
    {
    case GuiPowerState::OFF:
    case GuiPowerState::BOOTING_UP:
    case GuiPowerState::ACTIVE:
    case GuiPowerState::SHUTTING_DOWN:
    case GuiPowerState::SHUT_DOWN:
        _guiPowerState = static_cast<GuiPowerState>(code);
        if (_guiPowerState == GuiPowerState::ACTIVE)
        {
            _powerLED = 1;
            _powerLEDTimerMs = 0;
        }
        return true;
    default:
        return false;
    }
}


void PowerController::checkStartupState(bool guiOn)
{
    if (_guiPowerState == GuiPowerState::BOOTING_UP)
    {
        if (guiOn)
        {
            _guiPowerState = GuiPowerState::ACTIVE;
            emit(Notification::GUIPoweredOn);
        }
    }
}
