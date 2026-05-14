/**
 * PowerController - Implementation for friction tester.
 *
 * Uses:
 *  - POWER_BUTTON_SIGNAL: user power button (active when LOW).
 *  - CAN heartbeat (0x012): Pi CAN alive indication.
 *  - POWER_HOLD_PIN: latch power (HIGH to hold on, LOW to cut).
 *
 * poll10ms() must be called every 10 ms from the 100 Hz timer3 loop.
 */

#include "PowerController.h"

// Button timing in ms
static constexpr uint16_t POWER_BUTTON_PRESS_TIMEOUT_MS = 100;   // timeout between short presses
static constexpr uint16_t POWER_BUTTON_HOLD_OFF_TIME_MS = 2000; // hold to request shutdown
static constexpr uint32_t PI_CAN_HEARTBEAT_TIMEOUT_MS = 3000;    // 1 s heartbeat + jitter margin

PowerController::PowerController()
{
    pinMode(POWER_HOLD_PIN, OUTPUT);
    digitalWrite(POWER_HOLD_PIN, HIGH); // latch power on at boot

    pinMode(POWER_BUTTON_SIGNAL, INPUT);
}

void PowerController::poll10ms()
{
    // 10 ms per call
    _powerButtonPressTimerMs += 10;

    // Button assumed active LOW (pressed when reading LOW)
    _powerButtonIsPushed = (digitalRead(POWER_BUTTON_SIGNAL) == LOW) ? 1 : 0;
    bool guiOn = isGuiSignalOn();

    // Button edge/hold debug (runs regardless of GUI power state)
    const bool justPressed  = (_powerButtonIsPushed && !_previousPowerButtonIsPushed);
    const bool justReleased = (!_powerButtonIsPushed && _previousPowerButtonIsPushed);

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
        if (guiOn)
        {
            _guiPowerState = GuiPowerState::ACTIVE;
            USBSerial.println("GUI power detected (pin LOW), entering ACTIVE");
            emit(Notification::GUIPoweredOn);
        }
    }
    break;

    case GuiPowerState::BOOTING_UP:
    {
        if (!guiOn)
        {
            _guiPowerState = GuiPowerState::OFF;
        }
        else
        {
            _guiPowerState = GuiPowerState::ACTIVE;
            emit(Notification::GUIPoweredOn);
        }
    }
    break;

    case GuiPowerState::ACTIVE:
        if (!guiOn)
        {
            _guiPowerState = GuiPowerState::OFF;
            break;
        }

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
        // Wait for GUI to power off (GUI_SHUTDOWN_PIN HIGH) before cutting frame power
        if (!guiOn)
        {
            USBSerial.println("GUI off. Cutting main power.");
            _guiPowerState = GuiPowerState::SHUT_DOWN;
        }
        break;

    case GuiPowerState::SHUT_DOWN:
        digitalWrite(POWER_HOLD_PIN, LOW); // cut power
        if (!guiOn)
            _guiPowerState = GuiPowerState::OFF;
        break;
    }

    _previousPowerButtonIsPushed = _powerButtonIsPushed;
}

void PowerController::requestShutdownFromRemote()
{
    bool guiOn = isGuiSignalOn();
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

    // If GUI power signal is present and we receive a heartbeat, promote startup
    // states to ACTIVE so health checks can pass after normal boot.
    if (isGuiSignalOn() &&
        (_guiPowerState == GuiPowerState::OFF || _guiPowerState == GuiPowerState::BOOTING_UP))
    {
        _guiPowerState = GuiPowerState::ACTIVE;
        emit(Notification::GUIPoweredOn);
    }
}

bool PowerController::isGuiSignalOn() const
{
    return digitalRead(GUI_SHUTDOWN_PIN) == LOW;
}

bool PowerController::isPiCanAliveNow() const
{
    if (!_guiHeartbeatSeen)
        return false;
    return (millis() - _lastGuiHeartbeatMs) <= PI_CAN_HEARTBEAT_TIMEOUT_MS;
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
