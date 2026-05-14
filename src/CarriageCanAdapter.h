/**
 * CarriageCanAdapter - CAN (TWAI) ↔ CarriageController bridge.
 * Handles carriage motion messages only.
 */

#ifndef CARRIAGE_CAN_ADAPTER_H
#define CARRIAGE_CAN_ADAPTER_H

#include "CarriageController.h"
#include "FrameESP_CanAdapter.h"
#include "ICanRouter.h"
#include "CanCodec.h"
#include "SystemMode.h"
#include "ILimitSource.h"
#include <stdint.h>

static const uint32_t CAN_ID_CARRIAGE_STOP = 0x0A0u;
static const uint32_t CAN_ID_CARRIAGE_POSITION = 0x301u;
static const uint32_t CAN_ID_CARRIAGE_SET_ACCELERATION = 0x0A1u;
static const uint32_t CAN_ID_CARRIAGE_LIMIT_STATUS = 0x0A2u;
static const uint32_t CAN_ID_CARRIAGE_MOVE_ABS = 0x0A3u;
static const uint32_t CAN_ID_CARRIAGE_MOVE_REL = 0x0A4u;
static const uint32_t CAN_ID_CARRIAGE_HOME = 0x0A5u;
static const uint32_t CAN_ID_CARRIAGE_LIMITS_REQUEST = 0x1E1u;
static const uint32_t CAN_ID_CARRIAGE_HOMED = 0x2A4u;
static const uint32_t CAN_ID_CARRIAGE_LIMIT_HIT = 0x2A7u;
static const uint32_t CAN_ID_CARRIAGE_MOVE_DONE = 0x2AAu;

class CarriageCanAdapter : public ILimitSource
{
public:
   CarriageCanAdapter(CarriageController *controller, ICanRouter *router, FrameESP_CanAdapter *espCan);
   void begin();
   void tick();
   void requestLimitStatus();
   void onModeChanged(SystemMode mode) { _mode = mode; }
   SystemMode getMode() const { return _mode; }
   LimitSwitchState readLimitState() const override;

private:
   static constexpr uint32_t POSITION_TX_INTERVAL_MS = 50;
   CarriageController *_controller;
   ICanRouter *_router;
   FrameESP_CanAdapter *_espCan;
   float _defaultSpeed = 3000.0f;

   static void staticHandleMoveRel(const twai_message_t *msg, void *ctx);
   static void staticHandleMoveAbs(const twai_message_t *msg, void *ctx);
   static void staticHandleSetAcceleration(const twai_message_t *msg, void *ctx);
   static void staticHandleLimitStatus(const twai_message_t *msg, void *ctx);
   static void staticHandleHome(const twai_message_t *msg, void *ctx);
   static void staticHandleStop(const twai_message_t *msg, void *ctx);

   void handleMoveRel(const twai_message_t *msg);
   void handleMoveAbs(const twai_message_t *msg);
   void handleSetAcceleration(const twai_message_t *msg);
   void handleLimitStatus(const twai_message_t *msg);
   void handleHome(const twai_message_t *msg);
   void handleStop();
   void sendHomed(uint8_t result, float positionMm);
   void sendLimitsRequest();
   void sendLimitHit(float positionMm, uint8_t direction);
   void sendMoveDone(float positionMm);
   void sendPosition(float positionMm);

   SystemMode _mode = SystemMode::NORMAL;
   bool _absMoveDonePending = false;
   bool _absMoveWasMoving = false;
   volatile bool _remoteMinTriggered = false;
   volatile bool _remoteMaxTriggered = false;
   uint32_t _lastPositionTxMs = 0;
};

#endif // CARRIAGE_CAN_ADAPTER_H
