/**
 * CarriageCanAdapter - CAN (TWAI) ↔ CarriageController bridge.
 * Handles carriage motion messages only.
 */

#ifndef CARRIAGE_CAN_ADAPTER_H
#define CARRIAGE_CAN_ADAPTER_H

#include "CarriageController.h"
#include "FrameCanAdapter.h"
#include "ICanRouter.h"
#include "CanCodec.h"
#include "SystemMode.h"
#include <stdint.h>

static const uint32_t CAN_ID_CARRIAGE_STOP = 0x0A0u;
static const uint32_t CAN_ID_CARRIAGE_SET_ACCELERATION = 0x0A1u;
static const uint32_t CAN_ID_CARRIAGE_JOG = 0x0A2u;
static const uint32_t CAN_ID_CARRIAGE_MOVE_ABS = 0x0A3u;
static const uint32_t CAN_ID_CARRIAGE_MOVE_REL = 0x0A4u;
static const uint32_t CAN_ID_CARRIAGE_HOME = 0x0A5u;
static const uint32_t CAN_ID_CARRIAGE_MOVE_DONE = 0x2AAu;

class CarriageCanAdapter
{
public:
   CarriageCanAdapter(CarriageController *controller, ICanRouter *router, FrameCanAdapter *frameCan);
   void begin();
   void tick();
   void onModeChanged(SystemMode mode) { _mode = mode; }
   SystemMode getMode() const { return _mode; }

private:
   CarriageController *_controller;
   ICanRouter *_router;
   FrameCanAdapter *_frameCan;
   float _defaultSpeed = 1000.0f;

   static void staticHandleMoveRel(const twai_message_t *msg, void *ctx);
   static void staticHandleMoveAbs(const twai_message_t *msg, void *ctx);
   static void staticHandleSetAcceleration(const twai_message_t *msg, void *ctx);
   static void staticHandleJog(const twai_message_t *msg, void *ctx);
   static void staticHandleHome(const twai_message_t *msg, void *ctx);
   static void staticHandleStop(const twai_message_t *msg, void *ctx);

   void handleMoveRel(const twai_message_t *msg);
   void handleMoveAbs(const twai_message_t *msg);
   void handleSetAcceleration(const twai_message_t *msg);
   void handleJog(const twai_message_t *msg);
   void handleHome(const twai_message_t *msg);
   void handleStop();
   void sendMoveDone(float positionMm);

   SystemMode _mode = SystemMode::NORMAL;
   bool _absMoveDonePending = false;
   bool _absMoveWasMoving = false;
};

#endif // CARRIAGE_CAN_ADAPTER_H
