/**
 * WaterBathCanAdapter - CAN (TWAI) ↔ WaterBathController bridge.
 * Friction Tester CAN Communication Protocol: SET_WATER_TEMP (0x080), BATH_STATUS (0x280), FRAME_ACK (0x282).
 * WaterBathController remains CAN-agnostic; this adapter translates messages and calls controller API.
 *
 * Registration example (e.g. in main with CanRouter):
 *   CanRouter canRouter;
 *   WaterBathCanAdapter adapter(&waterBathController, &canRouter);
 *   adapter.begin();  // registers router.on(CAN_ID_SET_WATER_TEMP, ...)
 *   // In loop: adapter.tick(millis()); and canRouter.dispatch(&rx_message) when twai_receive returns a frame.
 */

#ifndef WATER_BATH_CAN_ADAPTER_H
#define WATER_BATH_CAN_ADAPTER_H

#include "WaterBathController.h"
#include "FrameCanAdapter.h"
#include "ICanRouter.h"
#include "CanCodec.h"
#include "SystemMode.h"
#include "driver/twai.h"
#include <stdint.h>

// -----------------------------------------------------------------------------
// CAN IDs (11-bit)
// -----------------------------------------------------------------------------
static const uint32_t CAN_ID_SET_WATER_BATH = 0x080u;
static const uint32_t CAN_ID_BATH_STATUS    = 0x280u;

// -----------------------------------------------------------------------------
// WaterBathCanAdapter
// -----------------------------------------------------------------------------
class WaterBathCanAdapter
{
public:
   /**
    * @param controller Water bath controller (must outlive adapter).
    * @param router     CAN router for registering 0x080 and sending frames.
    */
   WaterBathCanAdapter(WaterBathController *controller, ICanRouter *router, FrameCanAdapter *frameCan);

   /** Register 0x080 handler and set telemetry interval. Call once from setup. */
   void begin();

   /** Call from main loop; sends BATH_STATUS at configured rate (e.g. 5–10 Hz). */
   void tick(uint32_t now_ms);

   /** Gate: during FW_UPDATE, SET_WATER_TEMP is ignored (ACK with result=1). */
   void onModeChanged(SystemMode mode) { _mode = mode; }
   SystemMode getMode() const { return _mode; }

private:
   WaterBathController *_controller;
   ICanRouter *_router;
   FrameCanAdapter *_frameCan;
   SystemMode _mode = SystemMode::NORMAL;

   uint32_t _lastStatusMs = 0;
   uint32_t _statusIntervalMs = 500;  // 0.5 s, aligned with controller update
   uint32_t _flowSeq = 0;
   int8_t _lastLoggedHeater = -1;
   int8_t _lastLoggedEnabled = -1;

   void handleSetWaterBath(const twai_message_t *msg);
   void sendBathStatus();
   void sendAck(uint8_t result, uint8_t detailCode);

   static void staticHandleSetWaterBath(const twai_message_t *msg, void *ctx);
};

#endif // WATER_BATH_CAN_ADAPTER_H
