/**
 * WaterBathCanAdapter - CAN (TWAI) ↔ WaterBathController bridge.
 * Friction Tester CAN Communication Protocol: SET_WATER_TEMP (0x080), BATH_STATUS (0x280), FRAME_ACK (0x282).
 * WaterBathController remains CAN-agnostic; this adapter translates messages and calls controller API.
 *
 * Registration example (e.g. in main with CanRouter):
 *   CanRouter canRouter;
 *   WaterBathCanAdapter adapter(&waterBathController, &canRouter);
 *   adapter.setCirculatorRpmCallback(setCirculatorRpm);
 *   adapter.begin();  // registers router.on(CAN_ID_SET_WATER_TEMP, ...)
 *   // In loop: adapter.tick(millis()); and canRouter.dispatch(&rx_message) when twai_receive returns a frame.
 */

#ifndef WATER_BATH_CAN_ADAPTER_H
#define WATER_BATH_CAN_ADAPTER_H

#include "WaterBathController.h"
#include "driver/twai.h"
#include <stdint.h>

// -----------------------------------------------------------------------------
// CAN IDs (11-bit)
// -----------------------------------------------------------------------------
static const uint32_t CAN_ID_SET_WATER_TEMP = 0x080u;
static const uint32_t CAN_ID_BATH_STATUS    = 0x280u;
static const uint32_t CAN_ID_FRAME_ACK      = 0x282u;

// -----------------------------------------------------------------------------
// System mode: gate normal commands during FW_UPDATE
// -----------------------------------------------------------------------------
enum class SystemMode : uint8_t
{
   NORMAL = 0,
   FW_UPDATE
};

// -----------------------------------------------------------------------------
// CAN router interface: register by ID, send to TX queue
// -----------------------------------------------------------------------------
typedef void (*CanHandlerFn)(const twai_message_t *msg, void *ctx);

class ICanRouter
{
public:
   /** Register handler for 11-bit CAN ID. ctx passed to handler. */
   virtual void on(uint32_t id, CanHandlerFn fn, void *ctx) = 0;
   /** Enqueue frame for TX. Non-blocking. Returns true if queued. */
   virtual bool send(const twai_message_t *msg) = 0;
   virtual ~ICanRouter() = default;
};

// -----------------------------------------------------------------------------
// Little-endian pack / unpack helpers (for CAN payloads)
// -----------------------------------------------------------------------------
inline void packU16LE(uint8_t *buf, uint16_t val)
{
   buf[0] = (uint8_t)(val & 0xFF);
   buf[1] = (uint8_t)(val >> 8);
}

inline void packI16LE(uint8_t *buf, int16_t val)
{
   packU16LE(buf, (uint16_t)val);
}

inline uint16_t unpackU16LE(const uint8_t *buf)
{
   return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

inline int16_t unpackI16LE(const uint8_t *buf)
{
   return (int16_t)unpackU16LE(buf);
}

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
   WaterBathCanAdapter(WaterBathController *controller, ICanRouter *router);

   /** Wire circulator RPM (same as controller callback); call from main. */
   void setCirculatorRpmCallback(SetCirculatorRpmFn fn) { _setCirculatorRpm = fn; }

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
   SetCirculatorRpmFn _setCirculatorRpm = nullptr;
   SystemMode _mode = SystemMode::NORMAL;

   uint32_t _lastStatusMs = 0;
   uint32_t _statusIntervalMs = 500;  // 0.5 s, aligned with controller update
   uint8_t _ackSeq = 0;

   void handleSetWaterTemp(const twai_message_t *msg);
   void sendBathStatus();
   void sendAck(uint8_t result, uint8_t detailCode);

   static void staticHandleSetWaterTemp(const twai_message_t *msg, void *ctx);
};

#endif // WATER_BATH_CAN_ADAPTER_H
