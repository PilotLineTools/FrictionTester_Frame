/**
 * FrameESP_CanAdapter - shared system/frame-level CAN helpers.
 * Owns FRAME_ACK (0x282) formatting and sequence numbering.
 */

#ifndef FRAME_ESP_CAN_ADAPTER_H
#define FRAME_ESP_CAN_ADAPTER_H

#include "ICanRouter.h"
#include "CanCodec.h"
#include "SystemMode.h"
#include <stdint.h>

static const uint32_t FRAME_ESP_CAN_ID_HEARTBEAT = 0x013u;
static const uint32_t FRAME_ESP_CAN_ID_ACK = 0x282u;
static const uint32_t FRAME_ESP_CAN_ID_PING_REQUEST = 0x0E0u;
static const uint32_t FRAME_ESP_CAN_ID_FW_START = 0x0F0u;
static const uint32_t FRAME_ESP_CAN_ID_FW_END = 0x0F1u;
static const uint32_t FRAME_ESP_CAN_ID_FW_ABORT = 0x0F2u;
static const uint32_t FRAME_ESP_CAN_ID_FW_STATUS = 0x0F3u;

class FrameESP_CanAdapter
{
public:
   explicit FrameESP_CanAdapter(ICanRouter *router);
   void begin();

   /** Sends ACK with auto-incrementing sequence. */
   void sendAck(uint8_t result, uint8_t detailCode);
   void sendHeartbeat(float waterC, float blockC, float currentA, bool heaterOn, bool enabled);
   bool consumeModeChange(SystemMode &modeOut);

private:
   ICanRouter *_router;
   uint8_t _ackSeq = 0;
   SystemMode _mode = SystemMode::NORMAL;
   bool _modeChanged = false;
   uint32_t _imageSizeBytes = 0;
   uint32_t _bytesReceived = 0;
   uint32_t _imageCrc32 = 0;
   uint8_t _lastAbortReason = 0;

   static void staticHandlePingRequest(const twai_message_t *msg, void *ctx);
   static void staticHandleFwStart(const twai_message_t *msg, void *ctx);
   static void staticHandleFwEnd(const twai_message_t *msg, void *ctx);
   static void staticHandleFwAbort(const twai_message_t *msg, void *ctx);
   static void staticHandleFwStatus(const twai_message_t *msg, void *ctx);

   void handlePingRequest(const twai_message_t *msg);
   void handleFwStart(const twai_message_t *msg);
   void handleFwEnd(const twai_message_t *msg);
   void handleFwAbort(const twai_message_t *msg);
   void handleFwStatus(const twai_message_t *msg);

   void setMode(SystemMode mode);
   void sendFwStatus();
};

#endif // FRAME_ESP_CAN_ADAPTER_H
