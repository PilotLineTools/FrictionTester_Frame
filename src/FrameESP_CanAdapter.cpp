#include "FrameESP_CanAdapter.h"
#include <Arduino.h>
#include "driver/twai.h"

FrameESP_CanAdapter::FrameESP_CanAdapter(ICanRouter *router)
   : _router(router)
{
}

void FrameESP_CanAdapter::begin()
{
   if (!_router)
      return;
   _router->on(FRAME_ESP_CAN_ID_PING_REQUEST, &FrameESP_CanAdapter::staticHandlePingRequest, this);
   _router->on(FRAME_ESP_CAN_ID_FW_START, &FrameESP_CanAdapter::staticHandleFwStart, this);
   _router->on(FRAME_ESP_CAN_ID_FW_END, &FrameESP_CanAdapter::staticHandleFwEnd, this);
   _router->on(FRAME_ESP_CAN_ID_FW_ABORT, &FrameESP_CanAdapter::staticHandleFwAbort, this);
   _router->on(FRAME_ESP_CAN_ID_FW_STATUS, &FrameESP_CanAdapter::staticHandleFwStatus, this);
}

void FrameESP_CanAdapter::sendAck(uint8_t result, uint8_t detailCode)
{
   if (!_router)
      return;

   twai_message_t tx = {};
   tx.identifier = FRAME_ESP_CAN_ID_ACK;
   tx.data_length_code = 8;
   tx.flags = 0;
   tx.data[0] = _ackSeq;
   tx.data[1] = result;
   tx.data[2] = detailCode;
   tx.data[3] = 0;
   tx.data[4] = 0;
   tx.data[5] = 0;
   tx.data[6] = 0;
   tx.data[7] = 0;

   _router->send(&tx);
   _ackSeq++;
}

bool FrameESP_CanAdapter::consumeModeChange(SystemMode &modeOut)
{
   if (!_modeChanged)
      return false;
   modeOut = _mode;
   _modeChanged = false;
   return true;
}

void FrameESP_CanAdapter::staticHandlePingRequest(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameESP_CanAdapter *>(ctx);
   if (adapter)
      adapter->handlePingRequest(msg);
}

void FrameESP_CanAdapter::staticHandleFwStart(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameESP_CanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwStart(msg);
}

void FrameESP_CanAdapter::staticHandleFwEnd(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameESP_CanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwEnd(msg);
}

void FrameESP_CanAdapter::staticHandleFwAbort(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameESP_CanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwAbort(msg);
}

void FrameESP_CanAdapter::staticHandleFwStatus(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameESP_CanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwStatus(msg);
}

void FrameESP_CanAdapter::setMode(SystemMode mode)
{
   if (_mode == mode)
      return;
   _mode = mode;
   _modeChanged = true;
}

void FrameESP_CanAdapter::handlePingRequest(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
   {
      sendAck(1, 10);
      return;
   }

   uint8_t seq = msg->data[0];
   USBSerial.printf("FRAME: PING_REQUEST seq=%u\n", (unsigned)seq);
   sendAck(0, 0);
}

void FrameESP_CanAdapter::handleFwStart(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
   {
      sendAck(1, 11);
      return;
   }

   _imageSizeBytes = unpackU32LE(&msg->data[0]);
   _bytesReceived = 0;
   _imageCrc32 = 0;
   _lastAbortReason = 0;
   setMode(SystemMode::FW_UPDATE);
   USBSerial.printf("FRAME: FW_START image_size=%lu\n", (unsigned long)_imageSizeBytes);
   sendAck(0, 0);
   sendFwStatus();
}

void FrameESP_CanAdapter::handleFwEnd(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
   {
      sendAck(1, 12);
      return;
   }

   _imageCrc32 = unpackU32LE(&msg->data[0]);
   _bytesReceived = _imageSizeBytes;
   setMode(SystemMode::NORMAL);
   USBSerial.printf("FRAME: FW_END crc32=0x%08lX\n", (unsigned long)_imageCrc32);
   sendAck(0, 0);
   sendFwStatus();
}

void FrameESP_CanAdapter::handleFwAbort(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
      return;
   _lastAbortReason = (msg->data_length_code >= 1) ? msg->data[0] : 0;
   _bytesReceived = 0;
   setMode(SystemMode::NORMAL);
   USBSerial.printf("FRAME: FW_ABORT reason=%u\n", (unsigned)_lastAbortReason);
   sendFwStatus();
}

void FrameESP_CanAdapter::handleFwStatus(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
      return;
   sendFwStatus();
}

void FrameESP_CanAdapter::sendFwStatus()
{
   if (!_router)
      return;

   twai_message_t tx = {};
   tx.identifier = FRAME_ESP_CAN_ID_FW_STATUS;
   tx.data_length_code = 8;
   tx.flags = 0;
   packU32LE(&tx.data[0], _bytesReceived);
   tx.data[4] = 0;
   tx.data[5] = 0;
   tx.data[6] = 0;
   tx.data[7] = 0;
   _router->send(&tx);
}
