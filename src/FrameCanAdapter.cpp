#include "FrameCanAdapter.h"
#include <Arduino.h>
#include "driver/twai.h"

FrameCanAdapter::FrameCanAdapter(ICanRouter *router)
   : _router(router)
{
}

void FrameCanAdapter::begin()
{
   if (!_router)
      return;
   _router->on(CAN_ID_PING_REQUEST, &FrameCanAdapter::staticHandlePingRequest, this);
   _router->on(CAN_ID_FW_START, &FrameCanAdapter::staticHandleFwStart, this);
   _router->on(CAN_ID_FW_END, &FrameCanAdapter::staticHandleFwEnd, this);
   _router->on(CAN_ID_FW_ABORT, &FrameCanAdapter::staticHandleFwAbort, this);
   _router->on(CAN_ID_FW_STATUS, &FrameCanAdapter::staticHandleFwStatus, this);
}

void FrameCanAdapter::sendAck(uint8_t result, uint8_t detailCode)
{
   if (!_router)
      return;

   twai_message_t tx = {};
   tx.identifier = CAN_ID_FRAME_ACK;
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

bool FrameCanAdapter::consumeModeChange(SystemMode &modeOut)
{
   if (!_modeChanged)
      return false;
   modeOut = _mode;
   _modeChanged = false;
   return true;
}

void FrameCanAdapter::staticHandlePingRequest(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameCanAdapter *>(ctx);
   if (adapter)
      adapter->handlePingRequest(msg);
}

void FrameCanAdapter::staticHandleFwStart(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameCanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwStart(msg);
}

void FrameCanAdapter::staticHandleFwEnd(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameCanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwEnd(msg);
}

void FrameCanAdapter::staticHandleFwAbort(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameCanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwAbort(msg);
}

void FrameCanAdapter::staticHandleFwStatus(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameCanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwStatus(msg);
}

void FrameCanAdapter::setMode(SystemMode mode)
{
   if (_mode == mode)
      return;
   _mode = mode;
   _modeChanged = true;
}

void FrameCanAdapter::handlePingRequest(const twai_message_t *msg)
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

void FrameCanAdapter::handleFwStart(const twai_message_t *msg)
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

void FrameCanAdapter::handleFwEnd(const twai_message_t *msg)
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

void FrameCanAdapter::handleFwAbort(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
      return;
   _lastAbortReason = (msg->data_length_code >= 1) ? msg->data[0] : 0;
   _bytesReceived = 0;
   setMode(SystemMode::NORMAL);
   USBSerial.printf("FRAME: FW_ABORT reason=%u\n", (unsigned)_lastAbortReason);
   sendFwStatus();
}

void FrameCanAdapter::handleFwStatus(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
      return;
   sendFwStatus();
}

void FrameCanAdapter::sendFwStatus()
{
   if (!_router)
      return;

   twai_message_t tx = {};
   tx.identifier = CAN_ID_FW_STATUS;
   tx.data_length_code = 8;
   tx.flags = 0;
   packU32LE(&tx.data[0], _bytesReceived);
   tx.data[4] = 0;
   tx.data[5] = 0;
   tx.data[6] = 0;
   tx.data[7] = 0;
   _router->send(&tx);
}
