#include "FrameESP_CanAdapter.h"
#include <Arduino.h>
#include <algorithm>
#include "driver/twai.h"
#include "esp_system.h"
#include "esp_rom_crc.h"

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
   _router->on(FRAME_ESP_CAN_ID_FW_DATA, &FrameESP_CanAdapter::staticHandleFwData, this);
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

void FrameESP_CanAdapter::sendHeartbeat(float waterC, float blockC, float currentA, bool heaterOn, bool enabled)
{
   if (!_router)
      return;

   twai_message_t tx = {};
   tx.identifier = FRAME_ESP_CAN_ID_HEARTBEAT;
   tx.data_length_code = 8;
   tx.flags = 0;

   // Mirror BATH_STATUS payload packing:
   // [0..1]=waterC*100 (i16 LE), [2..3]=blockC*100 (i16 LE),
   // [4..5]=current_mA (u16 LE, clamped), [6]=heaterOn, [7]=enabled.
   int16_t water_x100 = (int16_t)(waterC * 100.0f);
   int16_t block_x100 = (int16_t)(blockC * 100.0f);
   int32_t current_mA = (int32_t)(currentA * 1000.0f);
   current_mA = std::max((int32_t)0, std::min((int32_t)65535, current_mA));
   uint16_t current_u16 = (uint16_t)current_mA;

   packI16LE(&tx.data[0], water_x100);
   packI16LE(&tx.data[2], block_x100);
   packU16LE(&tx.data[4], current_u16);
   tx.data[6] = heaterOn ? 1 : 0;
   tx.data[7] = enabled ? 1 : 0;

   _router->send(&tx);
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

void FrameESP_CanAdapter::staticHandleFwData(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<FrameESP_CanAdapter *>(ctx);
   if (adapter)
      adapter->handleFwData(msg);
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
      sendFwStatus(0xFF, 11, 0);
      return;
   }

   _imageSizeBytes = unpackU32LE(&msg->data[0]);
   _expectedImageCrc32 = unpackU32LE(&msg->data[4]);
   _bytesReceived = 0;
   _imageCrc32 = 0;
   _expectedTotalChunks = 0;
   _lastChunkSeq = 0;
   _lastAbortReason = 0;

   const esp_partition_t *nextPartition = esp_ota_get_next_update_partition(nullptr);
   if (!nextPartition)
   {
      USBSerial.println("FRAME: FW_START no OTA partition available");
      sendAck(1, 20);
      sendFwStatus(0xFF, 20, 0);
      return;
   }

   if (_otaInProgress)
   {
      esp_ota_abort(_otaHandle);
      _otaInProgress = false;
      _otaHandle = 0;
      _otaPartition = nullptr;
   }

   esp_err_t otaErr = esp_ota_begin(nextPartition, _imageSizeBytes, &_otaHandle);
   if (otaErr != ESP_OK)
   {
      USBSerial.printf("FRAME: FW_START ota_begin failed err=%ld\n", (long)otaErr);
      sendAck(1, 21);
      sendFwStatus(0xFF, 21, 0);
      return;
   }
   _otaPartition = nextPartition;
   _otaInProgress = true;

   setMode(SystemMode::FW_UPDATE);
   USBSerial.printf("FRAME: FW_START image_size=%lu expected_crc=0x%08lX\n",
                    (unsigned long)_imageSizeBytes,
                    (unsigned long)_expectedImageCrc32);
   sendAck(0, 0);
   sendFwStatus(1, 0, 0);
}

void FrameESP_CanAdapter::handleFwEnd(const twai_message_t *msg)
{
   if (_mode != SystemMode::FW_UPDATE || !_otaInProgress || !_otaPartition)
   {
      USBSerial.printf("FRAME: FW_END ignored (not in FW_UPDATE) mode=%u ota=%u\n",
                       (unsigned)_mode,
                       _otaInProgress ? 1u : 0u);
      sendAck(1, 22);
      sendFwStatus(0xFF, 22, 0);
      return;
   }

   if (msg->data_length_code < 5 || msg->data_length_code > 8)
   {
      sendAck(1, 12);
      sendFwStatus(0xFF, 12, 0);
      return;
   }

   const uint8_t lastSeq = msg->data[0];
   const uint32_t totalChunks = unpackU32LE(&msg->data[1]);
   _expectedTotalChunks = totalChunks;
   const bool byteCountOk = (_bytesReceived == _imageSizeBytes);
   const bool seqOk = (totalChunks == 0) ? false : ((lastSeq & 0xFFu) == ((totalChunks - 1u) & 0xFFu));

   if (!byteCountOk || !seqOk)
   {
      USBSerial.printf("FRAME: FW_END invalid totals bytes=%lu/%lu total_chunks=%lu last_seq=%u\n",
                       (unsigned long)_bytesReceived,
                       (unsigned long)_imageSizeBytes,
                       (unsigned long)totalChunks,
                       (unsigned)lastSeq);
      sendAck(1, 13);
      sendFwStatus(0xFF, 13, lastSeq);
      return;
   }

   if (_imageCrc32 != _expectedImageCrc32)
   {
      USBSerial.printf("FRAME: FW_END crc mismatch calc=0x%08lX expected=0x%08lX\n",
                       (unsigned long)_imageCrc32,
                       (unsigned long)_expectedImageCrc32);
      esp_ota_abort(_otaHandle);
      _otaInProgress = false;
      _otaHandle = 0;
      _otaPartition = nullptr;
      sendAck(1, 23);
      sendFwStatus(0xFF, 23, lastSeq);
      return;
   }

   esp_err_t otaErr = esp_ota_end(_otaHandle);
   if (otaErr != ESP_OK)
   {
      USBSerial.printf("FRAME: FW_END ota_end failed err=%ld\n", (long)otaErr);
      _otaInProgress = false;
      _otaHandle = 0;
      _otaPartition = nullptr;
      sendAck(1, 24);
      sendFwStatus(0xFF, 24, lastSeq);
      return;
   }

   otaErr = esp_ota_set_boot_partition(_otaPartition);
   if (otaErr != ESP_OK)
   {
      USBSerial.printf("FRAME: FW_END set_boot_partition failed err=%ld\n", (long)otaErr);
      _otaInProgress = false;
      _otaHandle = 0;
      _otaPartition = nullptr;
      sendAck(1, 25);
      sendFwStatus(0xFF, 25, lastSeq);
      return;
   }

   _otaInProgress = false;
   _otaHandle = 0;
   _otaPartition = nullptr;

   setMode(SystemMode::NORMAL);
   USBSerial.printf("FRAME: FW_END complete chunks=%lu bytes=%lu\n",
                    (unsigned long)totalChunks,
                    (unsigned long)_bytesReceived);
   sendAck(0, 0);
   sendFwStatus(2, 0, lastSeq);
   // Do not force immediate reboot here; host can request reboot after transfer.
   // This avoids reboot loops when FW_END traffic is replayed/retried on the bus.
   delay(100);
}

void FrameESP_CanAdapter::handleFwAbort(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
      return;
   _lastAbortReason = (msg->data_length_code >= 1) ? msg->data[0] : 0;
   _bytesReceived = 0;
   if (_otaInProgress)
   {
      esp_ota_abort(_otaHandle);
      _otaInProgress = false;
      _otaHandle = 0;
      _otaPartition = nullptr;
   }
   setMode(SystemMode::NORMAL);
   USBSerial.printf("FRAME: FW_ABORT reason=%u\n", (unsigned)_lastAbortReason);
   sendFwStatus(0xFF, _lastAbortReason, 0);
}

void FrameESP_CanAdapter::handleFwStatus(const twai_message_t *msg)
{
   if (msg->data_length_code != 8)
      return;
   const uint8_t state = (_mode == SystemMode::FW_UPDATE) ? 1 : 0;
   sendFwStatus(state, 0, _lastChunkSeq);
}


void FrameESP_CanAdapter::handleFwData(const twai_message_t *msg)
{
   if (_mode != SystemMode::FW_UPDATE)
   {
      USBSerial.printf("CAN frame reject: id=0x%03lX not in FW_UPDATE mode\n", (unsigned long)msg->identifier);
      sendAck(1, 2);
      sendFwStatus(0xFF, 2, 0);
      return;
   }

   if (msg->data_length_code < 2 || msg->data_length_code > 8)
   {
      USBSerial.printf("CAN frame reject: id=0x%03lX invalid DLC=%u\n", (unsigned long)msg->identifier, (unsigned)msg->data_length_code);
      sendAck(1, 3);
      sendFwStatus(0xFF, 3, 0);
      return;
   }

   const uint8_t seq = msg->data[0];
   const uint8_t chunkLen = msg->data[1];
   const uint8_t payloadBytes = (uint8_t)(msg->data_length_code - 2);
   if (chunkLen == 0 || chunkLen > 6 || chunkLen > payloadBytes)
   {
      USBSerial.printf("FRAME: FW_DATA invalid len seq=%u chunk_len=%u payload=%u\n",
                       (unsigned)seq,
                       (unsigned)chunkLen,
                       (unsigned)payloadBytes);
      sendAck(1, 4);
      sendFwStatus(0xFF, 4, seq);
      return;
   }

   if ((_bytesReceived + chunkLen) > _imageSizeBytes)
   {
      USBSerial.printf("FRAME: FW_DATA overflow seq=%u chunk_len=%u total=%lu size=%lu\n",
                       (unsigned)seq,
                       (unsigned)chunkLen,
                       (unsigned long)(_bytesReceived + chunkLen),
                       (unsigned long)_imageSizeBytes);
      sendAck(1, 5);
      sendFwStatus(0xFF, 5, seq);
      return;
   }

   _bytesReceived += chunkLen;
   _imageCrc32 = esp_rom_crc32_le(_imageCrc32, &msg->data[2], chunkLen);
   _lastChunkSeq = seq;

   if (!_otaInProgress)
   {
      sendAck(1, 26);
      sendFwStatus(0xFF, 26, seq);
      return;
   }

   esp_err_t otaErr = esp_ota_write(_otaHandle, &msg->data[2], chunkLen);
   if (otaErr != ESP_OK)
   {
      USBSerial.printf("FRAME: FW_DATA ota_write failed seq=%u err=%ld\n",
                       (unsigned)seq,
                       (long)otaErr);
      esp_ota_abort(_otaHandle);
      _otaInProgress = false;
      _otaHandle = 0;
      _otaPartition = nullptr;
      sendAck(1, 27);
      sendFwStatus(0xFF, 27, seq);
      return;
   }

   USBSerial.printf("FRAME: FW_DATA seq=%u len=%u total %lu/%lu\n",
                    (unsigned)seq,
                    (unsigned)chunkLen,
                    (unsigned long)_bytesReceived,
                    (unsigned long)_imageSizeBytes);
   sendAck(0, 0);
   sendFwStatus(1, 0, seq);
}


void FrameESP_CanAdapter::sendFwStatus(uint8_t state, uint8_t detail, uint8_t seqEcho)
{
   if (!_router)
      return;

   twai_message_t tx = {};
   tx.identifier = FRAME_ESP_CAN_ID_FW_STATUS;
   tx.data_length_code = 8;
   tx.flags = 0;
   tx.data[0] = state;
   tx.data[1] = detail;
   tx.data[2] = seqEcho;
   tx.data[3] = 0;
   tx.data[4] = (_mode == SystemMode::FW_UPDATE) ? 1 : 0;
   tx.data[5] = _lastAbortReason;
   tx.data[6] = 0;
   tx.data[7] = 0;
   _router->send(&tx);
}
