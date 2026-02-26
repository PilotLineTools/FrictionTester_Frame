/**
 * WaterBathCanAdapter - Implementation.
 * Handles 0x080 SET_WATER_BATH, sends 0x280 BATH_STATUS and 0x282 FRAME_ACK.
 */

#include "WaterBathCanAdapter.h"
#include <algorithm>

WaterBathCanAdapter::WaterBathCanAdapter(WaterBathController *controller, ICanRouter *router, FrameCanAdapter *frameCan)
   : _controller(controller),
     _router(router),
     _frameCan(frameCan)
{
}

void WaterBathCanAdapter::begin()
{
   if (_router)
      _router->on(CAN_ID_SET_WATER_BATH, &WaterBathCanAdapter::staticHandleSetWaterBath, this);
}

void WaterBathCanAdapter::tick(uint32_t now_ms)
{
   if (!_router || !_controller)
      return;
   if (now_ms - _lastStatusMs >= _statusIntervalMs)
   {
      sendBathStatus();
      _lastStatusMs = now_ms;
   }
}

void WaterBathCanAdapter::staticHandleSetWaterBath(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<WaterBathCanAdapter *>(ctx);
   if (adapter)
      adapter->handleSetWaterBath(msg);
}

void WaterBathCanAdapter::handleSetWaterBath(const twai_message_t *msg)
{
   const uint8_t *d = msg->data;
   const uint8_t dlc = msg->data_length_code;
   const uint32_t seq = ++_flowSeq;

   if (dlc != 8)
   {
      USBSerial.printf("WB FLOW[%lu] reject: bad_dlc=%u\n", (unsigned long)seq, (unsigned)dlc);
      sendAck(1, 0);
      return;
   }
   if (d[4] != 0 || d[5] != 0 || d[6] != 0 || d[7] != 0)
   {
      USBSerial.printf("WB FLOW[%lu] reject: reserved_nonzero=%02X %02X %02X %02X\n",
                       (unsigned long)seq, d[4], d[5], d[6], d[7]);
      sendAck(1, 0);
      return;
   }

   if (_mode == SystemMode::FW_UPDATE)
   {
      USBSerial.printf("WB FLOW[%lu] reject: mode=FW_UPDATE\n", (unsigned long)seq);
      sendAck(1, 0);
      return;
   }

   uint8_t heater_enable_request = d[0];
   int16_t temp_C_x100 = unpackI16LE(&d[1]);
   uint8_t circulator_speed = d[3];
   float targetC = (float)temp_C_x100 / 100.0f;

   USBSerial.printf("WB FLOW[%lu] RX SET_WATER_BATH heater_req=%u target=%.2fC circ_req=%u\n",
                    (unsigned long)seq,
                    (unsigned)heater_enable_request,
                    targetC,
                    (unsigned)circulator_speed);

   _controller->setTargetTemp(targetC);

   if (heater_enable_request)
   {
      _controller->setCirculatorTargetRpm((float)circulator_speed);
      _controller->enable();
   }
   else
   {
      _controller->setCirculatorTargetRpm(0.0f);
      _controller->disable();
   }

   USBSerial.printf("WB FLOW[%lu] after_cmd enabled=%u heater_var=%u err=%u\n",
                    (unsigned long)seq,
                    (unsigned)_controller->isEnabled(),
                    (unsigned)_controller->isHeaterOn(),
                    (unsigned)_controller->getErrorCode());

   WaterBathError err = _controller->getErrorCode();
   uint8_t detail = (err != WaterBathError::None) ? (uint8_t)err : 0;
   sendAck(0, detail);
   USBSerial.printf("WB FLOW[%lu] ack result=0 detail=%u\n", (unsigned long)seq, (unsigned)detail);
}

void WaterBathCanAdapter::sendBathStatus()
{
   twai_message_t tx = {};
   tx.identifier = CAN_ID_BATH_STATUS;
   tx.data_length_code = 8;
   tx.flags = 0;

   float waterC = _controller->getBathTempC();
   float blockC = _controller->getBlockTempC();
   float currentA = _controller->getHeaterCurrentA();
   int16_t water_x100 = (int16_t)(waterC * 100.0f);
   int16_t block_x100 = (int16_t)(blockC * 100.0f);
   int32_t current_mA = (int32_t)(currentA * 1000.0f);
   current_mA = std::max((int32_t)0, std::min((int32_t)65535, current_mA));
   uint16_t current_u16 = (uint16_t)current_mA;

   packI16LE(&tx.data[0], water_x100);
   packI16LE(&tx.data[2], block_x100);
   packU16LE(&tx.data[4], current_u16);
   tx.data[6] = _controller->isHeaterOn() ? 1 : 0;
   tx.data[7] = _controller->isEnabled() ? 1 : 0;

   if (((int8_t)tx.data[6] != _lastLoggedHeater) || ((int8_t)tx.data[7] != _lastLoggedEnabled))
   {
      USBSerial.printf("WB FLOW status heater=%u enabled=%u water=%.2f block=%.2f current=%.2fA\n",
                       (unsigned)tx.data[6],
                       (unsigned)tx.data[7],
                       waterC,
                       blockC,
                       currentA);
      _lastLoggedHeater = (int8_t)tx.data[6];
      _lastLoggedEnabled = (int8_t)tx.data[7];
   }

   if (_router)
      _router->send(&tx);
}

void WaterBathCanAdapter::sendAck(uint8_t result, uint8_t detailCode)
{
   if (_frameCan)
      _frameCan->sendAck(result, detailCode);
}
