/**
 * WaterBathCanAdapter - Implementation.
 * Handles 0x080 SET_WATER_TEMP, sends 0x280 BATH_STATUS and 0x282 FRAME_ACK.
 */

#include "WaterBathCanAdapter.h"
#include <algorithm>

WaterBathCanAdapter::WaterBathCanAdapter(WaterBathController *controller, ICanRouter *router)
   : _controller(controller),
     _router(router)
{
}

void WaterBathCanAdapter::begin()
{
   if (_router)
      _router->on(CAN_ID_SET_WATER_TEMP, &WaterBathCanAdapter::staticHandleSetWaterTemp, this);
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

void WaterBathCanAdapter::staticHandleSetWaterTemp(const twai_message_t *msg, void *ctx)
{
   auto *adapter = static_cast<WaterBathCanAdapter *>(ctx);
   if (adapter)
      adapter->handleSetWaterTemp(msg);
}

void WaterBathCanAdapter::handleSetWaterTemp(const twai_message_t *msg)
{
   const uint8_t *d = msg->data;
   const uint8_t dlc = msg->data_length_code;

   if (dlc < 8)
   {
      sendAck(1, 0);
      return;
   }
   if (d[4] != 0 || d[5] != 0 || d[6] != 0 || d[7] != 0)
   {
      sendAck(1, 0);
      return;
   }

   if (_mode == SystemMode::FW_UPDATE)
   {
      sendAck(1, 0);
      return;
   }

   uint8_t heater_enable = d[0];
   int16_t temp_C_x100 = unpackI16LE(&d[1]);
   uint8_t circulator_speed = d[3];

   _controller->setTargetTemp((float)temp_C_x100 / 100.0f);

   if (heater_enable)
   {
      _controller->enable();
      if (_setCirculatorRpm)
         _setCirculatorRpm((float)circulator_speed);
   }
   else
   {
      _controller->disable();
      if (_setCirculatorRpm)
         _setCirculatorRpm(0.0f);
   }

   WaterBathError err = _controller->getErrorCode();
   uint8_t detail = (err != WaterBathError::None) ? (uint8_t)err : 0;
   sendAck(0, detail);
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

   if (_router)
      _router->send(&tx);
}

void WaterBathCanAdapter::sendAck(uint8_t result, uint8_t detailCode)
{
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

   if (_router)
      _router->send(&tx);

   _ackSeq++;
}
