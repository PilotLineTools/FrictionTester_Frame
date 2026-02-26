/**
 * CanCodec - little-endian helpers for encoding/decoding CAN payload fields.
 */

#ifndef CAN_CODEC_H
#define CAN_CODEC_H

#include <cmath>
#include <cstring>
#include <stdint.h>

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

inline void packU32LE(uint8_t *buf, uint32_t val)
{
   buf[0] = (uint8_t)(val & 0xFF);
   buf[1] = (uint8_t)((val >> 8) & 0xFF);
   buf[2] = (uint8_t)((val >> 16) & 0xFF);
   buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

inline uint32_t unpackU32LE(const uint8_t *buf)
{
   return (uint32_t)buf[0] |
          ((uint32_t)buf[1] << 8) |
          ((uint32_t)buf[2] << 16) |
          ((uint32_t)buf[3] << 24);
}

inline bool unpackFloatLE(const uint8_t *buf, float &out)
{
   uint8_t raw[sizeof(float)];
   raw[0] = buf[0];
   raw[1] = buf[1];
   raw[2] = buf[2];
   raw[3] = buf[3];
   std::memcpy(&out, raw, sizeof(float));
   return std::isfinite(out);
}

#endif // CAN_CODEC_H
