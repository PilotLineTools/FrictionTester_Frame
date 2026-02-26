#ifndef AXIS_ID_H
#define AXIS_ID_H

#include <Arduino.h>
#include <stdint.h>

enum class AxisId : int8_t
{
   Carriage = 0,
   Invalid = -1
};

inline AxisId parseAxisId(const String &s)
{
   if (s == "CARRIAGE" || s == "X")
      return AxisId::Carriage;
   return AxisId::Invalid;
}

inline const char *axisToString(AxisId id)
{
   switch (id)
   {
   case AxisId::Carriage:
      return "CARRIAGE";
   default:
      return "INVALID";
   }
}

inline bool isValidAxisId(AxisId id)
{
   return id == AxisId::Carriage;
}

inline uint8_t axisToIndex(AxisId id)
{
   return static_cast<uint8_t>(id);
}

#endif // AXIS_ID_H
