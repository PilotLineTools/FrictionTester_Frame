#ifndef SYSTEM_MODE_H
#define SYSTEM_MODE_H

#include <stdint.h>

enum class SystemMode : uint8_t
{
   NORMAL = 0,
   FW_UPDATE
};

#endif // SYSTEM_MODE_H
