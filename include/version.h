#ifndef VERSION_H
#define VERSION_H

#include <stdint.h>

// Software version: MAJOR.MINOR.PATCH
// Do not edit these values by hand for releases; run release_firmware.py with
// the target tag so the header, firmware build, and release asset stay aligned.
// - MAJOR: breaking protocol/behavior changes that may require Pi-side updates
// - MINOR: backward-compatible features, such as new CAN messages or telemetry
// - PATCH: bug fixes or small safe changes that keep existing behavior compatible
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 6

// Build date and time
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// Helper macros for version string
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Full version string
#define VERSION_STRING TOSTRING(VERSION_MAJOR) "." TOSTRING(VERSION_MINOR) "." TOSTRING(VERSION_PATCH)

static constexpr uint8_t FIRMWARE_VERSION_MAJOR = VERSION_MAJOR;
static constexpr uint8_t FIRMWARE_VERSION_MINOR = VERSION_MINOR;
static constexpr uint8_t FIRMWARE_VERSION_PATCH = VERSION_PATCH;
static constexpr const char *FIRMWARE_VERSION_STRING = VERSION_STRING;

#endif // VERSION_H
