#ifndef VERSION_H
#define VERSION_H

// Software version
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 2

// Build date and time
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// Helper macros for version string
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Full version string
#define VERSION_STRING TOSTRING(VERSION_MAJOR) "." TOSTRING(VERSION_MINOR) "." TOSTRING(VERSION_PATCH)

#endif // VERSION_H