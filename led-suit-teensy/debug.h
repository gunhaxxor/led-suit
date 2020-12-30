#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x); Serial.print("\t")
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(x, y) Serial.printf(x, y)
// #define DEBUG_PRINTF(x, y, z) Serial.printf(x, y, z)
// #define DEBUG_PRINTF(x, y, z, w) Serial.printf(x, y, z, w)
// #define DEBUG_PRINTF(x, y, z, w, v) Serial.printf(x, y, z, w, v)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(x, y)
// #define DEBUG_PRINTF(x, y, z)
// #define DEBUG_PRINTF(x, y, z, w)
// #define DEBUG_PRINTF(x, y, z, w, v)
#endif



#endif
