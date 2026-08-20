// Host-test stand-in for the Arduino core header. See arduino_shim.h.
// Present only so firmware headers that `#include <Arduino.h>` compile under g++.
#ifndef ODY_HOST_ARDUINO_H
#define ODY_HOST_ARDUINO_H
#include "arduino_shim.h"
#endif
