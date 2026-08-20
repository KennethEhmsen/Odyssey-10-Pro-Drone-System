// =====================================================================================
//  Minimal Arduino shim for host-side unit testing.
//
//  The safety-critical algorithms in this project -- the desaturating mixer, the link
//  framing and CRC, the escalation-ordered state machine, the notch filter -- are pure
//  computation. They should not need an ESP32 to verify, and "it compiles for the
//  target" is not evidence that they are correct.
//
//  This header provides just enough of the Arduino and FreeRTOS surface for those
//  headers to compile and run under g++ on a desktop. It is a TEST FIXTURE. Nothing in
//  firmware/ includes it.
// =====================================================================================

#ifndef ODY_ARDUINO_SHIM_H
#define ODY_ARDUINO_SHIM_H

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <algorithm>

// ---- Arduino core --------------------------------------------------------------------
using std::min;
using std::max;

template <typename T, typename L, typename H>
static inline T constrain(T v, L lo, H hi) {
  return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v);
}

static inline long lroundf_shim(float f) { return std::lroundf(f); }
#ifndef lroundf
using std::lroundf;
#endif

// Test-controllable clock. Tests drive time explicitly rather than sleeping.
extern uint32_t g_millis;
static inline uint32_t millis() { return g_millis; }
static inline uint32_t micros() { return g_millis * 1000u; }
static inline void delay(uint32_t ms) { g_millis += ms; }

#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif
// newlib on ESP-IDF defines M_PI in <math.h>; MSYS2 g++ in strict -std=c++17 mode
// does not, so define it here rather than changing the firmware to suit the test.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f

#define HIGH 1
#define LOW  0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3

static inline void pinMode(int, int) {}
static inline void digitalWrite(int, int) {}
static inline int  digitalRead(int) { return LOW; }

// ---- FreeRTOS / ESP-IDF ----------------------------------------------------------------
// The state machine uses a portMUX spinlock. On the host a single-threaded no-op is
// sufficient: the tests exercise the escalation ORDERING, which is the property that
// makes the race harmless, not the locking primitive itself.
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
static inline void portENTER_CRITICAL(portMUX_TYPE*) {}
static inline void portEXIT_CRITICAL(portMUX_TYPE*) {}
static inline void portENTER_CRITICAL(const portMUX_TYPE*) {}
static inline void portEXIT_CRITICAL(const portMUX_TYPE*) {}

#endif // ODY_ARDUINO_SHIM_H
