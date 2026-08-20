// =====================================================================================
//  Odyssey-10 Pro -- Core runtime types
//  ------------------------------------------------------------------------------------
//  The central idea here is TimedValue<T>: no sensor reading is ever stored as a bare
//  number. Every sample carries the millisecond timestamp at which it was produced, and
//  consumers must ask isFresh() before using it.
//
//  This is a direct response to finding 15. The original firmware kept the forward
//  LiDAR distance in a plain uint16_t that was only ever written on a successful frame
//  and never invalidated, so a sensor failure latched the last reading for the rest of
//  the flight -- in one direction a permanent pitch clamp, in the other silently
//  disabled collision avoidance. The same pattern applied to every other sensor.
// =====================================================================================

#ifndef ODY_TYPES_H
#define ODY_TYPES_H

#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include "odyssey_link.h"

// -------------------------------------------------------------------------------------
//  A sensor reading that knows how old it is.
// -------------------------------------------------------------------------------------
template <typename T>
struct TimedValue {
  T        value{};
  uint32_t stampMs   = 0;
  bool     everValid = false;

  void set(const T& v, uint32_t nowMs) {
    value     = v;
    stampMs   = nowMs;
    everValid = true;
  }

  bool isFresh(uint32_t nowMs, uint32_t maxAgeMs) const {
    return everValid && (uint32_t)(nowMs - stampMs) <= maxAgeMs;
  }

  uint32_t ageMs(uint32_t nowMs) const {
    return everValid ? (uint32_t)(nowMs - stampMs) : 0xFFFFFFFFu;
  }

  // Returns the reading if fresh, otherwise the caller's fallback. Forces the caller
  // to state explicitly what "no data" means for their use case.
  T get(uint32_t nowMs, uint32_t maxAgeMs, const T& fallback) const {
    return isFresh(nowMs, maxAgeMs) ? value : fallback;
  }

  void invalidate() { everValid = false; stampMs = 0; }
};

// -------------------------------------------------------------------------------------
//  Vectors and attitude
// -------------------------------------------------------------------------------------
struct Vec3 {
  float x = 0, y = 0, z = 0;
  float magnitude() const { return sqrtf(x * x + y * y + z * z); }
};

struct Attitude {
  float rollDeg     = 0;
  float pitchDeg    = 0;
  float headingDeg  = 0;   // tilt-compensated magnetic heading, 0..360
};

struct GnssFix {
  double  latitude    = 0.0;
  double  longitude   = 0.0;
  float   altitudeMsl = 0.0f;
  float   groundSpeed = 0.0f;   // m/s
  float   courseDeg   = 0.0f;
  uint8_t satellites  = 0;
  uint8_t fixQuality  = 0;      // 0 none, 1 2D, 2 3D, 3 3D+DGPS
  bool    valid       = false;
};

// -------------------------------------------------------------------------------------
//  Battery state
//
//  Both a filtered voltage AND a coulomb count. Voltage alone sags under throttle,
//  which is why the original design's instantaneous comparison would have produced
//  spurious mode changes on every punch-out; the INA226 integration gives a load-
//  independent measure of what is actually left in the pack.
// -------------------------------------------------------------------------------------
struct BatteryState {
  float    packVolts        = 0.0f;   // filtered
  float    packVoltsRaw     = 0.0f;   // unfiltered, for the log
  float    currentAmps      = 0.0f;
  float    consumedMah      = 0.0f;
  float    remainingPercent = 100.0f;
  bool     currentSensorOk  = false;
};

// -------------------------------------------------------------------------------------
//  Perception
// -------------------------------------------------------------------------------------
struct Perception {
  TimedValue<uint16_t> forwardObstacleCm;   // TFmini-S
  TimedValue<float>    downwardAglM;        // VL53L1X
  TimedValue<float>    baroAglM;            // BMP280, relative to the armed reference
  float                varioMps = 0.0f;
};

// -------------------------------------------------------------------------------------
//  Aggregate vehicle snapshot shared between the two cores.
//  All access goes through the spinlock in main.cpp -- never read a field directly.
// -------------------------------------------------------------------------------------
struct VehicleSnapshot {
  Attitude     attitude;
  Vec3         gyroDps;
  Vec3         accelMps2;
  GnssFix      gnss;
  BatteryState battery;
  Perception   perception;

  float        distanceToHomeM   = 0.0f;
  float        bearingToHomeDeg  = 0.0f;
  uint16_t     sensorHealth      = 0;
  uint8_t      armBlockFlags     = 0;
  uint8_t      rcLinkQuality     = 0;
  bool         homeLocked        = false;
};

// -------------------------------------------------------------------------------------
//  Pilot input, normalised. Populated from CRSF, never from the LoRa link.
// -------------------------------------------------------------------------------------
struct PilotInput {
  uint16_t throttleUs = 1000;   // 1000..2000
  float    rollNorm   = 0.0f;   // -1..+1
  float    pitchNorm  = 0.0f;   // -1..+1, positive = nose up / pull back
  float    yawNorm    = 0.0f;   // -1..+1
  bool     armSwitch  = false;
  bool     rthSwitch  = false;
  uint8_t  linkQuality = 0;
  uint32_t stampMs    = 0;

  bool isFresh(uint32_t nowMs) const {
    return stampMs != 0 && (uint32_t)(nowMs - stampMs) <= CRSF_TIMEOUT_MS;
  }
};

// -------------------------------------------------------------------------------------
//  BlackBox record. Packed and versioned so tools/blackbox_decode.py can read it
//  without guessing. Motor fields hold the ACTUAL post-clamp outputs, not the raw
//  mixer sums -- otherwise saturation events are invisible in the log, which is
//  exactly when you most want to see them.
// -------------------------------------------------------------------------------------
#define BLACKBOX_MAGIC    0x4F445931u   // "ODY1"
#define BLACKBOX_VERSION   2

struct __attribute__((packed)) BlackBoxHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t recordBytes;
  uint32_t logRateHz;
  uint32_t bootTimeMs;
  char     airframe[24];
};

struct __attribute__((packed)) BlackBoxRecord {
  uint32_t timestampMs;
  int16_t  gyroX, gyroY, gyroZ;        // 0.1 deg/s
  int16_t  accelX, accelY, accelZ;     // 0.01 m/s^2
  int16_t  rollCentideg, pitchCentideg;
  int16_t  headingCentideg;
  uint16_t m1Pwm, m2Pwm, m3Pwm, m4Pwm; // post-clamp, as written to the ESCs
  uint16_t battMillivolts;
  int16_t  currentCentiAmps;
  uint16_t consumedMah;
  int16_t  baroAglCm;
  int16_t  tofAglCm;                   // -1 when the ToF is stale
  int16_t  varioCmS;
  uint16_t forwardObstacleCm;
  uint16_t sensorHealth;
  uint8_t  flightState;
  uint8_t  mixerSaturationPct;         // how much authority the mixer had to give up
};

#endif // ODY_TYPES_H
