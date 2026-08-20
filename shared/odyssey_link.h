// =====================================================================================
//  Odyssey-10 Pro -- Shared Link Protocol Definitions
//  ------------------------------------------------------------------------------------
//  This header is compiled into ALL FOUR firmware images (flight-controller,
//  beacon-node, remote-id, ground-station). It is the single source of truth for every
//  packet that crosses a wire or the air. Do not fork it per-project.
//
//  Three independent buses are defined here:
//
//    1. LORA CMD/TELEM LINK (433 MHz, SX1278, bi-directional, low rate)
//         FC -> GCS : TelemetryPacket   @ 2 Hz
//         GCS -> FC : CommandPacket     @ on-demand (PERMIT_LAND / ABORT / RTH)
//       This link is NOT the manual control link. See note below.
//
//    2. AUX BROADCAST BUS (LP-UART, 115200 8N1, FC TX -> many RX, one wire)
//         FC -> beacon-node    : AuxPositionFrame  @ 1 Hz  (fixes finding 7)
//         FC -> remote-id      : AuxPositionFrame  @ 2 Hz  (fixes finding 17)
//       Addressed, CRC-protected, transmit-only. Receivers never talk back, so the
//       bus cannot be jammed by a failed peripheral.
//
//    3. BEACON DOWNLINK (433 MHz, SX1278, TX only, duty-cycle limited)
//         beacon -> ground : BeaconPacket
//
//  IMPORTANT -- MANUAL CONTROL IS NOT ON THIS LINK.
//  Stick-rate control is carried by a dedicated ExpressLRS 2.4 GHz receiver over
//  CRSF. A 433 MHz LoRa packet at SF7/BW125 costs ~52 ms of airtime, so a 20 Hz stick
//  stream would need >100% duty cycle -- physically impossible and, at 26% duty, far
//  outside the 10% ISM limit for 433 MHz in ITU Region 1. The original design's
//  "LoRa RC link" could never have worked. See docs section 5.4.
// =====================================================================================

#ifndef ODYSSEY_LINK_H
#define ODYSSEY_LINK_H

#include <stdint.h>
#include <string.h>

// -------------------------------------------------------------------------------------
//  Protocol identity
// -------------------------------------------------------------------------------------
#define ODY_LINK_VERSION      2       // bump on any wire-format change
#define ODY_LINK_MAGIC        0xA7    // start-of-frame marker

// Frame type identifiers (the `type` field of OdyFrameHeader)
enum OdyFrameType : uint8_t {
  ODY_FRAME_TELEMETRY   = 0x10,   // FC  -> GCS   (LoRa)
  ODY_FRAME_COMMAND     = 0x20,   // GCS -> FC    (LoRa)
  ODY_FRAME_AUX_POS     = 0x30,   // FC  -> aux   (LP-UART broadcast)
  ODY_FRAME_BEACON      = 0x40,   // beacon -> ground (LoRa)
};

// Destination addresses for the AUX broadcast bus
enum OdyAuxAddr : uint8_t {
  ODY_AUX_ADDR_ALL      = 0x00,
  ODY_AUX_ADDR_BEACON   = 0x01,
  ODY_AUX_ADDR_REMOTEID = 0x02,
};

// -------------------------------------------------------------------------------------
//  Framing
//  Every frame is: [ OdyFrameHeader ][ payload (len bytes) ][ uint16 crc (LE) ]
//  The CRC covers the header (from `version` onward) and the entire payload.
// -------------------------------------------------------------------------------------
struct __attribute__((packed)) OdyFrameHeader {
  uint8_t  magic;      // ODY_LINK_MAGIC
  uint8_t  version;    // ODY_LINK_VERSION
  uint8_t  type;       // OdyFrameType
  uint8_t  dest;       // OdyAuxAddr (AUX bus only; 0 elsewhere)
  uint16_t seq;        // monotonic per-sender counter, wraps
  uint8_t  len;        // payload length in bytes
  uint8_t  reserved;   // zero; keeps the payload 4-byte aligned
};

#define ODY_FRAME_OVERHEAD  (sizeof(OdyFrameHeader) + 2u)
#define ODY_MAX_PAYLOAD     64u
#define ODY_MAX_FRAME       (ODY_MAX_PAYLOAD + ODY_FRAME_OVERHEAD)

// -------------------------------------------------------------------------------------
//  CRC-16/CCITT-FALSE  (poly 0x1021, init 0xFFFF, no reflection, no final xor)
//
//  Rationale: this is an INTEGRITY check, not authentication. It stops a corrupted or
//  mis-sized frame from being interpreted as a valid command -- the failure mode behind
//  finding 3. It does NOT stop a deliberate attacker. For flights where command
//  spoofing is a real threat, enable ODY_LINK_REQUIRE_MAC and provision a shared
//  secret; see docs section 5.5.
// -------------------------------------------------------------------------------------
static inline uint16_t odyCrc16(const uint8_t* data, uint32_t len, uint16_t crc = 0xFFFF) {
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// -------------------------------------------------------------------------------------
//  Flight state
//
//  The numeric values are WIRE VALUES -- the ground station decodes them and every
//  consumer must agree. `odyMotorsAreLive()` is the authoritative predicate for
//  "the propellers may be turning"; never open-code an ordinal comparison against
//  this enum (that mistake was finding 10).
//
//  The ordering is also the ESCALATION PRIORITY used by the flight controller's state
//  machine: a transition is only accepted if it moves to an equal or higher value,
//  which makes it impossible for a late FAILSAFE_LANDING request from the telemetry
//  core to clobber a PARACHUTE deployment raised by the flight core (finding 6).
//  DISARMED sits at the top because cutting the motors is always permitted.
// -------------------------------------------------------------------------------------
enum OdyFlightState : uint8_t {
  ODY_STATE_BOOT                = 0,
  ODY_STATE_CALIBRATING         = 1,
  ODY_STATE_PREFLIGHT_FAIL      = 2,
  ODY_STATE_PREFLIGHT_OK        = 3,
  ODY_STATE_ARMED               = 4,
  ODY_STATE_RTH_NAVIGATING      = 5,
  ODY_STATE_AWAITING_LAND_PERMIT= 6,
  ODY_STATE_FAILSAFE_LANDING    = 7,
  ODY_STATE_FREEFALL_PARACHUTE  = 8,
  ODY_STATE_DISARMED            = 9,
  ODY_STATE_COUNT
};

// True for every state in which the mixer may drive the ESCs above idle.
static inline bool odyMotorsAreLive(uint8_t s) {
  return s == ODY_STATE_ARMED
      || s == ODY_STATE_RTH_NAVIGATING
      || s == ODY_STATE_AWAITING_LAND_PERMIT
      || s == ODY_STATE_FAILSAFE_LANDING;
}

// True for every state in which the aircraft is (or may be) off the ground. This is
// the gate for free-fall / parachute detection -- it deliberately includes the
// parachute state itself so the detector's timer keeps running for the log.
static inline bool odyIsAirborneState(uint8_t s) {
  return odyMotorsAreLive(s) || s == ODY_STATE_FREEFALL_PARACHUTE;
}

static inline const char* odyStateName(uint8_t s) {
  switch (s) {
    case ODY_STATE_BOOT:                 return "BOOT";
    case ODY_STATE_CALIBRATING:          return "CALIBRATING";
    case ODY_STATE_PREFLIGHT_FAIL:       return "PREFLIGHT_FAIL";
    case ODY_STATE_PREFLIGHT_OK:         return "PREFLIGHT_OK";
    case ODY_STATE_ARMED:                return "ARMED";
    case ODY_STATE_RTH_NAVIGATING:       return "RTH_NAVIGATING";
    case ODY_STATE_AWAITING_LAND_PERMIT: return "AWAITING_LAND_PERMIT";
    case ODY_STATE_FAILSAFE_LANDING:     return "FAILSAFE_LANDING";
    case ODY_STATE_FREEFALL_PARACHUTE:   return "FREEFALL_PARACHUTE";
    case ODY_STATE_DISARMED:             return "DISARMED";
    default:                             return "UNKNOWN";
  }
}

// -------------------------------------------------------------------------------------
//  Sensor health bitfield (carried in telemetry, checked at arm time)
//
//  Every sensor that can influence a flight decision reports liveness here. A stale
//  reading is treated as a failed sensor, which is what the original firmware lacked:
//  a dead LiDAR silently latched its last distance forever (finding 15).
// -------------------------------------------------------------------------------------
enum OdySensorBit : uint16_t {
  ODY_SENS_IMU_PRIMARY   = 1u << 0,
  ODY_SENS_IMU_BACKUP    = 1u << 1,
  ODY_SENS_BARO          = 1u << 2,
  ODY_SENS_MAG           = 1u << 3,
  ODY_SENS_GNSS          = 1u << 4,
  ODY_SENS_LIDAR_FWD     = 1u << 5,
  ODY_SENS_TOF_DOWN      = 1u << 6,
  ODY_SENS_CURRENT       = 1u << 7,
  ODY_SENS_SDCARD        = 1u << 8,
  ODY_SENS_LORA          = 1u << 9,
  ODY_SENS_CRSF_RC       = 1u << 10,
  ODY_SENS_AUX_BUS       = 1u << 11,
  ODY_SENS_REMOTE_ID     = 1u << 12,
};

// Sensors that must be healthy before the aircraft is allowed to arm.
// LiDAR and downward ToF are deliberately excluded: they degrade capability
// (no obstacle bubble, no laser flare) but do not prevent a safe manual flight.
// Remote ID is included because flying without it is illegal in most jurisdictions.
#define ODY_ARM_REQUIRED_SENSORS ( ODY_SENS_IMU_PRIMARY | ODY_SENS_BARO   \
                                 | ODY_SENS_MAG         | ODY_SENS_GNSS   \
                                 | ODY_SENS_CURRENT     | ODY_SENS_CRSF_RC\
                                 | ODY_SENS_REMOTE_ID )

// -------------------------------------------------------------------------------------
//  FC -> GCS telemetry  (LoRa, 2 Hz)
// -------------------------------------------------------------------------------------
struct __attribute__((packed)) TelemetryPayload {
  uint32_t uptimeMs;
  int32_t  latitude1e7;          // degrees * 1e7
  int32_t  longitude1e7;         // degrees * 1e7
  int32_t  altitudeAglMm;        // mm above the armed launch point
  int16_t  varioCmS;             // cm/s, + is climb
  int16_t  groundSpeedCmS;       // cm/s
  int16_t  headingCentideg;      // 0..35999, tilt-compensated magnetic heading
  int16_t  bearingToHomeCentideg;
  uint32_t distanceToHomeCm;
  int16_t  rollCentideg;
  int16_t  pitchCentideg;
  uint16_t battMillivolts;       // filtered pack voltage
  int16_t  battCurrentCentiAmps; // 10 mA units
  uint16_t battConsumedMah;
  uint16_t battRemainingPercent; // 0..10000 (hundredths of a percent)
  uint16_t forwardObstacleCm;    // 0xFFFF when no valid reading
  uint16_t sensorHealth;         // OdySensorBit mask
  uint8_t  satellites;
  uint8_t  flightState;          // OdyFlightState
  uint8_t  rcLinkQuality;        // 0..100, from CRSF
  uint8_t  armFlags;             // OdyArmFlag mask -- why arming is blocked
};

// Reasons the aircraft is refusing to arm; surfaced to the pilot so a blocked arm is
// never a silent mystery in the field.
enum OdyArmFlag : uint8_t {
  ODY_ARMBLOCK_SENSORS    = 1u << 0,  // a required sensor is unhealthy
  ODY_ARMBLOCK_NO_HOME    = 1u << 1,  // GNSS home position not yet locked
  ODY_ARMBLOCK_THROTTLE   = 1u << 2,  // throttle stick is not at minimum (finding 5)
  ODY_ARMBLOCK_RC_STALE   = 1u << 3,  // no fresh CRSF frame
  ODY_ARMBLOCK_BATTERY    = 1u << 4,  // pack below the launch minimum
  ODY_ARMBLOCK_CALIB      = 1u << 5,  // calibration did not pass
  ODY_ARMBLOCK_ATTITUDE   = 1u << 6,  // aircraft is not level
};

// -------------------------------------------------------------------------------------
//  GCS -> FC command  (LoRa, on demand)
//
//  `seq` in the frame header is monotonic and the FC rejects any command whose seq is
//  not strictly greater than the last accepted one, which kills replays of a captured
//  ABORT frame. `sessionId` is randomised by the GCS at boot and echoed in telemetry,
//  so a command from a stale ground station is ignored rather than obeyed.
// -------------------------------------------------------------------------------------
enum OdyCommandId : uint8_t {
  ODY_CMD_NONE          = 0,
  ODY_CMD_PERMIT_LAND   = 1,   // pilot answers the REQ_LAND prompt
  ODY_CMD_DENY_LAND     = 2,   // pilot refuses; aircraft holds until critical
  ODY_CMD_RTH_NOW       = 3,   // operator-commanded return to home
  ODY_CMD_ABORT_TO_LAND = 4,   // operator-commanded immediate failsafe landing
  ODY_CMD_LINK_PING     = 5,   // keeps the command link measurably alive
};

struct __attribute__((packed)) CommandPayload {
  uint32_t sessionId;
  uint8_t  commandId;   // OdyCommandId
  uint8_t  argument;    // command-specific; zero when unused
  uint16_t reserved;
};

// -------------------------------------------------------------------------------------
//  FC -> aux modules  (LP-UART broadcast, 1-2 Hz)
//
//  This frame is the fix for finding 7. The original design's only FC-to-beacon
//  interface was a single GPIO latch pulse, which carries no data, so the beacon
//  transmitted latitude 0 / longitude 0 for its entire endurance. The beacon node now
//  holds a continuously refreshed copy of the aircraft's last known fix, written to
//  its RTC-backed memory so it survives the transition to 1S power.
// -------------------------------------------------------------------------------------
struct __attribute__((packed)) AuxPositionPayload {
  uint32_t uptimeMs;
  int32_t  latitude1e7;
  int32_t  longitude1e7;
  int32_t  altitudeMslMm;
  int32_t  altitudeAglMm;
  uint16_t groundSpeedCmS;
  uint16_t headingCentideg;
  uint16_t battMillivolts;
  uint8_t  satellites;
  uint8_t  fixQuality;      // 0 = none, 1 = 2D, 2 = 3D, 3 = 3D + DGPS
  uint8_t  flightState;     // OdyFlightState
  uint8_t  emergency;       // non-zero once the aircraft considers itself lost
};

// -------------------------------------------------------------------------------------
//  Beacon -> ground  (LoRa, duty-cycle limited; see beacon-node/src/main.cpp)
// -------------------------------------------------------------------------------------
struct __attribute__((packed)) BeaconPayload {
  char     tag[4];              // "BEAC"
  int32_t  latitude1e7;
  int32_t  longitude1e7;
  int32_t  altitudeMslMm;
  uint32_t secondsSinceLatch;   // how long the beacon has been running
  uint32_t fixAgeSeconds;       // how old the cached fix is -- 0 means it is current
  uint16_t beaconBattMv;        // the 1S cell, not the flight pack
  uint8_t  satellites;
  uint8_t  lastFlightState;     // OdyFlightState at the moment of the latch
};

// -------------------------------------------------------------------------------------
//  Frame encode / decode helpers
// -------------------------------------------------------------------------------------

// Serialises `payload` into `out`. Returns the total frame length, or 0 on overflow.
static inline uint32_t odyEncodeFrame(uint8_t* out, uint32_t outCap,
                                      uint8_t type, uint8_t dest, uint16_t seq,
                                      const void* payload, uint8_t payloadLen) {
  const uint32_t total = sizeof(OdyFrameHeader) + payloadLen + 2u;
  if (payloadLen > ODY_MAX_PAYLOAD || total > outCap) return 0;

  OdyFrameHeader h;
  h.magic    = ODY_LINK_MAGIC;
  h.version  = ODY_LINK_VERSION;
  h.type     = type;
  h.dest     = dest;
  h.seq      = seq;
  h.len      = payloadLen;
  h.reserved = 0;

  memcpy(out, &h, sizeof(h));
  memcpy(out + sizeof(h), payload, payloadLen);

  // CRC covers everything from `version` through the last payload byte. `magic` is
  // excluded so a resynchronising receiver can hunt for it without special-casing.
  const uint16_t crc = odyCrc16(out + 1, sizeof(h) - 1u + payloadLen);
  out[sizeof(h) + payloadLen]      = (uint8_t)(crc & 0xFF);
  out[sizeof(h) + payloadLen + 1u] = (uint8_t)(crc >> 8);
  return total;
}

// Validates a complete frame. On success writes the payload pointer/length and the
// header, and returns true. Rejects bad magic, wrong protocol version, truncated
// frames, length mismatches and CRC failures.
static inline bool odyDecodeFrame(const uint8_t* in, uint32_t inLen,
                                  OdyFrameHeader* hdrOut,
                                  const uint8_t** payloadOut, uint8_t* payloadLenOut) {
  if (inLen < ODY_FRAME_OVERHEAD) return false;
  if (in[0] != ODY_LINK_MAGIC)    return false;
  if (in[1] != ODY_LINK_VERSION)  return false;

  OdyFrameHeader h;
  memcpy(&h, in, sizeof(h));
  if (h.len > ODY_MAX_PAYLOAD) return false;

  const uint32_t total = sizeof(h) + h.len + 2u;
  if (total != inLen) return false;   // exact-length match; no trailing garbage

  const uint16_t want = (uint16_t)in[sizeof(h) + h.len]
                      | (uint16_t)((uint16_t)in[sizeof(h) + h.len + 1u] << 8);
  if (odyCrc16(in + 1, sizeof(h) - 1u + h.len) != want) return false;

  if (hdrOut)        *hdrOut        = h;
  if (payloadOut)    *payloadOut    = in + sizeof(h);
  if (payloadLenOut) *payloadLenOut = h.len;
  return true;
}

// Sequence comparison that tolerates the 16-bit wrap. Returns true when `candidate`
// is newer than `last`. Used to reject replayed and out-of-order commands.
static inline bool odySeqIsNewer(uint16_t candidate, uint16_t last) {
  return (uint16_t)(candidate - last) != 0u && (uint16_t)(candidate - last) < 0x8000u;
}

#endif // ODYSSEY_LINK_H
