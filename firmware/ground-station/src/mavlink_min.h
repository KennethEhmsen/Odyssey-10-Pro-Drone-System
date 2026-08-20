// =====================================================================================
//  Odyssey-10 Pro -- Minimal MAVLink v2 encoder
//  ------------------------------------------------------------------------------------
//  FIX FOR FINDING 9.
//
//  The original bridge wrote constant bytes where the checksum belongs:
//
//      Serial.write(0x55); Serial.write(0xAA);      // heartbeat
//      Serial.write(0x12); Serial.write(0x34);      // attitude
//
//  MAVLink requires a CRC-16/MCRF4XX computed over the header and payload, seeded with
//  a per-message CRC_EXTRA byte derived from the message's field signature. Every frame
//  the original emitted failed the receiver's CRC and was discarded, so QGroundControl
//  and Mission Planner showed nothing at all -- despite the section heading promising
//  compatibility with both.
//
//  Three further defects are fixed here:
//    * The start byte was 0xFE (MAVLink v1) although the heading claimed v2. v2 uses
//      0xFD with a 10-byte header and a 24-bit message id.
//    * The sequence byte was a constant (0 for heartbeat, 1 for attitude), so even a
//      CRC-correct link would have reported 100% packet loss. It is now a proper
//      per-link counter.
//    * Only two message types were sent. A GCS needs position, battery and GPS status
//      to be useful, all of which the telemetry frame already carries.
//
//  The CRC_EXTRA values below come from the MAVLink common dialect. For a production
//  build, prefer linking the generated headers from mavlink/c_library_v2 rather than
//  maintaining this table by hand; this file exists so the reference implementation is
//  self-contained and auditable.
// =====================================================================================

#ifndef ODY_MAVLINK_MIN_H
#define ODY_MAVLINK_MIN_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// -------------------------------------------------------------------------------------
//  Framing constants
// -------------------------------------------------------------------------------------
#define MAVLINK_V2_MAGIC        0xFD
#define MAVLINK_V2_HEADER_LEN   10
#define MAVLINK_MAX_PAYLOAD     255

// Message ids
#define MAVLINK_MSG_HEARTBEAT             0
#define MAVLINK_MSG_SYS_STATUS            1
#define MAVLINK_MSG_GPS_RAW_INT          24
#define MAVLINK_MSG_ATTITUDE             30
#define MAVLINK_MSG_GLOBAL_POSITION_INT  33
#define MAVLINK_MSG_VFR_HUD              74
#define MAVLINK_MSG_STATUSTEXT          253

// CRC_EXTRA, from the common dialect message definitions.
#define CRC_EXTRA_HEARTBEAT              50
#define CRC_EXTRA_SYS_STATUS            124
#define CRC_EXTRA_GPS_RAW_INT            24
#define CRC_EXTRA_ATTITUDE               39
#define CRC_EXTRA_GLOBAL_POSITION_INT   104
#define CRC_EXTRA_VFR_HUD                20
#define CRC_EXTRA_STATUSTEXT             83

// MAV_TYPE / MAV_AUTOPILOT / MAV_STATE
#define MAV_TYPE_QUADROTOR                2
#define MAV_AUTOPILOT_GENERIC             0
#define MAV_MODE_FLAG_SAFETY_ARMED      128
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#define MAV_STATE_UNINIT                  0
#define MAV_STATE_BOOT                    1
#define MAV_STATE_CALIBRATING             2
#define MAV_STATE_STANDBY                 3
#define MAV_STATE_ACTIVE                  4
#define MAV_STATE_CRITICAL                5
#define MAV_STATE_EMERGENCY               6

#define MAV_SEVERITY_CRITICAL             2
#define MAV_SEVERITY_WARNING              4
#define MAV_SEVERITY_INFO                 6

// MAV_SYS_STATUS_SENSOR bits we report on
#define MAV_SYS_STATUS_SENSOR_3D_GYRO          0x01
#define MAV_SYS_STATUS_SENSOR_3D_ACCEL         0x02
#define MAV_SYS_STATUS_SENSOR_3D_MAG           0x04
#define MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE 0x08
#define MAV_SYS_STATUS_SENSOR_GPS              0x20
#define MAV_SYS_STATUS_SENSOR_LASER_POSITION   0x40
#define MAV_SYS_STATUS_SENSOR_BATTERY      0x00020000

// -------------------------------------------------------------------------------------
//  CRC-16/MCRF4XX  (poly 0x1021 reflected, init 0xFFFF, no final xor)
//  This is the canonical MAVLink accumulator, byte at a time.
// -------------------------------------------------------------------------------------
static inline void mavlinkCrcAccumulate(uint8_t data, uint16_t* crc) {
  uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
  tmp ^= (uint8_t)(tmp << 4);
  *crc = (uint16_t)((*crc >> 8)
       ^ ((uint16_t)tmp << 8)
       ^ ((uint16_t)tmp << 3)
       ^ ((uint16_t)tmp >> 4));
}

// -------------------------------------------------------------------------------------
//  Frame writer
// -------------------------------------------------------------------------------------
class MavlinkWriter {
public:
  MavlinkWriter(Stream& out, uint8_t systemId, uint8_t componentId)
      : out_(out), sysId_(systemId), compId_(componentId) {}

  // Emits one complete MAVLink v2 frame with a correct CRC.
  void send(uint16_t msgId, uint8_t crcExtra,
            const uint8_t* payload, uint8_t payloadLen) {
    uint8_t frame[MAVLINK_V2_HEADER_LEN + MAVLINK_MAX_PAYLOAD + 2];

    frame[0] = MAVLINK_V2_MAGIC;
    frame[1] = payloadLen;
    frame[2] = 0;                    // incompat_flags: 0 means unsigned
    frame[3] = 0;                    // compat_flags
    frame[4] = seq_++;               // proper per-link sequence, not a constant
    frame[5] = sysId_;
    frame[6] = compId_;
    frame[7] = (uint8_t)(msgId & 0xFF);
    frame[8] = (uint8_t)((msgId >> 8) & 0xFF);
    frame[9] = (uint8_t)((msgId >> 16) & 0xFF);
    memcpy(&frame[MAVLINK_V2_HEADER_LEN], payload, payloadLen);

    // CRC runs from the length byte through the last payload byte, then the
    // message-specific CRC_EXTRA is folded in.
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 1; i < MAVLINK_V2_HEADER_LEN + payloadLen; ++i) {
      mavlinkCrcAccumulate(frame[i], &crc);
    }
    mavlinkCrcAccumulate(crcExtra, &crc);

    frame[MAVLINK_V2_HEADER_LEN + payloadLen]     = (uint8_t)(crc & 0xFF);
    frame[MAVLINK_V2_HEADER_LEN + payloadLen + 1] = (uint8_t)(crc >> 8);

    out_.write(frame, MAVLINK_V2_HEADER_LEN + payloadLen + 2);
  }

  // ---- Message builders -------------------------------------------------------------
  // Field order follows the MAVLink wire order: fields sorted by descending native
  // size, then by declaration order within each size class.

  void heartbeat(uint8_t baseMode, uint8_t systemStatus, uint32_t customMode) {
    uint8_t p[9];
    memcpy(&p[0], &customMode, 4);
    p[4] = MAV_TYPE_QUADROTOR;
    p[5] = MAV_AUTOPILOT_GENERIC;
    p[6] = baseMode;
    p[7] = systemStatus;
    p[8] = 3;                                  // MAVLink version
    send(MAVLINK_MSG_HEARTBEAT, CRC_EXTRA_HEARTBEAT, p, sizeof(p));
  }

  void sysStatus(uint32_t sensorsPresent, uint32_t sensorsEnabled, uint32_t sensorsHealth,
                 uint16_t voltageMv, int16_t currentCentiAmps, int8_t batteryPercent,
                 uint16_t dropRateComm) {
    uint8_t p[31];
    uint16_t load = 500;                       // 50.0%
    uint16_t zero16 = 0;
    memcpy(&p[0],  &sensorsPresent, 4);
    memcpy(&p[4],  &sensorsEnabled, 4);
    memcpy(&p[8],  &sensorsHealth,  4);
    memcpy(&p[12], &load,           2);
    memcpy(&p[14], &voltageMv,      2);
    memcpy(&p[16], &currentCentiAmps, 2);
    memcpy(&p[18], &dropRateComm,   2);
    memcpy(&p[20], &zero16, 2);                // errors_comm
    memcpy(&p[22], &zero16, 2);                // errors_count1
    memcpy(&p[24], &zero16, 2);                // errors_count2
    memcpy(&p[26], &zero16, 2);                // errors_count3
    memcpy(&p[28], &zero16, 2);                // errors_count4
    p[30] = (uint8_t)batteryPercent;
    send(MAVLINK_MSG_SYS_STATUS, CRC_EXTRA_SYS_STATUS, p, sizeof(p));
  }

  void attitude(uint32_t timeBootMs, float roll, float pitch, float yaw,
                float rollSpeed, float pitchSpeed, float yawSpeed) {
    uint8_t p[28];
    memcpy(&p[0],  &timeBootMs, 4);
    memcpy(&p[4],  &roll,       4);
    memcpy(&p[8],  &pitch,      4);
    memcpy(&p[12], &yaw,        4);
    memcpy(&p[16], &rollSpeed,  4);
    memcpy(&p[20], &pitchSpeed, 4);
    memcpy(&p[24], &yawSpeed,   4);
    send(MAVLINK_MSG_ATTITUDE, CRC_EXTRA_ATTITUDE, p, sizeof(p));
  }

  void globalPosition(uint32_t timeBootMs, int32_t lat1e7, int32_t lon1e7,
                      int32_t altMm, int32_t relAltMm,
                      int16_t vxCmS, int16_t vyCmS, int16_t vzCmS, uint16_t hdgCentideg) {
    uint8_t p[28];
    memcpy(&p[0],  &timeBootMs, 4);
    memcpy(&p[4],  &lat1e7,     4);
    memcpy(&p[8],  &lon1e7,     4);
    memcpy(&p[12], &altMm,      4);
    memcpy(&p[16], &relAltMm,   4);
    memcpy(&p[20], &vxCmS,      2);
    memcpy(&p[22], &vyCmS,      2);
    memcpy(&p[24], &vzCmS,      2);
    memcpy(&p[26], &hdgCentideg, 2);
    send(MAVLINK_MSG_GLOBAL_POSITION_INT, CRC_EXTRA_GLOBAL_POSITION_INT, p, sizeof(p));
  }

  void vfrHud(float airspeed, float groundspeed, float altMsl, float climbMps,
              int16_t headingDeg, uint16_t throttlePercent) {
    uint8_t p[20];
    memcpy(&p[0],  &airspeed,    4);
    memcpy(&p[4],  &groundspeed, 4);
    memcpy(&p[8],  &altMsl,      4);
    memcpy(&p[12], &climbMps,    4);
    memcpy(&p[16], &headingDeg,  2);
    memcpy(&p[18], &throttlePercent, 2);
    send(MAVLINK_MSG_VFR_HUD, CRC_EXTRA_VFR_HUD, p, sizeof(p));
  }

  void gpsRawInt(uint64_t timeUsec, int32_t lat1e7, int32_t lon1e7, int32_t altMm,
                 uint8_t fixType, uint8_t satellites, uint16_t eph) {
    uint8_t p[30];
    uint16_t invalid = 0xFFFF;
    memcpy(&p[0],  &timeUsec, 8);
    memcpy(&p[8],  &lat1e7,   4);
    memcpy(&p[12], &lon1e7,   4);
    memcpy(&p[16], &altMm,    4);
    memcpy(&p[20], &eph,      2);
    memcpy(&p[22], &invalid,  2);              // epv unknown
    memcpy(&p[24], &invalid,  2);              // vel unknown
    memcpy(&p[26], &invalid,  2);              // cog unknown
    p[28] = fixType;
    p[29] = satellites;
    send(MAVLINK_MSG_GPS_RAW_INT, CRC_EXTRA_GPS_RAW_INT, p, sizeof(p));
  }

  void statusText(uint8_t severity, const char* text) {
    uint8_t p[51];
    memset(p, 0, sizeof(p));
    p[0] = severity;
    strncpy((char*)&p[1], text, 50);
    send(MAVLINK_MSG_STATUSTEXT, CRC_EXTRA_STATUSTEXT, p, sizeof(p));
  }

private:
  Stream& out_;
  uint8_t sysId_;
  uint8_t compId_;
  uint8_t seq_ = 0;
};

#endif // ODY_MAVLINK_MIN_H
