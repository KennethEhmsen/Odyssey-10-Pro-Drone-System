// =====================================================================================
//  Odyssey-10 Pro -- Ground Station Bridge  (ESP32 + SX1278)
//  ------------------------------------------------------------------------------------
//  Two jobs:
//
//    1. RECEIVE telemetry from the aircraft over 433 MHz LoRa and republish it as
//       valid MAVLink v2 on USB serial, so QGroundControl or Mission Planner sees a
//       real vehicle.  (FINDING 9, FINDING 10)
//
//    2. TRANSMIT operator commands back to the aircraft.  (FINDING 3)
//
//  The second job did not exist at all in the original. Section 10.3's loop() only
//  called LoRa.parsePacket() and wrote to serial -- it never built or sent a command,
//  which meant the PERMIT_LAND branch that section 5's whole failsafe flowchart hinges
//  on was unreachable code. The pilot had no way to answer the aircraft's request.
//
//  Commands are raised by three physical buttons. A button is the correct interface
//  for a safety decision: it cannot be triggered by a GCS software glitch, and the
//  operator's hand is on it. Each command carries the session id, a monotonic sequence
//  number and a CRC, so a corrupted or replayed frame cannot be mistaken for a real
//  ABORT (see shared/odyssey_link.h).
// =====================================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <esp_random.h>

#include "odyssey_link.h"
#include "mavlink_min.h"

// -------------------------------------------------------------------------------------
//  Pin map
// -------------------------------------------------------------------------------------
#define PIN_LORA_SCK        18
#define PIN_LORA_MISO       19
#define PIN_LORA_MOSI       23
#define PIN_LORA_NSS        5
#define PIN_LORA_RST        14
#define PIN_LORA_DIO0       2

#define PIN_BTN_PERMIT      25      // grant the aircraft's landing request
#define PIN_BTN_RTH         26      // command return to home
#define PIN_BTN_ABORT       27      // command an immediate landing where it is
#define PIN_LED_LINK        13

#define BUTTON_DEBOUNCE_MS  40u
#define BUTTON_HOLD_MS      600u    // ABORT and RTH need a deliberate hold

#define MAVLINK_SYS_ID      1
#define MAVLINK_COMP_ID     1
#define HEARTBEAT_PERIOD_MS 1000u
#define LINK_TIMEOUT_MS     5000u

// -------------------------------------------------------------------------------------
//  State
// -------------------------------------------------------------------------------------
static MavlinkWriter mav(Serial, MAVLINK_SYS_ID, MAVLINK_COMP_ID);

static TelemetryPayload latest;
static bool     haveTelemetry   = false;
static uint32_t lastTelemetryMs = 0;
static uint32_t lastHeartbeatMs = 0;
static uint32_t telemetryCount  = 0;
static uint32_t rejectedCount   = 0;
static uint16_t lastRxSeq       = 0;
static bool     haveRxSeq       = false;

static uint32_t sessionId = 0;      // randomised at boot; the aircraft binds to it
static uint16_t txSeq     = 0;

static uint8_t  rxBuf[ODY_MAX_FRAME];

// =====================================================================================
//  MAVLink state mapping  --  FIX FOR FINDING 10
//
//  The original wrote:
//
//      bool armed = (tIn.flightState == 3 || tIn.flightState == 4);
//
//  Those two ordinals were ARMED and RTH_NAVIGATING. It omitted AWAITING_LAND_PERMIT
//  and FAILSAFE_LANDING -- states in which the aircraft is airborne with all four
//  propellers turning. The operator saw "disarmed" at exactly the moment section 5
//  expects them to approve a landing, and might walk up to a live aircraft.
//
//  The predicate now lives in shared/odyssey_link.h as odyMotorsAreLive(), so the
//  aircraft and the ground station cannot drift apart, and no caller open-codes an
//  ordinal comparison ever again.
// =====================================================================================
static uint8_t mavBaseMode(uint8_t flightState) {
  uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  if (odyMotorsAreLive(flightState)) mode |= MAV_MODE_FLAG_SAFETY_ARMED;
  return mode;
}

static uint8_t mavSystemStatus(uint8_t flightState) {
  switch (flightState) {
    case ODY_STATE_BOOT:                 return MAV_STATE_BOOT;
    case ODY_STATE_CALIBRATING:          return MAV_STATE_CALIBRATING;
    case ODY_STATE_PREFLIGHT_FAIL:       return MAV_STATE_CRITICAL;
    case ODY_STATE_PREFLIGHT_OK:
    case ODY_STATE_DISARMED:             return MAV_STATE_STANDBY;
    case ODY_STATE_ARMED:                return MAV_STATE_ACTIVE;
    case ODY_STATE_RTH_NAVIGATING:
    case ODY_STATE_AWAITING_LAND_PERMIT: return MAV_STATE_CRITICAL;
    case ODY_STATE_FAILSAFE_LANDING:
    case ODY_STATE_FREEFALL_PARACHUTE:   return MAV_STATE_EMERGENCY;
    default:                             return MAV_STATE_UNINIT;
  }
}

// Translates the aircraft's sensor health bits into the MAVLink sensor bitmask so the
// GCS sensor panel reflects reality instead of showing everything green.
static uint32_t mavSensorMask(uint16_t health) {
  uint32_t m = 0;
  if (health & ODY_SENS_IMU_PRIMARY) m |= MAV_SYS_STATUS_SENSOR_3D_GYRO
                                       |  MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  if (health & ODY_SENS_MAG)         m |= MAV_SYS_STATUS_SENSOR_3D_MAG;
  if (health & ODY_SENS_BARO)        m |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
  if (health & ODY_SENS_GNSS)        m |= MAV_SYS_STATUS_SENSOR_GPS;
  if (health & ODY_SENS_TOF_DOWN)    m |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
  if (health & ODY_SENS_CURRENT)     m |= MAV_SYS_STATUS_SENSOR_BATTERY;
  return m;
}

// =====================================================================================
//  Command transmission  --  FIX FOR FINDING 3
// =====================================================================================
static bool sendCommand(uint8_t commandId, uint8_t argument) {
  CommandPayload cmd{};
  cmd.sessionId = sessionId;
  cmd.commandId = commandId;
  cmd.argument  = argument;

  uint8_t frame[ODY_MAX_FRAME];
  const uint32_t len = odyEncodeFrame(frame, sizeof(frame),
                                      ODY_FRAME_COMMAND, 0, txSeq++,
                                      &cmd, (uint8_t)sizeof(cmd));
  if (len == 0) return false;

  if (!LoRa.beginPacket()) return false;
  LoRa.write(frame, len);
  LoRa.endPacket();          // blocking; the command must actually leave the radio

  // Repeat safety-critical commands. The uplink is unacknowledged and the aircraft
  // rejects duplicates by sequence number, so sending three costs nothing but
  // materially improves the odds of one arriving through interference.
  for (int i = 0; i < 2; ++i) {
    delay(60);
    const uint32_t l2 = odyEncodeFrame(frame, sizeof(frame),
                                       ODY_FRAME_COMMAND, 0, txSeq++,
                                       &cmd, (uint8_t)sizeof(cmd));
    if (l2 && LoRa.beginPacket()) {
      LoRa.write(frame, l2);
      LoRa.endPacket();
    }
  }
  return true;
}

// -------------------------------------------------------------------------------------
//  Debounced button with an optional hold requirement
// -------------------------------------------------------------------------------------
struct Button {
  uint8_t  pin;
  uint32_t holdMs;
  uint32_t downSince = 0;
  bool     fired     = false;

  bool pressed(uint32_t nowMs) {
    const bool down = digitalRead(pin) == LOW;
    if (!down) { downSince = 0; fired = false; return false; }
    if (downSince == 0) { downSince = nowMs; return false; }
    if (fired) return false;
    if ((uint32_t)(nowMs - downSince) >= max(holdMs, BUTTON_DEBOUNCE_MS)) {
      fired = true;
      return true;
    }
    return false;
  }
};

static Button btnPermit{PIN_BTN_PERMIT, BUTTON_DEBOUNCE_MS};
static Button btnRth   {PIN_BTN_RTH,    BUTTON_HOLD_MS};
static Button btnAbort {PIN_BTN_ABORT,  BUTTON_HOLD_MS};

// =====================================================================================
//  Telemetry reception
// =====================================================================================
static void serviceLora(uint32_t nowMs) {
  const int size = LoRa.parsePacket();
  if (size <= 0) return;
  if (size > (int)sizeof(rxBuf)) { ++rejectedCount; return; }

  int n = 0;
  while (LoRa.available() && n < size) rxBuf[n++] = (uint8_t)LoRa.read();

  OdyFrameHeader hdr;
  const uint8_t* payload = nullptr;
  uint8_t payloadLen = 0;
  if (!odyDecodeFrame(rxBuf, (uint32_t)n, &hdr, &payload, &payloadLen)) {
    ++rejectedCount;
    return;
  }
  if (hdr.type != ODY_FRAME_TELEMETRY || payloadLen != sizeof(TelemetryPayload)) {
    ++rejectedCount;
    return;
  }

  // Count genuine loss rather than reordering. A frame older than the last one we saw
  // is a duplicate from the aircraft's repeat logic, not a new sample.
  if (haveRxSeq && !odySeqIsNewer(hdr.seq, lastRxSeq)) return;
  lastRxSeq = hdr.seq;
  haveRxSeq = true;

  memcpy(&latest, payload, sizeof(latest));
  haveTelemetry   = true;
  lastTelemetryMs = nowMs;
  ++telemetryCount;
}

// =====================================================================================
//  MAVLink republishing
// =====================================================================================
static void publishMavlink(uint32_t nowMs) {
  const bool linkUp = haveTelemetry
                   && (uint32_t)(nowMs - lastTelemetryMs) < LINK_TIMEOUT_MS;
  digitalWrite(PIN_LED_LINK, linkUp ? HIGH : LOW);

  const uint8_t state = linkUp ? latest.flightState : (uint8_t)ODY_STATE_BOOT;

  mav.heartbeat(mavBaseMode(state), mavSystemStatus(state), (uint32_t)state);

  if (!linkUp) {
    mav.statusText(MAV_SEVERITY_CRITICAL, "Odyssey: telemetry link down");
    return;
  }

  const uint32_t sensors = mavSensorMask(latest.sensorHealth);
  // drop_rate_comm is expressed in hundredths of a percent. Two frames per second
  // expected; anything less is loss.
  const uint16_t expected = 1;
  static uint32_t lastCount = 0;
  const uint32_t got = telemetryCount - lastCount;
  lastCount = telemetryCount;
  const uint16_t dropRate = (got >= expected) ? 0
                          : (uint16_t)((expected - got) * 10000u / expected);

  mav.sysStatus(sensors, sensors, sensors,
                latest.battMillivolts,
                latest.battCurrentCentiAmps,
                (int8_t)(latest.battRemainingPercent / 100u),
                dropRate);

  mav.attitude(latest.uptimeMs,
               (float)latest.rollCentideg  / 100.0f * (float)DEG_TO_RAD,
               (float)latest.pitchCentideg / 100.0f * (float)DEG_TO_RAD,
               (float)latest.headingCentideg / 100.0f * (float)DEG_TO_RAD,
               0.0f, 0.0f, 0.0f);

  mav.globalPosition(latest.uptimeMs,
                     latest.latitude1e7, latest.longitude1e7,
                     latest.altitudeAglMm, latest.altitudeAglMm,
                     latest.groundSpeedCmS, 0, -latest.varioCmS,
                     (uint16_t)latest.headingCentideg);

  mav.vfrHud(0.0f,
             (float)latest.groundSpeedCmS / 100.0f,
             (float)latest.altitudeAglMm / 1000.0f,
             (float)latest.varioCmS / 100.0f,
             (int16_t)(latest.headingCentideg / 100),
             0);

  mav.gpsRawInt((uint64_t)latest.uptimeMs * 1000ULL,
                latest.latitude1e7, latest.longitude1e7, latest.altitudeAglMm,
                latest.satellites >= 6 ? 3 : (latest.satellites >= 4 ? 2 : 0),
                latest.satellites, 0xFFFF);

  // Surface the states that need a human decision as GCS alerts, not just numbers.
  static uint8_t lastAnnouncedState = 0xFF;
  if (state != lastAnnouncedState) {
    lastAnnouncedState = state;
    char msg[50];
    snprintf(msg, sizeof(msg), "Odyssey: %s", odyStateName(state));
    const uint8_t sev = (state == ODY_STATE_AWAITING_LAND_PERMIT ||
                         state == ODY_STATE_FREEFALL_PARACHUTE)
                          ? MAV_SEVERITY_CRITICAL : MAV_SEVERITY_INFO;
    mav.statusText(sev, msg);

    if (state == ODY_STATE_AWAITING_LAND_PERMIT) {
      mav.statusText(MAV_SEVERITY_CRITICAL,
                     "LAND PERMISSION REQUESTED - press PERMIT");
    }
  }

  if (latest.armFlags != 0 && state == ODY_STATE_PREFLIGHT_OK) {
    char msg[50];
    snprintf(msg, sizeof(msg), "Arm blocked: 0x%02X", latest.armFlags);
    mav.statusText(MAV_SEVERITY_WARNING, msg);
  }
}

// =====================================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BTN_PERMIT, INPUT_PULLUP);
  pinMode(PIN_BTN_RTH,    INPUT_PULLUP);
  pinMode(PIN_BTN_ABORT,  INPUT_PULLUP);
  pinMode(PIN_LED_LINK,   OUTPUT);
  digitalWrite(PIN_LED_LINK, LOW);

  // A fresh session id every boot. The aircraft binds to the first station it hears
  // and rejects everyone else, so a second transmitter cannot take over mid-flight.
  sessionId = esp_random();

  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    // Do not spin forever in a while(1) as the original did -- that leaves the
    // operator with a dead box and no explanation. Keep emitting MAVLink status text
    // so the failure is visible in the GCS.
    for (;;) {
      mav.heartbeat(0, MAV_STATE_CRITICAL, 0);
      mav.statusText(MAV_SEVERITY_CRITICAL, "Ground station: LoRa radio failed");
      delay(1000);
    }
  }

  // Must match the aircraft exactly.
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(20);
  LoRa.enableCrc();
  LoRa.setSyncWord(0x4F);

  mav.statusText(MAV_SEVERITY_INFO, "Odyssey ground station ready");
}

void loop() {
  const uint32_t nowMs = millis();

  serviceLora(nowMs);

  // ---- Operator commands -------------------------------------------------------------
  if (btnPermit.pressed(nowMs)) {
    if (haveTelemetry && latest.flightState == ODY_STATE_AWAITING_LAND_PERMIT) {
      sendCommand(ODY_CMD_PERMIT_LAND, 0);
      mav.statusText(MAV_SEVERITY_INFO, "PERMIT_LAND sent");
    } else {
      mav.statusText(MAV_SEVERITY_WARNING, "PERMIT ignored: no request pending");
    }
  }

  if (btnRth.pressed(nowMs)) {
    sendCommand(ODY_CMD_RTH_NOW, 0);
    mav.statusText(MAV_SEVERITY_INFO, "RTH_NOW sent");
  }

  if (btnAbort.pressed(nowMs)) {
    sendCommand(ODY_CMD_ABORT_TO_LAND, 0);
    mav.statusText(MAV_SEVERITY_CRITICAL, "ABORT_TO_LAND sent");
  }

  // ---- MAVLink at 1 Hz ----------------------------------------------------------------
  if ((uint32_t)(nowMs - lastHeartbeatMs) >= HEARTBEAT_PERIOD_MS) {
    lastHeartbeatMs = nowMs;
    publishMavlink(nowMs);
  }
}
