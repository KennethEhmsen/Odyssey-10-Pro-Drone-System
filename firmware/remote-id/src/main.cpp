// =====================================================================================
//  Odyssey-10 Pro -- Direct Remote ID broadcaster  (ESP32-C6)
//  ------------------------------------------------------------------------------------
//  FIX FOR FINDING 17.
//
//  Section 1 of the original specification listed a "Native Direct Remote ID (DRI)
//  OpenDroneID Broadcaster" among the core avionics. The string appeared exactly once
//  in the entire 1258-line document and nothing implemented it. At an all-up weight of
//  1773 g the aircraft is over the 250 g threshold in every jurisdiction that has one,
//  so flying it without Remote ID is not a missing feature -- it is illegal in the
//  United States (14 CFR Part 89), the European Union (EU 2019/945 direct remote
//  identification) and the United Kingdom.
//
//  There is also a hardware reason it could never have worked as drawn: the ESP32-P4
//  has no radio. It has no Wi-Fi and no Bluetooth at all, so no amount of firmware on
//  the flight controller can broadcast anything. Remote ID needs its own radio, which
//  is why the BOM now carries an ESP32-C6 module on the AUX broadcast bus.
//
//  ------------------------------------------------------------------------------------
//  MESSAGE ENCODING
//
//  The ASTM F3411 / ASD-STAN prEN 4709-002 message layout is bit-exact and easy to get
//  subtly wrong. Rather than hand-roll it, this firmware links the reference encoder,
//  opendroneid/opendroneid-core-c, which is the same library the standard's own
//  conformance tools use. See platformio.ini lib_deps.
//
//  Broadcast media (both enabled, as recommended for maximum receiver compatibility):
//    * Bluetooth 5 Long Range, Coded PHY, extended advertising -- ASTM service data
//      under UUID 0xFFFA with application code 0x0D
//    * Wi-Fi NAN / Beacon vendor-specific IE
//
//  Rates required by the standard: Location/Vector at 1 Hz minimum, static messages
//  (Basic ID, System, Operator ID) at least every 3 s. The AUX bus feeds this module
//  at 2 Hz, giving comfortable margin on the dynamic message.
//
//  ------------------------------------------------------------------------------------
//  BEFORE FLYING: set OPERATOR_ID and UAS_SERIAL_NUMBER below to the values registered
//  with your civil aviation authority. Broadcasting a placeholder is itself a
//  violation. See docs section 12.
// =====================================================================================

#include <Arduino.h>
#include <opendroneid.h>
#include "odyssey_link.h"
#include "odid_transport.h"

// -------------------------------------------------------------------------------------
//  OPERATOR CONFIGURATION -- MUST BE SET BEFORE FLIGHT
// -------------------------------------------------------------------------------------
// ANSI/CTA-2063-A serial number issued by the manufacturer, or your own if you built
// the aircraft. Format: 4-char manufacturer code, 1-char length code, then the serial.
#define UAS_SERIAL_NUMBER   "ODY1P0000000000000000"

// The operator registration number issued by your CAA (FAA, EASA member state, CAA UK).
#define OPERATOR_ID         "SET-YOUR-OPERATOR-ID"

// Self-declared UA category and class. 1773 g places this aircraft in EU open category
// A3 / class C3 at best; check your own jurisdiction.
#define UA_CATEGORY_EU      ODID_CATEGORY_EU_OPEN
#define UA_CLASS_EU         ODID_CLASS_EU_CLASS_3

#define PIN_AUX_RX          4       // AUX broadcast bus from the flight controller

// Health line back to the flight controller. Driven HIGH only while BOTH radios are
// advertising AND a Location message has gone out within the last few seconds. The
// flight controller reads it with a pull-down, so a missing, crashed or silently
// failed module reads as unhealthy and blocks arming -- which is the point, since
// flying this aircraft without Remote ID is unlawful (docs section 12.1).
#define PIN_HEALTH_OUT      5

// Broadcast cadence
#define LOCATION_PERIOD_MS  1000UL  // 1 Hz, the standard's minimum for dynamic data
#define STATIC_PERIOD_MS    3000UL  // Basic ID / System / Operator ID

// -------------------------------------------------------------------------------------
//  State
// -------------------------------------------------------------------------------------
static HardwareSerial AuxSerial(1);

static ODID_UAS_Data uasData;
static uint8_t  rxBuf[ODY_MAX_FRAME];
static uint8_t  rxIdx = 0;
static uint32_t lastLocationMs = 0;
static uint32_t lastStaticMs   = 0;
static uint32_t lastAuxFrameMs = 0;
static uint8_t  msgCounter[ODID_MSG_COUNTER_AMOUNT] = {0};
static uint32_t lastLocationSentMs = 0;
static bool     operatorIdConfigured = false;

// =====================================================================================
//  Static message population
// =====================================================================================
static void buildStaticMessages() {
  odid_initUasData(&uasData);

  // ---- Basic ID --------------------------------------------------------------------
  uasData.BasicID[0].UAType   = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
  uasData.BasicID[0].IDType   = ODID_IDTYPE_SERIAL_NUMBER;
  strncpy(uasData.BasicID[0].UASID, UAS_SERIAL_NUMBER,
          sizeof(uasData.BasicID[0].UASID) - 1);
  uasData.BasicIDValid[0] = 1;

  // ---- System ----------------------------------------------------------------------
  // OperatorLocation is filled from the home position once the flight controller
  // reports one; until then the standard's "takeoff location" type is not yet valid.
  uasData.System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
  uasData.System.ClassificationType   = ODID_CLASSIFICATION_TYPE_EU;
  uasData.System.CategoryEU           = UA_CATEGORY_EU;
  uasData.System.ClassEU              = UA_CLASS_EU;
  uasData.System.AreaCount            = 1;
  uasData.System.AreaRadius           = 0;
  uasData.System.AreaCeiling          = INV_ALT;
  uasData.System.AreaFloor            = INV_ALT;
  uasData.SystemValid = 1;

  // ---- Operator ID -----------------------------------------------------------------
  uasData.OperatorID.OperatorIdType = ODID_OPERATOR_ID;
  strncpy(uasData.OperatorID.OperatorId, OPERATOR_ID,
          sizeof(uasData.OperatorID.OperatorId) - 1);
  uasData.OperatorIDValid = 1;
}

// =====================================================================================
//  AUX bus reception
// =====================================================================================
static void applyAuxPosition(const AuxPositionPayload& p, uint32_t nowMs) {
  lastAuxFrameMs = nowMs;

  ODID_Location_data& loc = uasData.Location;

  // Status. The standard distinguishes ground, airborne and emergency; the flight
  // controller's state maps onto that directly.
  if (p.emergency) {
    loc.Status = ODID_STATUS_EMERGENCY;
  } else if (odyMotorsAreLive(p.flightState)) {
    loc.Status = (p.altitudeAglMm > 2000) ? ODID_STATUS_AIRBORNE : ODID_STATUS_GROUND;
  } else {
    loc.Status = ODID_STATUS_GROUND;
  }

  loc.Direction      = (float)p.headingCentideg / 100.0f;
  loc.SpeedHorizontal = (float)p.groundSpeedCmS / 100.0f;
  loc.SpeedVertical  = INV_SPEED_V;              // not carried on the AUX frame
  loc.Latitude       = (double)p.latitude1e7  / 1e7;
  loc.Longitude      = (double)p.longitude1e7 / 1e7;
  loc.AltitudeBaro   = INV_ALT;
  loc.AltitudeGeo    = (float)p.altitudeMslMm / 1000.0f;
  loc.HeightType     = ODID_HEIGHT_REF_OVER_TAKEOFF;
  loc.Height         = (float)p.altitudeAglMm / 1000.0f;

  // Accuracy fields. Reporting UNKNOWN is permitted and is honest; claiming a tighter
  // figure than the BN-220 delivers would be worse than saying nothing.
  loc.HorizAccuracy  = (p.satellites >= 9) ? ODID_HOR_ACC_3_METER
                     : (p.satellites >= 6) ? ODID_HOR_ACC_10_METER
                                           : ODID_HOR_ACC_UNKNOWN;
  loc.VertAccuracy   = ODID_VER_ACC_UNKNOWN;
  loc.BaroAccuracy   = ODID_VER_ACC_UNKNOWN;
  loc.SpeedAccuracy  = ODID_SPEED_ACC_UNKNOWN;
  loc.TSAccuracy     = ODID_TIME_ACC_UNKNOWN;
  loc.TimeStamp      = (float)((nowMs / 100u) % 36000u) / 10.0f;   // tenths past the hour

  uasData.LocationValid = 1;

  // Latch the takeoff location the first time we see a valid airborne fix.
  if (!uasData.System.OperatorLatitude && p.satellites >= 6) {
    uasData.System.OperatorLatitude  = loc.Latitude;
    uasData.System.OperatorLongitude = loc.Longitude;
    uasData.System.OperatorAltitudeGeo = loc.AltitudeGeo;
    uasData.System.Timestamp = (uint32_t)(nowMs / 1000u);
  }
}

static void serviceAuxBus(uint32_t nowMs) {
  while (AuxSerial.available()) {
    const uint8_t b = (uint8_t)AuxSerial.read();

    if (rxIdx == 0) {
      if (b != ODY_LINK_MAGIC) continue;
      rxBuf[rxIdx++] = b;
      continue;
    }
    if (rxIdx < sizeof(rxBuf)) rxBuf[rxIdx++] = b;
    else { rxIdx = 0; continue; }

    if (rxIdx < sizeof(OdyFrameHeader)) continue;
    const OdyFrameHeader* h = (const OdyFrameHeader*)rxBuf;
    if (h->len > ODY_MAX_PAYLOAD) { rxIdx = 0; continue; }

    const uint32_t total = sizeof(OdyFrameHeader) + h->len + 2u;
    if (rxIdx < total) continue;

    OdyFrameHeader hdr;
    const uint8_t* payload = nullptr;
    uint8_t payloadLen = 0;
    if (odyDecodeFrame(rxBuf, total, &hdr, &payload, &payloadLen) &&
        hdr.type == ODY_FRAME_AUX_POS &&
        (hdr.dest == ODY_AUX_ADDR_REMOTEID || hdr.dest == ODY_AUX_ADDR_ALL) &&
        payloadLen == sizeof(AuxPositionPayload)) {
      AuxPositionPayload p;
      memcpy(&p, payload, sizeof(p));
      applyAuxPosition(p, nowMs);
    }
    rxIdx = 0;
  }
}

// =====================================================================================
//  Broadcast
// =====================================================================================
static void broadcastLocation() {
  ODID_Location_encoded enc;
  if (encodeLocationMessage(&enc, &uasData.Location) != ODID_SUCCESS) return;
  odidTransportSend(ODID_MESSAGETYPE_LOCATION,
                    (const uint8_t*)&enc, sizeof(enc),
                    msgCounter[ODID_MSG_COUNTER_LOCATION]++);
  lastLocationSentMs = millis();
}

// Asserts the health line only when the module is genuinely doing its job. A module
// that boots, fails to bring up BLE and then sits there must NOT report healthy.
static void updateHealthLine(uint32_t nowMs) {
  const bool broadcasting = odidTransportHealthy()
                         && lastLocationSentMs != 0
                         && (uint32_t)(nowMs - lastLocationSentMs) < 3000u;
  digitalWrite(PIN_HEALTH_OUT,
               (broadcasting && operatorIdConfigured) ? HIGH : LOW);
}

static void broadcastStatic() {
  ODID_BasicID_encoded basic;
  if (encodeBasicIDMessage(&basic, &uasData.BasicID[0]) == ODID_SUCCESS) {
    odidTransportSend(ODID_MESSAGETYPE_BASIC_ID,
                      (const uint8_t*)&basic, sizeof(basic),
                      msgCounter[ODID_MSG_COUNTER_BASIC_ID]++);
  }

  ODID_System_encoded sys;
  if (encodeSystemMessage(&sys, &uasData.System) == ODID_SUCCESS) {
    odidTransportSend(ODID_MESSAGETYPE_SYSTEM,
                      (const uint8_t*)&sys, sizeof(sys),
                      msgCounter[ODID_MSG_COUNTER_SYSTEM]++);
  }

  ODID_OperatorID_encoded op;
  if (encodeOperatorIDMessage(&op, &uasData.OperatorID) == ODID_SUCCESS) {
    odidTransportSend(ODID_MESSAGETYPE_OPERATOR_ID,
                      (const uint8_t*)&op, sizeof(op),
                      msgCounter[ODID_MSG_COUNTER_OPERATOR_ID]++);
  }
}

// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Odyssey-10 Pro Remote ID broadcaster ===");

  pinMode(PIN_HEALTH_OUT, OUTPUT);
  digitalWrite(PIN_HEALTH_OUT, LOW);

  operatorIdConfigured = (strcmp(OPERATOR_ID, "SET-YOUR-OPERATOR-ID") != 0);
  if (!operatorIdConfigured) {
    Serial.println("*** WARNING: OPERATOR_ID is still the placeholder. ***");
    Serial.println("*** Broadcasting a placeholder ID is a regulatory violation. ***");
    Serial.println("*** The health line stays LOW, so the aircraft will refuse to arm. ***");
  }

  AuxSerial.begin(115200, SERIAL_8N1, PIN_AUX_RX, -1);   // receive only

  buildStaticMessages();
  odidTransportBegin();

  Serial.printf("Serial: %s\nOperator: %s\n", UAS_SERIAL_NUMBER, OPERATOR_ID);
}

void loop() {
  const uint32_t nowMs = millis();
  serviceAuxBus(nowMs);

  // If the AUX bus goes quiet the position we hold is stale. The standard requires the
  // broadcast to continue, so keep transmitting, but mark the fix invalid rather than
  // repeating a position the aircraft has long since left.
  if (lastAuxFrameMs != 0 && (uint32_t)(nowMs - lastAuxFrameMs) > 5000u) {
    uasData.Location.Latitude  = INV_LAT;
    uasData.Location.Longitude = INV_LON;
    uasData.Location.HorizAccuracy = ODID_HOR_ACC_UNKNOWN;
  }

  if ((uint32_t)(nowMs - lastLocationMs) >= LOCATION_PERIOD_MS) {
    lastLocationMs = nowMs;
    broadcastLocation();
  }

  if ((uint32_t)(nowMs - lastStaticMs) >= STATIC_PERIOD_MS) {
    lastStaticMs = nowMs;
    broadcastStatic();
  }

  updateHealthLine(nowMs);
  delay(10);
}
