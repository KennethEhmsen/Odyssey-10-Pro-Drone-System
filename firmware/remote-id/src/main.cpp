// =====================================================================================
//  Odyssey-10 Pro -- Direct Remote ID broadcaster  (ESP32-C6)
//  ------------------------------------------------------------------------------------
//  FIX FOR FINDING 17.
//
//  Section 1 of the original specification listed a "Native Direct Remote ID (DRI)
//  OpenDroneID Broadcaster" among the core avionics. The string appeared exactly once
//  in the entire 1258-line document and nothing implemented it.
//
//  There is also a hardware reason it could never have worked as drawn: the ESP32-P4
//  has no radio. No Wi-Fi, no Bluetooth. No firmware on the flight controller could
//  broadcast anything. Remote ID needs its own radio, which is why the BOM carries this
//  ESP32-C6 module on the AUX broadcast bus.
//
//  ------------------------------------------------------------------------------------
//  WHETHER YOU ACTUALLY NEED THIS -- READ docs SECTION 12.2 FIRST
//
//  Revision 2.0 of the specification asserted that flying above 250 g without Remote ID
//  is unlawful in the EU. That was an over-claim, and it is corrected in revision 2.1.
//
//  Under EU 2019/945 the Direct Remote ID requirement attaches to CLASS-MARKED aircraft
//  (C1/C2/C3). EASA separately permits PRIVATELY BUILT UAS in the open category -- A1
//  below 250 g and A3 below 25 kg -- and a privately built aircraft is not class-marked,
//  so the C-class DRI requirement does not attach to it by that route.
//
//  A home-built 1773 g aircraft flown in A3 may therefore not require Direct Remote ID
//  today. It IS required in the specific category. Denmark has also proposed national
//  rules broadening electronic visibility (consultation closed 21 August 2026, proposed
//  effect 1 January 2027) which would change the position. Check the current rules for
//  your own operation rather than trusting this comment or a specification section.
//
//  ------------------------------------------------------------------------------------
//  WHAT THIS MODULE IS AND IS NOT
//
//  It broadcasts correctly-encoded OpenDroneID messages. That is NOT the same as being
//  a compliant Direct Remote Identification add-on under EU 2019/945 Part 6, which
//  additionally requires tamper resistance, the serial physically associated with the
//  module, operator-ID upload with validity checking, and manufacturer instructions --
//  plus product-conformity obligations if placed on the EU market. Linking the reference
//  encoder gets the wire format right; it does not confer compliance.
//
//  Message encoding uses opendroneid/opendroneid-core-c rather than a hand-rolled bit
//  layout, because the ASTM F3411 / ASD-STAN field packing is bit-exact and easy to get
//  subtly wrong.
//
//  Broadcast media, both enabled for receiver compatibility:
//    * Bluetooth 5 Long Range, Coded PHY, extended advertising (ASTM service data,
//      UUID 0xFFFA, application code 0x0D)
//    * Wi-Fi beacon vendor-specific IE
//
//  Rates: Location/Vector at 1 Hz minimum, static messages at least every 3 s. The AUX
//  bus feeds this module at 2 Hz, giving margin on the dynamic message.
// =====================================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <opendroneid.h>
#include "odyssey_link.h"
#include "odid_transport.h"
#include "identity.h"

// -------------------------------------------------------------------------------------
//  IDENTITY CONFIGURATION
//
//  Two identifiers, handled very differently. See identity.h for the full explanation.
//
//  1. UAS ID identifies the AIRCRAFT. There are two practical routes and you almost
//     certainly want the second one.
//
//     ODY_UAS_ID_CTA_SERIAL
//         A CTA-2063-A serial: [ICAO MFR code: 4][length code: 1][serial: 1..15].
//         The 4-character manufacturer code is issued by ICAO to MANUFACTURERS.
//         Use this if you are manufacturing airframes or Remote ID modules for
//         others -- or if you fitted a BOUGHT Remote ID module, in which case it
//         already carries a serial from its own manufacturer and you apply for
//         nothing.
//
//     ODY_UAS_ID_CAA_REGISTRATION   <-- DEFAULT, and the route for a home build
//         The registration issued by your civil aviation authority. Needs no ICAO
//         involvement at all.
//
//     Revision 2.1 hard-coded the CTA route and told you to write to
//     OPSInbox@icao.int. For a privately built aircraft that was wrong: building one
//     aircraft for yourself does not make you a manufacturer, and there is nothing to
//     apply for.
//
//  2. OPERATOR REGISTRATION NUMBER identifies the PERSON and is provisioned at RUNTIME
//     into NVS. It is never compiled in.
//
//     *** THE THREE CHARACTERS AFTER THE HYPHEN ARE SECRET. ***
//     EASA is explicit that they must not be shared, and Trafikstyrelsen calls the
//     Danish equivalent a "security code". A full operator ID in this file would publish
//     that secret to anyone with repository access, and git history would keep it there
//     after any later "fix". So there is no #define for it, and there never should be.
//
//     Provision over the serial console:  SETOPERATOR DNKxxxxxxxxxxxxx-yyy
//     Only the public 16 characters are broadcast; the secret is stored, never sent.
// -------------------------------------------------------------------------------------
#define ODY_UAS_ID_CAA_REGISTRATION  0
#define ODY_UAS_ID_CTA_SERIAL        1

// Which identifier this aircraft broadcasts in the Basic ID message.
#define UAS_ID_TYPE          ODY_UAS_ID_CAA_REGISTRATION

// Used when UAS_ID_TYPE is ODY_UAS_ID_CAA_REGISTRATION.
// In the EU open category a privately built aircraft is not individually registered --
// the OPERATOR is. The operator registration is therefore the identifier in practice,
// and the Operator ID message (type 5) carries it as well. If your authority issues a
// distinct per-aircraft registration, put that here instead.
#define UAS_CAA_REGISTRATION "SET-YOUR-CAA-REGISTRATION"

// Used when UAS_ID_TYPE is ODY_UAS_ID_CTA_SERIAL. Left deliberately invalid.
#define UAS_SERIAL_NUMBER    "SET-YOUR-CTA-SERIAL"

#define NVS_NAMESPACE       "odyrid"
#define NVS_KEY_OPERATOR    "operator"

// Self-declared UA category and class.
//
// NOTE: a privately built aircraft in the open category is NOT class-marked. Declaring
// a C-class here would assert a conformity assessment that a home build has not been
// through, so the class is left undeclared. See docs section 12.2.
#define UA_CATEGORY_EU      ODID_CATEGORY_EU_OPEN
#define UA_CLASS_EU         ODID_CLASS_EU_CLASS_UNDECLARED

#define PIN_AUX_RX          4       // AUX broadcast bus from the flight controller

// Health line back to the flight controller. Driven HIGH only while both radios are
// advertising, a Location message has gone out recently, the CTA serial is structurally
// valid, and an operator registration has been provisioned. The flight controller reads
// it with a pull-down, so a missing or silently failed module reads as unhealthy.
#define PIN_HEALTH_OUT      5

#define LOCATION_PERIOD_MS  1000UL  // 1 Hz, the standard's minimum for dynamic data
#define STATIC_PERIOD_MS    3000UL  // Basic ID / System / Operator ID

// -------------------------------------------------------------------------------------
//  State
// -------------------------------------------------------------------------------------
static HardwareSerial AuxSerial(1);

static ODID_UAS_Data uasData;
static uint8_t  rxBuf[ODY_MAX_FRAME];
static uint8_t  rxIdx = 0;
static uint32_t lastLocationMs     = 0;
static uint32_t lastStaticMs       = 0;
static uint32_t lastAuxFrameMs     = 0;
static uint32_t lastLocationSentMs = 0;
static uint8_t  msgCounter[ODID_MSG_COUNTER_AMOUNT] = {0};
static bool     operatorIdConfigured = false;
static bool     uasIdValid           = false;

// =====================================================================================
//  Static message population
// =====================================================================================
static void buildStaticMessages() {
  odid_initUasData(&uasData);

  uasData.BasicID[0].UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
#if UAS_ID_TYPE == ODY_UAS_ID_CTA_SERIAL
  uasData.BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;
  strncpy(uasData.BasicID[0].UASID, UAS_SERIAL_NUMBER,
          sizeof(uasData.BasicID[0].UASID) - 1);
#else
  uasData.BasicID[0].IDType = ODID_IDTYPE_CAA_REGISTRATION_ID;
  strncpy(uasData.BasicID[0].UASID, UAS_CAA_REGISTRATION,
          sizeof(uasData.BasicID[0].UASID) - 1);
#endif
  uasData.BasicIDValid[0] = 1;

  uasData.System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
  uasData.System.ClassificationType   = ODID_CLASSIFICATION_TYPE_EU;
  uasData.System.CategoryEU           = UA_CATEGORY_EU;
  uasData.System.ClassEU              = UA_CLASS_EU;
  uasData.System.AreaCount            = 1;
  uasData.System.AreaRadius           = 0;
  uasData.System.AreaCeiling          = INV_ALT;
  uasData.System.AreaFloor            = INV_ALT;
  uasData.SystemValid = 1;

  // Operator ID is filled by loadOperatorId() from NVS, never from a constant.
  uasData.OperatorID.OperatorIdType = ODID_OPERATOR_ID;
  uasData.OperatorID.OperatorId[0]  = 0;
  uasData.OperatorIDValid = 0;
}

// -------------------------------------------------------------------------------------
//  Operator identity
//
//  Only the PUBLIC 16 characters ever reach uasData, so the secret cannot leak into a
//  broadcast frame even if a future edit gets careless.
// -------------------------------------------------------------------------------------
static bool loadOperatorId() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) return false;

  char full[ODY_OPERATOR_FULL_LEN + 1] = {0};
  const size_t n = prefs.getString(NVS_KEY_OPERATOR, full, sizeof(full));
  prefs.end();
  if (n == 0) return false;

  char pub[ODY_OPERATOR_PUBLIC_LEN + 1];
  if (odySplitOperatorId(full, pub, sizeof(pub), nullptr, 0) != ODY_ID_OK) {
    Serial.println("[ODID] stored operator ID is malformed");
    return false;
  }

  // Non-strict: the Luhn implementation is validated against a single published
  // example, so a checksum disagreement must not silently disable Remote ID.
  const uint8_t v = odyValidateOperatorIdPublic(pub, false);
  if (v != ODY_ID_OK) {
    Serial.printf("[ODID] stored operator ID is invalid: %s\n", odyIdResultText(v));
    return false;
  }
  if (odyValidateOperatorIdPublic(pub, true) != ODY_ID_OK) {
    Serial.println("[ODID] NOTE: check character does not match the Luhn mod 36 "
                   "implementation. Broadcasting anyway -- verify the identifier "
                   "against your authority's specification.");
  }

  strncpy(uasData.OperatorID.OperatorId, pub,
          sizeof(uasData.OperatorID.OperatorId) - 1);
  uasData.OperatorIDValid = 1;
  Serial.printf("[ODID] operator ID loaded: %s (secret withheld)\n", pub);
  return true;
}

// Provisioning console. Deliberately never echoes the secret.
static void serviceProvisioning() {
  static char line[64];
  static uint8_t len = 0;

  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (len < sizeof(line) - 1) line[len++] = c;
      continue;
    }
    line[len] = '\0';
    len = 0;

    if (strncmp(line, "SETOPERATOR ", 12) == 0) {
      const char* value = line + 12;
      const uint8_t v = odyValidateOperatorIdFull(value, false);
      if (v != ODY_ID_OK) {
        Serial.printf("[ODID] rejected: %s\n", odyIdResultText(v));
      } else {
        Preferences prefs;
        if (prefs.begin(NVS_NAMESPACE, false)) {
          prefs.putString(NVS_KEY_OPERATOR, value);
          prefs.end();
          operatorIdConfigured = loadOperatorId();
          Serial.println("[ODID] operator ID stored.");
        }
      }
      // Scrub the buffer so the secret does not linger in RAM.
      memset(line, 0, sizeof(line));

    } else if (strcmp(line, "STATUS") == 0) {
      Serial.printf("[ODID] uas_id=%s (%s, %s)  operator=%s  broadcasting=%s\n",
#if UAS_ID_TYPE == ODY_UAS_ID_CTA_SERIAL
                    UAS_SERIAL_NUMBER, "CTA-2063-A serial",
                    odyIdResultText(odyValidateCtaSerial(UAS_SERIAL_NUMBER)),
#else
                    UAS_CAA_REGISTRATION, "CAA registration",
                    odyIdResultText(odyValidateCaaRegistration(UAS_CAA_REGISTRATION)),
#endif
                    uasData.OperatorIDValid ? uasData.OperatorID.OperatorId : "<unset>",
                    odidTransportHealthy() ? "yes" : "no");

    } else if (line[0]) {
      Serial.println("[ODID] commands: SETOPERATOR <16char-sec> | STATUS");
    }
  }
}

// =====================================================================================
//  AUX bus reception
// =====================================================================================
static void applyAuxPosition(const AuxPositionPayload& p, uint32_t nowMs) {
  lastAuxFrameMs = nowMs;

  ODID_Location_data& loc = uasData.Location;

  if (p.emergency) {
    loc.Status = ODID_STATUS_EMERGENCY;
  } else if (odyMotorsAreLive(p.flightState)) {
    loc.Status = (p.altitudeAglMm > 2000) ? ODID_STATUS_AIRBORNE : ODID_STATUS_GROUND;
  } else {
    loc.Status = ODID_STATUS_GROUND;
  }

  loc.Direction       = (float)p.headingCentideg / 100.0f;
  loc.SpeedHorizontal = (float)p.groundSpeedCmS / 100.0f;
  loc.SpeedVertical   = INV_SPEED_V;
  loc.Latitude        = (double)p.latitude1e7  / 1e7;
  loc.Longitude       = (double)p.longitude1e7 / 1e7;
  loc.AltitudeBaro    = INV_ALT;
  loc.AltitudeGeo     = (float)p.altitudeMslMm / 1000.0f;
  loc.HeightType      = ODID_HEIGHT_REF_OVER_TAKEOFF;
  loc.Height          = (float)p.altitudeAglMm / 1000.0f;

  // Reporting UNKNOWN accuracy is permitted and honest; claiming tighter figures than
  // a BN-220 delivers would be worse than saying nothing.
  loc.HorizAccuracy = (p.satellites >= 9) ? ODID_HOR_ACC_3_METER
                    : (p.satellites >= 6) ? ODID_HOR_ACC_10_METER
                                          : ODID_HOR_ACC_UNKNOWN;
  loc.VertAccuracy  = ODID_VER_ACC_UNKNOWN;
  loc.BaroAccuracy  = ODID_VER_ACC_UNKNOWN;
  loc.SpeedAccuracy = ODID_SPEED_ACC_UNKNOWN;
  loc.TSAccuracy    = ODID_TIME_ACC_UNKNOWN;
  loc.TimeStamp     = (float)((nowMs / 100u) % 36000u) / 10.0f;

  uasData.LocationValid = 1;

  if (uasData.System.OperatorLatitude == 0.0 && p.satellites >= 6) {
    uasData.System.OperatorLatitude    = loc.Latitude;
    uasData.System.OperatorLongitude   = loc.Longitude;
    uasData.System.OperatorAltitudeGeo = loc.AltitudeGeo;
    uasData.System.Timestamp           = (uint32_t)(nowMs / 1000u);
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
  if (!uasIdValid) return;      // see broadcastStatic()
  ODID_Location_encoded enc;
  if (encodeLocationMessage(&enc, &uasData.Location) != ODID_SUCCESS) return;
  odidTransportSend(ODID_MESSAGETYPE_LOCATION,
                    (const uint8_t*)&enc, sizeof(enc),
                    msgCounter[ODID_MSG_COUNTER_LOCATION]++);
  lastLocationSentMs = millis();
}

static void broadcastStatic() {
  // An invalid UAS ID is worse than no broadcast: a receiver logs an identifier that
  // cannot be traced back to anyone. Stay silent instead.
  if (!uasIdValid) return;

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

  // Only transmitted once a valid operator registration has been provisioned.
  if (uasData.OperatorIDValid) {
    ODID_OperatorID_encoded op;
    if (encodeOperatorIDMessage(&op, &uasData.OperatorID) == ODID_SUCCESS) {
      odidTransportSend(ODID_MESSAGETYPE_OPERATOR_ID,
                        (const uint8_t*)&op, sizeof(op),
                        msgCounter[ODID_MSG_COUNTER_OPERATOR_ID]++);
    }
  }
}

// Asserts the health line only when the module is genuinely doing its job.
static void updateHealthLine(uint32_t nowMs) {
  const bool broadcasting = odidTransportHealthy()
                         && lastLocationSentMs != 0
                         && (uint32_t)(nowMs - lastLocationSentMs) < 3000u;
  digitalWrite(PIN_HEALTH_OUT,
               (broadcasting && operatorIdConfigured && uasIdValid) ? HIGH : LOW);
}

// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Odyssey-10 Pro Remote ID broadcaster ===");

  pinMode(PIN_HEALTH_OUT, OUTPUT);
  digitalWrite(PIN_HEALTH_OUT, LOW);

  AuxSerial.begin(115200, SERIAL_8N1, PIN_AUX_RX, -1);   // receive only

  buildStaticMessages();
  odidTransportBegin();

#if UAS_ID_TYPE == ODY_UAS_ID_CTA_SERIAL
  const uint8_t idCheck = odyValidateCtaSerial(UAS_SERIAL_NUMBER);
  uasIdValid = (idCheck == ODY_ID_OK);
  if (!uasIdValid) {
    Serial.printf("*** CTA-2063-A serial invalid: %s ***\n", odyIdResultText(idCheck));
    Serial.println("*** Set UAS_SERIAL_NUMBER, or switch UAS_ID_TYPE to");
    Serial.println("*** ODY_UAS_ID_CAA_REGISTRATION if this is a home build -- that");
    Serial.println("*** route needs no ICAO manufacturer code. ***");
  } else {
    Serial.printf("UAS ID (CTA-2063-A serial): %s\n", UAS_SERIAL_NUMBER);
  }
#else
  const uint8_t idCheck = odyValidateCaaRegistration(UAS_CAA_REGISTRATION);
  uasIdValid = (idCheck == ODY_ID_OK);
  if (!uasIdValid) {
    Serial.printf("*** CAA registration invalid: %s ***\n", odyIdResultText(idCheck));
    Serial.println("*** Set UAS_CAA_REGISTRATION to the registration issued by your");
    Serial.println("*** civil aviation authority. No ICAO application is involved. ***");
  } else {
    Serial.printf("UAS ID (CAA registration): %s\n", UAS_CAA_REGISTRATION);
  }
#endif

  operatorIdConfigured = loadOperatorId();
  if (!operatorIdConfigured) {
    Serial.println("*** No operator registration number provisioned. ***");
    Serial.println("*** Provision with:  SETOPERATOR DNKxxxxxxxxxxxxx-yyy ***");
  }

  if (!uasIdValid || !operatorIdConfigured) {
    Serial.println("*** Remote ID is NOT broadcasting. ***");
    Serial.println("*** The health line stays LOW. Whether that blocks arming depends");
    Serial.println("*** on REQUIRE_REMOTE_ID_TO_ARM in the flight controller's config.h,");
    Serial.println("*** which defaults to 0 because a privately built aircraft under");
    Serial.println("*** 25 kg in EU open category A3 does not require Direct Remote ID.");
    Serial.println("*** Set it to 1 if you fly in the specific category, are");
    Serial.println("*** class-marked, or are in a jurisdiction that requires it.");
  }
}

void loop() {
  const uint32_t nowMs = millis();

  serviceAuxBus(nowMs);
  serviceProvisioning();

  // If the AUX bus goes quiet the position we hold is stale. The standard requires the
  // broadcast to continue, so keep transmitting, but mark the fix invalid rather than
  // repeating a position the aircraft has long since left.
  if (lastAuxFrameMs != 0 && (uint32_t)(nowMs - lastAuxFrameMs) > 5000u) {
    uasData.Location.Latitude      = INV_LAT;
    uasData.Location.Longitude     = INV_LON;
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
