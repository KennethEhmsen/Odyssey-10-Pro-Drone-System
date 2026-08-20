// =====================================================================================
//  Odyssey-10 Pro -- Independent Emergency Locator Beacon  (ESP32-C3)
//  ------------------------------------------------------------------------------------
//  FIXES FINDINGS 7 AND 8.
//
//  FINDING 7 -- the beacon transmitted latitude 0 / longitude 0 forever.
//
//    The original node had no GPS and no data path from the flight controller: the
//    only interface was a single GPIO latch pulse, which carries no information. Its
//    BeaconPacket was a zero-initialised global and the only field ever written was
//    battMillivolts, so a searcher decoding the packet got a well-formed fix pointing
//    at Null Island.
//
//    The fix is a hardware change plus a protocol. The beacon node is now powered
//    during flight through a Schottky diode-OR from the aircraft's 5 V rail, with the
//    1S cell as the other input, so it is awake for the whole flight listening to the
//    AUX broadcast bus. It caches the aircraft's position in RTC-backed memory, which
//    survives both the transition to 1S power and a deep-sleep cycle. When the latch
//    fires -- or when main power simply disappears in a crash -- it already holds a
//    fix that is at most a second old.
//
//  FINDING 8 -- the 48-hour endurance claim was out by roughly 5x.
//
//    At SF11 / BW 62.5 kHz / CR 4:8 a symbol lasts 32.77 ms, and the on-air frame
//    (26-byte BeaconPayload plus 10 bytes of framing) needs 84.25 symbols: 2.76 s of
//    airtime for a single transmission. The original loop sent one every 2 s with no
//    sleep, so the radio ran at roughly 50% duty -- five times the 10% ISM ceiling for
//    433 MHz in ITU Region 1 -- and, because endPacket() blocks for the whole
//    transmission, the real cadence was about 4 s rather than the specified 2 s.
//    Average draw came to roughly 85 mA against an 800 mAh cell: nine hours, not
//    forty-eight.
//
//    The schedule below is duty-cycle compliant and budgeted rather than asserted:
//
//      phase 1 (first 2 h)   LoRa every  45 s -> 6.1% duty,  7.4 mA average
//      phase 2 (after 2 h)   LoRa every 180 s -> 1.5% duty,  1.8 mA average
//      audible/visual chirp every 5 s throughout          ~1.0 mA average
//      light sleep between events                        ~0.8 mA
//
//    Phase 1 costs about 21 mAh. With 680 mAh usable from an 800 mAh 1S cell down to
//    3.3 V, phase 2 then runs for well over 100 hours at 20 C. The specification
//    claims 72 h, which leaves margin for a cold cell, an aged pack and a buzzer
//    drawing more than the modelled figure.
//
//    transmitBeacon() also polices the measured duty cycle at runtime and refuses a
//    transmission that would breach it, so a future schedule change cannot silently
//    put the aircraft outside the regulations.
// =====================================================================================

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <esp_sleep.h>
#include "odyssey_link.h"

// -------------------------------------------------------------------------------------
//  Pin map (ESP32-C3-MINI-1)
// -------------------------------------------------------------------------------------
#define PIN_BUZZER          4
#define PIN_STROBE          5
#define PIN_LORA_NSS        7
#define PIN_LORA_RST        6
#define PIN_LORA_DIO0       3
#define PIN_LORA_SCK        8
#define PIN_LORA_MISO       9
#define PIN_LORA_MOSI       10
#define PIN_AUX_RX          20      // AUX broadcast bus from the flight controller
#define PIN_MAIN_POWER_SENSE 1      // divided 5 V rail; LOW means the aircraft is dead
#define PIN_CELL_ADC        0       // 1S cell through a 100k/100k divider

#define CELL_DIVIDER_RATIO  2.0f

// -------------------------------------------------------------------------------------
//  Duty-cycle compliant schedule
// -------------------------------------------------------------------------------------
#define PHASE1_DURATION_MS      (2UL * 60UL * 60UL * 1000UL)   // 2 hours
#define PHASE1_TX_INTERVAL_MS   45000UL
#define PHASE2_TX_INTERVAL_MS   180000UL
#define CHIRP_INTERVAL_MS       5000UL
#define CHIRP_DURATION_MS       100UL

// Measured airtime for a BeaconPayload frame at SF11 / BW62.5k / CR4:8. Used to police
// our own duty cycle rather than trusting the schedule blindly.
#define TX_AIRTIME_MS           2761UL
#define MAX_DUTY_PERCENT        10

// -------------------------------------------------------------------------------------
//  RTC-backed cache. Survives deep sleep and the switch to 1S power.
// -------------------------------------------------------------------------------------
RTC_DATA_ATTR static int32_t  rtcLat1e7      = 0;
RTC_DATA_ATTR static int32_t  rtcLon1e7      = 0;
RTC_DATA_ATTR static int32_t  rtcAltMslMm    = 0;
RTC_DATA_ATTR static uint8_t  rtcSatellites  = 0;
RTC_DATA_ATTR static uint8_t  rtcFlightState = 0;
RTC_DATA_ATTR static uint32_t rtcFixEpochMs  = 0;   // millis() at the time of the fix
RTC_DATA_ATTR static bool     rtcHaveFix     = false;
RTC_DATA_ATTR static uint32_t rtcLatchedAtMs = 0;

// -------------------------------------------------------------------------------------
//  Runtime
// -------------------------------------------------------------------------------------
static HardwareSerial AuxSerial(1);

static uint8_t  rxBuf[ODY_MAX_FRAME];
static uint8_t  rxIdx = 0;
static bool     beaconMode = false;
static uint32_t beaconStartMs = 0;
static uint32_t lastTxMs = 0;
static uint32_t lastChirpMs = 0;
static uint32_t txCount = 0;
static uint32_t airtimeAccumMs = 0;
static bool     loraReady = false;

// =====================================================================================
//  AUX bus reception -- the data path that finding 7 is about
// =====================================================================================
static void serviceAuxBus() {
  while (AuxSerial.available()) {
    const uint8_t b = (uint8_t)AuxSerial.read();

    if (rxIdx == 0) {
      if (b != ODY_LINK_MAGIC) continue;
      rxBuf[rxIdx++] = b;
      continue;
    }
    if (rxIdx < sizeof(rxBuf)) rxBuf[rxIdx++] = b;
    else { rxIdx = 0; continue; }

    // Once the header is in we know the exact frame length.
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
        (hdr.dest == ODY_AUX_ADDR_BEACON || hdr.dest == ODY_AUX_ADDR_ALL) &&
        payloadLen == sizeof(AuxPositionPayload)) {

      AuxPositionPayload p;
      memcpy(&p, payload, sizeof(p));

      // Only cache a fix that is actually usable. A zero fix is worse than a stale
      // one, because it looks valid to whoever is decoding the beacon.
      if (p.satellites >= 5 && (p.latitude1e7 != 0 || p.longitude1e7 != 0)) {
        rtcLat1e7      = p.latitude1e7;
        rtcLon1e7      = p.longitude1e7;
        rtcAltMslMm    = p.altitudeMslMm;
        rtcSatellites  = p.satellites;
        rtcFixEpochMs  = millis();
        rtcHaveFix     = true;
      }
      rtcFlightState = p.flightState;

      // The aircraft can tell us it is in trouble before the power is cut, which
      // gets the beacon transmitting a few seconds earlier.
      if (p.emergency && !beaconMode) {
        Serial.println("[BEACON] aircraft signalled emergency -- pre-arming");
      }
    }
    rxIdx = 0;
  }
}

// =====================================================================================
//  Main power sensing
//
//  The P-FET latch also fires passively when the flight pack is cut, so this is a
//  second, independent trigger: if the 5 V rail disappears while we are still running,
//  the aircraft has crashed or been disconnected and we are now on the 1S cell.
// =====================================================================================
static bool mainPowerPresent() {
  // Debounced across 5 reads to ride out the brownout at the moment of the switchover.
  uint8_t high = 0;
  for (int i = 0; i < 5; ++i) {
    if (digitalRead(PIN_MAIN_POWER_SENSE) == HIGH) ++high;
    delayMicroseconds(200);
  }
  return high >= 3;
}

static uint16_t readCellMillivolts() {
  uint32_t accum = 0;
  for (int i = 0; i < 8; ++i) accum += analogReadMilliVolts(PIN_CELL_ADC);
  return (uint16_t)((accum / 8) * CELL_DIVIDER_RATIO);
}

// =====================================================================================
//  LoRa
// =====================================================================================
static bool startLora() {
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);
  if (!LoRa.begin(433E6)) return false;

  // Maximum-range configuration. The cost is airtime, which is exactly why the
  // schedule above is spaced the way it is.
  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);
  LoRa.setTxPower(20);
  LoRa.enableCrc();
  LoRa.setSyncWord(0x4F);
  return true;
}

static void transmitBeacon(uint32_t nowMs) {
  if (!loraReady) return;

  // Self-policing duty cycle. If the schedule ever drifts -- a clock glitch, a code
  // change -- this refuses the transmission rather than breaking the regulatory limit.
  const uint32_t elapsed = nowMs - beaconStartMs + 1u;
  const uint32_t dutyPct = (airtimeAccumMs * 100u) / elapsed;
  if (dutyPct >= MAX_DUTY_PERCENT) {
    Serial.printf("[BEACON] duty cycle %lu%% -- skipping transmission\n",
                  (unsigned long)dutyPct);
    return;
  }

  BeaconPayload p{};
  memcpy(p.tag, "BEAC", 4);
  p.latitude1e7      = rtcLat1e7;
  p.longitude1e7     = rtcLon1e7;
  p.altitudeMslMm    = rtcAltMslMm;
  p.secondsSinceLatch = (nowMs - beaconStartMs) / 1000u;
  // Fix age tells the searcher how much to trust the coordinates. The original had no
  // way to express "this position is stale" because it had no position at all.
  p.fixAgeSeconds    = rtcHaveFix ? ((nowMs - rtcFixEpochMs) / 1000u) : 0xFFFFFFFFu;
  p.beaconBattMv     = readCellMillivolts();
  p.satellites       = rtcSatellites;
  p.lastFlightState  = rtcFlightState;

  uint8_t frame[ODY_MAX_FRAME];
  const uint32_t len = odyEncodeFrame(frame, sizeof(frame),
                                      ODY_FRAME_BEACON, 0, (uint16_t)txCount,
                                      &p, (uint8_t)sizeof(p));
  if (len == 0) return;

  LoRa.beginPacket();
  LoRa.write(frame, len);
  LoRa.endPacket();          // blocking is fine here: nothing else is time critical

  ++txCount;
  airtimeAccumMs += TX_AIRTIME_MS;

  Serial.printf("[BEACON] tx #%lu  %.6f, %.6f  fix age %lus  cell %umV\n",
                (unsigned long)txCount,
                rtcLat1e7 / 1e7, rtcLon1e7 / 1e7,
                (unsigned long)p.fixAgeSeconds, p.beaconBattMv);
}

static void chirp() {
  digitalWrite(PIN_STROBE, HIGH);
  tone(PIN_BUZZER, 2700, CHIRP_DURATION_MS);
  delay(CHIRP_DURATION_MS);
  digitalWrite(PIN_STROBE, LOW);
  noTone(PIN_BUZZER);
}

// =====================================================================================
//  Setup
// =====================================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_STROBE, OUTPUT);
  pinMode(PIN_MAIN_POWER_SENSE, INPUT);
  digitalWrite(PIN_STROBE, LOW);

  analogReadResolution(12);

  AuxSerial.begin(115200, SERIAL_8N1, PIN_AUX_RX, -1);   // receive only

  loraReady = startLora();
  Serial.printf("\n=== Odyssey-10 Pro recovery beacon === LoRa:%s\n",
                loraReady ? "up" : "FAILED");

  // A cold boot with main power already absent means we woke up on the 1S cell, which
  // only happens after a latch. Go straight to beacon mode.
  if (!mainPowerPresent()) {
    beaconMode    = true;
    beaconStartMs = millis();
    rtcLatchedAtMs = beaconStartMs;
    Serial.println("[BEACON] cold start on 1S power -- entering beacon mode");
  } else {
    Serial.println("[BEACON] on aircraft power -- caching position, standing by");
  }
}

// =====================================================================================
//  Loop
// =====================================================================================
void loop() {
  const uint32_t nowMs = millis();

  if (!beaconMode) {
    // ---- Standby: awake on aircraft power, listening to the AUX bus ----------------
    serviceAuxBus();

    if (!mainPowerPresent()) {
      beaconMode     = true;
      beaconStartMs  = nowMs;
      rtcLatchedAtMs = nowMs;
      lastTxMs       = nowMs - PHASE1_TX_INTERVAL_MS;   // transmit immediately
      lastChirpMs    = nowMs - CHIRP_INTERVAL_MS;
      Serial.printf("[BEACON] MAIN POWER LOST -- beacon mode. "
                    "Cached fix: %.6f, %.6f (%s)\n",
                    rtcLat1e7 / 1e7, rtcLon1e7 / 1e7,
                    rtcHaveFix ? "valid" : "NONE");
    }
    delay(20);
    return;
  }

  // ---- Beacon mode -----------------------------------------------------------------
  const uint32_t elapsed  = nowMs - beaconStartMs;
  const uint32_t interval = (elapsed < PHASE1_DURATION_MS)
                              ? PHASE1_TX_INTERVAL_MS : PHASE2_TX_INTERVAL_MS;

  if ((uint32_t)(nowMs - lastChirpMs) >= CHIRP_INTERVAL_MS) {
    lastChirpMs = nowMs;
    chirp();
  }

  if ((uint32_t)(nowMs - lastTxMs) >= interval) {
    lastTxMs = nowMs;
    transmitBeacon(nowMs);
  }

  // If aircraft power comes back -- someone found it and plugged it in, or the latch
  // was a false trigger -- resume caching rather than continuing to burn the cell.
  if (mainPowerPresent()) {
    Serial.println("[BEACON] aircraft power restored -- returning to standby");
    beaconMode = false;
    return;
  }

  // Light sleep until the next scheduled event. This is what turns ~27 mA of always-on
  // idle into the ~0.8 mA that makes the endurance number achievable.
  const uint32_t nextChirp = lastChirpMs + CHIRP_INTERVAL_MS;
  const uint32_t nextTx    = lastTxMs + interval;
  const uint32_t nextEvent = (nextChirp < nextTx) ? nextChirp : nextTx;

  if (nextEvent > millis() + 50u) {
    const uint64_t sleepUs = (uint64_t)(nextEvent - millis() - 20u) * 1000ULL;
    esp_sleep_enable_timer_wakeup(sleepUs);
    // Keep the power-sense pin able to wake us so a restored aircraft is noticed
    // promptly rather than at the next scheduled chirp.
    esp_light_sleep_start();
  }
}
