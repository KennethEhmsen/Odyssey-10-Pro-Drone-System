// =====================================================================================
//  Odyssey-10 Pro -- Radio links
//  ------------------------------------------------------------------------------------
//  FIX FOR FINDING 3.
//
//  The original design had no manual control link at all. RCCommandPacket was only ever
//  *received*; nothing anywhere constructed or transmitted one, and the ground station
//  sketch was receive-only. Since setup() seeded lastRcPacketTime = millis(), the very
//  first thing an armed aircraft did was fire RC LINK LOSS TIMEOUT 1.2 s later.
//
//  It also could not have been fixed by writing a transmitter for the existing radio.
//  A 16-byte LoRa packet at SF7/BW125/CR4-5 costs about 52 ms of airtime; a 20 Hz stick
//  stream needs more than 100% duty cycle, and even 5 Hz would sit at 26% -- far past
//  the 10% ISM limit for 433 MHz in ITU Region 1.
//
//  So the control link is now split by what each medium is actually good at:
//
//    CrsfReceiver  -- ExpressLRS 2.4 GHz over CRSF at 420 kbaud. Stick data at
//                     50-500 Hz with explicit link statistics and a hardware failsafe
//                     flag. This is the manual control path.
//
//    LoraLink      -- 433 MHz SX1278. Telemetry down at 2 Hz, operator commands up on
//                     demand (PERMIT_LAND, ABORT, RTH). This is the long-range
//                     supervisory path and the answer to "the pilot must be able to
//                     approve a landing", which the original could not do either.
//
//  Commands carry a session id, a monotonic sequence number and a CRC (see
//  shared/odyssey_link.h), so a corrupted frame or a replayed capture cannot be
//  mistaken for a valid ABORT.
// =====================================================================================

#ifndef ODY_RADIO_LINK_H
#define ODY_RADIO_LINK_H

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "odyssey_link.h"

// =====================================================================================
//  CRSF (ExpressLRS) receiver
// =====================================================================================
#define CRSF_BAUD                 420000
#define CRSF_SYNC                 0xC8
#define CRSF_FRAME_RC_CHANNELS    0x16
#define CRSF_FRAME_LINK_STATS     0x14
#define CRSF_MAX_FRAME_LEN        64
#define CRSF_CHANNEL_COUNT        16

// Channel assignment, matching the handset model set up in docs section 11.
#define CRSF_CH_ROLL              0
#define CRSF_CH_PITCH             1
#define CRSF_CH_THROTTLE          2
#define CRSF_CH_YAW               3
#define CRSF_CH_ARM               4
#define CRSF_CH_RTH               5

class CrsfReceiver {
public:
  void begin(HardwareSerial& port);
  void service(uint32_t nowMs);

  // Returns false when no valid frame has arrived within CRSF_TIMEOUT_MS, or when the
  // receiver has told us it is in failsafe. Callers must treat false as "no pilot".
  bool getPilotInput(uint32_t nowMs, PilotInput& out) const;

  bool     linkUp(uint32_t nowMs) const;
  uint8_t  linkQuality() const { return linkQuality_; }
  int8_t   rssiDbm()     const { return rssiDbm_; }
  uint32_t framesReceived() const { return frameCount_; }

private:
  void handleFrame(const uint8_t* frame, uint8_t len, uint32_t nowMs);
  static uint8_t crc8Dvb(const uint8_t* data, uint8_t len);

  HardwareSerial* port_ = nullptr;
  uint8_t  buf_[CRSF_MAX_FRAME_LEN];
  uint8_t  idx_        = 0;
  uint8_t  expected_   = 0;
  uint16_t channels_[CRSF_CHANNEL_COUNT] = {0};
  uint32_t lastFrameMs_ = 0;
  uint32_t frameCount_  = 0;
  uint8_t  linkQuality_ = 0;
  int8_t   rssiDbm_     = -128;
  bool     failsafe_    = true;
};

// =====================================================================================
//  LoRa supervisory link
// =====================================================================================
struct ReceivedCommand {
  uint8_t  commandId = ODY_CMD_NONE;
  uint8_t  argument  = 0;
  uint32_t stampMs   = 0;
  bool     valid     = false;
};

class LoraLink {
public:
  bool begin();

  // Non-blocking receive. Validates magic, version, length, CRC, session id and
  // sequence before reporting a command. Returns true only for a command that is
  // new, intact and from the current ground station session.
  bool poll(uint32_t nowMs, ReceivedCommand& out);

  // Sends a telemetry frame. Returns false if the radio is busy or unhealthy.
  bool sendTelemetry(const TelemetryPayload& payload);

  bool     healthy() const { return healthy_; }
  uint32_t lastCommandMs() const { return lastAcceptedMs_; }
  uint32_t rejectedFrames() const { return rejected_; }

  // The ground station's session id, learned from the first accepted command. Echoed
  // in telemetry so the operator can confirm they are talking to their own aircraft.
  uint32_t peerSession() const { return peerSession_; }

private:
  bool     healthy_        = false;
  uint16_t txSeq_          = 0;
  uint16_t lastRxSeq_      = 0;
  bool     haveRxSeq_      = false;
  uint32_t peerSession_    = 0;
  uint32_t lastAcceptedMs_ = 0;
  uint32_t rejected_       = 0;
  uint8_t  rxBuf_[ODY_MAX_FRAME];
};

// =====================================================================================
//  AUX broadcast bus (FC -> beacon node and Remote ID module)
//
//  Transmit only, one wire, addressed frames. This is the data path that finding 7 is
//  about: without it the beacon node has no way to learn where the aircraft is, and it
//  broadcasts latitude 0 / longitude 0 for its entire endurance.
// =====================================================================================
class AuxBroadcast {
public:
  void begin(HardwareSerial& port);
  bool send(uint8_t destAddr, const AuxPositionPayload& payload);
  uint32_t framesSent() const { return sent_; }

private:
  HardwareSerial* port_ = nullptr;
  uint16_t seq_  = 0;
  uint32_t sent_ = 0;
};

extern CrsfReceiver crsf;
extern LoraLink     lora;
extern AuxBroadcast auxBus;

#endif // ODY_RADIO_LINK_H
