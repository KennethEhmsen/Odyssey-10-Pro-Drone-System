// =====================================================================================
//  Odyssey-10 Pro -- Radio link implementation
// =====================================================================================

#include "radio_link.h"
#include <SPI.h>
#include <LoRa.h>

CrsfReceiver crsf;
LoraLink     lora;
AuxBroadcast auxBus;

// =====================================================================================
//  CRSF
// =====================================================================================

// CRSF uses CRC-8 with polynomial 0xD5 (DVB-S2), initial value 0.
uint8_t CrsfReceiver::crc8Dvb(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void CrsfReceiver::begin(HardwareSerial& port) {
  port_ = &port;
  port_->begin(CRSF_BAUD, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  idx_ = 0;
  expected_ = 0;
  failsafe_ = true;
}

void CrsfReceiver::service(uint32_t nowMs) {
  if (!port_) return;

  while (port_->available()) {
    const uint8_t b = (uint8_t)port_->read();

    if (idx_ == 0) {
      if (b != CRSF_SYNC) continue;            // hunt for the sync byte
      buf_[idx_++] = b;
      continue;
    }
    if (idx_ == 1) {
      // Length byte counts type + payload + CRC.
      if (b < 2 || b > CRSF_MAX_FRAME_LEN - 2) { idx_ = 0; continue; }
      expected_ = b;
      buf_[idx_++] = b;
      continue;
    }

    buf_[idx_++] = b;
    if (idx_ < expected_ + 2) continue;

    // Frame complete. CRC covers type through the last payload byte.
    const uint8_t crcGiven = buf_[expected_ + 1];
    if (crc8Dvb(&buf_[2], expected_ - 1) == crcGiven) {
      handleFrame(&buf_[2], (uint8_t)(expected_ - 1), nowMs);
    }
    idx_ = 0;
  }
}

void CrsfReceiver::handleFrame(const uint8_t* frame, uint8_t len, uint32_t nowMs) {
  const uint8_t type = frame[0];

  if (type == CRSF_FRAME_RC_CHANNELS && len >= 23) {
    // 16 channels packed into 22 bytes, 11 bits each, little-endian bit order.
    const uint8_t* p = &frame[1];
    uint32_t bits = 0;
    uint8_t  bitsAvailable = 0;
    uint8_t  byteIndex = 0;
    for (int ch = 0; ch < CRSF_CHANNEL_COUNT; ++ch) {
      while (bitsAvailable < 11) {
        bits |= (uint32_t)p[byteIndex++] << bitsAvailable;
        bitsAvailable += 8;
      }
      channels_[ch] = (uint16_t)(bits & 0x7FF);
      bits >>= 11;
      bitsAvailable -= 11;
    }
    lastFrameMs_ = nowMs;
    failsafe_    = false;
    ++frameCount_;

  } else if (type == CRSF_FRAME_LINK_STATS && len >= 11) {
    // Byte layout: [1] uplink RSSI ant1, [2] ant2, [3] uplink LQ, [4] uplink SNR ...
    rssiDbm_     = -(int8_t)frame[1];
    linkQuality_ = frame[3];
    // ExpressLRS reports LQ 0 when the link is genuinely gone.
    if (linkQuality_ == 0) failsafe_ = true;
  }
}

bool CrsfReceiver::linkUp(uint32_t nowMs) const {
  return !failsafe_
      && lastFrameMs_ != 0
      && (uint32_t)(nowMs - lastFrameMs_) <= CRSF_TIMEOUT_MS;
}

bool CrsfReceiver::getPilotInput(uint32_t nowMs, PilotInput& out) const {
  if (!linkUp(nowMs)) return false;

  // CRSF channel units: 172 = -100%, 992 = centre, 1811 = +100%.
  auto toUs = [](uint16_t v) -> uint16_t {
    const int32_t us = ((int32_t)v * 1024) / 1639 + 881;
    return (uint16_t)constrain(us, 988, 2012);
  };
  auto toNorm = [](uint16_t v) -> float {
    return constrain(((float)v - 992.0f) / 819.0f, -1.0f, 1.0f);
  };

  out.throttleUs  = toUs(channels_[CRSF_CH_THROTTLE]);
  out.rollNorm    = toNorm(channels_[CRSF_CH_ROLL]);
  out.pitchNorm   = toNorm(channels_[CRSF_CH_PITCH]);
  out.yawNorm     = toNorm(channels_[CRSF_CH_YAW]);
  out.armSwitch   = channels_[CRSF_CH_ARM] > 1400;
  out.rthSwitch   = channels_[CRSF_CH_RTH] > 1400;
  out.linkQuality = linkQuality_;
  out.stampMs     = lastFrameMs_;
  return true;
}

// =====================================================================================
//  LoRa supervisory link
// =====================================================================================
bool LoraLink::begin() {
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    healthy_ = false;
    Serial.println("[LORA] radio did not respond");
    return false;
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(20);
  LoRa.enableCrc();                 // hardware CRC on top of our own
  LoRa.setSyncWord(0x4F);           // keeps foreign 433 MHz traffic out of the parser

  healthy_ = true;
  Serial.println("[LORA] 433 MHz supervisory link up");
  return true;
}

bool LoraLink::poll(uint32_t nowMs, ReceivedCommand& out) {
  out = ReceivedCommand{};
  if (!healthy_) return false;

  const int size = LoRa.parsePacket();
  if (size <= 0) return false;
  if (size > (int)sizeof(rxBuf_)) { ++rejected_; return false; }

  int n = 0;
  while (LoRa.available() && n < size) rxBuf_[n++] = (uint8_t)LoRa.read();

  OdyFrameHeader hdr;
  const uint8_t* payload = nullptr;
  uint8_t payloadLen = 0;
  if (!odyDecodeFrame(rxBuf_, (uint32_t)n, &hdr, &payload, &payloadLen)) {
    ++rejected_;
    return false;
  }
  if (hdr.type != ODY_FRAME_COMMAND || payloadLen != sizeof(CommandPayload)) {
    ++rejected_;
    return false;
  }

  CommandPayload cmd;
  memcpy(&cmd, payload, sizeof(cmd));

  // Bind to the first ground station we hear, then reject anything else. A second
  // transmitter cannot take over the aircraft mid-flight without a power cycle.
  if (peerSession_ == 0) {
    peerSession_ = cmd.sessionId;
    Serial.printf("[LORA] bound to ground station session %08lX\n",
                  (unsigned long)peerSession_);
  } else if (cmd.sessionId != peerSession_) {
    ++rejected_;
    return false;
  }

  // Replay protection: the sequence must strictly advance.
  if (haveRxSeq_ && !odySeqIsNewer(hdr.seq, lastRxSeq_)) {
    ++rejected_;
    return false;
  }
  lastRxSeq_ = hdr.seq;
  haveRxSeq_ = true;

  out.commandId = cmd.commandId;
  out.argument  = cmd.argument;
  out.stampMs   = nowMs;
  out.valid     = true;
  lastAcceptedMs_ = nowMs;
  return true;
}

bool LoraLink::sendTelemetry(const TelemetryPayload& payload) {
  if (!healthy_) return false;

  uint8_t frame[ODY_MAX_FRAME];
  const uint32_t len = odyEncodeFrame(frame, sizeof(frame),
                                      ODY_FRAME_TELEMETRY, 0, txSeq_++,
                                      &payload, (uint8_t)sizeof(payload));
  if (len == 0) return false;

  // beginPacket() returns 0 when the radio is still transmitting. Dropping this
  // telemetry frame is correct: blocking here would stall the telemetry task.
  if (!LoRa.beginPacket()) return false;
  LoRa.write(frame, len);
  LoRa.endPacket(true);          // async: returns immediately, TX completes in the background
  return true;
}

// =====================================================================================
//  AUX broadcast bus
// =====================================================================================
void AuxBroadcast::begin(HardwareSerial& port) {
  port_ = &port;
  // TX-only: passing -1 for RX leaves the pin free and makes it structurally
  // impossible for a faulty aux module to feed data back into the flight controller.
  port_->begin(115200, SERIAL_8N1, -1, PIN_AUX_BUS_TX);
}

bool AuxBroadcast::send(uint8_t destAddr, const AuxPositionPayload& payload) {
  if (!port_) return false;

  uint8_t frame[ODY_MAX_FRAME];
  const uint32_t len = odyEncodeFrame(frame, sizeof(frame),
                                      ODY_FRAME_AUX_POS, destAddr, seq_++,
                                      &payload, (uint8_t)sizeof(payload));
  if (len == 0) return false;

  // 115200 baud gives ~11.5 kB/s; a 50-byte frame is 4.3 ms of wire time. The write
  // is buffered, so this does not block the telemetry task.
  port_->write(frame, len);
  ++sent_;
  return true;
}
