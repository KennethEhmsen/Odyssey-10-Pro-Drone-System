// =====================================================================================
//  Odyssey-10 Pro -- DShot, and bidirectional DShot telemetry
//  ------------------------------------------------------------------------------------
//  WHY THIS EXISTS
//
//  §4.3 has recorded bidirectional DShot as the highest-value change to the propulsion
//  stack since revision 2.0, for one reason: it returns per-motor RPM. Shaft frequency
//  is then KNOWN rather than inferred.
//
//  Everything the gyro notch does today is an attempt to recover that number from the
//  noise it produces. A sliding DFT hunts for a peak, gates it on height and stability,
//  clamps it to a band, and slews toward it -- an elaborate apparatus whose entire job
//  is to estimate a quantity the ESC already measures. RPM telemetry replaces the
//  estimate with the measurement, and it does so for each motor separately rather than
//  for the four of them blurred together.
//
//  It also answers §8.3.3. The second harmonic cannot be SEEN on most builds because
//  2*f0 lands above the IMU's anti-alias corner, but it does not need to be seen if the
//  fundamental is known: the harmonic is at a known multiple of a known number.
//
//  ------------------------------------------------------------------------------------
//  WHAT IS AND IS NOT VERIFIED HERE
//
//  Everything in this header is pure arithmetic on integers, and it is covered by host
//  tests against the published frame format: encoding, the CRC in both its normal and
//  inverted forms, the GCR telemetry decode, and the eRPM-to-Hz conversion.
//
//  The part that CANNOT be verified without hardware is the timing -- driving the
//  ESP32-P4's RMT peripheral to within a fraction of a microsecond, and turning the line
//  around fast enough to catch the ESC's reply. That code is in dshot_rmt.cpp, it is
//  behind DSHOT_ENABLE, and DSHOT_ENABLE DEFAULTS TO 0.
//
//  That default is deliberate. This code drives motors. Untested timing that misses a
//  bit boundary does not fail politely -- a corrupted frame is a throttle value the ESC
//  will happily act on. Analog PWM stays the default until someone has watched this on
//  a scope and then on a thrust stand with the propellers off.
// =====================================================================================

#ifndef ODY_DSHOT_H
#define ODY_DSHOT_H

#include <Arduino.h>
#include "config.h"

// -------------------------------------------------------------------------------------
//  Frame format
//
//      bit  15..5   11-bit value
//      bit      4   telemetry request
//      bit   3..0   CRC
//
//  Values 0..47 are commands; 48..2047 are throttle. That leaves 2000 usable throttle
//  steps against the 1000 that 1000-2000 us PWM offers, which is the lesser reason to
//  want DShot and the one most often quoted.
// -------------------------------------------------------------------------------------
#define DSHOT_CMD_DISARM              0
#define DSHOT_CMD_BEEP1               1
#define DSHOT_CMD_ESC_INFO            6
#define DSHOT_CMD_SPIN_DIRECTION_1    7
#define DSHOT_CMD_SPIN_DIRECTION_2    8
#define DSHOT_CMD_SAVE_SETTINGS      12
#define DSHOT_CMD_EDT_ENABLE         13
#define DSHOT_CMD_SPIN_NORMAL        20
#define DSHOT_CMD_SPIN_REVERSED      21

#define DSHOT_MIN_THROTTLE           48
#define DSHOT_MAX_THROTTLE         2047

/** True for values the ESC treats as throttle rather than as a command. */
static inline bool dshotIsThrottle(uint16_t value) {
    return value >= DSHOT_MIN_THROTTLE && value <= DSHOT_MAX_THROTTLE;
}

/**
 * The 4-bit checksum, over the 12-bit value-plus-telemetry field.
 *
 * Bidirectional DShot INVERTS it. That is not decoration: inverting the checksum is how
 * an ESC distinguishes a frame it should reply to from an ordinary one, so getting this
 * backwards does not corrupt the frame, it silently disables telemetry while everything
 * still appears to fly.
 */
static inline uint8_t dshotChecksum(uint16_t payload12, bool bidirectional) {
    const uint16_t x = payload12 & 0x0FFF;
    uint8_t crc = (uint8_t)((x ^ (x >> 4) ^ (x >> 8)) & 0x0F);
    if (bidirectional) crc = (uint8_t)((~crc) & 0x0F);
    return crc;
}

/**
 * Builds a complete 16-bit frame.
 *
 * @param value          0..2047, command or throttle
 * @param requestTelem   set the telemetry-request bit (the ESC replies on its own wire)
 * @param bidirectional  invert the checksum, as bidirectional DShot requires
 */
static inline uint16_t dshotFrame(uint16_t value, bool requestTelem, bool bidirectional) {
    const uint16_t payload = (uint16_t)(((value & 0x07FF) << 1) | (requestTelem ? 1 : 0));
    return (uint16_t)((payload << 4) | dshotChecksum(payload, bidirectional));
}

/** Recovers the 11-bit value from a frame, without validating it. */
static inline uint16_t dshotValueOf(uint16_t frame) { return (frame >> 5) & 0x07FF; }

/** True if the frame's checksum is self-consistent. */
static inline bool dshotFrameValid(uint16_t frame, bool bidirectional) {
    const uint16_t payload = (frame >> 4) & 0x0FFF;
    return (frame & 0x0F) == dshotChecksum(payload, bidirectional);
}

/**
 * Maps the mixer's PWM-domain output onto DShot's throttle range.
 *
 * The mixer still works in microseconds because that is what §4.2's desaturation
 * arithmetic is written in, and changing that would mean re-deriving the mixer to gain
 * nothing. Anything at or below PWM_MIN is a disarm rather than the lowest throttle:
 * DShot value 0 means "stop", and 48 means "the slowest speed you can hold", which are
 * different requests and must not be conflated.
 */
static inline uint16_t dshotFromPwmMicros(uint16_t pwmUs) {
    if (pwmUs <= PWM_MIN) return DSHOT_CMD_DISARM;
    if (pwmUs >= PWM_MAX) return DSHOT_MAX_THROTTLE;
    const uint32_t span = (uint32_t)(PWM_MAX - PWM_MIN);
    const uint32_t step = (uint32_t)(pwmUs - PWM_MIN);
    const uint32_t range = DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE;
    return (uint16_t)(DSHOT_MIN_THROTTLE + (step * range + span / 2) / span);
}

// -------------------------------------------------------------------------------------
//  Return telemetry
//
//  The ESC replies with a 21-bit GCR-encoded burst. Decoded, it is 16 bits:
//
//      eee mmmmmmmmm cccc      exponent, mantissa, checksum
//
//  and the period between commutations is  mantissa << exponent  microseconds.
//
//  Note that this is the ELECTRICAL period. Converting to a shaft rate needs the pole
//  count, and getting that wrong scales every derived frequency by a constant -- which
//  looks exactly like a mis-tuned notch rather than like a units error.
// -------------------------------------------------------------------------------------
#define DSHOT_ERPM_INVALID      0xFFFFFFFFu
#define DSHOT_ERPM_NOT_SPINNING 0x0FFFu      // the "period is effectively infinite" code

/** GCR: each 5-bit group maps back to 4 bits. Invalid groups decode to 0xFF. */
static inline uint8_t dshotGcrDecodeNibble(uint8_t quintet) {
    switch (quintet & 0x1F) {
        case 0x19: return 0x00;  case 0x1B: return 0x01;
        case 0x12: return 0x02;  case 0x13: return 0x03;
        case 0x1D: return 0x04;  case 0x15: return 0x05;
        case 0x16: return 0x06;  case 0x17: return 0x07;
        case 0x1A: return 0x08;  case 0x09: return 0x09;
        case 0x0A: return 0x0A;  case 0x0B: return 0x0B;
        case 0x1E: return 0x0C;  case 0x0D: return 0x0D;
        case 0x0E: return 0x0E;  case 0x0F: return 0x0F;
        default:   return 0xFF;  // not a legal group
    }
}

/**
 * Undoes the line coding on a 21-bit reply and returns the 16-bit telemetry word.
 *
 * The wire carries the value XORed with itself shifted right one bit, so the first step
 * is to undo that before the GCR groups mean anything.
 *
 * @return the 16-bit word, or 0xFFFF if any group was illegal
 */
static inline uint16_t dshotDecodeGcr21(uint32_t raw21) {
    uint32_t value = raw21 & 0x1FFFFF;
    value = value ^ (value >> 1);          // undo the GCR pre-scramble

    // Four 5-bit groups, LOW group first: quintet at bits 0-4 carries the low nibble.
    // Reading them the other way round decodes to a legal-looking but wrong value,
    // which is the worst kind of wrong for an RPM feeding a notch filter.
    uint16_t out = 0;
    for (int i = 0; i < 4; ++i) {
        const uint8_t quintet = (uint8_t)((value >> (i * 5)) & 0x1F);
        const uint8_t nibble = dshotGcrDecodeNibble(quintet);
        if (nibble == 0xFF) return 0xFFFF;
        out |= (uint16_t)nibble << (i * 4);
    }
    return out;
}

/** The telemetry word carries its own 4-bit checksum, computed the same way. */
static inline bool dshotTelemetryValid(uint16_t word) {
    const uint16_t payload = (word >> 4) & 0x0FFF;
    const uint8_t crc = (uint8_t)((~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0x0F);
    return (word & 0x0F) == crc;
}

/**
 * Electrical RPM from a validated telemetry word.
 *
 * @return eRPM, 0 if the motor is stopped, or DSHOT_ERPM_INVALID if the word is bad
 */
static inline uint32_t dshotErpm(uint16_t word) {
    if (!dshotTelemetryValid(word)) return DSHOT_ERPM_INVALID;
    const uint16_t period12 = (word >> 4) & 0x0FFF;
    if (period12 == DSHOT_ERPM_NOT_SPINNING) return 0;

    const uint32_t mantissa = period12 & 0x1FF;
    const uint32_t exponent = (period12 >> 9) & 0x07;
    const uint32_t periodUs = mantissa << exponent;
    if (periodUs == 0) return DSHOT_ERPM_INVALID;

    return 60000000u / periodUs;          // 60e6 us per minute
}

/** Shaft RPM. eRPM counts electrical revolutions, of which there are one per pole pair. */
static inline float dshotShaftRpm(uint32_t erpm) {
    if (erpm == DSHOT_ERPM_INVALID) return 0.0f;
    return (float)erpm / (float)MOTOR_POLE_PAIRS;
}

/**
 * Shaft frequency in Hz -- the number the gyro notch spends a sliding DFT trying to find.
 *
 * This is the whole point of the file. §8.3 estimates it from two models that disagree
 * by 7%; §8.3.1 measures it from the noise floor with a bounded peak tracker. This
 * computes it in one division from a value the ESC already knew.
 */
static inline float dshotShaftHz(uint32_t erpm) { return dshotShaftRpm(erpm) / 60.0f; }

// -------------------------------------------------------------------------------------
//  Turning four motors' telemetry into one notch frequency
//
//  The four shafts do not turn at the same speed. In a hover they are close; under a
//  roll command one diagonal pair speeds up while the other slows, and the spread can
//  be large. So a single notch derived from RPM is only honest while the motors AGREE.
//
//  Betaflight solves this with a filter per motor per axis -- twelve, plus harmonics.
//  That is the right answer and it is not free. What this does instead is narrower and
//  states its own limits: report the mean, report the spread, and declare the reading
//  COHERENT only when the spread is small enough that one notch covers all four. When
//  it is not, the caller falls back to the sliding DFT, which sees the blend of all
//  four shafts anyway and does not care that they differ.
//
//  This is the part of DShot that pays. It converts the notch frequency from something
//  estimated by two models that disagree by 7% into something read off the ESCs.
// -------------------------------------------------------------------------------------
#define DSHOT_TELEM_MAX_SPREAD    0.08f    // 8% between fastest and slowest motor
#define DSHOT_TELEM_STALE_MS      100u     // older than this and the reading is not used

struct DShotTelemetryState {
    float shaftHz[4]   = { 0, 0, 0, 0 };
    float meanHz       = 0.0f;
    float spread       = 0.0f;   // (max-min)/mean
    uint8_t validCount = 0;
    bool  coherent     = false;  // one notch can cover all four
};

class DShotTelemetry {
public:
    /** Feeds one motor's decoded telemetry word. */
    void ingest(uint8_t motor, uint16_t word, uint32_t nowMs) {
        if (motor >= 4) return;
        const uint32_t erpm = dshotErpm(word);
        if (erpm == DSHOT_ERPM_INVALID) return;    // keep the previous, let it age
        hz_[motor] = dshotShaftHz(erpm);
        stamp_[motor] = nowMs;
    }

    /** Recomputes the aggregate. Call once per loop, after ingesting all four. */
    void update(uint32_t nowMs) {
        float sum = 0.0f, lo = 1e9f, hi = 0.0f;
        uint8_t n = 0;
        for (int i = 0; i < 4; ++i) {
            // A motor that has stopped replying must not keep contributing the last
            // value it sent. This is finding 15 again, in a different subsystem.
            if (stamp_[i] == 0 || (uint32_t)(nowMs - stamp_[i]) > DSHOT_TELEM_STALE_MS) {
                state_.shaftHz[i] = 0.0f;
                continue;
            }
            state_.shaftHz[i] = hz_[i];
            sum += hz_[i];
            if (hz_[i] < lo) lo = hz_[i];
            if (hz_[i] > hi) hi = hz_[i];
            ++n;
        }

        state_.validCount = n;
        // Fewer than all four and the mean is not describing the aircraft any more.
        if (n < 4 || sum <= 0.0f) {
            state_.meanHz = 0.0f;
            state_.spread = 0.0f;
            state_.coherent = false;
            return;
        }

        state_.meanHz = sum / (float)n;
        state_.spread = (state_.meanHz > 0.0f) ? (hi - lo) / state_.meanHz : 0.0f;
        state_.coherent = state_.spread <= DSHOT_TELEM_MAX_SPREAD;
    }

    const DShotTelemetryState& state() const { return state_; }

    /** The measured notch frequency, or 0 when the motors do not agree closely enough. */
    float notchHz() const { return state_.coherent ? state_.meanHz : 0.0f; }

    void reset() {
        for (int i = 0; i < 4; ++i) { hz_[i] = 0.0f; stamp_[i] = 0; }
        state_ = DShotTelemetryState{};
    }

private:
    float    hz_[4]    = { 0, 0, 0, 0 };
    uint32_t stamp_[4] = { 0, 0, 0, 0 };
    DShotTelemetryState state_;
};

#endif // ODY_DSHOT_H
