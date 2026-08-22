// =====================================================================================
//  Odyssey-10 Pro -- DShot over the ESP32-P4 RMT peripheral
//  ------------------------------------------------------------------------------------
//  THE LINE THIS FILE IS DRAWN ALONG
//
//  §4.3.1 said the driver needs hardware to verify. That is true of the PERIPHERAL, and
//  it is not true of most of the code. So the two are separated here deliberately:
//
//    * everything above the "hardware" divider is integer arithmetic -- choosing a
//      clock resolution, converting a frame into RMT symbols, converting captured
//      symbols back into a bit stream. All of it is host-tested, and it is where the
//      bugs actually live. The GCR ordering defect in dshot.h was exactly this kind.
//
//    * everything below it is ESP-IDF calls, and cannot be compiled here, let alone
//      run. It is written, it is commented with what it assumes, and it is guarded by
//      DSHOT_ENABLE, WHICH STILL DEFAULTS TO 0.
//
//  Nothing in this file has driven a motor. The bring-up procedure in §4.3.1 exists
//  because writing the code is the easy half.
//
//  ------------------------------------------------------------------------------------
//  WHY RMT AND NOT BIT-BANGING
//
//  A DShot300 bit is 3.33 us and the tolerance on the high time is a few percent. A
//  bit-banged frame would have to hold that through 16 bits with interrupts disabled,
//  four times per loop, on a core also running the control law. RMT emits the whole
//  frame from a symbol list in hardware once started, so jitter in when the frame BEGINS
//  costs nothing, and jitter inside the frame does not exist.
//
//  ------------------------------------------------------------------------------------
//  BIDIRECTIONAL DSHOT INVERTS THE LINE
//
//  Ordinary DShot idles low and pulses high. Bidirectional idles HIGH and pulses low, so
//  that the ESC can pull the line down to reply without the two fighting. Getting this
//  backwards produces a signal an ESC will simply ignore -- no motion, no error, and
//  nothing on the wire that looks obviously wrong at a glance.
// =====================================================================================

#ifndef ODY_DSHOT_RMT_H
#define ODY_DSHOT_RMT_H

#include <Arduino.h>
#include "config.h"
#include "dshot.h"

// -------------------------------------------------------------------------------------
//  Timing arithmetic  -- pure, and host-tested
// -------------------------------------------------------------------------------------

// 10 MHz gives 0.1 us per tick. It is chosen because 80 MHz divides by 8 exactly, so the
// divider is integral on every clock source the P4 offers, and because a DShot300 bit
// then lands within 1% of nominal -- comfortably inside the tolerance an ESC allows.
// A "nicer" 12 MHz would make the bit exactly 40 ticks but needs a fractional divider.
#ifndef DSHOT_RMT_RESOLUTION_HZ
#define DSHOT_RMT_RESOLUTION_HZ   10000000u
#endif

/** Ticks in one bit period, rounded to nearest. */
static inline uint32_t dshotBitTicks(uint32_t resolutionHz, uint32_t bitrateKhz) {
    const uint32_t bps = bitrateKhz * 1000u;
    return (resolutionHz + bps / 2u) / bps;
}

/** High time for a 1 bit: 75% of the period. */
static inline uint32_t dshotOneHighTicks(uint32_t bitTicks) {
    return (bitTicks * 3u + 2u) / 4u;
}

/** High time for a 0 bit: 37.5% of the period. */
static inline uint32_t dshotZeroHighTicks(uint32_t bitTicks) {
    return (bitTicks * 3u + 4u) / 8u;
}

/**
 * How far the realised bit period sits from nominal, in parts per million.
 *
 * This is the number that decides whether a chosen resolution is usable at all. An ESC
 * auto-detects the bitrate from the frame, so a consistent error is tolerated far better
 * than an inconsistent one -- but past a few percent the detection itself fails.
 */
static inline uint32_t dshotBitErrorPpm(uint32_t resolutionHz, uint32_t bitrateKhz) {
    const uint32_t ticks = dshotBitTicks(resolutionHz, bitrateKhz);
    const uint64_t realisedPs = (uint64_t)ticks * 1000000000000ull / resolutionHz;
    const uint64_t nominalPs  = 1000000000ull / bitrateKhz;
    const uint64_t diff = realisedPs > nominalPs ? realisedPs - nominalPs
                                                 : nominalPs - realisedPs;
    return (uint32_t)(diff * 1000000ull / nominalPs);
}

// -------------------------------------------------------------------------------------
//  Symbols
//
//  An RMT symbol is a pair of (level, duration) halves. One DShot bit is one symbol: a
//  high half and a low half that together make the bit period. Bidirectional flips both
//  levels, which is the whole of the inversion.
// -------------------------------------------------------------------------------------
struct DShotRmtSymbol {
    uint16_t firstTicks;    // duration of the first half
    uint8_t  firstLevel;    // its level, 0 or 1
    uint16_t secondTicks;
    uint8_t  secondLevel;
};

#define DSHOT_FRAME_BITS   16
#define DSHOT_TELEM_BITS   21

/**
 * Expands a frame into 16 symbols, MSB first.
 *
 * @param frame          the 16-bit DShot frame
 * @param bitTicks       ticks per bit, from dshotBitTicks()
 * @param inverted       true for bidirectional DShot: idle high, pulse low
 * @param out            receives DSHOT_FRAME_BITS symbols
 */
static inline void dshotBuildSymbols(uint16_t frame, uint32_t bitTicks, bool inverted,
                                     DShotRmtSymbol out[DSHOT_FRAME_BITS]) {
    const uint32_t oneHigh  = dshotOneHighTicks(bitTicks);
    const uint32_t zeroHigh = dshotZeroHighTicks(bitTicks);

    for (int i = 0; i < DSHOT_FRAME_BITS; ++i) {
        // MSB first. Sending these the other way round is a frame the ESC will reject
        // on checksum, which is at least a loud failure rather than a quiet one.
        const bool bit = (frame >> (DSHOT_FRAME_BITS - 1 - i)) & 1u;
        const uint32_t active = bit ? oneHigh : zeroHigh;

        out[i].firstTicks  = (uint16_t)active;
        out[i].firstLevel  = inverted ? 0u : 1u;
        out[i].secondTicks = (uint16_t)(bitTicks - active);
        out[i].secondLevel = inverted ? 1u : 0u;
    }
}

// -------------------------------------------------------------------------------------
//  Decoding the reply
//
//  The ESC answers on the same wire at 5/4 of the outgoing bitrate. RMT hands back
//  durations at alternating levels rather than bits, so the durations have to be
//  divided by the reply's bit period to recover how many identical bits each run held.
//
//  This is where a receiver usually goes wrong, and it is entirely testable.
// -------------------------------------------------------------------------------------

/** Ticks per bit of the ESC's reply: it runs at 5/4 the outgoing rate, so 4/5 the period. */
static inline uint32_t dshotTelemBitTicks(uint32_t bitTicks) {
    return (bitTicks * 4u + 2u) / 5u;
}

/**
 * Turns captured run-lengths into the 21-bit reply.
 *
 * @param runTicks   duration of each run, in ticks
 * @param runLevels  the level held during each run
 * @param runCount   how many runs were captured
 * @param telemTicks ticks per reply bit, from dshotTelemBitTicks()
 * @return the 21-bit value, or 0 if the capture could not be made sense of
 *
 * The ESC's clock is not ours, so run lengths are rounded rather than assumed exact. A
 * run that rounds to zero bits is a glitch; a total that is not 21 bits means the
 * capture was truncated or noisy, and both are rejected rather than padded -- a padded
 * reply decodes to a plausible RPM, which is far worse than no reply at all.
 */
static inline uint32_t dshotDecodeRuns(const uint32_t* runTicks, const uint8_t* runLevels,
                                       int runCount, uint32_t telemTicks) {
    if (runCount <= 0 || telemTicks == 0) return 0;

    uint32_t value = 0;
    int bitsSeen = 0;

    for (int i = 0; i < runCount; ++i) {
        // Round to the nearest whole number of bit periods.
        const uint32_t bits = (runTicks[i] + telemTicks / 2u) / telemTicks;
        if (bits == 0) return 0;                       // a glitch, not a symbol
        if (bitsSeen + (int)bits > DSHOT_TELEM_BITS) return 0;   // overran

        for (uint32_t b = 0; b < bits; ++b) {
            value = (value << 1) | (runLevels[i] ? 1u : 0u);
            ++bitsSeen;
        }
    }

    // The reply always ends with the line returning to idle, and the capture may stop
    // as soon as it does. Anything short of a full 21 bits is padded with the idle
    // level ONLY when the shortfall is the trailing idle itself.
    if (bitsSeen < DSHOT_TELEM_BITS) {
        const int missing = DSHOT_TELEM_BITS - bitsSeen;
        if (missing > 3) return 0;                     // too much is missing to trust
        value = (value << missing) | ((1u << missing) - 1u);
    }
    return value & 0x1FFFFF;
}

/**
 * The whole reply path in one call: runs in, eRPM out.
 *
 * @return eRPM, 0 if stopped, or DSHOT_ERPM_INVALID if anything did not add up
 */
static inline uint32_t dshotErpmFromRuns(const uint32_t* runTicks,
                                         const uint8_t* runLevels, int runCount,
                                         uint32_t telemTicks) {
    const uint32_t raw = dshotDecodeRuns(runTicks, runLevels, runCount, telemTicks);
    if (raw == 0) return DSHOT_ERPM_INVALID;
    const uint16_t word = dshotDecodeGcr21(raw);
    if (word == 0xFFFF) return DSHOT_ERPM_INVALID;
    return dshotErpm(word);
}

// =====================================================================================
//  HARDWARE  --  below this line nothing has been compiled, let alone run
// =====================================================================================
#if DSHOT_ENABLE

/**
 * Four DShot channels on the RMT peripheral.
 *
 * ASSUMPTIONS, every one of which the bring-up in §4.3.1 has to confirm:
 *
 *   * the P4 has at least 4 RMT TX channels free after LED and IR uses
 *   * rmt_new_tx_channel() accepts DSHOT_RMT_RESOLUTION_HZ on the default clock source
 *   * the copy encoder emits symbols back-to-back with no inter-symbol gap
 *   * a channel can be reconfigured to RX and back inside the ~30 us turnaround
 *
 * The last one is the doubtful one. If the RMT driver cannot turn a channel around that
 * fast, bidirectional telemetry needs either a second channel bound to the same pin or a
 * different capture mechanism entirely, and this class needs restructuring rather than
 * tuning. That is a hardware question and it is not answerable here.
 */
class DShotRmt {
public:
    bool begin(const uint8_t pins[4]);

    /** Queues one frame per motor. Non-blocking; the peripheral emits them. */
    void write(const uint16_t values[4], bool requestTelemetry);

    /** Collects whatever replies arrived since the last write(). */
    void pollTelemetry(DShotTelemetry& sink, uint32_t nowMs);

    /** Sends a DShot command to every motor, repeated as the protocol requires. */
    void sendCommand(uint16_t command, int repeats = 10);

    bool ready() const { return ready_; }

private:
    bool ready_ = false;
    uint8_t pins_[4] = { 0, 0, 0, 0 };
    DShotRmtSymbol symbols_[4][DSHOT_FRAME_BITS];
};

extern DShotRmt dshotRmt;

#endif // DSHOT_ENABLE

#endif // ODY_DSHOT_RMT_H
