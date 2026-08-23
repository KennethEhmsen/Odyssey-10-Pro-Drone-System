// =====================================================================================
//  Odyssey-10 Pro -- DShot over RMT, the hardware half
//  ------------------------------------------------------------------------------------
//  READ THIS BEFORE ENABLING IT
//
//  Nothing in this file has been compiled. There is no ESP-IDF in the environment it was
//  written in, so it has not been through a compiler, let alone onto a board, let alone
//  onto a board with motors attached. The arithmetic it depends on IS tested -- see
//  dshot_rmt.h and the 40-odd assertions covering bit timing, symbol construction and
//  reply decoding -- but everything below is ESP-IDF API usage and peripheral behaviour,
//  and both are assumptions until someone checks them.
//
//  The whole file is inside `#if DSHOT_ENABLE`, which defaults to 0. With it off this
//  translation unit is empty and the aircraft flies on analog PWM exactly as before.
//
//  §4.3.1 has the bring-up procedure. The short version: scope first, then a thrust
//  stand with the PROPELLERS OFF, and only then anything that could spin a blade.
// =====================================================================================

#include "config.h"

#if DSHOT_ENABLE

#include "dshot_rmt.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"
#include "soc/soc_caps.h"
#include "esp_log.h"

static const char* TAG = "dshot";

DShotRmt dshotRmt;

// -------------------------------------------------------------------------------------
//  Channel state
//
//  TX and RX are separate channels bound to the SAME GPIO. That is the arrangement
//  bidirectional DShot needs and it is ASSUMPTION NUMBER ONE: the IDF driver may refuse
//  two channels on one pin, in which case this needs restructuring rather than tuning.
//  It is the first thing to find out on real hardware, and it is why begin() reports
//  every failure individually instead of returning a single bool.
// -------------------------------------------------------------------------------------
namespace {

struct Channel {
    rmt_channel_handle_t tx = nullptr;
    rmt_channel_handle_t rx = nullptr;
    rmt_symbol_word_t    words[DSHOT_FRAME_BITS];
    rmt_symbol_word_t    rxBuf[64];
    volatile size_t      rxCount = 0;
    volatile bool        rxDone = false;
};

Channel g_ch[4];
rmt_encoder_handle_t g_copy = nullptr;

uint32_t g_bitTicks   = 0;
uint32_t g_telemTicks = 0;

/**
 * RX completion callback. Runs in ISR context, so it does nothing but record.
 *
 * ASSUMPTION TWO: that the callback signature and the ISR-context restriction are as
 * documented for the P4's RMT driver. Allocating, logging or blocking here would fault.
 */
bool IRAM_ATTR onRxDone(rmt_channel_handle_t, const rmt_rx_done_event_data_t* ev,
                        void* user) {
    Channel* c = static_cast<Channel*>(user);
    c->rxCount = ev->num_symbols;
    c->rxDone = true;
    return false;      // no task woken
}

} // namespace

// -------------------------------------------------------------------------------------
bool DShotRmt::begin(const uint8_t pins[4]) {
    g_bitTicks   = dshotBitTicks(DSHOT_RMT_RESOLUTION_HZ, DSHOT_BITRATE_KHZ);
    g_telemTicks = dshotTelemBitTicks(g_bitTicks);

    // A copy encoder emits a symbol list verbatim. ASSUMPTION THREE is that it does so
    // back to back with no inter-symbol gap -- a gap inside a DShot frame is a bit
    // boundary missed, and the ESC would reject the frame on checksum.
    rmt_copy_encoder_config_t enc = {};
    if (rmt_new_copy_encoder(&enc, &g_copy) != ESP_OK) {
        ESP_LOGE(TAG, "copy encoder allocation failed");
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        pins_[i] = pins[i];

        rmt_tx_channel_config_t txc = {};
        txc.gpio_num          = (gpio_num_t)pins[i];
        txc.clk_src           = RMT_CLK_SRC_DEFAULT;
        txc.resolution_hz     = DSHOT_RMT_RESOLUTION_HZ;
        //  ONE BLOCK PER CHANNEL, AND THE NUMBER IS NOT ARBITRARY.
        //
        //  The ESP32-P4 has SOC_RMT_TX_CANDIDATES_PER_GROUP = 4 and
        //  SOC_RMT_MEM_WORDS_PER_CHANNEL = 48. Asking for 64 symbols spans TWO
        //  blocks, which halves the usable channels to two -- and this driver
        //  needs four, one per motor. The first hardware run failed exactly
        //  there:
        //
        //      rmt_tx_register_to_group: no free tx channels
        //      dshot: motor 2: TX channel on GPIO 6 failed
        //
        //  A DShot frame is 16 symbols. 48 is ample and costs one block.
        txc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
        txc.trans_queue_depth = 2;
        // THE INVERSION LIVES HERE, AND ONLY HERE.
        //
        // Bidirectional DShot idles HIGH and pulses low. dshotBuildSymbols() can invert
        // in software, but software cannot set the level the line rests at BETWEEN
        // frames, and that idle level is exactly what lets the ESC pull the line down to
        // reply. So the hardware does it, and the symbols are built un-inverted.
        //
        // Doing both cancels out, and the result is a signal an ESC silently ignores.
        txc.flags.invert_out  = DSHOT_BIDIRECTIONAL ? 1 : 0;
        txc.flags.with_dma    = 0;

        if (rmt_new_tx_channel(&txc, &g_ch[i].tx) != ESP_OK) {
            ESP_LOGE(TAG, "motor %d: TX channel on GPIO %d failed", i, pins[i]);
            return false;
        }
        if (rmt_enable(g_ch[i].tx) != ESP_OK) {
            ESP_LOGE(TAG, "motor %d: TX enable failed", i);
            return false;
        }

#if DSHOT_BIDIRECTIONAL
        rmt_rx_channel_config_t rxc = {};
        rxc.gpio_num          = (gpio_num_t)pins[i];
        rxc.clk_src           = RMT_CLK_SRC_DEFAULT;
        rxc.resolution_hz     = DSHOT_RMT_RESOLUTION_HZ;
        //  Same reasoning as the TX channel above. With four TX and four RX
        //  channels this driver uses the entire RMT peripheral -- 8 of 8 -- so
        //  nothing else in the firmware may claim an RMT channel.
        rxc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
        rxc.flags.invert_in   = 1;     // the reply is low-going, like the outgoing frame

        if (rmt_new_rx_channel(&rxc, &g_ch[i].rx) != ESP_OK) {
            // This is the failure that means the approach is wrong rather than
            // mis-tuned. Say so plainly instead of retrying.
            ESP_LOGE(TAG, "motor %d: RX channel on GPIO %d failed -- if this is because "
                          "a pin cannot carry both TX and RX, bidirectional telemetry "
                          "needs a different capture mechanism, not different settings",
                     i, pins[i]);
            return false;
        }

        rmt_rx_event_callbacks_t cbs = {};
        cbs.on_recv_done = onRxDone;
        if (rmt_rx_register_event_callbacks(g_ch[i].rx, &cbs, &g_ch[i]) != ESP_OK ||
            rmt_enable(g_ch[i].rx) != ESP_OK) {
            ESP_LOGE(TAG, "motor %d: RX setup failed", i);
            return false;
        }
#endif
    }

    ESP_LOGI(TAG, "DShot%u on %u MHz RMT: %u ticks/bit (%u ppm error), %s",
             (unsigned)DSHOT_BITRATE_KHZ, (unsigned)(DSHOT_RMT_RESOLUTION_HZ / 1000000u),
             (unsigned)g_bitTicks,
             (unsigned)dshotBitErrorPpm(DSHOT_RMT_RESOLUTION_HZ, DSHOT_BITRATE_KHZ),
             DSHOT_BIDIRECTIONAL ? "bidirectional" : "output only");

    ready_ = true;
    return true;
}

// -------------------------------------------------------------------------------------
void DShotRmt::write(const uint16_t values[4], bool requestTelemetry) {
    if (!ready_) return;

    for (int i = 0; i < 4; ++i) {
        const uint16_t frame = dshotFrame(values[i], requestTelemetry,
                                          DSHOT_BIDIRECTIONAL != 0);

        // Symbols are built UN-inverted; the channel's invert_out does the flipping.
        // See the note in begin().
        dshotBuildSymbols(frame, g_bitTicks, /*inverted=*/false, symbols_[i]);

        for (int b = 0; b < DSHOT_FRAME_BITS; ++b) {
            g_ch[i].words[b].level0    = symbols_[i][b].firstLevel;
            g_ch[i].words[b].duration0 = symbols_[i][b].firstTicks;
            g_ch[i].words[b].level1    = symbols_[i][b].secondLevel;
            g_ch[i].words[b].duration1 = symbols_[i][b].secondTicks;
        }

        rmt_transmit_config_t tc = {};
        tc.loop_count = 0;             // one frame, not a repeating pattern

        // Non-blocking: the frame is queued and the peripheral emits it. The flight loop
        // must not wait here -- 16 bits at DShot300 is 53 us, which is 5% of a 1000 Hz
        // period spent doing nothing.
        rmt_transmit(g_ch[i].tx, g_copy, g_ch[i].words,
                     sizeof(g_ch[i].words), &tc);

#if DSHOT_BIDIRECTIONAL
        // ASSUMPTION FOUR, and the shakiest one. The ESC replies about 30 us after the
        // frame ends, on the same wire. Arming the receiver here means it is listening
        // while we are still transmitting, and it will therefore capture our own frame
        // first -- pollTelemetry() has to skip past that.
        //
        // If the RMT driver cannot have RX armed on a pin its TX channel is driving,
        // this is where it will show up, and the answer is a different capture
        // mechanism rather than a different delay.
        g_ch[i].rxDone = false;
        g_ch[i].rxCount = 0;

        rmt_receive_config_t rc = {};
        // A reply bit is ~2.7 us at DShot300. Anything an order of magnitude shorter is
        // a glitch; anything much longer than the whole 21-bit reply is the line idle.
        rc.signal_range_min_ns = 200;
        rc.signal_range_max_ns = (uint32_t)DSHOT_TELEM_TIMEOUT_US * 1000u;
        rmt_receive(g_ch[i].rx, g_ch[i].rxBuf, sizeof(g_ch[i].rxBuf), &rc);
#endif
    }
}

// -------------------------------------------------------------------------------------
void DShotRmt::pollTelemetry(DShotTelemetry& sink, uint32_t nowMs) {
#if DSHOT_BIDIRECTIONAL
    if (!ready_) return;

    for (int i = 0; i < 4; ++i) {
        if (!g_ch[i].rxDone) continue;              // nothing back yet; do not block
        g_ch[i].rxDone = false;

        const size_t n = g_ch[i].rxCount;
        if (n == 0 || n > 64) continue;

        // Flatten the captured symbols into runs. Each RMT symbol is two runs, so the
        // reply's alternating levels come out as a straightforward sequence.
        uint32_t ticks[128];
        uint8_t  levels[128];
        int runs = 0;

        for (size_t s = 0; s < n && runs < 126; ++s) {
            const rmt_symbol_word_t& w = g_ch[i].rxBuf[s];
            if (w.duration0 == 0) break;            // end marker
            ticks[runs]  = w.duration0;
            levels[runs] = (uint8_t)w.level0;
            ++runs;
            if (w.duration1 == 0) break;
            ticks[runs]  = w.duration1;
            levels[runs] = (uint8_t)w.level1;
            ++runs;
        }

        // Skip our own transmitted frame, which the receiver heard because it was armed
        // during transmission. The gap between the frame ending and the ESC answering is
        // the longest quiet stretch in the capture, so it is the separator.
        int start = 0;
        for (int r = 0; r < runs; ++r) {
            if (ticks[r] > g_telemTicks * 4u) start = r + 1;
        }
        if (start >= runs) continue;

        const uint32_t erpm = dshotErpmFromRuns(&ticks[start], &levels[start],
                                                runs - start, g_telemTicks);
        if (erpm == DSHOT_ERPM_INVALID) continue;   // let the value age instead

        // ingest() wants a telemetry word rather than an eRPM, so hand it one that
        // carries the same period. Re-encoding here keeps DShotTelemetry's validation
        // in one place rather than giving it a second entry point that skips the checks.
        const uint32_t periodUs = erpm ? (60000000u / erpm) : 0x0FFFu;
        uint32_t e = 0;
        uint32_t p = periodUs;
        while ((p >> e) > 0x1FF && e < 7) ++e;
        const uint16_t p12 = (uint16_t)(((e & 0x7) << 9) | ((p >> e) & 0x1FF));
        const uint8_t crc = (uint8_t)((~(p12 ^ (p12 >> 4) ^ (p12 >> 8))) & 0x0F);
        sink.ingest((uint8_t)i, (uint16_t)((p12 << 4) | crc), nowMs);
    }
#else
    (void)sink; (void)nowMs;
#endif
}

// -------------------------------------------------------------------------------------
void DShotRmt::sendCommand(uint16_t command, int repeats) {
    if (!ready_) return;
    // Configuration commands must be repeated before an ESC acts on them, and several
    // of them are persistent. This is why sendCommand() is not reachable from the
    // flight loop: DSHOT_CMD_SAVE_SETTINGS sent by accident writes the ESC's EEPROM.
    const uint16_t values[4] = { command, command, command, command };
    for (int r = 0; r < repeats; ++r) {
        write(values, false);
        delayMicroseconds(FLIGHT_LOOP_PERIOD_US);
    }
}

#endif // DSHOT_ENABLE
