// =====================================================================================
//  Odyssey-10 Pro -- Dynamic gyro notch
//  ------------------------------------------------------------------------------------
//  WHY THIS EXISTS
//
//  The notch centre frequency has been the most troublesome number in this project. It
//  moved from 80 Hz to 95, then 100, then 120 Hz as the airframe, motors and propellers
//  changed. Two independent models of it disagreed by 7%. It spans 88 Hz to 180 Hz
//  across the ten characterised build combinations. And when it moved from 80 to 120 Hz
//  the IMU anti-alias filter was left behind at 94 Hz, which put the low-pass BELOW the
//  notch and quietly negated both -- a defect that survived four revisions.
//
//  Every one of those problems has the same root cause: a fixed constant standing in for
//  a quantity that is not fixed. Hover shaft speed varies with mass, air density, pack
//  voltage, payload and how hard the aircraft is working. A number chosen at compile
//  time is wrong the moment any of those changes.
//
//  So measure it instead. A sliding DFT over the gyro signal finds where the motor noise
//  actually is, several times a second, and the notch follows it.
//
//  ------------------------------------------------------------------------------------
//  WHY A SLIDING DFT RATHER THAN AN FFT
//
//  An FFT needs a whole buffer before it produces anything, which means a burst of work
//  every N samples -- exactly the kind of periodic spike that ruins a hard real-time
//  loop's worst-case timing. A sliding DFT updates every bin by a constant amount on
//  every sample, so the cost is uniform and the worst case equals the average case.
//
//  With N = 128 bins at the 500 Hz loop rate: 3.9 Hz resolution across 0-250 Hz. One
//  complex multiply-accumulate per 64 usable bins per sample is about 0.5 MFLOP/s --
//  under 1% of a 400 MHz core, and 3 KB of state.
//
//  One tracker, on the roll axis, retunes all three notches. The peak being hunted is
//  the four motor shafts and they are the same four shafts whichever axis you look
//  down, so a tracker per axis would triple the cost to rediscover the same number.
//
//  ------------------------------------------------------------------------------------
//  WHY IT IS BOUNDED
//
//  This is untested on hardware. A tracker that chased a spurious peak could put the
//  notch somewhere useless, or worse, somewhere that removes real control signal.
//
//  So it cannot do worse than the static configuration it replaces: the tracked centre
//  is CLAMPED to a band around the compiled NOTCH_CENTER_HZ, a peak must be both tall
//  enough and REPEAT in the same bin before it is believed at all, and the centre slews
//  rather than jumping. If tracking fails or the signal is quiet, it falls back to
//  exactly the static value. The worst case is the behaviour we already had.
// =====================================================================================

#ifndef ODY_DYNAMIC_NOTCH_H
#define ODY_DYNAMIC_NOTCH_H

#include <Arduino.h>
#include "config.h"
#include "filters.h"

// -------------------------------------------------------------------------------------
//  Sliding DFT
//
//  Each bin holds a running complex accumulator updated as
//
//      S_k <- (S_k - x[n-N] + x[n]) * W_k        where W_k = exp(j*2*pi*k/N)
//
//  A pure recursion drifts as float rounding accumulates, so the twiddle is damped
//  slightly below unit magnitude. That makes the recursion stable at the cost of a small
//  known bias -- the standard fix, and far cheaper than periodically recomputing.
// -------------------------------------------------------------------------------------
template <int N>
class SlidingDft {
public:
    void begin(float sampleRateHz) {
        sampleRate_ = sampleRateHz;
        for (int k = 0; k < BINS; ++k) {
            const float w = 2.0f * (float)M_PI * (float)k / (float)N;
            // 0.9999 damping: stable recursion, negligible amplitude bias.
            cosW_[k] = 0.9999f * cosf(w);
            sinW_[k] = 0.9999f * sinf(w);
            re_[k] = im_[k] = 0.0f;
        }
        for (int i = 0; i < N; ++i) delay_[i] = 0.0f;
        head_ = 0;
        primed_ = 0;
    }

    /** Feeds one sample. Cost is O(BINS) and identical every call. */
    void push(float x) {
        const float oldest = delay_[head_];
        delay_[head_] = x;
        head_ = (head_ + 1) % N;
        if (primed_ < N) ++primed_;

        const float diff = x - oldest;
        for (int k = 0; k < BINS; ++k) {
            const float r = re_[k] + diff;
            const float i = im_[k];
            re_[k] = r * cosW_[k] - i * sinW_[k];
            im_[k] = r * sinW_[k] + i * cosW_[k];
        }
    }

    bool ready() const { return primed_ >= N; }

    float magnitudeSq(int k) const { return re_[k] * re_[k] + im_[k] * im_[k]; }

    float binToHz(float k) const { return k * sampleRate_ / (float)N; }
    int   hzToBin(float hz) const {
        const int k = (int)(hz * (float)N / sampleRate_ + 0.5f);
        return constrain(k, 0, BINS - 1);
    }

    static constexpr int BINS = N / 2;   // above N/2 mirrors, so only half is useful

private:
    float re_[BINS], im_[BINS];
    float cosW_[BINS], sinW_[BINS];
    float delay_[N];
    int   head_ = 0;
    int   primed_ = 0;
    float sampleRate_ = 500.0f;
};

// -------------------------------------------------------------------------------------
//  Peak tracker
// -------------------------------------------------------------------------------------
struct DynamicNotchState {
    float centreHz     = 0.0f;   // what the notch is currently set to
    float detectedHz   = 0.0f;   // raw peak, before clamping and slewing
    float confidence   = 0.0f;   // peak magnitude over the band average
    bool  tracking     = false;  // false means it fell back to the static value
    uint32_t updates   = 0;

    // ---- second harmonic -------------------------------------------------------------
    float harmonicHz        = 0.0f;   // tracked 2*f0, or 0 when not engaged
    float harmonicConf      = 0.0f;
    bool  harmonicTracking  = false;  // a confident lock on the harmonic
    bool  harmonicObservable = false; // 2*f0 clears the DLPF corner and Nyquist margin
    bool  fromTelemetry      = false; // the centre came from RPM, not from the DFT
};

// -------------------------------------------------------------------------------------
//  WHY A MAGNITUDE THRESHOLD ALONE IS NOT ENOUGH
//
//  The obvious test for "is there really a peak here" is peak/mean magnitude. It is not
//  sufficient. For white noise the expected ratio of the largest bin to the band mean is
//  the harmonic number H_n, which across the ~30 bins of the search band is already about
//  4 -- and it exceeds that half the time. A threshold set anywhere near that either
//  rejects real peaks or believes noise.
//
//  What actually separates them is not height but STABILITY. A motor peak stays in the
//  same bin update after update, because shaft speed changes on the timescale of the
//  aircraft's mass. A noise peak lands in a different bin every time. So the tracker
//  requires the peak to REPEAT within a couple of bins before it is believed at all.
// -------------------------------------------------------------------------------------

class DynamicNotchTracker {
public:
    /**
     * @param staticCentreHz  the compiled NOTCH_CENTER_HZ, used as the fallback and as
     *                        the centre of the permitted band
     */
    void begin(float sampleRateHz, float staticCentreHz) {
        dft_.begin(sampleRateHz);
        sampleRate_  = sampleRateHz;
        staticHz_    = staticCentreHz;
        minHz_       = staticCentreHz * DYN_NOTCH_BAND_LOW;
        maxHz_       = staticCentreHz * DYN_NOTCH_BAND_HIGH;

        // Never search above the DLPF corner: there is nothing above it but the filter's
        // own roll-off, and a peak found there would be an artefact.
        ceilingHz_ = min((float)IMU_DLPF_HZ,
                         sampleRate_ * 0.5f * DYN_NOTCH_H2_NYQUIST_FRAC);
        if (maxHz_ > ceilingHz_) maxHz_ = ceilingHz_;

        state_.centreHz = staticHz_;
        state_.detectedHz = staticHz_;
        state_.harmonicHz = 0.0f;
        state_.harmonicTracking = false;
        state_.harmonicObservable =
            (staticHz_ * DYN_NOTCH_H2_MULTIPLE) <= ceilingHz_;
        smoothed_ = staticHz_;
        h2PrevK_ = -1;
        h2HavePrev_ = false;
    }

    /** Feed every gyro sample from one axis, RAW -- upstream of the notch it tunes. */
    void push(float gyroDps) { dft_.push(gyroDps); }

    /**
     * Supplies a fundamental MEASURED from ESC RPM telemetry, instead of searching.
     *
     * This is what the whole sliding-DFT apparatus is a substitute for. When the ESCs
     * report and agree, there is nothing to estimate.
     *
     * It is still CLAMPED to the same band, and still slews. Not because the measurement
     * is doubted, but because the conversion from it is not: MOTOR_POLE_PAIRS scales
     * every derived frequency, and a wrong pole count produces a confident, precise,
     * wrong number. Landing outside the band the models predict is exactly the symptom,
     * and clamping turns it into a bounded error rather than a notch in the wrong place.
     *
     * Gyro samples must keep being push()ed even while this is in use -- the harmonic
     * search still runs on the spectrum, and the DFT has to stay primed in case
     * telemetry drops out mid-flight.
     *
     * @return true if the notch centre moved enough to be worth retuning
     */
    bool applyMeasured(float hz) {
        if (!(hz > 0.0f)) return false;
        state_.detectedHz = hz;
        state_.tracking = true;
        state_.fromTelemetry = true;
        state_.confidence = 0.0f;      // not a peak-to-mean ratio; do not pretend it is
        slewTowards(constrain(hz, minHz_, maxHz_));
        const bool moved = commit();
        updateHarmonic();
        return moved;
    }

    /**
     * Re-evaluates the peak. Call at DYN_NOTCH_UPDATE_HZ, not every sample -- the
     * spectrum does not change faster than the aircraft's mass does.
     *
     * @return true if the notch centre moved enough to be worth retuning the biquad
     */
    bool update() {
        state_.fromTelemetry = false;
        if (!dft_.ready()) return false;

        const int kLo = dft_.hzToBin(minHz_);
        const int kHi = dft_.hzToBin(maxHz_);
        if (kHi <= kLo + 1) return false;

        float peakMag = 0.0f, sumMag = 0.0f;
        int   peakK = kLo;
        for (int k = kLo; k <= kHi; ++k) {
            const float m = dft_.magnitudeSq(k);
            sumMag += m;
            if (m > peakMag) { peakMag = m; peakK = k; }
        }

        const int   count = kHi - kLo + 1;
        const float mean  = sumMag / (float)count;

        // A peak that is not clearly above the band's own average is noise, not a motor.
        // Falling back to the static value is the safe answer, not guessing.
        state_.confidence = (mean > 1e-12f) ? (peakMag / mean) : 0.0f;

        // Two independent gates, and BOTH must pass.
        //
        //   1. Height  -- the peak must stand clear of the band's own average.
        //   2. Stability -- it must land in roughly the same bin as last time.
        //
        // Gate 2 is the one that does the real work. See the note above the state
        // struct: a noise peak wanders, a motor peak does not.
        const bool tallEnough = state_.confidence >= DYN_NOTCH_MIN_CONFIDENCE;
        const int  drift      = peakK - prevPeakK_;
        const bool repeated   = havePrev_ && drift >= -DYN_NOTCH_PEAK_TOL_BINS
                                          && drift <=  DYN_NOTCH_PEAK_TOL_BINS;
        prevPeakK_ = peakK;
        havePrev_  = true;

        if (!tallEnough || !repeated) {
            state_.tracking = false;
            slewTowards(staticHz_);
            return commit();
        }

        // Parabolic interpolation across the peak and its neighbours, so the resolution
        // is not limited to the 3.9 Hz bin spacing.
        float kEst = (float)peakK;
        if (peakK > kLo && peakK < kHi) {
            const float a = sqrtf(dft_.magnitudeSq(peakK - 1));
            const float b = sqrtf(dft_.magnitudeSq(peakK));
            const float c = sqrtf(dft_.magnitudeSq(peakK + 1));
            const float denom = a - 2.0f * b + c;
            if (fabsf(denom) > 1e-9f) {
                const float delta = 0.5f * (a - c) / denom;
                if (fabsf(delta) < 1.0f) kEst += delta;
            }
        }

        state_.detectedHz = dft_.binToHz(kEst);
        state_.tracking = true;
        slewTowards(constrain(state_.detectedHz, minHz_, maxHz_));
        const bool moved = commit();
        updateHarmonic();
        return moved;
    }

    /**
     * Looks for the second harmonic, in a narrow window around twice the tracked
     * fundamental.
     *
     * Searching a WINDOW rather than simply notching at exactly 2x the fundamental is
     * the point: a propeller's overtone is not an exact integer multiple once blade
     * flex and frame modes are involved, and a notch placed by arithmetic rather than by
     * measurement is the mistake this whole subsystem exists to stop repeating.
     */
    void updateHarmonic() {
#if DYN_NOTCH_HARMONIC
        state_.harmonicTracking = false;

        // Judged against the TRACKED fundamental, not the compiled one. If the real f0
        // is lower than the model assumed, the harmonic may be visible when the compiled
        // value said it would not be -- and the reverse.
        const float target = state_.centreHz * DYN_NOTCH_H2_MULTIPLE;
        state_.harmonicObservable = (target <= ceilingHz_);
        if (!state_.harmonicObservable || !state_.tracking) {
            state_.harmonicHz = 0.0f;
            state_.harmonicConf = 0.0f;
            h2HavePrev_ = false;
            return;
        }

        const int kLo = dft_.hzToBin(target * (1.0f - DYN_NOTCH_H2_SEARCH));
        const int kHi = dft_.hzToBin(target * (1.0f + DYN_NOTCH_H2_SEARCH));
        if (kHi <= kLo + 1) {
            state_.harmonicHz = 0.0f;
            h2HavePrev_ = false;
            return;
        }

        float peakMag = 0.0f, sumMag = 0.0f;
        int peakK = kLo;
        for (int k = kLo; k <= kHi; ++k) {
            const float m = dft_.magnitudeSq(k);
            sumMag += m;
            if (m > peakMag) { peakMag = m; peakK = k; }
        }
        const float mean = sumMag / (float)(kHi - kLo + 1);
        state_.harmonicConf = (mean > 1e-12f) ? (peakMag / mean) : 0.0f;

        // Same two gates as the fundamental: tall enough, and in the same place twice.
        const int drift = peakK - h2PrevK_;
        const bool repeated = h2HavePrev_ && drift >= -DYN_NOTCH_PEAK_TOL_BINS
                                          && drift <=  DYN_NOTCH_PEAK_TOL_BINS;
        h2PrevK_ = peakK;
        h2HavePrev_ = true;

        if (state_.harmonicConf < DYN_NOTCH_H2_MIN_CONF || !repeated) {
            state_.harmonicHz = 0.0f;
            return;
        }

        float kEst = (float)peakK;
        if (peakK > kLo && peakK < kHi) {
            const float a = sqrtf(dft_.magnitudeSq(peakK - 1));
            const float b = sqrtf(dft_.magnitudeSq(peakK));
            const float c = sqrtf(dft_.magnitudeSq(peakK + 1));
            const float denom = a - 2.0f * b + c;
            if (fabsf(denom) > 1e-9f) {
                const float delta = 0.5f * (a - c) / denom;
                if (fabsf(delta) < 1.0f) kEst += delta;
            }
        }

        const float found = dft_.binToHz(kEst);
        state_.harmonicHz = constrain(found, target * (1.0f - DYN_NOTCH_H2_SEARCH),
                                      min(target * (1.0f + DYN_NOTCH_H2_SEARCH),
                                          ceilingHz_));
        state_.harmonicTracking = true;
#endif
    }

    const DynamicNotchState& state() const { return state_; }
    float centreHz() const { return state_.centreHz; }
    float harmonicHz() const { return state_.harmonicHz; }

    /**
     * The highest frequency worth searching.
     *
     * Above the IMU's anti-alias corner there is nothing but the filter's own roll-off,
     * and near Nyquist the SDFT's top bins collect whatever aliased. A peak found in
     * either region is an artefact, and notching an artefact costs real phase lag.
     */
    float ceilingHz() const { return ceilingHz_; }

    float bandLowHz()  const { return minHz_; }
    float bandHighHz() const { return maxHz_; }

private:
    void slewTowards(float target) {
        // Rate-limited so a transient cannot yank the notch across the band. At the
        // default 40 Hz/s it takes about a second to traverse the whole permitted range.
        const float maxStep = DYN_NOTCH_SLEW_HZ_PER_S / (float)DYN_NOTCH_UPDATE_HZ;
        const float delta = constrain(target - smoothed_, -maxStep, maxStep);
        smoothed_ = constrain(smoothed_ + delta, minHz_, maxHz_);
    }

    bool commit() {
        ++state_.updates;
        // Retuning a biquad is cheap but not free, and a centre that jitters by a
        // fraction of a hertz is not worth the churn.
        if (fabsf(smoothed_ - state_.centreHz) < DYN_NOTCH_RETUNE_HZ) return false;
        state_.centreHz = smoothed_;
        return true;
    }

    SlidingDft<DYN_NOTCH_BINS> dft_;
    DynamicNotchState state_;
    float sampleRate_ = 500.0f;
    float staticHz_   = 100.0f;
    float ceilingHz_  = 184.0f;
    int   prevPeakK_  = -1;
    bool  havePrev_   = false;
    int   h2PrevK_    = -1;
    bool  h2HavePrev_ = false;
    float minHz_      = 60.0f;
    float maxHz_      = 200.0f;
    float smoothed_   = 100.0f;
};

#endif // ODY_DYNAMIC_NOTCH_H
