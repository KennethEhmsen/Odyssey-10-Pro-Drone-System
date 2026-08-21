// =====================================================================================
//  Odyssey-10 Pro -- Signal filters
//  ------------------------------------------------------------------------------------
//  The bi-quad notch maths in the original document was actually correct; it is kept
//  here with the coefficient names corrected to the conventional b (numerator) /
//  a (denominator) convention, because the original swapped them and that makes the
//  implementation very hard to check against a reference.
//
//  The one real defect is fixed: the filter was configured inside runAutoCalibration(),
//  so if calibration bailed out early the flight task still ran apply() against
//  zero-initialised coefficients and every gyro sample became 0.0. configured() now
//  makes that state explicit and the flight task refuses to use an unconfigured filter.
// =====================================================================================

#ifndef ODY_FILTERS_H
#define ODY_FILTERS_H

#include <Arduino.h>
#include <math.h>

// -------------------------------------------------------------------------------------
//  2nd-order bi-quad notch (RBJ cookbook)
//
//      H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
//
//  with, for a notch of centre w0 and quality Q:
//      alpha = sin(w0) / (2Q)
//      b0 = 1,  b1 = -2cos(w0),  b2 = 1
//      a0 = 1 + alpha,  a1 = -2cos(w0),  a2 = 1 - alpha
//  all divided through by a0.
// -------------------------------------------------------------------------------------
class BiQuadNotch {
public:
  void configure(float centreHz, float sampleHz, float q) {
    // Guard the degenerate cases rather than emitting NaNs into the control loop.
    if (!(centreHz > 0.0f) || !(sampleHz > 0.0f) || !(q > 0.0f) ||
        centreHz >= sampleHz * 0.5f) {
      setPassthrough();
      return;
    }

    const float w0    = 2.0f * (float)M_PI * centreHz / sampleHz;
    const float cosW0 = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * q);
    const float a0    = 1.0f + alpha;

    b0_ =  1.0f      / a0;
    b1_ = -2.0f * cosW0 / a0;
    b2_ =  1.0f      / a0;
    a1_ = -2.0f * cosW0 / a0;
    a2_ = (1.0f - alpha) / a0;

    reset();
    configured_ = true;
    centreHz_   = centreHz;
  }

  // Retunes the notch without clearing the delay line, so a moving motor-noise peak
  // can be tracked without punching a transient through the rate loop.
  void retune(float centreHz, float sampleHz, float q) {
    const float x1 = x1_, x2 = x2_, y1 = y1_, y2 = y2_;
    configure(centreHz, sampleHz, q);
    x1_ = x1; x2_ = x2; y1_ = y1; y2_ = y2;
  }

  float apply(float in) {
    if (!configured_) return in;      // never silently zero the gyro
    const float out = b0_ * in + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_; x1_ = in;
    y2_ = y1_; y1_ = out;
    return out;
  }

  void reset() { x1_ = x2_ = y1_ = y2_ = 0.0f; }

  /**
   * Disengages the filter, returning it to a pass-through.
   *
   * A notch that is no longer needed must be REMOVED, not left tuned to wherever it
   * last was. The dynamic harmonic notch can legitimately stop having anything to
   * filter -- the overtone fades, or the tracked fundamental rises until twice it no
   * longer clears the anti-alias corner -- and a notch left sitting on empty spectrum
   * contributes phase lag in the control band in exchange for nothing. That is the
   * defect of section 8.3 in miniature.
   */
  void bypass() { setPassthrough(); }

  bool  configured() const { return configured_; }
  float centreHz()   const { return centreHz_; }

private:
  void setPassthrough() {
    b0_ = 1.0f; b1_ = b2_ = a1_ = a2_ = 0.0f;
    reset();
    configured_ = false;
  }

  float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f;
  float a1_ = 0.0f, a2_ = 0.0f;
  float x1_ = 0.0f, x2_ = 0.0f, y1_ = 0.0f, y2_ = 0.0f;
  bool  configured_ = false;
  float centreHz_   = 0.0f;
};

// -------------------------------------------------------------------------------------
//  First-order low pass, expressed as a time constant so the behaviour does not change
//  when the caller's loop rate changes.
// -------------------------------------------------------------------------------------
class LowPass {
public:
  explicit LowPass(float tauSeconds = 0.01f) : tau_(tauSeconds) {}

  void  setTau(float tauSeconds) { tau_ = tauSeconds; }
  void  reset(float v = 0.0f)    { y_ = v; primed_ = (v != 0.0f); }

  float apply(float in, float dt) {
    if (!primed_) { y_ = in; primed_ = true; return y_; }
    if (dt <= 0.0f) return y_;
    const float a = dt / (dt + tau_);
    y_ += (in - y_) * a;
    return y_;
  }

  float value() const { return y_; }

private:
  float tau_    = 0.01f;
  float y_      = 0.0f;
  bool  primed_ = false;
};

// -------------------------------------------------------------------------------------
//  Exponential moving average with an explicit alpha, plus slew limiting.
//  Used for the pack voltage: filtering alone is not enough, because a single
//  bad ADC conversion can still step the output. Slew limiting bounds how fast the
//  reported voltage may move, so one glitch cannot latch an irreversible mode change.
// -------------------------------------------------------------------------------------
class SlewLimitedEma {
public:
  SlewLimitedEma(float alpha, float maxDeltaPerSample)
      : alpha_(alpha), maxDelta_(maxDeltaPerSample) {}

  void reset(float v) { y_ = v; primed_ = true; }

  float apply(float in) {
    if (!primed_) { y_ = in; primed_ = true; return y_; }
    float delta = in - y_;
    if (delta >  maxDelta_) delta =  maxDelta_;
    if (delta < -maxDelta_) delta = -maxDelta_;
    y_ += alpha_ * delta;
    return y_;
  }

  float value() const { return y_; }
  bool  primed() const { return primed_; }

private:
  float alpha_;
  float maxDelta_;
  float y_      = 0.0f;
  bool  primed_ = false;
};

// -------------------------------------------------------------------------------------
//  Debounced threshold.
//
//  A condition must hold continuously for holdMs before latching. This is what turns
//  the battery comparison from "one sagging ADC sample commits the aircraft to an
//  irreversible RTH" into a decision that reflects sustained pack state.
// -------------------------------------------------------------------------------------
class DebouncedCondition {
public:
  explicit DebouncedCondition(uint32_t holdMs) : holdMs_(holdMs) {}

  bool update(bool condition, uint32_t nowMs) {
    if (!condition) { since_ = 0; return false; }
    if (since_ == 0) { since_ = nowMs ? nowMs : 1; return false; }
    return (uint32_t)(nowMs - since_) >= holdMs_;
  }

  void reset() { since_ = 0; }

  uint32_t heldMs(uint32_t nowMs) const {
    return since_ ? (uint32_t)(nowMs - since_) : 0;
  }

private:
  uint32_t holdMs_;
  uint32_t since_ = 0;
};

#endif // ODY_FILTERS_H
