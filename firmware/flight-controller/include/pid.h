// =====================================================================================
//  Odyssey-10 Pro -- PID controllers
//  ------------------------------------------------------------------------------------
//  Changes from the original implementation:
//
//    * Derivative is taken on the MEASUREMENT, not the error. Differentiating the error
//      means every setpoint step (a stick flick, or the RTH controller handing over a
//      new target) puts a spike through the D term proportional to the step size.
//    * Integral anti-windup is conditional: the I term stops accumulating while the
//      output is saturated in the direction that would make the saturation worse.
//    * dt is validated. The original divided by dt without checking it, so a scheduling
//      hiccup that produced a near-zero dt would generate an enormous D term.
// =====================================================================================

#ifndef ODY_PID_H
#define ODY_PID_H

#include <Arduino.h>
#include "filters.h"

struct PidGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float outMin = -1.0f;
  float outMax =  1.0f;
  float iMax   =  0.0f;   // absolute clamp on the integral accumulator
  float dTau   =  0.01f;  // derivative low-pass time constant, seconds
};

class Pid {
public:
  Pid() = default;
  explicit Pid(const PidGains& g) : g_(g) { dLpf_.setTau(g.dTau); }

  void setGains(const PidGains& g) {
    g_ = g;
    dLpf_.setTau(g.dTau);
  }

  const PidGains& gains() const { return g_; }

  float update(float setpoint, float measurement, float dt) {
    // A bad dt is a scheduling fault, not a control input. Fall back to the nominal
    // period rather than letting it propagate into the derivative.
    if (!(dt > 0.0f) || dt > 0.05f) dt = 1.0f / 500.0f;

    const float error = setpoint - measurement;
    const float pTerm = g_.kp * error;

    // Derivative on measurement: d(error)/dt == -d(measurement)/dt for a constant
    // setpoint, so the sign is negated here.
    float dRaw = 0.0f;
    if (primed_) dRaw = -(measurement - prevMeasurement_) / dt;
    prevMeasurement_ = measurement;
    primed_ = true;

    const float dTerm = g_.kd * dLpf_.apply(dRaw, dt);

    // Provisional output without the new integral contribution.
    const float provisional = pTerm + integral_ + dTerm;

    // Conditional integration: only accumulate if we are not already pinned against
    // the limit we would be pushing further into.
    const bool pinnedHigh = provisional >= g_.outMax && error > 0.0f;
    const bool pinnedLow  = provisional <= g_.outMin && error < 0.0f;
    if (!pinnedHigh && !pinnedLow) {
      integral_ += g_.ki * error * dt;
      if (g_.iMax > 0.0f) integral_ = constrain(integral_, -g_.iMax, g_.iMax);
    }

    lastOutput_ = constrain(pTerm + integral_ + dTerm, g_.outMin, g_.outMax);
    return lastOutput_;
  }

  // Called on every disarm and on every state transition that changes what the
  // controller is being asked to do, so stale integral does not leak across modes.
  void reset() {
    integral_        = 0.0f;
    prevMeasurement_ = 0.0f;
    lastOutput_      = 0.0f;
    primed_          = false;
    dLpf_.reset(0.0f);
  }

  // Pre-loads the integrator so a mode change hands over without a step. Used when
  // switching from manual to RTH at a non-zero attitude.
  void preload(float outputValue) {
    integral_ = constrain(outputValue, g_.outMin, g_.outMax);
    if (g_.iMax > 0.0f) integral_ = constrain(integral_, -g_.iMax, g_.iMax);
  }

  float integral()   const { return integral_; }
  float lastOutput() const { return lastOutput_; }

private:
  PidGains g_{};
  LowPass  dLpf_{0.01f};
  float    integral_        = 0.0f;
  float    prevMeasurement_ = 0.0f;
  float    lastOutput_      = 0.0f;
  bool     primed_          = false;
};

// -------------------------------------------------------------------------------------
//  Tuned gain sets. Rate loops operate in degrees/second in and PWM counts out.
// -------------------------------------------------------------------------------------
namespace GainSets {

inline PidGains rollRate() {
  PidGains g;
  g.kp = 0.85f; g.ki = 0.04f; g.kd = 0.015f;
  g.outMin = -350.0f; g.outMax = 350.0f; g.iMax = 100.0f; g.dTau = 0.010f;
  return g;
}

inline PidGains pitchRate() { return rollRate(); }

inline PidGains yawRate() {
  PidGains g;
  g.kp = 1.80f; g.ki = 0.08f; g.kd = 0.0f;
  g.outMin = -250.0f; g.outMax = 250.0f; g.iMax = 80.0f; g.dTau = 0.010f;
  return g;
}

// Vertical speed -> throttle counts.
inline PidGains verticalRate() {
  PidGains g;
  g.kp = 220.0f; g.ki = 90.0f; g.kd = 25.0f;
  g.outMin = -400.0f; g.outMax = 400.0f; g.iMax = 250.0f; g.dTau = 0.050f;
  return g;
}

} // namespace GainSets

#endif // ODY_PID_H
