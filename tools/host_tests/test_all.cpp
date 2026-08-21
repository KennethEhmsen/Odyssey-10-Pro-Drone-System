// =====================================================================================
//  Odyssey-10 Pro -- host-side verification of the safety-critical algorithms
//  ------------------------------------------------------------------------------------
//  Build and run:
//      sh run_tests.sh
//
//  These tests exist because several of the review findings were defects that no amount
//  of reading the code would reliably catch, but that a two-line assertion catches every
//  time. Each test names the finding it guards.
// =====================================================================================

#include "arduino_shim.h"

uint32_t g_millis = 0;

#include "odyssey_link.h"
#include "config.h"
#include "types.h"
#include "mixer.h"
#include "filters.h"
#include "pid.h"
#include "state_machine.h"
#include "identity.h"
#include "dynamic_notch.h"

#include <cstdio>
#include <vector>
#include <string>
#include <cmath>

// -------------------------------------------------------------------------------------
static int g_pass = 0, g_fail = 0;
static std::string g_section;

static void section(const char* s) {
  g_section = s;
  printf("\n%s\n", s);
  for (size_t i = 0; i < strlen(s); ++i) putchar('-');
  putchar('\n');
}

static void check(bool cond, const char* what) {
  if (cond) { ++g_pass; printf("  PASS  %s\n", what); }
  else      { ++g_fail; printf("  FAIL  %s\n", what); }
}

static void checkNear(float got, float want, float tol, const char* what) {
  const bool ok = fabsf(got - want) <= tol;
  if (ok) { ++g_pass; printf("  PASS  %s  (%.4f)\n", what, got); }
  else    { ++g_fail; printf("  FAIL  %s  got %.4f want %.4f +/- %.4f\n",
                            what, got, want, tol); }
}

// Some checks build their description from the case being run, so the helpers accept a
// std::string too rather than every caller having to write .c_str().
static void check(bool cond, const std::string& what) { check(cond, what.c_str()); }
static void checkNear(float got, float want, float tol, const std::string& what) {
  checkNear(got, want, tol, what.c_str());
}

// =====================================================================================
//  FINDING 1 -- battery thresholds must be 6S values
// =====================================================================================
static void testBatteryThresholds() {
  section("Finding 1: battery thresholds are 6S, not 3S");

  check(CELL_COUNT == 6, "airframe is configured 6S");
  checkNear(PACK_CRITICAL_V, 19.80f, 0.01f, "critical cutoff is 19.80 V");
  checkNear(PACK_WARN_V,     20.40f, 0.01f, "warning threshold is 20.40 V");

  // The exact failure mode of the original: 9.9 V on a 6S pack.
  check(PACK_CRITICAL_V > 15.0f,
        "critical cutoff is not a 3S value (the original bug)");
  check(PACK_CRITICAL_V < PACK_WARN_V && PACK_WARN_V < PACK_LAUNCH_MIN_V,
        "thresholds are correctly ordered");

  // The divider must not saturate the ADC at a full pack.
  checkNear(PACK_FULL_V / VOLTAGE_DIVIDER_RATIO, 2.29f, 0.01f,
            "full pack presents 2.29 V to the ADC");
  check(PACK_FULL_V / VOLTAGE_DIVIDER_RATIO < 3.10f,
        "divider leaves ADC headroom");

  // A 6S pack in normal flight must actually be able to cross the thresholds.
  const float atLanding = 3.5f * CELL_COUNT;   // 21.0 V, a realistic end-of-flight pack
  check(atLanding > PACK_CRITICAL_V, "a 3.5 V/cell pack is above critical");
  const float depleted = 3.25f * CELL_COUNT;   // 19.5 V
  check(depleted < PACK_CRITICAL_V, "a 3.25 V/cell pack trips critical -- reachable");
}

// =====================================================================================
//  FINDING 2 / 13 -- mixer geometry and desaturation
// =====================================================================================
static void testMixerGeometry() {
  section("Finding 2: mixer geometry matches the props-out motor map");

  // Yaw: diagonal pairs must share a sign. M1 (rear-right, CCW) pairs with
  // M4 (front-left, CCW); M2 (front-right, CW) pairs with M3 (rear-left, CW).
  check(Mixer::kYaw[0] == Mixer::kYaw[3], "yaw: M1 and M4 share a sign (CCW pair)");
  check(Mixer::kYaw[1] == Mixer::kYaw[2], "yaw: M2 and M3 share a sign (CW pair)");
  check(Mixer::kYaw[0] == -Mixer::kYaw[1], "yaw: the two pairs oppose");

  // Roll: the right-hand motors (M1, M2) must respond together and oppose the left.
  check(Mixer::kRoll[0] == Mixer::kRoll[1], "roll: M1 and M2 are the right-hand pair");
  check(Mixer::kRoll[2] == Mixer::kRoll[3], "roll: M3 and M4 are the left-hand pair");
  check(Mixer::kRoll[0] == -Mixer::kRoll[2], "roll: the sides oppose");

  // Pitch: the rear motors (M1, M3) must respond together and oppose the front.
  check(Mixer::kPitch[0] == Mixer::kPitch[2], "pitch: M1 and M3 are the rear pair");
  check(Mixer::kPitch[1] == Mixer::kPitch[3], "pitch: M2 and M4 are the front pair");
  check(Mixer::kPitch[0] == -Mixer::kPitch[1], "pitch: front and rear oppose");

  // Each axis must sum to zero across the four motors, otherwise an attitude command
  // would change total thrust and the aircraft would climb when you roll.
  float sr = 0, sp = 0, sy = 0;
  for (int i = 0; i < 4; ++i) {
    sr += Mixer::kRoll[i]; sp += Mixer::kPitch[i]; sy += Mixer::kYaw[i];
  }
  checkNear(sr, 0.0f, 1e-6f, "roll column sums to zero");
  checkNear(sp, 0.0f, 1e-6f, "pitch column sums to zero");
  checkNear(sy, 0.0f, 1e-6f, "yaw column sums to zero");
}

// Reproduces the original independent-clamping behaviour so the two can be compared
// on identical inputs.
static void naiveMix(float throttleCounts, float r, float p, float y, int out[4]) {
  const float base = (float)PWM_MIN + throttleCounts;
  const float m[4] = {
    base + r * Mixer::kRoll[0] + p * Mixer::kPitch[0] + y * Mixer::kYaw[0],
    base + r * Mixer::kRoll[1] + p * Mixer::kPitch[1] + y * Mixer::kYaw[1],
    base + r * Mixer::kRoll[2] + p * Mixer::kPitch[2] + y * Mixer::kYaw[2],
    base + r * Mixer::kRoll[3] + p * Mixer::kPitch[3] + y * Mixer::kYaw[3],
  };
  for (int i = 0; i < 4; ++i) out[i] = (int)constrain((int)lroundf(m[i]), PWM_MIN, PWM_MAX);
}

static float rollDifferential(const int m[4]) {
  // Roll moment is proportional to (left side) - (right side).
  return (float)((m[2] + m[3]) - (m[0] + m[1]));
}

static void testMixerDesaturation() {
  section("Finding 13: mixer desaturation preserves commanded moments");

  // The exact scenario from the finding: base throttle near the ceiling, full roll.
  const float throttle = (float)(PWM_MAX - 300 - PWM_MIN);   // 2976 - PWM_MIN
  const float roll = 350.0f;

  int naive[4];
  naiveMix(throttle, roll, 0.0f, 0.0f, naive);
  const MixerOutput fixed = Mixer::mix(throttle, roll, 0.0f, 0.0f);

  int fixedArr[4];
  for (int i = 0; i < 4; ++i) fixedArr[i] = fixed.motor[i];

  const float wantDiff  = 4.0f * roll;    // two motors up by 350, two down by 350
  const float naiveDiff = rollDifferential(naive);
  const float fixedDiff = rollDifferential(fixedArr);

  printf("      commanded roll differential : %.0f counts\n", wantDiff);
  printf("      original (clamp per motor)  : %.0f counts  (%.1f%% lost)\n",
         naiveDiff, 100.0f * (wantDiff - naiveDiff) / wantDiff);
  printf("      fixed (desaturating mixer)  : %.0f counts  (%.1f%% lost)\n",
         fixedDiff, 100.0f * (wantDiff - fixedDiff) / wantDiff);

  check(naiveDiff < wantDiff - 1.0f,
        "the original mixer DOES lose roll authority here (bug reproduced)");
  checkNear(fixedDiff, wantDiff, 2.0f,
            "the fixed mixer delivers the full commanded roll differential");

  // Nothing may ever leave the endpoints.
  for (int i = 0; i < 4; ++i) {
    check(fixed.motor[i] >= PWM_MIN && fixed.motor[i] <= PWM_MAX,
          "fixed mixer output is inside the PWM endpoints");
  }

  // Sweep: across the whole throttle and correction space, the fixed mixer must never
  // deliver a moment whose SIGN differs from the command. That is the failure the
  // finding describes -- rolling away from the correction.
  int signErrors = 0, magnitudeLoss = 0;
  for (int t = 0; t <= PWM_RANGE; t += 40) {
    for (float r = -350.0f; r <= 350.0f; r += 25.0f) {
      const MixerOutput o = Mixer::mix((float)t, r, 0.0f, 0.0f);
      int a[4]; for (int i = 0; i < 4; ++i) a[i] = o.motor[i];
      const float d = rollDifferential(a);
      if (fabsf(r) > 1.0f) {
        if (d * r < -1.0f) ++signErrors;
        if (fabsf(d) < fabsf(4.0f * r) - 2.0f) ++magnitudeLoss;
      }
    }
  }
  printf("      swept %d throttle x roll combinations\n",
         (PWM_RANGE / 40 + 1) * 29);
  check(signErrors == 0, "no commanded roll ever produces an opposing moment");
  check(magnitudeLoss == 0, "no commanded roll is ever attenuated");
}

// Peak-to-peak span of the correction-only mix, which is what stage 1 must fit into
// PWM_RANGE.
static float correctionRange(float r, float p, float y) {
  float m[4];
  for (int i = 0; i < 4; ++i) {
    m[i] = r * Mixer::kRoll[i] + p * Mixer::kPitch[i] + y * Mixer::kYaw[i];
  }
  float lo = m[0], hi = m[0];
  for (int i = 1; i < 4; ++i) { lo = min(lo, m[i]); hi = max(hi, m[i]); }
  return hi - lo;
}

static void testMixerYawSacrifice() {
  section("Finding 13: correction scaling, and the headroom the gains actually leave");

  // First, an empirical fact worth knowing: with the SHIPPED PID output limits
  // (roll/pitch +/-350, yaw +/-250), what is the worst-case correction span?
  float worst = 0.0f;
  float wr = 0, wp = 0, wy = 0;
  for (float r = -350.0f; r <= 350.0f; r += 10.0f) {
    for (float p = -350.0f; p <= 350.0f; p += 10.0f) {
      for (float y = -250.0f; y <= 250.0f; y += 10.0f) {
        const float range = correctionRange(r, p, y);
        if (range > worst) { worst = range; wr = r; wp = p; wy = y; }
      }
    }
  }
  printf("      worst-case correction span at the shipped gains: %.0f counts\n", worst);
  printf("      (roll %.0f, pitch %.0f, yaw %.0f) against PWM_RANGE = %d\n",
         wr, wp, wy, PWM_RANGE);
  check(worst <= (float)PWM_RANGE,
        "the shipped PID limits cannot saturate the correction stage");
  printf("      headroom: %.0f counts (%.0f%%)\n",
         (float)PWM_RANGE - worst, 100.0f * ((float)PWM_RANGE - worst) / PWM_RANGE);

  // Stage 1 is therefore unreachable unless the gains are raised. Drive it deliberately
  // past the limits to prove it still behaves, because a future retune could get there.
  const float throttle = (float)PWM_RANGE / 2.0f;
  const float r = 600.0f, p = 600.0f, y = 500.0f;
  check(correctionRange(r, p, y) > (float)PWM_RANGE,
        "the over-driven demand genuinely exceeds the motor range");

  const MixerOutput o = Mixer::mix(throttle, r, p, y);
  int a[4]; for (int i = 0; i < 4; ++i) a[i] = o.motor[i];
  const float rollDiff  = rollDifferential(a);
  const float pitchDiff = (float)((a[1] + a[3]) - (a[0] + a[2]));

  printf("      over-driven: commanded roll %.0f, delivered %.0f\n", 4.0f * r, rollDiff);
  printf("      over-driven: commanded pitch %.0f, delivered %.0f\n", 4.0f * p, pitchDiff);
  printf("      reported saturation: %.0f%%\n", o.saturation * 100.0f);

  check(o.saturation > 0.0f, "the mixer reports that it gave authority up");
  check(o.saturation < 1.0f, "...but not all of it");

  // The critical property: scaling must be UNIFORM, so the ratio between the axes --
  // and therefore the direction of the resulting moment -- is preserved. This is what
  // independent clamping destroys.
  checkNear(rollDiff / pitchDiff, r / p, 0.02f,
            "roll:pitch ratio is preserved under scaling");
  check(rollDiff > 0.0f && pitchDiff > 0.0f,
        "both axes keep the sign that was commanded");

  for (int i = 0; i < 4; ++i) {
    check(o.motor[i] >= PWM_MIN && o.motor[i] <= PWM_MAX,
          "saturated output is still inside the endpoints");
  }
}

// =====================================================================================
//  FINDING 6 -- escalation-ordered state machine
// =====================================================================================
static void testStateEscalation() {
  section("Finding 6: FAILSAFE_LANDING cannot overwrite FREEFALL_PARACHUTE");

  FlightStateMachine sm;
  sm.begin();

  check(sm.request(ODY_STATE_PREFLIGHT_OK, "ok"), "BOOT -> PREFLIGHT_OK accepted");
  check(sm.request(ODY_STATE_ARMED, "armed"),     "PREFLIGHT_OK -> ARMED accepted");
  check(sm.get() == ODY_STATE_ARMED, "state is ARMED");

  // The race: core 1 deploys the parachute, core 0 then tries to force a failsafe
  // landing from a stale read. The second request must be refused.
  check(sm.request(ODY_STATE_FREEFALL_PARACHUTE, "free fall"), "parachute deployed");
  check(!sm.request(ODY_STATE_FAILSAFE_LANDING, "stale failsafe"),
        "a late FAILSAFE_LANDING request is REFUSED (the original bug)");
  check(sm.get() == ODY_STATE_FREEFALL_PARACHUTE,
        "state remains FREEFALL_PARACHUTE -- motors stay off under the canopy");

  // Disarming is always permitted.
  check(sm.request(ODY_STATE_DISARMED, "landed"), "DISARMED is always accepted");
  check(sm.get() == ODY_STATE_DISARMED, "state is DISARMED");

  // No back-door de-escalation except the explicit reset.
  check(!sm.request(ODY_STATE_ARMED, "sneaky rearm"),
        "cannot re-arm through the escalate-only path");
  check(sm.resetFromDisarmed(ODY_STATE_PREFLIGHT_OK, "next flight"),
        "explicit reset out of DISARMED is allowed");
  check(sm.get() == ODY_STATE_PREFLIGHT_OK, "ready for another flight");

  // The reset must not be usable to jump straight back into a flying state.
  sm.request(ODY_STATE_ARMED, "armed");
  sm.request(ODY_STATE_DISARMED, "landed");
  check(!sm.resetFromDisarmed(ODY_STATE_ARMED, "cheat"),
        "reset cannot jump directly to ARMED");

  // Every ordering relation the design depends on.
  check(ODY_STATE_FREEFALL_PARACHUTE > ODY_STATE_FAILSAFE_LANDING,
        "parachute outranks failsafe landing");
  check(ODY_STATE_DISARMED > ODY_STATE_FREEFALL_PARACHUTE,
        "disarm outranks everything");
  check(ODY_STATE_FAILSAFE_LANDING > ODY_STATE_AWAITING_LAND_PERMIT,
        "failsafe landing outranks the land-permission hold");
}

// =====================================================================================
//  FINDING 10 -- the motors-live predicate
// =====================================================================================
static void testMotorsLivePredicate() {
  section("Finding 10: motors-live predicate covers every airborne state");

  check(odyMotorsAreLive(ODY_STATE_ARMED),                "ARMED is live");
  check(odyMotorsAreLive(ODY_STATE_RTH_NAVIGATING),       "RTH_NAVIGATING is live");
  check(odyMotorsAreLive(ODY_STATE_AWAITING_LAND_PERMIT),
        "AWAITING_LAND_PERMIT is live (the original said disarmed)");
  check(odyMotorsAreLive(ODY_STATE_FAILSAFE_LANDING),
        "FAILSAFE_LANDING is live (the original said disarmed)");

  check(!odyMotorsAreLive(ODY_STATE_DISARMED),      "DISARMED is not live");
  check(!odyMotorsAreLive(ODY_STATE_PREFLIGHT_OK),  "PREFLIGHT_OK is not live");
  check(!odyMotorsAreLive(ODY_STATE_FREEFALL_PARACHUTE),
        "FREEFALL_PARACHUTE is not live -- motors are cut");

  // The original's predicate, for contrast.
  int missedByOriginal = 0;
  for (uint8_t s = 0; s < ODY_STATE_COUNT; ++s) {
    const bool original = (s == 3 || s == 4);
    if (odyMotorsAreLive(s) && !original) ++missedByOriginal;
  }
  printf("      the original predicate missed %d live states\n", missedByOriginal);
  check(missedByOriginal > 0, "the original predicate is demonstrably wrong");

  // Airborne must be a superset of motors-live.
  for (uint8_t s = 0; s < ODY_STATE_COUNT; ++s) {
    if (odyMotorsAreLive(s)) check(odyIsAirborneState(s),
                                   "every motors-live state is also airborne");
  }
}

// =====================================================================================
//  FINDINGS 3 / 5 -- link framing, CRC and replay rejection
// =====================================================================================
static void testLinkFraming() {
  section("Findings 3 and 5: link framing, CRC and replay rejection");

  CommandPayload cmd{};
  cmd.sessionId = 0xDEADBEEF;
  cmd.commandId = ODY_CMD_ABORT_TO_LAND;

  uint8_t frame[ODY_MAX_FRAME];
  const uint32_t len = odyEncodeFrame(frame, sizeof(frame),
                                      ODY_FRAME_COMMAND, 0, 42,
                                      &cmd, sizeof(cmd));
  check(len == sizeof(OdyFrameHeader) + sizeof(cmd) + 2, "frame length is as expected");

  OdyFrameHeader hdr; const uint8_t* pl = nullptr; uint8_t plLen = 0;
  check(odyDecodeFrame(frame, len, &hdr, &pl, &plLen), "a clean frame decodes");
  check(hdr.type == ODY_FRAME_COMMAND && hdr.seq == 42, "header round-trips");
  CommandPayload back; memcpy(&back, pl, plLen);
  check(back.sessionId == 0xDEADBEEF && back.commandId == ODY_CMD_ABORT_TO_LAND,
        "payload round-trips");

  // Single-bit corruption anywhere in the frame must be rejected. This is the property
  // that stops a garbled frame from being read as a valid ABORT.
  int missed = 0, tested = 0;
  for (uint32_t byteIdx = 0; byteIdx < len; ++byteIdx) {
    for (int bit = 0; bit < 8; ++bit) {
      uint8_t bad[ODY_MAX_FRAME];
      memcpy(bad, frame, len);
      bad[byteIdx] ^= (uint8_t)(1u << bit);
      ++tested;
      if (odyDecodeFrame(bad, len, &hdr, &pl, &plLen)) ++missed;
    }
  }
  printf("      tested %d single-bit corruptions\n", tested);
  check(missed == 0, "every single-bit corruption is rejected");

  // Truncation and over-length must both be rejected.
  check(!odyDecodeFrame(frame, len - 1, &hdr, &pl, &plLen), "a truncated frame is rejected");
  check(!odyDecodeFrame(frame, len + 1, &hdr, &pl, &plLen), "an over-long frame is rejected");

  // Wrong magic and wrong protocol version.
  uint8_t bad[ODY_MAX_FRAME];
  memcpy(bad, frame, len); bad[0] ^= 0xFF;
  check(!odyDecodeFrame(bad, len, &hdr, &pl, &plLen), "bad magic is rejected");
  memcpy(bad, frame, len); bad[1] = 99;
  check(!odyDecodeFrame(bad, len, &hdr, &pl, &plLen), "wrong protocol version is rejected");

  // Replay protection across the 16-bit wrap.
  check(odySeqIsNewer(43, 42),         "43 is newer than 42");
  check(!odySeqIsNewer(42, 42),        "a replay of the same sequence is rejected");
  check(!odySeqIsNewer(41, 42),        "an out-of-order sequence is rejected");
  check(odySeqIsNewer(0, 65535),       "sequence wrap is handled");
  check(!odySeqIsNewer(65535, 0),      "a replay across the wrap is rejected");
}

// =====================================================================================
//  Notch filter -- the "silently zeroed the gyro" defect
// =====================================================================================
static void testNotchFilter() {
  section("Notch filter: unconfigured is pass-through, not zero");

  BiQuadNotch n;
  check(!n.configured(), "a fresh filter reports itself unconfigured");
  checkNear(n.apply(123.4f), 123.4f, 1e-3f,
            "an unconfigured filter passes the gyro through (was: returned 0)");

  n.configure(NOTCH_CENTER_HZ, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
  check(n.configured(), "configure() takes effect");

  // Drive a sine at the notch centre and one well away from it, and compare the
  // steady-state amplitude that survives.
  auto amplitudeAt = [](float hz) {
    BiQuadNotch f;
    f.configure(NOTCH_CENTER_HZ, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
    float peak = 0.0f;
    const int N = 4000;
    for (int i = 0; i < N; ++i) {
      const float t = (float)i / (float)FLIGHT_LOOP_HZ;
      const float y = f.apply(sinf(2.0f * PI * hz * t));
      if (i > N / 2) peak = max(peak, fabsf(y));
    }
    return peak;
  };

  const float atNotch = amplitudeAt(NOTCH_CENTER_HZ);
  const float atLow   = amplitudeAt(8.0f);
  printf("      amplitude at %.0f Hz (notch centre): %.4f\n", NOTCH_CENTER_HZ, atNotch);
  printf("      amplitude at 8 Hz (control band)   : %.4f\n", atLow);

  check(atNotch < 0.15f, "the notch attenuates its centre frequency by >16 dB");
  check(atLow   > 0.90f, "the control band passes essentially untouched");

  // Degenerate configurations must fall back to pass-through, not emit NaN.
  BiQuadNotch bad;
  bad.configure(0.0f, 500.0f, 4.0f);
  check(!bad.configured(), "a zero centre frequency is refused");
  checkNear(bad.apply(7.0f), 7.0f, 1e-3f, "...and the filter passes through");
  bad.configure(400.0f, 500.0f, 4.0f);      // above Nyquist
  check(!bad.configured(), "a centre above Nyquist is refused");
}

// =====================================================================================
//  PID -- derivative on measurement, anti-windup
// =====================================================================================
static void testPid() {
  section("PID: derivative on measurement, conditional anti-windup");

  PidGains g = GainSets::rollRate();
  Pid pid(g);

  // A setpoint step must not produce a derivative spike. With derivative-on-error the
  // first sample after a step would contribute kd * (step/dt), which at kd = 0.015,
  // step = 100 and dt = 0.002 is 750 counts -- more than twice the output limit.
  pid.reset();
  pid.update(0.0f, 0.0f, 0.002f);        // settle
  pid.update(0.0f, 0.0f, 0.002f);
  const float afterStep = pid.update(100.0f, 0.0f, 0.002f);
  printf("      output on a 100 dps setpoint step: %.1f counts\n", afterStep);
  checkNear(afterStep, g.kp * 100.0f, 5.0f,
            "a setpoint step produces only the proportional term");

  // Anti-windup: hold a large sustained error and confirm the integrator is bounded.
  pid.reset();
  for (int i = 0; i < 5000; ++i) pid.update(1000.0f, 0.0f, 0.002f);
  printf("      integrator after 10 s of saturation: %.1f (limit %.1f)\n",
         pid.integral(), g.iMax);
  check(fabsf(pid.integral()) <= g.iMax + 0.01f, "the integrator respects iMax");
  check(pid.lastOutput() <= g.outMax + 0.01f,    "the output respects outMax");

  // And it must recover promptly once the error reverses -- a wound-up integrator that
  // cannot unwind is what makes a saturated axis feel "sticky".
  for (int i = 0; i < 500; ++i) pid.update(-1000.0f, 0.0f, 0.002f);
  check(pid.lastOutput() < 0.0f, "the controller reverses when the error reverses");

  // A pathological dt must not blow up the derivative.
  pid.reset();
  pid.update(0.0f, 0.0f, 0.002f);
  const float weird = pid.update(0.0f, 50.0f, 0.0f);     // dt of zero
  check(std::isfinite(weird), "a zero dt does not produce NaN or infinity");
  const float huge = pid.update(0.0f, 50.0f, 1000.0f);   // absurd dt
  check(std::isfinite(huge), "an absurd dt does not produce NaN or infinity");
}

// =====================================================================================
//  Debounce -- the battery threshold latch
// =====================================================================================
static void testDebounce() {
  section("Debounce: a transient sag cannot latch a mode change");

  DebouncedCondition d(BATT_DEBOUNCE_MS);
  g_millis = 1000;

  // A 1-second sag, well inside the 3-second window, must not trip.
  bool tripped = false;
  for (int i = 0; i < 50; ++i) { g_millis += 20; if (d.update(true, g_millis)) tripped = true; }
  check(!tripped, "a 1 s transient does not trip the threshold");

  d.update(false, g_millis);       // sag recovers
  check(d.heldMs(g_millis) == 0, "the timer resets when the condition clears");

  // A sustained condition must trip after the full window.
  tripped = false;
  const uint32_t start = g_millis;
  while (g_millis - start < BATT_DEBOUNCE_MS + 100) {
    g_millis += 20;
    if (d.update(true, g_millis)) { tripped = true; break; }
  }
  check(tripped, "a sustained condition trips after the debounce window");
  check(g_millis - start >= BATT_DEBOUNCE_MS, "...and not before");
}

// =====================================================================================
//  DYNAMIC NOTCH
//
//  The justification for this feature is that a measured centre beats a compiled one.
//  That only holds if the tracker genuinely finds the right frequency, and if a tracker
//  that goes wrong is no worse than the constant it replaced. Both claims are tested
//  here against synthetic gyro signals whose true peak is known exactly.
// =====================================================================================

// Drives a tracker with a tone plus deterministic broadband noise and returns it.
// Noise matters: a tracker that only works on a clean sine is not evidence of anything,
// because a real gyro trace never looks like that.
static void driveTracker(DynamicNotchTracker& tr, float sampleRate, float toneHz,
                         float toneAmp, float noiseAmp, float seconds) {
  const int samples   = (int)(sampleRate * seconds);
  const int perUpdate = (int)(sampleRate / DYN_NOTCH_UPDATE_HZ);
  uint32_t rng = 12345u;
  for (int i = 0; i < samples; ++i) {
    rng = rng * 1664525u + 1013904223u;
    const float noise = (((float)(rng >> 8) / 8388608.0f) - 1.0f) * noiseAmp;
    const float tsec  = (float)i / sampleRate;
    tr.push(toneAmp * sinf(2.0f * (float)M_PI * toneHz * tsec) + noise);
    if (perUpdate > 0 && i % perUpdate == 0) tr.update();
  }
}

static void testDynamicNotchTracking() {
  section("Dynamic notch: does it find the peak");

  DynamicNotchTracker probe;
  probe.begin(500.0f, 120.0f);
  checkNear(probe.bandLowHz(), 72.0f, 0.5f,
            "search band starts at 0.6x the static centre");
  check(probe.bandHighHz() <= (float)IMU_DLPF_HZ + 0.01f,
        "search band is capped at the IMU DLPF corner, not extended past it");
  checkNear(probe.centreHz(), 120.0f, 0.01f,
            "before any samples arrive it sits on the static centre");

  const float tones[] = { 90.0f, 105.0f, 120.0f, 140.0f, 160.0f };
  for (float tone : tones) {
    DynamicNotchTracker tr;
    tr.begin(500.0f, 120.0f);
    driveTracker(tr, 500.0f, tone, 10.0f, 0.5f, 8.0f);
    check(tr.state().tracking,
          "tracking engages on a " + std::to_string((int)tone) + " Hz tone");
    checkNear(tr.centreHz(), tone, 6.0f,
              "locks onto a " + std::to_string((int)tone) + " Hz tone");
  }

  // Parabolic interpolation should beat the 3.9 Hz bin spacing.
  DynamicNotchTracker odd;
  odd.begin(500.0f, 120.0f);
  driveTracker(odd, 500.0f, 117.3f, 10.0f, 0.3f, 10.0f);
  checkNear(odd.centreHz(), 117.3f, 4.0f,
            "resolves a frequency that falls between two bins");

  // A peak buried in comparable noise is not a peak worth chasing.
  DynamicNotchTracker buried;
  buried.begin(500.0f, 120.0f);
  driveTracker(buried, 500.0f, 140.0f, 0.4f, 10.0f, 8.0f);
  check(buried.centreHz() >= buried.bandLowHz() - 0.01f &&
        buried.centreHz() <= buried.bandHighHz() + 0.01f,
        "a tone buried in noise leaves the centre inside the permitted band");
}

static void testDynamicNotchRejectsNoise() {
  section("Dynamic notch: what it refuses to believe");

  // Broadband noise with no motor in it. The magnitude gate alone would not catch this
  // -- across ~30 bins white noise produces a peak-to-mean near 4 about half the time.
  // The bin-repeat gate is what rejects it.
  DynamicNotchTracker noisy;
  noisy.begin(500.0f, 120.0f);
  driveTracker(noisy, 500.0f, 0.0f, 0.0f, 10.0f, 10.0f);
  check(!noisy.state().tracking, "pure broadband noise does not engage tracking");
  checkNear(noisy.centreHz(), 120.0f, 3.0f,
            "and the centre stays on the static value");

  // A silent gyro is the same case with no ambiguity at all.
  DynamicNotchTracker quiet;
  quiet.begin(500.0f, 120.0f);
  driveTracker(quiet, 500.0f, 0.0f, 0.0f, 0.0f, 6.0f);
  check(!quiet.state().tracking, "a silent gyro does not engage tracking");
  checkNear(quiet.centreHz(), 120.0f, 0.01f,
            "and the centre stays exactly on the static value");

  // Nothing is reported before the window has filled.
  DynamicNotchTracker cold;
  cold.begin(500.0f, 120.0f);
  for (int i = 0; i < DYN_NOTCH_BINS / 2; ++i) {
    cold.push(10.0f * sinf(2.0f * (float)M_PI * 140.0f * (float)i / 500.0f));
    cold.update();
  }
  check(!cold.state().tracking, "no verdict before the analysis window has filled");
  checkNear(cold.centreHz(), 120.0f, 0.01f, "and no movement either");
}

static void testDynamicNotchIsBounded() {
  section("Dynamic notch: it cannot do worse than the static value");

  // This is the safety argument for enabling it by default, so it gets tested hardest.
  // A strong tone well OUTSIDE the search band must not drag the notch out with it.
  const float outOfBand[] = { 10.0f, 30.0f, 45.0f, 200.0f, 230.0f, 245.0f };
  for (float tone : outOfBand) {
    DynamicNotchTracker tr;
    tr.begin(500.0f, 120.0f);
    driveTracker(tr, 500.0f, tone, 15.0f, 0.5f, 10.0f);
    check(tr.centreHz() >= tr.bandLowHz() - 0.01f &&
          tr.centreHz() <= tr.bandHighHz() + 0.01f,
          "a " + std::to_string((int)tone) + " Hz out-of-band tone cannot pull the "
          "centre outside the permitted band");
  }

  // The centre must never leave the band under any input, including absurd ones.
  DynamicNotchTracker wild;
  wild.begin(500.0f, 120.0f);
  for (int i = 0; i < 2000; ++i) wild.push((i % 2) ? 1.0e6f : -1.0e6f);
  wild.update();
  check(std::isfinite(wild.centreHz()), "extreme input does not produce NaN or inf");
  check(wild.centreHz() >= wild.bandLowHz() - 0.01f &&
        wild.centreHz() <= wild.bandHighHz() + 0.01f,
        "extreme input leaves the centre inside the permitted band");

  // Slew limiting: one update cannot traverse the band.
  DynamicNotchTracker slew;
  slew.begin(500.0f, 120.0f);
  const float startHz = slew.centreHz();
  driveTracker(slew, 500.0f, 180.0f, 15.0f, 0.5f, 0.30f);
  const float maxStep = DYN_NOTCH_SLEW_HZ_PER_S / (float)DYN_NOTCH_UPDATE_HZ;
  check(fabsf(slew.centreHz() - startHz) <= maxStep * 8.0f,
        "the centre is rate-limited, not teleported, when the peak moves");

  // The band itself must always straddle the static centre, whatever it is set to.
  const float statics[] = { 88.0f, 90.0f, 100.0f, 105.0f, 120.0f, 150.0f, 180.0f };
  for (float s : statics) {
    DynamicNotchTracker tr;
    tr.begin(500.0f, s);
    check(tr.bandLowHz() < s, "band floor sits below a " +
          std::to_string((int)s) + " Hz static centre");
    check(tr.bandHighHz() > tr.bandLowHz(),
          "band is non-empty for a " + std::to_string((int)s) + " Hz static centre");
  }
}

static void testDynamicNotchAcrossBuilds() {
  section("Dynamic notch: every characterised build");

  // The notch spans 88-180 Hz across the ten frame/motor/propeller combinations, and
  // the 7-inch runs a 1000 Hz loop. The tracker has to work at all of them, not just
  // at the one the tests happen to be compiled for.
  struct Case { float fs; float staticHz; float tone; const char* name; };
  const Case cases[] = {
    { 1000.0f, 180.0f, 172.0f, "7-inch 2-blade, 1000 Hz loop" },
    { 1000.0f, 150.0f, 155.0f, "7-inch 3-blade, 1000 Hz loop" },
    {  500.0f, 120.0f, 124.0f, "9-inch 2-blade (default)" },
    {  500.0f, 100.0f, 103.0f, "9-inch 3-blade" },
    {  500.0f, 105.0f, 100.0f, "10-inch 2-blade" },
    {  500.0f,  90.0f,  88.0f, "10-inch 3-blade" },
  };
  for (const Case& c : cases) {
    DynamicNotchTracker tr;
    tr.begin(c.fs, c.staticHz);
    driveTracker(tr, c.fs, c.tone, 10.0f, 0.5f, 8.0f);
    check(tr.state().tracking && fabsf(tr.centreHz() - c.tone) < 10.0f,
          std::string(c.name) + ": locks near " + std::to_string((int)c.tone) +
          " Hz (got " + std::to_string((int)tr.centreHz()) + " Hz)");
  }
}

static void testNotchIsNotObservableInTheLog() {
  section("BlackBox: why the notch verdict is logged, not the spectrum");

  // Section 8.3 records a defect that survived every revision to 2.9: the reader was
  // told to find the motor peak in a BlackBox gyro trace. These assertions state, in
  // the firmware's own constants, why that could never work -- so the arithmetic lives
  // next to the code rather than only in prose.
  const float logNyquist = BLACKBOX_LOG_HZ / 2.0f;

  check(PROP_NOTCH_DEFAULT_HZ > logNyquist,
        "the motor peak is above the log's Nyquist limit, so it cannot appear in a "
        "gyro trace at its own frequency");

  // Where it does appear, if anyone tries.
  float aliased = fmodf(PROP_NOTCH_DEFAULT_HZ, (float)BLACKBOX_LOG_HZ);
  if (aliased > logNyquist) aliased = BLACKBOX_LOG_HZ - aliased;
  check(fabsf(aliased - PROP_NOTCH_DEFAULT_HZ) > 1.0f,
        "the peak aliases to a different frequency entirely in the log");

  // The verdict, by contrast, is a slow scalar and samples fine at the log rate.
  check((float)DYN_NOTCH_UPDATE_HZ < logNyquist,
        "the tracker's update rate IS below the log's Nyquist limit, which is why "
        "logging its verdict works where logging the spectrum cannot");

  // And the record must actually carry it.
  BlackBoxRecord rec{};
  rec.notchCentreDeciHz = 1187;
  rec.notchFlags = ODY_NOTCH_FLAG_TRACKING | ODY_NOTCH_FLAG_DYNAMIC;
  checkNear(rec.notchCentreDeciHz / 10.0f, 118.7f, 0.01f,
            "the notch centre round-trips through the record at 0.1 Hz resolution");
  check((rec.notchFlags & ODY_NOTCH_FLAG_TRACKING) != 0,
        "the tracking flag is readable back out");
  check((rec.notchFlags & ODY_NOTCH_FLAG_DYNAMIC) != 0,
        "the dynamic-enabled flag is independent of it");

  // The layout the decoder assumes. The consistency check verifies this against the
  // compiler field by field; this catches it at test time too.
  check(sizeof(BlackBoxRecord) == 54,
        "the v4 record is 54 bytes, as the decoder's format string expects");
  check(BLACKBOX_VERSION == 4, "the firmware writes log format v4");

  // The harmonic's own fields. The OBSERVABLE flag is the one that matters on this
  // hardware: it separates "the tracker found nothing" from "the IMU cannot see it",
  // and on most builds the answer is the second.
  rec.notchHarmonicDeciHz = 1760;
  rec.notchFlags |= ODY_NOTCH_FLAG_H2_TRACKING | ODY_NOTCH_FLAG_H2_VISIBLE;
  checkNear(rec.notchHarmonicDeciHz / 10.0f, 176.0f, 0.01f,
            "the harmonic centre round-trips at 0.1 Hz resolution");
  check((rec.notchFlags & ODY_NOTCH_FLAG_H2_TRACKING) != 0,
        "the harmonic tracking flag is readable back out");
  check((rec.notchFlags & ODY_NOTCH_FLAG_H2_VISIBLE) != 0,
        "the harmonic observability flag is independent of it");
  check(ODY_NOTCH_FLAG_TRACKING != ODY_NOTCH_FLAG_H2_TRACKING &&
        ODY_NOTCH_FLAG_DYNAMIC  != ODY_NOTCH_FLAG_H2_VISIBLE,
        "all four notch flags occupy distinct bits");

  // On this build, can the harmonic be seen at all? State the answer either way.
  const float h2 = PROP_NOTCH_DEFAULT_HZ * DYN_NOTCH_H2_MULTIPLE;
  const float ceiling = min((float)IMU_DLPF_HZ,
                            (float)FLIGHT_LOOP_HZ * 0.5f * DYN_NOTCH_H2_NYQUIST_FRAC);
  if (h2 <= ceiling) {
    check(true, "this build CAN observe its second harmonic");
  } else {
    check(h2 > IMU_DLPF_HZ || h2 > (float)FLIGHT_LOOP_HZ * 0.5f * DYN_NOTCH_H2_NYQUIST_FRAC,
          "this build cannot observe its second harmonic, and the reason is the IMU "
          "anti-alias corner or the Nyquist margin -- not a tracker failure");
  }
}

// Same driver, plus energy at a second frequency -- a real propeller puts a harmonic
// there and the tracker has to tell it apart from the fundamental.
static void driveTracker2(DynamicNotchTracker& tr, float sampleRate,
                          float toneHz, float toneAmp,
                          float harmHz, float harmAmp,
                          float noiseAmp, float seconds) {
  const int samples   = (int)(sampleRate * seconds);
  const int perUpdate = (int)(sampleRate / DYN_NOTCH_UPDATE_HZ);
  uint32_t rng = 12345u;
  for (int i = 0; i < samples; ++i) {
    rng = rng * 1664525u + 1013904223u;
    const float noise = (((float)(rng >> 8) / 8388608.0f) - 1.0f) * noiseAmp;
    const float tsec  = (float)i / sampleRate;
    float x = toneAmp * sinf(2.0f * (float)M_PI * toneHz * tsec) + noise;
    if (harmAmp > 0.0f) x += harmAmp * sinf(2.0f * (float)M_PI * harmHz * tsec);
    tr.push(x);
    if (perUpdate > 0 && i % perUpdate == 0) tr.update();
  }
}

static void testHarmonicObservability() {
  section("Second harmonic: only tracked where the hardware can see it");

  // The ceiling is the IMU's anti-alias corner, or a margin below Nyquist, whichever
  // is lower. Above it there is nothing but the DLPF's own roll-off and aliased energy.
  DynamicNotchTracker probe;
  probe.begin(500.0f, 120.0f);
  const float expected = min((float)IMU_DLPF_HZ,
                             500.0f * 0.5f * DYN_NOTCH_H2_NYQUIST_FRAC);
  checkNear(probe.ceilingHz(), expected, 0.01f,
            "the search ceiling is the lower of the DLPF corner and the Nyquist margin");

  // The 9-inch default: f0 = 120, so the harmonic would be 240 Hz -- above a 184 Hz
  // DLPF corner and at 96% of Nyquist. It must NOT be tracked, however strong it is.
  DynamicNotchTracker deaf;
  deaf.begin(500.0f, 120.0f);
  driveTracker2(deaf, 500.0f, 120.0f, 10.0f, 240.0f, 10.0f, 0.5f, 8.0f);
  check(deaf.state().tracking, "the fundamental still locks at 120 Hz");
  check(!deaf.state().harmonicObservable,
        "a 240 Hz harmonic is correctly judged unobservable on the 9-inch default");
  check(!deaf.state().harmonicTracking,
        "...so it is not tracked, however much energy is put there");
  checkNear(deaf.harmonicHz(), 0.0f, 0.01f, "...and no harmonic notch is requested");

  // The 10-inch 3-blade: f0 = 90, harmonic at 180 Hz, under the 184 Hz corner.
  DynamicNotchTracker sees;
  sees.begin(500.0f, 88.0f);
  check(sees.state().harmonicObservable,
        "a 176 Hz harmonic IS judged observable on the 10-inch 3-blade");
  driveTracker2(sees, 500.0f, 88.0f, 10.0f, 176.0f, 6.0f, 0.5f, 10.0f);
  check(sees.state().tracking, "the fundamental locks at 88 Hz");
#if DYN_NOTCH_HARMONIC
  check(sees.state().harmonicTracking, "and the harmonic locks too");
  checkNear(sees.harmonicHz(), 176.0f, 8.0f, "the harmonic is found near 2x f0");
#else
  // Compiled out. The fundamental must be unaffected and no harmonic notch may ever
  // be requested -- the disable switch has to actually disable.
  check(!sees.state().harmonicTracking,
        "with DYN_NOTCH_HARMONIC=0 the harmonic never locks, even when it is there");
  checkNear(sees.harmonicHz(), 0.0f, 0.01f,
            "...and no harmonic notch is ever requested");
#endif
}

static void testHarmonicIsNotAssumed() {
  section("Second harmonic: measured, not calculated");

  // A harmonic notch placed at exactly 2x the fundamental by arithmetic would be the
  // same mistake as a fixed notch. The tracker searches a window, so a harmonic that
  // is not an exact multiple is still found where it actually is.
  DynamicNotchTracker tr;
  tr.begin(500.0f, 88.0f);
  driveTracker2(tr, 500.0f, 88.0f, 10.0f, 168.0f, 6.0f, 0.5f, 10.0f);
#if DYN_NOTCH_HARMONIC
  check(tr.state().harmonicTracking, "an off-multiple harmonic still locks");
  check(fabsf(tr.harmonicHz() - 168.0f) < fabsf(tr.harmonicHz() - 176.0f),
        "it is found at 168 Hz where the energy is, not at the arithmetic 176 Hz");
#else
  check(!tr.state().harmonicTracking, "compiled out, so nothing locks");
  check(tr.harmonicHz() == 0.0f, "compiled out, so no notch is requested");
#endif

  // No harmonic energy at all: nothing to notch, so nothing is notched.
  DynamicNotchTracker none;
  none.begin(500.0f, 88.0f);
  driveTracker2(none, 500.0f, 88.0f, 10.0f, 0.0f, 0.0f, 0.5f, 10.0f);
  check(none.state().tracking, "the fundamental locks");
  check(!none.state().harmonicTracking,
        "a clean fundamental with no overtone does not produce a harmonic notch");

  // Broadband noise in the harmonic window must not be mistaken for an overtone --
  // same bin-repeat gate as the fundamental.
  DynamicNotchTracker noisy;
  noisy.begin(500.0f, 88.0f);
  driveTracker2(noisy, 500.0f, 88.0f, 10.0f, 0.0f, 0.0f, 8.0f, 10.0f);
  check(!noisy.state().harmonicTracking,
        "noise in the harmonic window is not mistaken for an overtone");
}

static void testHarmonicStaysInBounds() {
  section("Second harmonic: bounded like the fundamental");

  // Whatever it locks onto, it can never sit above the ceiling -- that is the whole
  // basis for enabling it by default.
  const float statics[] = { 60.0f, 75.0f, 88.0f, 90.0f, 100.0f, 120.0f, 150.0f };
  for (float s : statics) {
    DynamicNotchTracker tr;
    tr.begin(500.0f, s);
    driveTracker2(tr, 500.0f, s, 12.0f, 2.0f * s, 10.0f, 1.0f, 8.0f);
    check(tr.harmonicHz() <= tr.ceilingHz() + 0.01f,
          "a " + std::to_string((int)s) + " Hz fundamental never puts the harmonic "
          "notch above the ceiling");
    check(tr.harmonicHz() == 0.0f || tr.harmonicHz() > tr.centreHz(),
          "the harmonic notch is never placed below the fundamental it belongs to");
  }

  // If the fundamental never locks, there is no fundamental to take a harmonic of.
  DynamicNotchTracker unlocked;
  unlocked.begin(500.0f, 88.0f);
  driveTracker2(unlocked, 500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 8.0f);
  check(!unlocked.state().tracking, "the fundamental does not lock on pure noise");
  check(!unlocked.state().harmonicTracking,
        "and no harmonic is claimed without a fundamental to derive it from");
}

// =====================================================================================
//  FINDING 17 -- Remote ID identifiers
// =====================================================================================
static void testCtaSerial() {
  section("Remote ID: CTA-2063-A hardware serial");

  // The worked example: 4-char ICAO code, F = 15 characters, then 15 characters.
  check(odyValidateCtaSerial("K7E3F000000000000001") == ODY_ID_OK,
        "a valid 20-character serial is accepted");
  check(strlen("K7E3F000000000000001") == ODY_CTA_SERIAL_MAX,
        "...and it fills the 20-byte OpenDroneID UAS ID field exactly");

  // Shorter length codes.
  check(odyValidateCtaSerial("K7E31A") == ODY_ID_OK,
        "length code '1' with a 1-character serial");
  check(odyValidateCtaSerial("K7E3C00000000ABCD") == ODY_ID_OK,
        "length code 'C' with a 12-character serial");

  // The placeholder that shipped in revision 2.0 was malformed: 'P' is not a valid
  // length code, and the serial that followed it was 16 characters -- over the 15 max.
  check(odyValidateCtaSerial("ODY1P0000000000000000") != ODY_ID_OK,
        "the original malformed placeholder is REJECTED");
  checkNear((float)odyValidateCtaSerial("ODY1P0000000000000000"),
            (float)ODY_ID_ERR_LENGTH, 0.01f,
            "...rejected for being over the maximum length");

  // Length code must agree with the actual serial length.
  check(odyValidateCtaSerial("K7E3F0001") == ODY_ID_ERR_LENGTH_MISMATCH,
        "a declared length that disagrees with the serial is rejected");
  check(odyValidateCtaSerial("K7E3G0001") == ODY_ID_ERR_LENGTH_CODE,
        "'G' is not a valid length code (max is F = 15)");

  // I and O are excluded from the alphabet to avoid confusion with 1 and 0.
  check(odyValidateCtaSerial("K7E34ABIC") == ODY_ID_ERR_CHARSET,
        "the letter I is rejected");
  check(odyValidateCtaSerial("K7E34ABOC") == ODY_ID_ERR_CHARSET,
        "the letter O is rejected");
  check(odyIsCtaChar('0') && odyIsCtaChar('9') && odyIsCtaChar('A') && odyIsCtaChar('Z'),
        "digits and A-Z are permitted");
  check(!odyIsCtaChar('I') && !odyIsCtaChar('O') && !odyIsCtaChar('a'),
        "I, O and lower case are not permitted");

  check(odyValidateCtaSerial(nullptr) == ODY_ID_ERR_NULL, "a null serial is rejected");
  check(odyValidateCtaSerial("") == ODY_ID_ERR_LENGTH, "an empty serial is rejected");

  // Construction round-trips through validation.
  char built[ODY_CTA_SERIAL_MAX + 1];
  check(odyMakeCtaSerial("K7E3", "7C2P4K8M1X6R3T9", built, sizeof(built)) == ODY_ID_OK,
        "a 15-character serial is built");
  printf("      built: %s\n", built);
  check(strcmp(built, "K7E3F7C2P4K8M1X6R3T9") == 0, "...with the expected layout");
  check(odyValidateCtaSerial(built) == ODY_ID_OK, "...and it validates");

  check(odyMakeCtaSerial("K7E3", "0123456789ABCDEF", built, sizeof(built))
          == ODY_ID_ERR_LENGTH,
        "a 16-character serial is refused (15 is the maximum)");
  check(odyMakeCtaSerial("K7E3", "ABIC", built, sizeof(built)) == ODY_ID_ERR_CHARSET,
        "a serial containing I is refused at construction");
  check(odyMakeCtaSerial("K7", "0001", built, sizeof(built)) == ODY_ID_ERR_MFR_CODE,
        "a short manufacturer code is refused");
}

static void testCaaRegistration() {
  section("Remote ID: CAA registration -- the route that needs no ICAO code");

  check(odyValidateCaaRegistration("DNK87astrdge12k8") == ODY_ID_OK,
        "a Danish operator registration is accepted as the UAS ID");
  check(odyValidateCaaRegistration("G-ABCD") == ODY_ID_OK,
        "a short national registration is accepted");
  check(odyValidateCaaRegistration("FA3X9-771.2_A") == ODY_ID_OK,
        "hyphens, dots and underscores are accepted");

  // The placeholder must never reach the air.
  check(odyValidateCaaRegistration("SET-YOUR-CAA-REGISTRATION") == ODY_ID_ERR_PLACEHOLDER,
        "the shipped placeholder is rejected");

  check(odyValidateCaaRegistration("") == ODY_ID_ERR_LENGTH,
        "an empty registration is rejected");
  check(odyValidateCaaRegistration("123456789012345678901") == ODY_ID_ERR_LENGTH,
        "21 characters exceeds the 20-byte ODID field");
  check(odyValidateCaaRegistration("ABC DEF") == ODY_ID_ERR_CHARSET,
        "a space is rejected");
  check(odyValidateCaaRegistration(nullptr) == ODY_ID_ERR_NULL,
        "a null registration is rejected");

  // Exactly 20 characters must fit -- that is the ODID field width.
  check(odyValidateCaaRegistration("12345678901234567890") == ODY_ID_OK,
        "exactly 20 characters fits the ODID UAS ID field");

  // The point of this route: no ICAO manufacturer code is involved anywhere.
  check(odyValidateCaaRegistration("DNK87astrdge12k8") == ODY_ID_OK
        && odyValidateCtaSerial("DNK87astrdge12k8") != ODY_ID_OK,
        "a CAA registration need not be a valid CTA serial -- different schemes");
}

static void testOperatorId() {
  section("Remote ID: EU / Danish operator registration number");

  // EASA's published example. This is the ONLY vector available, which is why the
  // checksum is advisory rather than blocking -- see identity.h.
  const char* easa = "FIN87astrdge12k8";
  check(odyValidateOperatorIdPublic(easa, false) == ODY_ID_OK,
        "the EASA example passes structural validation");

  char payload[13];
  memcpy(payload, easa + 3, 12);
  payload[12] = '\0';
  const char ck = odyOperatorChecksum(payload);
  printf("      payload '%s' -> check '%c' (published: '%c')\n",
         payload, ck, easa[15]);
  check(ck == easa[15], "the Luhn mod 36 implementation reproduces the published check");
  check(odyValidateOperatorIdPublic(easa, true) == ODY_ID_OK,
        "...so strict validation also passes");

  // A Danish-prefixed identifier of the same shape.
  check(odyValidateOperatorIdPublic("DNK87astrdge12k8", false) == ODY_ID_OK,
        "a DNK-prefixed identifier passes structural validation");

  // Structure.
  check(odyValidateOperatorIdPublic("FIN87astrdge12k", false) == ODY_ID_ERR_LENGTH,
        "15 characters is rejected");
  check(odyValidateOperatorIdPublic("FIN87astrdge12k88", false) == ODY_ID_ERR_LENGTH,
        "17 characters is rejected");
  check(odyValidateOperatorIdPublic("F1N87astrdge12k8", false) == ODY_ID_ERR_COUNTRY,
        "a country prefix containing a digit is rejected");
  check(odyValidateOperatorIdPublic("FIN87astrdge12k-", false) == ODY_ID_ERR_CHARSET,
        "a non-alphanumeric body character is rejected");

  // A wrong check character is caught in strict mode and tolerated otherwise. This
  // asymmetry is deliberate: strict mode must never sit in a path that can block flight.
  check(odyValidateOperatorIdPublic("FIN87astrdge12k9", true) == ODY_ID_ERR_CHECKSUM,
        "a wrong check character fails STRICT validation");
  check(odyValidateOperatorIdPublic("FIN87astrdge12k9", false) == ODY_ID_OK,
        "...and passes non-strict validation, so it cannot ground the aircraft");

  // Splitting the public part from the secret.
  char pub[ODY_OPERATOR_PUBLIC_LEN + 1], sec[ODY_OPERATOR_SECRET_LEN + 1];
  check(odySplitOperatorId("FIN87astrdge12k8-xyz", pub, sizeof(pub),
                           sec, sizeof(sec)) == ODY_ID_OK,
        "a full identifier splits");
  check(strcmp(pub, "FIN87astrdge12k8") == 0, "...the public part is the first 16");
  check(strcmp(sec, "xyz") == 0,              "...the secret is the trailing 3");
  check(odyValidateOperatorIdFull("FIN87astrdge12k8-xyz", false) == ODY_ID_OK,
        "the full identifier validates");

  check(odySplitOperatorId("FIN87astrdge12k8xyz", pub, sizeof(pub),
                           sec, sizeof(sec)) == ODY_ID_ERR_SEPARATOR,
        "a missing separator is rejected");
  check(odyValidateOperatorIdFull("FIN87astrdge12k8-xy", false) == ODY_ID_ERR_LENGTH,
        "a 2-character secret is rejected");

  // The secret must never appear in the broadcast identity. Guard the public accessor.
  check(odyValidateOperatorIdPublic("FIN87astrdge12k8-xyz", false) == ODY_ID_ERR_LENGTH,
        "a full identifier is not accepted where only the public part belongs");
}

// =====================================================================================
int main() {
  printf("=====================================================================\n");
  printf(" Odyssey-10 Pro -- host verification of review-finding fixes\n");
  printf("=====================================================================\n");

  testBatteryThresholds();
  testMixerGeometry();
  testMixerDesaturation();
  testMixerYawSacrifice();
  testStateEscalation();
  testMotorsLivePredicate();
  testLinkFraming();
  testNotchFilter();
  testPid();
  testDebounce();
  testNotchIsNotObservableInTheLog();
  testDynamicNotchTracking();
  testDynamicNotchRejectsNoise();
  testDynamicNotchIsBounded();
  testDynamicNotchAcrossBuilds();
  testHarmonicObservability();
  testHarmonicIsNotAssumed();
  testHarmonicStaysInBounds();
  testCtaSerial();
  testCaaRegistration();
  testOperatorId();

  printf("\n=====================================================================\n");
  printf(" %d passed, %d failed\n", g_pass, g_fail);
  printf("=====================================================================\n");
  return g_fail == 0 ? 0 : 1;
}
