// =====================================================================================
//  Odyssey-10 Pro -- Quad-X mixer with desaturation
//  ------------------------------------------------------------------------------------
//  FIX FOR FINDING 13.
//
//  The original mixer summed throttle and the three axis corrections into each motor
//  and then clamped each motor independently inside writeMotorPWM():
//
//      m1 = base - roll + pitch + yaw;      ...      ledcWrite(pin, constrain(m, MIN, MAX));
//
//  That is only correct while nothing clips. With base near its ceiling of
//  PWM_MAX - 300 = 2976 and a full roll correction of 350, motors 3 and 4 compute to
//  3326 and get clamped to 3276 (losing 50 counts) while motors 1 and 2 drop cleanly
//  to 2626. The realised roll differential is 650 instead of the commanded 700. Stack
//  roll + pitch + yaw (up to 950 counts) and the shortfall grows large enough to
//  reverse the net moment on an axis: the aircraft rolls AWAY from the correction.
//
//  The fix is the standard two-stage approach:
//
//    1. Build the correction-only mix (no throttle). If its peak-to-peak range exceeds
//       the available motor range, scale ALL corrections by a single common factor.
//       Scaling uniformly preserves the direction of the commanded moment, which is the
//       property independent clamping destroys.
//
//    2. Choose a throttle offset that fits the scaled mix inside [PWM_MIN, PWM_MAX].
//       Throttle is the term we are willing to give up; attitude authority is not.
//
//  Yaw is scaled back before roll and pitch when there is not enough range for all
//  three, because losing yaw authority costs heading and losing roll or pitch authority
//  costs the aircraft.
// =====================================================================================

#ifndef ODY_MIXER_H
#define ODY_MIXER_H

#include <Arduino.h>
#include "config.h"

struct MixerOutput {
  uint16_t motor[4] = { PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN };
  float    saturation = 0.0f;   // 0 = full authority, 1 = all correction authority lost
  bool     throttleClipped = false;
};

class Mixer {
public:
  // Motor index order is [M1, M2, M3, M4] = rear-right, front-right, rear-left,
  // front-left, matching config.h and the section 4 airframe diagram.
  //
  //   roll  : positive rolls right, so the right-hand motors (M1, M2) give up thrust
  //   pitch : positive pitches nose up, so the rear motors (M1, M3) give up thrust
  //   yaw   : positive yaws nose right; the CCW pair (M1 rear-right, M4 front-left)
  //           spins up, matching the props-out rotation defined in config.h
  //
  // The yaw column is the one the original section 4 diagram would have inverted.
  static constexpr float kRoll [4] = { -1.0f, -1.0f, +1.0f, +1.0f };
  static constexpr float kPitch[4] = { -1.0f, +1.0f, -1.0f, +1.0f };
  static constexpr float kYaw  [4] = { +1.0f, -1.0f, -1.0f, +1.0f };

  // throttleCounts is the desired offset above PWM_MIN, in the range
  // [0, PWM_RANGE]. rollOut/pitchOut/yawOut come straight from the rate PIDs.
  static MixerOutput mix(float throttleCounts,
                         float rollOut, float pitchOut, float yawOut) {
    MixerOutput out;

    // ---- Stage 1: correction-only mix -------------------------------------------
    float m[4];
    buildMix(m, rollOut, pitchOut, yawOut);

    float lo = m[0], hi = m[0];
    for (int i = 1; i < 4; ++i) { lo = min(lo, m[i]); hi = max(hi, m[i]); }
    float range = hi - lo;

    const float available = (float)PWM_RANGE;

    if (range > available) {
      // Not enough motor range for everything. Sacrifice yaw first: recompute with
      // yaw scaled down and see whether roll and pitch now fit on their own.
      const float rpRange = axisRange(rollOut, pitchOut, 0.0f);

      if (rpRange <= available) {
        // Roll and pitch fit. Find the largest yaw that still fits.
        float yawScale = 1.0f;
        const float yawHeadroom = available - rpRange;
        const float yawRange    = axisRange(0.0f, 0.0f, yawOut);
        if (yawRange > 0.0f) yawScale = constrain(yawHeadroom / yawRange, 0.0f, 1.0f);

        buildMix(m, rollOut, pitchOut, yawOut * yawScale);
        out.saturation = 1.0f - yawScale;
      } else {
        // Even roll and pitch alone will not fit. Scale everything by one common
        // factor so the commanded moment keeps its direction.
        const float scale = available / range;
        buildMix(m, rollOut * scale, pitchOut * scale, yawOut * scale);
        out.saturation = 1.0f - scale;
      }

      lo = m[0]; hi = m[0];
      for (int i = 1; i < 4; ++i) { lo = min(lo, m[i]); hi = max(hi, m[i]); }
    }

    // ---- Stage 2: fit the throttle offset ---------------------------------------
    // Any offset in [-lo, PWM_RANGE - hi] keeps all four motors inside the endpoints.
    const float offsetMin = -lo;
    const float offsetMax = available - hi;
    float offset = constrain(throttleCounts, offsetMin, offsetMax);
    out.throttleClipped = (offset != throttleCounts);

    for (int i = 0; i < 4; ++i) {
      const float v = (float)PWM_MIN + offset + m[i];
      // This constrain is now a belt-and-braces guard against float rounding only;
      // the arithmetic above has already guaranteed the range.
      out.motor[i] = (uint16_t)constrain((int)lroundf(v), PWM_MIN, PWM_MAX);
    }
    return out;
  }

  // Maps a 1000..2000 us stick position onto the usable throttle band. The top of the
  // band is held back by MIXER_HEADROOM_COUNTS so that full stick still leaves room to
  // stabilise -- without it, stage 2 would be forced to steal throttle on every
  // correction at full stick, which the pilot feels as the aircraft sagging when they
  // roll hard.
  static float throttleFromStick(uint16_t stickUs) {
    const uint16_t clamped = constrain(stickUs, (uint16_t)1000, (uint16_t)2000);
    const float    span    = (float)(PWM_RANGE - MIXER_HEADROOM_COUNTS)
                           - (float)(PWM_ARM_IDLE - PWM_MIN);
    return (float)(PWM_ARM_IDLE - PWM_MIN)
         + ((float)(clamped - 1000) / 1000.0f) * span;
  }

  static MixerOutput allIdle() {
    MixerOutput out;
    for (int i = 0; i < 4; ++i) out.motor[i] = PWM_MIN;
    return out;
  }

private:
  static void buildMix(float* m, float r, float p, float y) {
    for (int i = 0; i < 4; ++i) {
      m[i] = r * kRoll[i] + p * kPitch[i] + y * kYaw[i];
    }
  }

  static float axisRange(float r, float p, float y) {
    float m[4];
    for (int i = 0; i < 4; ++i) m[i] = r * kRoll[i] + p * kPitch[i] + y * kYaw[i];
    float lo = m[0], hi = m[0];
    for (int i = 1; i < 4; ++i) { lo = min(lo, m[i]); hi = max(hi, m[i]); }
    return hi - lo;
  }
};

#endif // ODY_MIXER_H
