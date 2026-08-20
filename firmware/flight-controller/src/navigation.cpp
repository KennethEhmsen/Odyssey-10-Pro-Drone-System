// =====================================================================================
//  Odyssey-10 Pro -- Navigation implementation
// =====================================================================================

#include "navigation.h"

static constexpr double kEarthRadiusM = 6371008.8;   // WGS-84 mean radius
static constexpr double kDegToRad     = M_PI / 180.0;
static constexpr double kRadToDeg     = 180.0 / M_PI;

// -------------------------------------------------------------------------------------
//  Geodesy
// -------------------------------------------------------------------------------------
double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double dLat = (lat2 - lat1) * kDegToRad;
  const double dLon = (lon2 - lon1) * kDegToRad;
  const double sLat = sin(dLat * 0.5);
  const double sLon = sin(dLon * 0.5);
  const double a = sLat * sLat
                 + cos(lat1 * kDegToRad) * cos(lat2 * kDegToRad) * sLon * sLon;
  return 2.0 * kEarthRadiusM * atan2(sqrt(a), sqrt(1.0 - a));
}

double initialBearingDeg(double lat1, double lon1, double lat2, double lon2) {
  const double phi1 = lat1 * kDegToRad;
  const double phi2 = lat2 * kDegToRad;
  const double dLon = (lon2 - lon1) * kDegToRad;
  const double y = sin(dLon) * cos(phi2);
  const double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon);
  double brg = atan2(y, x) * kRadToDeg;
  if (brg < 0.0) brg += 360.0;
  return brg;
}

float wrapAngleDeg(float degrees) {
  while (degrees >  180.0f) degrees -= 360.0f;
  while (degrees <= -180.0f) degrees += 360.0f;
  return degrees;
}

// -------------------------------------------------------------------------------------
//  Energy budget
// -------------------------------------------------------------------------------------
EnergyBudget evaluateEnergy(const BatteryState& batt, float distanceToHomeM) {
  EnergyBudget b;

  // Time to fly home at cruise, plus a fixed allowance for the descent and landing.
  b.returnTimeSec = (distanceToHomeM / RTH_CRUISE_SPEED_MPS) + RTH_DESCENT_BUFFER_S;

  // Currency 1: pack voltage. Where the aircraft must still be when it lands, plus
  // the sag it will accumulate on the way, plus a reserve.
  //
  //   V_req = V_critical + (t_return * burn_rate) + V_reserve
  //
  // With 6S thresholds these numbers are now reachable: a 4500 mAh 6S pack at
  // 0.003 V/s loses about 0.18 V over a 60 s return, so V_req sits near 20.6 V
  // against a 19.8 V floor.
  static constexpr float kBurnRateVoltsPerSec = 0.003f;
  b.requiredVolts = PACK_CRITICAL_V
                  + (b.returnTimeSec * kBurnRateVoltsPerSec)
                  + PACK_RESERVE_V;

  // Currency 2: charge. Load-independent, so it does not false-trigger on a punch-out.
  b.requiredMah  = (CRUISE_CURRENT_A * 1000.0f) * (b.returnTimeSec / 3600.0f);
  b.remainingMah = PACK_USABLE_MAH - batt.consumedMah;

  const bool voltageSaysOk = batt.packVolts > b.requiredVolts;
  const bool chargeSaysOk  = !batt.currentSensorOk
                           || b.remainingMah > b.requiredMah * 1.2f;   // 20% margin

  b.canReachHome = voltageSaysOk && chargeSaysOk;
  b.belowWarnThreshold     = batt.packVolts <= PACK_WARN_V;
  b.belowCriticalThreshold = batt.packVolts <= PACK_CRITICAL_V
                          || (batt.currentSensorOk && b.remainingMah <= 0.0f);
  return b;
}

// -------------------------------------------------------------------------------------
//  RTH navigator
// -------------------------------------------------------------------------------------
void RthNavigator::begin() {
  PidGains g;

  // Distance error -> ground speed demand. Pure P with a hard ceiling at cruise;
  // integral here would just wind up while the aircraft is still climbing.
  g = PidGains{};
  g.kp = RTH_POS_P; g.ki = 0.0f; g.kd = 0.0f;
  g.outMin = 0.0f;  g.outMax = RTH_CRUISE_SPEED_MPS;
  posToVel_.setGains(g);

  // Speed error -> pitch demand.
  g = PidGains{};
  g.kp = RTH_VEL_P; g.ki = 0.35f; g.kd = 0.0f;
  g.outMin = -RTH_MAX_PITCH_DEG; g.outMax = RTH_MAX_PITCH_DEG;
  g.iMax = RTH_MAX_PITCH_DEG * 0.5f;
  velToPitch_.setGains(g);

  // Heading error -> yaw rate demand.
  g = PidGains{};
  g.kp = RTH_YAW_P; g.ki = 0.0f; g.kd = 0.0f;
  g.outMin = -YAW_RATE_MAX_DPS; g.outMax = YAW_RATE_MAX_DPS;
  headingToYaw_.setGains(g);

  // Altitude error -> climb rate demand.
  g = PidGains{};
  g.kp = RTH_ALT_P; g.ki = 0.10f; g.kd = 0.0f;
  g.outMin = -2.0f; g.outMax = 3.0f; g.iMax = 1.0f;
  altToClimb_.setGains(g);

  phase_ = RthPhase::Idle;
}

void RthNavigator::engage(float currentAltitudeAgl, float currentHeadingDeg) {
  (void)currentHeadingDeg;
  posToVel_.reset();
  velToPitch_.reset();
  headingToYaw_.reset();
  altToClimb_.reset();

  // If the aircraft is already at or above the safe return altitude there is nothing
  // to climb for; go straight to translating.
  phase_ = (currentAltitudeAgl >= RTH_SAFE_ALTITUDE_M - 2.0f)
             ? RthPhase::Translate : RthPhase::Climb;
  phaseEnteredMs_ = millis();
  lastDistanceM_  = 0.0f;
}

void RthNavigator::disengage() {
  phase_ = RthPhase::Idle;
  posToVel_.reset();
  velToPitch_.reset();
  headingToYaw_.reset();
  altToClimb_.reset();
}

const char* RthNavigator::phaseName() const {
  switch (phase_) {
    case RthPhase::Idle:      return "IDLE";
    case RthPhase::Climb:     return "CLIMB";
    case RthPhase::Translate: return "TRANSLATE";
    case RthPhase::Descend:   return "DESCEND";
    case RthPhase::Arrived:   return "ARRIVED";
  }
  return "?";
}

NavigationDemand RthNavigator::update(const VehicleSnapshot& snap,
                                      double homeLat, double homeLon,
                                      float dt, uint32_t nowMs) {
  NavigationDemand d;

  // Without a position solution there is nothing to navigate towards. Report invalid
  // and let the caller fall back to a level-and-descend landing -- which is exactly
  // what the original RTH did unconditionally.
  if (!snap.gnss.valid || !snap.homeLocked) {
    d.valid = false;
    return d;
  }

  const double distance = haversineMeters(snap.gnss.latitude, snap.gnss.longitude,
                                          homeLat, homeLon);
  const double bearing  = initialBearingDeg(snap.gnss.latitude, snap.gnss.longitude,
                                            homeLat, homeLon);
  lastDistanceM_ = (float)distance;

  float altitudeAgl = 0.0f;
  if (snap.perception.baroAglM.everValid) altitudeAgl = snap.perception.baroAglM.value;

  // ---- Phase sequencing -------------------------------------------------------------
  switch (phase_) {
    case RthPhase::Idle:
      phase_ = RthPhase::Climb;
      phaseEnteredMs_ = nowMs;
      break;

    case RthPhase::Climb:
      // Move on once at altitude, or after 20 s if the climb is not making progress
      // (better to start heading home at a lower altitude than to burn the pack
      // climbing into a headwind).
      if (altitudeAgl >= RTH_SAFE_ALTITUDE_M - 2.0f ||
          (uint32_t)(nowMs - phaseEnteredMs_) > 20000u) {
        phase_ = RthPhase::Translate;
        phaseEnteredMs_ = nowMs;
      }
      break;

    case RthPhase::Translate:
      if (distance <= RTH_ARRIVAL_RADIUS_M) {
        phase_ = RthPhase::Descend;
        phaseEnteredMs_ = nowMs;
      }
      break;

    case RthPhase::Descend:
      if (altitudeAgl <= FLARE_ENGAGE_AGL_M) {
        phase_ = RthPhase::Arrived;
        phaseEnteredMs_ = nowMs;
      }
      break;

    case RthPhase::Arrived:
      break;
  }

  // ---- Heading control --------------------------------------------------------------
  // Point the nose at home during the translate phase so the forward-looking LiDAR is
  // actually looking along the flight path. During climb and descent hold heading.
  if (phase_ == RthPhase::Translate) {
    const float headingError = wrapAngleDeg((float)bearing - snap.attitude.headingDeg);
    d.targetYawRate = headingToYaw_.update(headingError, 0.0f, dt);
  } else {
    d.targetYawRate = 0.0f;
  }

  // ---- Translation control ----------------------------------------------------------
  if (phase_ == RthPhase::Translate) {
    // Position -> velocity. Slow to the approach speed as the arrival radius nears.
    float speedDemand = posToVel_.update((float)distance, 0.0f, dt);
    if (distance < 30.0) speedDemand = min(speedDemand, RTH_APPROACH_SPEED_MPS);

    // Velocity -> pitch. Negative pitch is nose-down, which is forward flight.
    const float speedError = speedDemand - snap.gnss.groundSpeed;
    const float pitchCmd   = velToPitch_.update(speedError, 0.0f, dt);
    d.targetPitchDeg = -constrain(pitchCmd, 0.0f, RTH_MAX_PITCH_DEG);

    // Only translate once the nose is roughly on the bearing; crabbing sideways at
    // 12 m/s with a forward-only obstacle sensor is how you fly into things.
    const float headingError = fabsf(wrapAngleDeg((float)bearing - snap.attitude.headingDeg));
    if (headingError > 45.0f) d.targetPitchDeg *= 0.2f;
  } else {
    d.targetPitchDeg = 0.0f;
  }

  d.targetRollDeg = 0.0f;   // wings level; heading is corrected with yaw, not bank

  // ---- Vertical control -------------------------------------------------------------
  float altTarget;
  switch (phase_) {
    case RthPhase::Climb:
    case RthPhase::Translate: altTarget = RTH_SAFE_ALTITUDE_M; break;
    case RthPhase::Descend:   altTarget = 0.0f;                break;
    case RthPhase::Arrived:   altTarget = 0.0f;                break;
    default:                  altTarget = altitudeAgl;         break;
  }
  d.targetClimbMps = altToClimb_.update(altTarget, altitudeAgl, dt);
  if (phase_ == RthPhase::Descend || phase_ == RthPhase::Arrived) {
    d.targetClimbMps = max(d.targetClimbMps, -FAILSAFE_DESCENT_MPS);
  }

  d.valid = true;
  return d;
}

// -------------------------------------------------------------------------------------
//  Obstacle response
// -------------------------------------------------------------------------------------
ObstacleResponse evaluateObstacle(bool lidarHealthy, uint16_t distanceCm,
                                  float stepOverRemainingM) {
  ObstacleResponse r;

  // A stale or failed sensor disables avoidance outright. It must NOT fall back to the
  // last known distance -- that is the latch behaviour finding 15 describes, where a
  // sensor that died inside 3.5 m clamped forward pitch for the rest of the flight.
  if (!lidarHealthy) {
    r.avoidanceActive = false;
    return r;
  }
  r.avoidanceActive = true;

  if (distanceCm <= OBSTACLE_STOP_CM) {
    // Inside the hard bubble: brake, and climb over.
    r.pitchLimitDeg = OBSTACLE_BRAKE_PITCH_DEG;   // forward pitch is clamped to reverse
    r.velocityScale = 0.0f;
    r.braking       = true;
    r.stepOverClimbMps = (stepOverRemainingM > 0.05f) ? OBSTACLE_STEPOVER_RATE_MPS : 0.0f;
  } else if (distanceCm <= OBSTACLE_SLOW_CM) {
    // Proportional band: scale = (d - stop) / (slow - stop)
    const float span = (float)(OBSTACLE_SLOW_CM - OBSTACLE_STOP_CM);
    r.velocityScale  = constrain(((float)distanceCm - (float)OBSTACLE_STOP_CM) / span,
                                 0.0f, 1.0f);
    r.pitchLimitDeg  = ANGLE_MAX_DEG * r.velocityScale;
  }
  return r;
}
