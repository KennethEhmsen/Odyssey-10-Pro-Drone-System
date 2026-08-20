// =====================================================================================
//  Odyssey-10 Pro -- Return-to-home navigator and energy budget
//  ------------------------------------------------------------------------------------
//  FIX FOR FINDING 4.
//
//  The original STATE_RTH_NAVIGATING branch was:
//
//      rollOut  = pidRoll.update((0 - rollEst) * 4.5f, gx, dt);
//      pitchOut = pidPitch.update((0 - pitchEst) * 4.5f, gy, dt);
//      yawOut   = pidYaw.update(0.0f, gz, dt);
//      holdThrottle = PWM_ARM_IDLE + 450;
//
//  That levels the aircraft and holds a fixed throttle. It computes no bearing, never
//  yaws, never pitches forward. distHome therefore never decreases, so the documented
//  exit condition (distHome <= 5.0) is unreachable, and because the battery block was
//  gated on `currentState == STATE_ARMED` there was no low-voltage escalation once RTH
//  was entered. The aircraft would hover in place until the pack was flat.
//
//  This class implements the three-phase return the specification actually described:
//
//      CLIMB      -- gain RTH_SAFE_ALTITUDE_M AGL before translating, so the return
//                    path clears whatever the aircraft flew under on the way out
//      TRANSLATE  -- yaw to the home bearing, then pitch forward under a
//                    position -> velocity -> attitude cascade
//      DESCEND    -- inside the arrival radius, hand over to the landing controller
//
//  Bearing and distance use the same haversine/great-circle maths as before, but the
//  heading now comes from the QMC5883L (implemented in sensors.cpp) rather than being
//  absent entirely.
// =====================================================================================

#ifndef ODY_NAVIGATION_H
#define ODY_NAVIGATION_H

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "pid.h"

// Great-circle distance in metres between two WGS-84 coordinates.
double haversineMeters(double lat1, double lon1, double lat2, double lon2);

// Initial great-circle bearing from point 1 to point 2, degrees true, 0..360.
double initialBearingDeg(double lat1, double lon1, double lat2, double lon2);

// Shortest signed angular difference `target - current`, wrapped to (-180, +180].
float wrapAngleDeg(float degrees);

enum class RthPhase : uint8_t {
  Idle      = 0,
  Climb     = 1,
  Translate = 2,
  Descend   = 3,
  Arrived   = 4,
};

struct NavigationDemand {
  float targetRollDeg  = 0.0f;
  float targetPitchDeg = 0.0f;
  float targetYawRate  = 0.0f;
  float targetClimbMps = 0.0f;
  bool  valid          = false;   // false means "no usable position solution"
};

// -------------------------------------------------------------------------------------
//  Energy budget
//
//  Answers one question: can the aircraft still reach home from where it is now?
//
//  It answers it twice, in two independent currencies, and takes the pessimistic
//  result. Voltage alone sags under throttle, which is why the original design's
//  instantaneous comparison would trip on a punch-out; the coulomb count from the
//  INA226 is load-independent but drifts if the shunt calibration is off. Requiring
//  both to agree that there is enough left avoids each one's failure mode.
// -------------------------------------------------------------------------------------
struct EnergyBudget {
  float returnTimeSec        = 0.0f;
  float requiredVolts        = 0.0f;
  float requiredMah          = 0.0f;
  float remainingMah         = 0.0f;
  bool  canReachHome         = true;
  bool  belowWarnThreshold   = false;
  bool  belowCriticalThreshold = false;
};

EnergyBudget evaluateEnergy(const BatteryState& batt, float distanceToHomeM);

// -------------------------------------------------------------------------------------
//  The navigator itself
// -------------------------------------------------------------------------------------
class RthNavigator {
public:
  void begin();

  // Called when the aircraft first enters RTH. Captures the entry attitude so the
  // controllers hand over without a step.
  void engage(float currentAltitudeAgl, float currentHeadingDeg);

  void disengage();

  // Runs the outer navigation cascade. `dt` is the outer-loop period (50 Hz).
  // Returns attitude and climb-rate demands for the inner rate controllers.
  NavigationDemand update(const VehicleSnapshot& snap,
                          double homeLat, double homeLon,
                          float dt, uint32_t nowMs);

  RthPhase phase() const { return phase_; }
  const char* phaseName() const;

private:
  RthPhase phase_ = RthPhase::Idle;
  Pid      posToVel_;      // distance error -> ground speed demand
  Pid      velToPitch_;    // speed error    -> pitch demand
  Pid      headingToYaw_;  // heading error  -> yaw rate demand
  Pid      altToClimb_;    // altitude error -> climb rate demand
  uint32_t phaseEnteredMs_ = 0;
  float    lastDistanceM_  = 0.0f;
};

// -------------------------------------------------------------------------------------
//  Obstacle response
//
//  FIX FOR FINDING 15, part two. The original code implemented only the pitch clamp;
//  the "+5.0 m vertical step-over climb" that section 6 specified, and that the
//  section 11 checklist asked the tester to verify, was never commanded anywhere.
//
//  `lidarHealthy` is passed in explicitly. When the sensor is stale the response is
//  to disable avoidance and say so, NOT to reuse the last distance -- reusing it was
//  the actual bug.
// -------------------------------------------------------------------------------------
struct ObstacleResponse {
  float pitchLimitDeg   = 90.0f;   // maximum permitted forward (nose-down) pitch
  float velocityScale   = 1.0f;    // proportional slow-down in the outer band
  float stepOverClimbMps = 0.0f;   // commanded vertical escape rate
  bool  braking         = false;
  bool  avoidanceActive = false;   // false when the sensor is unhealthy
};

ObstacleResponse evaluateObstacle(bool lidarHealthy, uint16_t distanceCm,
                                  float stepOverRemainingM);

#endif // ODY_NAVIGATION_H
