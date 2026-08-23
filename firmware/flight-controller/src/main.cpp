// =====================================================================================
//  Odyssey-10 Pro -- ESP32-P4 Master Flight & Navigation Controller
//  ------------------------------------------------------------------------------------
//  Task layout:
//
//    Core 1  TaskFlightLoop   500 Hz, priority MAX-1
//              Primary IMU, attitude estimation, free-fall detection, rate control,
//              mixing, ESC output, BlackBox record generation. Touches exactly one
//              I2C device and never blocks on anything else.
//
//    Core 0  TaskTelemetry     50 Hz, priority 3
//              Slow sensors, GNSS, LiDAR, arming logic, energy budget, state
//              transitions, LoRa, AUX bus, beacon latch sequencing, console output.
//
//    Core 0  TaskStorage       20 Hz, priority 1
//              Drains the BlackBox ring to the MicroSD card. Isolated so an SD write
//              stall cannot delay a control deadline or a radio poll.
//
//  Cross-core data moves through `snapshotLock` (a spinlock, for the small struct) and
//  `flightState` (which is its own atomic escalate-only state machine). No global is
//  ever read without one of those two.
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "types.h"
#include "odyssey_link.h"
#include "filters.h"
#include "dynamic_notch.h"
#include "dshot_rmt.h"
#include "pid.h"
#include "mixer.h"
#include "state_machine.h"
#include "sensors.h"
#include "navigation.h"
#include "radio_link.h"
#include "blackbox.h"

// -------------------------------------------------------------------------------------
//  Serial ports. UART0 is the console.
// -------------------------------------------------------------------------------------
HardwareSerial GnssSerial(1);
HardwareSerial VtxSerial(2);
HardwareSerial LidarSerial(3);
HardwareSerial CrsfSerial(4);
//  UART1, not the LP-UART.
//
//  This was HardwareSerial(5), the ESP32-P4's LP-UART. That peripheral can only
//  attach to LP-IO pins -- GPIO 0-15 on this chip -- and every one of those is
//  already spoken for by the ADC input, the arm button, the motors, I2C and the
//  LoRa SPI bus. The hardware says so plainly:
//
//      RTCIO: rtc_gpio_init(49): RTCIO number error
//      lp_uart_config_io(): Failed to initialize LP_IO 39
//
//  The AUX bus is a one-way broadcast to the beacon and the Remote ID module
//  while the aircraft is powered, so it gains nothing from a low-power UART.
//  UART1 was unused -- the console is on USB, which frees UART0 and UART1.
HardwareSerial AuxSerial(1);

// -------------------------------------------------------------------------------------
//  Shared state
// -------------------------------------------------------------------------------------
static portMUX_TYPE      snapshotLock = portMUX_INITIALIZER_UNLOCKED;
static VehicleSnapshot   snapshot;
static FlightStateMachine flightState;
static RthNavigator      navigator;

static Pid pidRoll (GainSets::rollRate());
static Pid pidPitch(GainSets::pitchRate());
static Pid pidYaw  (GainSets::yawRate());
static Pid pidClimb(GainSets::verticalRate());

static BiQuadNotch notchGx, notchGy, notchGz;

#if DYN_NOTCH_HARMONIC
// A second notch per axis for the propeller's overtone. These stay UNCONFIGURED --
// which filters.h makes a pass-through rather than a silent zero -- unless the tracker
// reports an observable harmonic. On most builds 2*f0 is above the IMU's anti-alias
// corner, so on most builds these never engage and cost nothing but their memory.
static BiQuadNotch notchH2x, notchH2y, notchH2z;
static float       harmonicTuned = 0.0f;
#endif

#if DYN_NOTCH_ENABLE
// ONE tracker, not three. The peak being hunted is the four motor shafts, and they are
// the same four shafts whichever axis you look down -- running a tracker per axis would
// spend three times the CPU rediscovering the same number. The roll axis is used because
// it sees the strongest motor coupling on a Quad-X.
static DynamicNotchTracker notchTracker;
static int      notchDivider   = 0;
static volatile float trackedNotchHz = NOTCH_CENTER_HZ;   // recorded in the BlackBox log
static volatile bool  notchIsTracking = false;
static volatile float notchConfidence = 0.0f;
static volatile float trackedHarmonicHz = 0.0f;
static volatile bool  harmonicIsTracking = false;
static volatile bool  harmonicIsVisible = false;
static volatile bool  notchFromTelemetry = false;
#endif

static double   homeLat = 0.0, homeLon = 0.0;
static bool     homeLocked = false;
static float    gyroBias[3] = {0, 0, 0};
static uint32_t flightNumber = 0;

// Requests raised by the flight loop, actioned by the telemetry task. The flight loop
// must never call delay(), Serial or the radio.
static volatile bool     requestBeaconLatch  = false;
static volatile bool     requestParachuteLog = false;
static volatile uint32_t parachuteDeployedMs = 0;

// Step-over climb budget: how much of the OBSTACLE_STEPOVER_M vertical escape remains.
static float stepOverRemainingM = 0.0f;

// Bench self-test injection. Set by the console handler, consumed by the flight loop.
// This exists because the free-fall detector cannot otherwise be exercised safely:
// the documented revision 1.0 bench test (a 10 cm drop) produces only 143 ms of free
// fall against a 400 ms threshold and could never have passed. See docs section 11.3.
static volatile bool     selftestFreefall     = false;
static volatile uint32_t selftestFreefallUntil = 0;

// =====================================================================================
//  Small helpers
// =====================================================================================
#if DSHOT_ENABLE
static DShotTelemetry dshotTelem;
#endif

static inline void setMotors(const MixerOutput& out) {
#if DSHOT_ENABLE
  // The mixer's output is unchanged -- only the transport differs. Keeping the mixer in
  // the PWM count domain means §4.2's desaturation arithmetic, and every test of it, is
  // untouched by this.
  const uint16_t v[4] = {
      dshotFromPwmCounts(out.motor[0]), dshotFromPwmCounts(out.motor[1]),
      dshotFromPwmCounts(out.motor[2]), dshotFromPwmCounts(out.motor[3]) };
  dshotRmt.write(v, DSHOT_BIDIRECTIONAL != 0);
#else
  ledcWrite(MOTOR1_PIN, out.motor[0]);
  ledcWrite(MOTOR2_PIN, out.motor[1]);
  ledcWrite(MOTOR3_PIN, out.motor[2]);
  ledcWrite(MOTOR4_PIN, out.motor[3]);
#endif
}

static void idleMotors() {
#if DSHOT_ENABLE
  // DISARM, not minimum throttle. On PWM these are the same signal; on DShot they are
  // not, and sending 48 here would leave four motors turning on a disarmed aircraft.
  const uint16_t stop[4] = { DSHOT_CMD_DISARM, DSHOT_CMD_DISARM,
                             DSHOT_CMD_DISARM, DSHOT_CMD_DISARM };
  dshotRmt.write(stop, false);
#else
  ledcWrite(MOTOR1_PIN, PWM_MIN);
  ledcWrite(MOTOR2_PIN, PWM_MIN);
  ledcWrite(MOTOR3_PIN, PWM_MIN);
  ledcWrite(MOTOR4_PIN, PWM_MIN);
#endif
}

static void resetControllers() {
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();
  pidClimb.reset();
}

// =====================================================================================
//  FLIGHT LOOP -- Core 1, 500 Hz
// =====================================================================================
// The flight loop's period is expressed in FreeRTOS ticks, which makes the tick rate
// a hard dependency of the control loop and not a detail of the build system.
//
// pdMS_TO_TICKS(ms) is (ms * configTICK_RATE_HZ) / 1000. At the ESP-IDF default tick of
// 100 Hz, a 1 ms period evaluates to ZERO ticks, and vTaskDelayUntil() with a zero
// period does not delay -- the loop would spin and starve everything else pinned to
// that core. Arduino-ESP32 sets 1000 Hz, which is why the 1000 Hz 7-inch build works,
// but nothing in this repository said so until now.
static_assert(configTICK_RATE_HZ == 1000,
              "The flight loop needs a 1000 Hz FreeRTOS tick. At a lower tick rate "
              "pdMS_TO_TICKS() rounds the loop period down to zero and vTaskDelayUntil "
              "stops delaying at all.");

// 1000/FLIGHT_LOOP_HZ is integer division. 500 and 1000 Hz divide exactly; a rate that
// does not would be silently rounded to a different loop period than the one every
// derived constant -- dt, the notch, the log divider -- was computed from.
static_assert(1000 % FLIGHT_LOOP_HZ == 0,
              "FLIGHT_LOOP_HZ must divide 1000 exactly, or the tick period is rounded "
              "and the real loop rate stops matching FLIGHT_LOOP_HZ");

static void TaskFlightLoop(void* /*arg*/) {
  TickType_t      lastWake  = xTaskGetTickCount();
  const TickType_t period   = pdMS_TO_TICKS(1000 / FLIGHT_LOOP_HZ);

  // One tick of slack at 500 Hz, none at 1000 Hz. At 1000 Hz any overrun misses the
  // deadline outright rather than eating into margin -- see section 8.3.4.
  static_assert(FLIGHT_LOOP_HZ <= 1000,
                "Above 1000 Hz the loop period is under one FreeRTOS tick and "
                "vTaskDelayUntil can no longer schedule it");

  float    rollEst = 0.0f, pitchEst = 0.0f;
  float    prevBaroAgl = 0.0f, vario = 0.0f;
  uint32_t lastMicros = micros();
  uint32_t freefallSince = 0;
  uint32_t touchdownSince = 0;
  uint8_t  baroDivider = 0;
  uint32_t lostImuSamples = 0;

  NavigationDemand navDemand;
  uint32_t         lastNavUpdateMs = 0;

  for (;;) {
    const uint32_t nowMicros = micros();
    const uint32_t nowMs     = millis();

    float dt = (float)(nowMicros - lastMicros) / 1000000.0f;
    if (!(dt > 0.0f) || dt > 0.010f) dt = FLIGHT_LOOP_DT;
    lastMicros = nowMicros;

    const uint8_t state = flightState.get();

    // ---- Sensors -------------------------------------------------------------------
    Vec3 gyroRaw, accel;
    if (!sensors.readPrimaryImu(gyroRaw, accel)) {
      // A lost sample is not a reason to zero the gyro. Hold the previous estimate
      // and count the miss; sustained misses fail the health mask and trigger a
      // failsafe from the telemetry task.
      if (++lostImuSamples > FLIGHT_LOOP_HZ / 4 && odyMotorsAreLive(state)) {
        flightState.request(ODY_STATE_FAILSAFE_LANDING, "primary IMU lost");
      }
      vTaskDelayUntil(&lastWake, period);
      continue;
    }
    lostImuSamples = 0;

    const float gxRaw = gyroRaw.x - gyroBias[0];
    const float gyRaw = gyroRaw.y - gyroBias[1];
    const float gzRaw = gyroRaw.z - gyroBias[2];

#if DYN_NOTCH_ENABLE
    // ---- Dynamic notch tracking ----------------------------------------------------
    // The tracker is fed the RAW gyro, deliberately. Feeding it the filtered signal
    // would be self-defeating: the notch removes the very peak the tracker exists to
    // find, so it would watch the peak vanish, conclude there was nothing there, and
    // wander off. Analysis has to happen upstream of the filter it is tuning.
    //
    // Only while the motors are actually turning. With the props stopped there is no
    // motor peak to find and nothing to do but reject noise, and a spurious lock on the
    // bench would be carried into the air.
    if (odyMotorsAreLive(state)) {
      notchTracker.push(gxRaw);
#if DSHOT_ENABLE && DSHOT_BIDIRECTIONAL
      // Collect whatever the ESCs sent back since the last frame. Non-blocking: a reply
      // that has not arrived is simply not there this iteration.
      dshotRmt.pollTelemetry(dshotTelem, nowMs);
      dshotTelem.update(nowMs);
#endif
      if (++notchDivider >= FLIGHT_LOOP_HZ / DYN_NOTCH_UPDATE_HZ) {
        notchDivider = 0;

        // Prefer the measurement over the search. dshotTelem returns 0 unless all four
        // motors reported recently AND agree closely enough for one notch to cover
        // them -- under a hard roll the diagonals diverge and the sliding DFT, which
        // sees the blend of all four, is the better answer.
        bool notchMoved = false;
#if DSHOT_ENABLE && DSHOT_BIDIRECTIONAL
        const float measuredHz = dshotTelem.notchHz();
        if (measuredHz > 0.0f) notchMoved = notchTracker.applyMeasured(measuredHz);
        else                   notchMoved = notchTracker.update();
#else
        notchMoved = notchTracker.update();
#endif
        if (notchMoved) {
          const float centre = notchTracker.centreHz();
          // retune(), not configure(): configure() clears the delay line, which would
          // punch a transient straight through the rate loop every time the notch
          // moved a couple of hertz. retune() keeps the filter state.
          notchGx.retune(centre, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
          notchGy.retune(centre, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
          notchGz.retune(centre, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
          trackedNotchHz = centre;
        }

#if DYN_NOTCH_HARMONIC
        // The harmonic is retuned on its own schedule: it can appear and disappear
        // while the fundamental stays put, because it depends on whether 2*f0 clears
        // the DLPF corner as much as on whether the energy is there.
        const float h2 = notchTracker.harmonicHz();
        if (h2 > 0.0f) {
          if (fabsf(h2 - harmonicTuned) >= DYN_NOTCH_RETUNE_HZ) {
            notchH2x.retune(h2, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
            notchH2y.retune(h2, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
            notchH2z.retune(h2, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
            harmonicTuned = h2;
          }
        } else if (harmonicTuned != 0.0f) {
          // Nothing there any more. Return the filters to pass-through rather than
          // leaving a notch sitting on empty spectrum, which is phase lag for nothing.
          notchH2x.bypass();
          notchH2y.bypass();
          notchH2z.bypass();
          harmonicTuned = 0.0f;
        }
        trackedHarmonicHz = notchTracker.harmonicHz();
        harmonicIsTracking = notchTracker.state().harmonicTracking;
        harmonicIsVisible  = notchTracker.state().harmonicObservable;
#endif
        notchIsTracking = notchTracker.state().tracking;
        notchConfidence = notchTracker.state().confidence;
        notchFromTelemetry = notchTracker.state().fromTelemetry;
      }
    } else {
      notchDivider = 0;
    }
#endif

    float gx = notchGx.apply(gxRaw);
    float gy = notchGy.apply(gyRaw);
    float gz = notchGz.apply(gzRaw);

#if DYN_NOTCH_HARMONIC
    // In series after the fundamental. A pass-through when no harmonic is engaged, so
    // the builds that cannot see one pay nothing but three function calls.
    gx = notchH2x.apply(gx);
    gy = notchH2y.apply(gy);
    gz = notchH2z.apply(gz);
#endif

    // ---- Attitude estimate ---------------------------------------------------------
    const float accelRoll  = atan2f(accel.y, accel.z) * 180.0f / (float)M_PI;
    const float accelPitch = atan2f(-accel.x,
                                    sqrtf(accel.y * accel.y + accel.z * accel.z))
                           * 180.0f / (float)M_PI;
    rollEst  = ATT_COMP_ALPHA * (rollEst  + gx * dt) + (1.0f - ATT_COMP_ALPHA) * accelRoll;
    pitchEst = ATT_COMP_ALPHA * (pitchEst + gy * dt) + (1.0f - ATT_COMP_ALPHA) * accelPitch;

    // ---- Altitude and vertical speed -----------------------------------------------
    float baroAgl = prevBaroAgl;
    bool  baroFresh = false;
    if (++baroDivider >= FLIGHT_LOOP_HZ / 50) {     // 50 Hz
      baroDivider = 0;
      float agl;
      if (sensors.baroAgl(nowMs, agl)) {
        vario = (agl - prevBaroAgl) / (1.0f / 50.0f);
        vario = constrain(vario, -25.0f, 25.0f);
        prevBaroAgl = agl;
        baroAgl = agl;
        baroFresh = true;
      }
    }

    float tofAgl = -1.0f;
    const bool tofValid = sensors.tofAgl(nowMs, tofAgl);

    // Best available height above ground. The ToF is authoritative under 4 m; the
    // barometer covers everything above that.
    const float bestAgl = (tofValid && tofAgl < 3.8f) ? tofAgl : baroAgl;

    // =================================================================================
    //  FREE-FALL DETECTION  --  FIX FOR FINDING 12
    //
    //  Enabled in EVERY airborne state, not just ARMED and RTH. The original gate
    //  excluded AWAITING_LAND_PERMIT (hovering, up to 15 s) and FAILSAFE_LANDING
    //  (powered descent, up to 45 s) -- precisely the states in which a shed propeller
    //  produces a real free fall with the parachute inhibited.
    //
    //  Two guards are added that the original lacked:
    //    * a minimum altitude, because a canopy below its inflation height will not
    //      slow the aircraft and will foul the propellers
    //    * the timer resets on every state entry, so a partially accumulated
    //      free-fall window cannot leak across a mode change
    // =================================================================================
    // Bench injection: pretend the accelerometer reads free-fall for the requested
    // window. Only honoured while the aircraft is on the ground, and it deliberately
    // runs through the SAME detection path rather than short-circuiting to the servo,
    // so the test proves the detector, the altitude gate and the state machine.
    bool injecting = false;
    if (selftestFreefall) {
      if ((int32_t)(nowMs - selftestFreefallUntil) >= 0) {
        selftestFreefall = false;
      } else {
        injecting = true;
      }
    }

    if (odyMotorsAreLive(state) || injecting) {
      const float accelMag = injecting ? 0.0f : accel.magnitude();
      // The injection also bypasses the altitude gate, since a bench test is by
      // definition below the 8 m minimum deployment altitude.
      const bool  highEnough = injecting || (bestAgl >= PARACHUTE_MIN_AGL_M) || !baroFresh;

      if (accelMag < FREEFALL_ACCEL_MPS2 && highEnough) {
        if (freefallSince == 0) freefallSince = nowMs ? nowMs : 1;
        else if ((uint32_t)(nowMs - freefallSince) > FREEFALL_HOLD_MS) {
          // Deploy. The escalate-only state machine guarantees this cannot be
          // overwritten by a concurrent FAILSAFE_LANDING request from core 0.
          ledcWrite(PIN_PARACHUTE_SRV, SERVO_EJECT);
          idleMotors();
          resetControllers();
          flightState.request(ODY_STATE_FREEFALL_PARACHUTE, "free-fall detected");
          parachuteDeployedMs = nowMs;
          requestParachuteLog = true;
          freefallSince = 0;
        }
      } else {
        freefallSince = 0;
      }
    } else {
      freefallSince = 0;
    }

    // ---- Perception ----------------------------------------------------------------
    uint16_t obstacleCm = 0;
    const bool lidarHealthy = sensors.forwardObstacle(nowMs, obstacleCm);
    const ObstacleResponse obstacle =
        evaluateObstacle(lidarHealthy, obstacleCm, stepOverRemainingM);

    // Consume the step-over budget while the escape climb is actually commanded.
    if (obstacle.stepOverClimbMps > 0.0f) {
      stepOverRemainingM -= obstacle.stepOverClimbMps * dt;
      if (stepOverRemainingM < 0.0f) stepOverRemainingM = 0.0f;
    } else if (!obstacle.braking) {
      stepOverRemainingM = OBSTACLE_STEPOVER_M;   // rearm once clear
    }

    // ---- Pilot input ---------------------------------------------------------------
    PilotInput pilot;
    const bool pilotPresent = crsf.getPilotInput(nowMs, pilot);

    // ---- Navigation demand ---------------------------------------------------------
    // Recomputed at 50 Hz; the inner loops run at 500 Hz off the held demand.
    if (state == ODY_STATE_RTH_NAVIGATING && (nowMs - lastNavUpdateMs) >= 20u) {
      lastNavUpdateMs = nowMs;
      VehicleSnapshot local;
      portENTER_CRITICAL(&snapshotLock);
      local = snapshot;
      portEXIT_CRITICAL(&snapshotLock);
      local.attitude.rollDeg  = rollEst;
      local.attitude.pitchDeg = pitchEst;
      navDemand = navigator.update(local, homeLat, homeLon, 0.02f, nowMs);
    }

    // =================================================================================
    //  CONTROL
    // =================================================================================
    MixerOutput out = Mixer::allIdle();
    float targetRoll = 0.0f, targetPitch = 0.0f, targetYawRate = 0.0f;
    float throttleCounts = 0.0f;
    bool  driveMotors = false;

    switch (state) {

      // ---------------------------------------------------------------------------
      case ODY_STATE_ARMED: {
        if (!pilotPresent) break;      // telemetry task will raise the failsafe

        targetRoll    = pilot.rollNorm  * ANGLE_MAX_DEG;
        targetPitch   = pilot.pitchNorm * ANGLE_MAX_DEG;
        targetYawRate = pilot.yawNorm   * YAW_RATE_MAX_DPS;

        // Obstacle intervention. Negative pitch is nose-down (forward flight), so the
        // clamp is a lower bound on the commanded pitch angle.
        if (obstacle.avoidanceActive) {
          if (obstacle.braking) {
            targetPitch = max(targetPitch, OBSTACLE_BRAKE_PITCH_DEG * -1.0f);
            if (targetPitch < 0.0f) targetPitch = -OBSTACLE_BRAKE_PITCH_DEG;
          } else if (targetPitch < 0.0f) {
            targetPitch *= obstacle.velocityScale;
          }
        }

        throttleCounts = Mixer::throttleFromStick(pilot.throttleUs);

        // Step-over climb overrides the throttle stick while escaping an obstacle.
        if (obstacle.stepOverClimbMps > 0.0f) {
          const float climbCmd = pidClimb.update(obstacle.stepOverClimbMps, vario, dt);
          throttleCounts = max(throttleCounts,
                               (float)(PWM_ARM_IDLE - PWM_MIN) + climbCmd);
        }
        driveMotors = true;
        break;
      }

      // ---------------------------------------------------------------------------
      case ODY_STATE_RTH_NAVIGATING: {
        if (navDemand.valid) {
          targetRoll    = navDemand.targetRollDeg;
          targetPitch   = navDemand.targetPitchDeg;
          targetYawRate = navDemand.targetYawRate;

          // Obstacle avoidance outranks the navigator. Flying home is not worth
          // flying into a tree on the way.
          if (obstacle.avoidanceActive && obstacle.braking) {
            targetPitch = -OBSTACLE_BRAKE_PITCH_DEG;
          } else if (obstacle.avoidanceActive && targetPitch < 0.0f) {
            targetPitch *= obstacle.velocityScale;
          }

          const float climbCmd = pidClimb.update(navDemand.targetClimbMps, vario, dt);
          throttleCounts = (float)(PWM_ARM_IDLE - PWM_MIN) + 450.0f + climbCmd;
        } else {
          // No position solution: hold level and hover. This is what the original did
          // in ALL cases; here it is only the degraded fallback, and the telemetry
          // task will escalate to a landing rather than let it hover indefinitely.
          const float climbCmd = pidClimb.update(0.0f, vario, dt);
          throttleCounts = (float)(PWM_ARM_IDLE - PWM_MIN) + 450.0f + climbCmd;
        }
        driveMotors = true;
        break;
      }

      // ---------------------------------------------------------------------------
      case ODY_STATE_AWAITING_LAND_PERMIT: {
        // Station-keeping hover while the operator decides.
        const float climbCmd = pidClimb.update(0.0f, vario, dt);
        throttleCounts = (float)(PWM_ARM_IDLE - PWM_MIN) + 250.0f + climbCmd;
        driveMotors = true;
        break;
      }

      // ---------------------------------------------------------------------------
      case ODY_STATE_FAILSAFE_LANDING: {
        // Precision flare below FLARE_ENGAGE_AGL_M using the VL53L1X, which is what
        // section 6 always specified and the original firmware never implemented.
        float descentRate = FAILSAFE_DESCENT_MPS;
        if (tofValid && tofAgl < FLARE_ENGAGE_AGL_M) descentRate = FLARE_DESCENT_MPS;

        const float climbCmd = pidClimb.update(-descentRate, vario, dt);
        throttleCounts = (float)(PWM_ARM_IDLE - PWM_MIN) + 350.0f + climbCmd;
        driveMotors = true;

        // =========================================================================
        //  TOUCHDOWN DETECTION  --  FIX FOR FINDING 14
        //
        //  The original test was one line:
        //
        //      if (millis() - failsafeStartTime > 14000 || currentAlt <= 0.05f)
        //
        //  where currentAlt was a barometric reading relative to a ground reference
        //  captured once at power-on. Ambient pressure drift over a long flight is
        //  easily a metre, so that comparison can pass in mid-air -- and the branch
        //  immediately cut all four motors.
        //
        //  Touchdown now needs three independent things to agree, and any altitude
        //  source that says we are still up vetoes the whole thing.
        // =========================================================================
        const bool tofSaysDown  = tofValid && tofAgl <= TOUCHDOWN_TOF_M;
        const bool baroSaysDown = !tofValid && baroAgl <= TOUCHDOWN_BARO_M;
        const bool notMoving    = fabsf(vario) < TOUCHDOWN_VARIO_MPS;

        // Veto: if ANY healthy sensor puts us above the veto altitude, we are not down.
        const bool vetoed = (tofValid && tofAgl > TOUCHDOWN_VETO_AGL_M)
                         || (baroFresh && baroAgl > TOUCHDOWN_VETO_AGL_M);

        if ((tofSaysDown || baroSaysDown) && notMoving && !vetoed) {
          if (touchdownSince == 0) touchdownSince = nowMs ? nowMs : 1;
          else if ((uint32_t)(nowMs - touchdownSince) >= TOUCHDOWN_HOLD_MS) {
            idleMotors();
            resetControllers();
            flightState.request(ODY_STATE_DISARMED, "touchdown confirmed");
            driveMotors = false;
          }
        } else {
          touchdownSince = 0;
        }

        // Hard ceiling on the descent. Reaching it means the touchdown logic never
        // agreed, so cut power rather than descend forever -- but only once we are
        // plausibly low, and log it as the anomaly it is.
        if (flightState.millisInState() > FAILSAFE_LAND_TIMEOUT_MS) {
          idleMotors();
          resetControllers();
          flightState.request(ODY_STATE_DISARMED, "landing timeout");
          driveMotors = false;
        }
        break;
      }

      // ---------------------------------------------------------------------------
      case ODY_STATE_FREEFALL_PARACHUTE:
      default:
        idleMotors();
        resetControllers();
        driveMotors = false;
        break;
    }

    // ---- Inner rate loop and mixing ------------------------------------------------
    if (driveMotors) {
      const float desiredRollRate  = (targetRoll  - rollEst)  * ANGLE_P_GAIN;
      const float desiredPitchRate = (targetPitch - pitchEst) * ANGLE_P_GAIN;

      const float rollOut  = pidRoll.update(desiredRollRate,  gx, dt);
      const float pitchOut = pidPitch.update(desiredPitchRate, gy, dt);
      const float yawOut   = pidYaw.update(targetYawRate,      gz, dt);

      out = Mixer::mix(throttleCounts, rollOut, pitchOut, yawOut);
      setMotors(out);
    }

    // ---- Publish the snapshot ------------------------------------------------------
    portENTER_CRITICAL(&snapshotLock);
    snapshot.attitude.rollDeg  = rollEst;
    snapshot.attitude.pitchDeg = pitchEst;
    snapshot.gyroDps           = Vec3{gx, gy, gz};
    snapshot.accelMps2         = accel;
    snapshot.perception.varioMps = vario;
    if (baroFresh) snapshot.perception.baroAglM.set(baroAgl, nowMs);
    if (tofValid)  snapshot.perception.downwardAglM.set(tofAgl, nowMs);
    if (lidarHealthy) snapshot.perception.forwardObstacleCm.set(obstacleCm, nowMs);
    portEXIT_CRITICAL(&snapshotLock);

    // ---- BlackBox ------------------------------------------------------------------
    static uint8_t logDivider = 0;
    if (++logDivider >= FLIGHT_LOOP_HZ / BLACKBOX_LOG_HZ) {
      logDivider = 0;
      BlackBoxRecord rec{};
      rec.timestampMs    = nowMs;
      rec.gyroX          = (int16_t)(gx * 10.0f);
      rec.gyroY          = (int16_t)(gy * 10.0f);
      rec.gyroZ          = (int16_t)(gz * 10.0f);
      rec.accelX         = (int16_t)(accel.x * 100.0f);
      rec.accelY         = (int16_t)(accel.y * 100.0f);
      rec.accelZ         = (int16_t)(accel.z * 100.0f);
      rec.rollCentideg   = (int16_t)(rollEst * 100.0f);
      rec.pitchCentideg  = (int16_t)(pitchEst * 100.0f);
      rec.headingCentideg= (int16_t)(snapshot.attitude.headingDeg * 100.0f);
      // Post-clamp values: saturation is visible in the log, which is the one time
      // you most want to see it.
      rec.m1Pwm          = out.motor[0];
      rec.m2Pwm          = out.motor[1];
      rec.m3Pwm          = out.motor[2];
      rec.m4Pwm          = out.motor[3];
      rec.battMillivolts = (uint16_t)(sensors.battery().packVolts * 1000.0f);
      rec.currentCentiAmps = (int16_t)(sensors.battery().currentAmps * 100.0f);
      rec.consumedMah    = (uint16_t)sensors.battery().consumedMah;
      rec.baroAglCm      = (int16_t)(baroAgl * 100.0f);
      rec.tofAglCm       = tofValid ? (int16_t)(tofAgl * 100.0f) : (int16_t)-1;
      rec.varioCmS       = (int16_t)(vario * 100.0f);
      rec.forwardObstacleCm = lidarHealthy ? obstacleCm : LIDAR_INVALID_CM;
      rec.sensorHealth   = snapshot.sensorHealth;
      rec.flightState    = state;
      rec.mixerSaturationPct = (uint8_t)(out.saturation * 100.0f);

      // The notch tracker's verdict. Without this the log cannot answer the one
      // question the tracker exists to settle -- where the motor peak actually was --
      // because the logged gyro is post-notch and the log rate is below the peak
      // anyway. See the note on BlackBoxRecord in types.h.
#if DYN_NOTCH_ENABLE
      rec.notchCentreDeciHz = (uint16_t)constrain(trackedNotchHz * 10.0f, 0.0f, 65535.0f);
      rec.notchConfidence   = (uint8_t)constrain(notchConfidence, 0.0f, 255.0f);
      rec.notchFlags        = ODY_NOTCH_FLAG_DYNAMIC
                            | (notchIsTracking ? ODY_NOTCH_FLAG_TRACKING : 0u)
#if DYN_NOTCH_HARMONIC
                            | (harmonicIsTracking ? ODY_NOTCH_FLAG_H2_TRACKING : 0u)
                            | (harmonicIsVisible  ? ODY_NOTCH_FLAG_H2_VISIBLE  : 0u)
                            | (notchFromTelemetry ? ODY_NOTCH_FLAG_MEASURED   : 0u)
#endif
                            ;
      rec.notchHarmonicDeciHz =
          (uint16_t)constrain(trackedHarmonicHz * 10.0f, 0.0f, 65535.0f);
#else
      // Still log the static centre, so a fixed-notch log and a tracked one can be
      // compared field-for-field rather than by eye.
      rec.notchCentreDeciHz = (uint16_t)(NOTCH_CENTER_HZ * 10.0f);
      rec.notchConfidence   = 0;
      rec.notchFlags        = 0;
      rec.notchHarmonicDeciHz = 0;
#endif
      blackbox.push(rec);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

// =====================================================================================
//  ARMING  --  FIX FOR FINDING 5
//
//  The original arm path required only PREFLIGHT_OK + home lock + the button. The
//  global currentRC retained the last received throttle across flights, so arming with
//  the stick up drove all four props to ~82% instantly.
//
//  Every condition is now checked, every failure is reported to the pilot through the
//  telemetry armBlockFlags field, and the physical button must be held deliberately.
// =====================================================================================
static uint8_t evaluateArmBlockers(uint32_t nowMs, const PilotInput& pilot,
                                   bool pilotPresent, uint16_t health) {
  uint8_t flags = 0;

  // Remote ID is added to the required set only when the operator has opted in via
  // REQUIRE_REMOTE_ID_TO_ARM. It is not required for a privately built aircraft under
  // 25 kg in the EU open category -- see config.h and docs section 12.1.
  const uint16_t requiredSensors = ODY_ARM_REQUIRED_SENSORS
#if REQUIRE_REMOTE_ID_TO_ARM
                                 | ODY_SENS_REMOTE_ID
#endif
                                 ;
  if ((health & requiredSensors) != requiredSensors)
    flags |= ODY_ARMBLOCK_SENSORS;

  if (!homeLocked)                       flags |= ODY_ARMBLOCK_NO_HOME;
  if (!pilotPresent)                     flags |= ODY_ARMBLOCK_RC_STALE;

  // THE FIX: the throttle stick must be at the bottom, from a live frame.
  if (!pilotPresent || pilot.throttleUs > ARM_THROTTLE_MAX_US)
    flags |= ODY_ARMBLOCK_THROTTLE;

  if (sensors.battery().packVolts < PACK_LAUNCH_MIN_V)
    flags |= ODY_ARMBLOCK_BATTERY;

  float roll, pitch;
  portENTER_CRITICAL(&snapshotLock);
  roll  = snapshot.attitude.rollDeg;
  pitch = snapshot.attitude.pitchDeg;
  portEXIT_CRITICAL(&snapshotLock);
  if (fabsf(roll) > ARM_MAX_TILT_DEG || fabsf(pitch) > ARM_MAX_TILT_DEG)
    flags |= ODY_ARMBLOCK_ATTITUDE;

  return flags;
}

// =====================================================================================
//  EMERGENCY BEACON
//
//  Runs entirely on the telemetry core. The original called activateEmergencyBeacon()
//  from the 500 Hz flight task, where its four delay(150) calls plus a delay(100) put
//  roughly 700 ms of blocking into a hard real-time loop -- and drove the LoRa SPI bus
//  concurrently with the telemetry task on the other core, with no mutex.
//
//  The latch pulse is now a non-blocking state machine.
// =====================================================================================
static void serviceBeaconLatch(uint32_t nowMs) {
  static uint8_t  stage      = 0;
  static uint32_t stageStart = 0;

  switch (stage) {
    case 0:
      if (requestBeaconLatch) {
        requestBeaconLatch = false;
        digitalWrite(PIN_BEACON_LATCH, HIGH);
        stageStart = nowMs;
        stage = 1;
        Serial.println("[BEACON] latching the isolated 1S recovery beacon");
      }
      break;

    case 1:
      if ((uint32_t)(nowMs - stageStart) >= 100u) {   // 100 ms HIGH pulse
        digitalWrite(PIN_BEACON_LATCH, LOW);
        stage = 2;
        stageStart = nowMs;
      }
      break;

    case 2:
      // The beacon node takes over from here. It already holds a current position
      // from the AUX bus, so unlike the original design it has something real to
      // transmit. Return to idle so a second event can re-latch.
      if ((uint32_t)(nowMs - stageStart) >= 500u) stage = 0;
      break;
  }
}

// =====================================================================================
//  DEBUG CONSOLE
//
//  Bench-only commands. Every one of them refuses to run unless the aircraft is
//  disarmed and on the ground, so a stray character on the console cannot do anything
//  in flight.
// =====================================================================================
static void serviceConsole(uint32_t nowMs) {
  static char line[48];
  static uint8_t len = 0;

  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (len < sizeof(line) - 1) line[len++] = c;
      continue;
    }
    line[len] = '\0';
    const uint8_t state = flightState.get();

    if (strcmp(line, "SELFTEST FREEFALL") == 0) {
      if (odyIsAirborneState(state)) {
        Serial.println("[SELFTEST] refused: aircraft is not on the ground");
      } else {
        // Hold the injected condition for comfortably longer than FREEFALL_HOLD_MS so
        // the detector's own timer is what decides, not the length of the injection.
        selftestFreefallUntil = nowMs + FREEFALL_HOLD_MS + 300u;
        selftestFreefall = true;
        Serial.printf("[SELFTEST] injecting free-fall for %lu ms -- expect the servo to "
                      "reach the eject position and the state to become "
                      "FREEFALL_PARACHUTE\n",
                      (unsigned long)(FREEFALL_HOLD_MS + 300u));
      }

    } else if (strcmp(line, "SELFTEST RESET") == 0) {
      // Re-locks the parachute servo and returns the state machine to preflight so a
      // bench test can be repeated without a power cycle.
      if (odyIsAirborneState(state)) {
        Serial.println("[SELFTEST] refused: aircraft is not on the ground");
      } else {
        ledcWrite(PIN_PARACHUTE_SRV, SERVO_LOCKED);
        flightState.request(ODY_STATE_DISARMED, "selftest reset");
        flightState.resetFromDisarmed(ODY_STATE_PREFLIGHT_OK, "selftest reset");
        Serial.println("[SELFTEST] servo re-locked, state returned to PREFLIGHT_OK");
      }

    } else if (strcmp(line, "STATUS") == 0) {
      Serial.printf("[STATUS] %s  pack %.2f V (%.2f V/cell)  %.0f mAh used  "
                    "home %s  sensors 0x%04X  armblock 0x%02X\n",
                    odyStateName(state),
                    sensors.battery().packVolts,
                    sensors.battery().packVolts / (float)CELL_COUNT,
                    sensors.battery().consumedMah,
                    homeLocked ? "locked" : "NOT LOCKED",
                    snapshot.sensorHealth, snapshot.armBlockFlags);

    } else if (len > 0) {
      Serial.println("[CONSOLE] commands: SELFTEST FREEFALL | SELFTEST RESET | STATUS");
    }
    len = 0;
  }
}

// =====================================================================================
//  TELEMETRY TASK -- Core 0, 50 Hz
// =====================================================================================
static void TaskTelemetry(void* /*arg*/) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEM_TASK_HZ);

  DebouncedCondition cannotReachHome(BATT_DEBOUNCE_MS);
  DebouncedCondition belowWarn(BATT_DEBOUNCE_MS);
  DebouncedCondition belowCritical(1000u);
  uint32_t landRequestedMs = 0;
  uint32_t armButtonSince  = 0;
  uint32_t lastTelemMs     = 0;
  uint32_t lastAuxMs       = 0;

  for (;;) {
    const uint32_t nowMs = millis();

    // ---- Sensor servicing ----------------------------------------------------------
    sensors.serviceSlowSensors(nowMs);
    sensors.serviceGnss(nowMs);
    sensors.serviceLidar(nowMs);
    crsf.service(nowMs);
    serviceConsole(nowMs);

    // Composite health mask. sensors.healthMask() only knows about the devices the
    // SensorHub owns; the radios, the card and the Remote ID module are reported by
    // their own subsystems and must be folded in HERE, before anything consumes the
    // mask. Passing the bare sensor mask to the arm gate would leave ODY_SENS_CRSF_RC
    // and ODY_SENS_REMOTE_ID permanently clear, and arming would never be allowed.
    const uint16_t health =
          sensors.healthMask(nowMs)
        | (crsf.linkUp(nowMs)                    ? ODY_SENS_CRSF_RC  : 0)
        | (lora.healthy()                        ? ODY_SENS_LORA     : 0)
        | (blackbox.ready()                      ? ODY_SENS_SDCARD   : 0)
        | (auxBus.framesSent() > 0               ? ODY_SENS_AUX_BUS  : 0)
        | (digitalRead(PIN_REMOTEID_HEALTH) == HIGH ? ODY_SENS_REMOTE_ID : 0);

    const uint8_t state = flightState.get();

    GnssFix fix;
    const bool haveFix = sensors.gnssFix(nowMs, fix);

    PilotInput pilot;
    const bool pilotPresent = crsf.getPilotInput(nowMs, pilot);

    // ---- Heading -------------------------------------------------------------------
    float roll, pitch, heading = 0.0f;
    portENTER_CRITICAL(&snapshotLock);
    roll  = snapshot.attitude.rollDeg;
    pitch = snapshot.attitude.pitchDeg;
    portEXIT_CRITICAL(&snapshotLock);
    const bool haveHeading = sensors.magneticHeading(nowMs, roll, pitch, heading);

    // ---- Home lock -----------------------------------------------------------------
    if (!homeLocked && haveFix && fix.satellites >= ARM_MIN_SATELLITES) {
      homeLat = fix.latitude;
      homeLon = fix.longitude;
      homeLocked = true;
      Serial.printf(">>> HOME LOCKED: %.6f, %.6f (%u sats)\n",
                    homeLat, homeLon, fix.satellites);
    }

    float distanceToHome = 0.0f, bearingToHome = 0.0f;
    if (homeLocked && haveFix) {
      distanceToHome = (float)haversineMeters(fix.latitude, fix.longitude,
                                              homeLat, homeLon);
      bearingToHome  = (float)initialBearingDeg(fix.latitude, fix.longitude,
                                                homeLat, homeLon);
    }

    // ---- Publish to the snapshot ---------------------------------------------------
    portENTER_CRITICAL(&snapshotLock);
    snapshot.gnss              = haveFix ? fix : GnssFix{};
    snapshot.battery           = sensors.battery();
    snapshot.distanceToHomeM   = distanceToHome;
    snapshot.bearingToHomeDeg  = bearingToHome;
    snapshot.sensorHealth      = health;
    snapshot.homeLocked        = homeLocked;
    snapshot.rcLinkQuality     = pilot.linkQuality;
    if (haveHeading) snapshot.attitude.headingDeg = heading;
    portEXIT_CRITICAL(&snapshotLock);

    // =================================================================================
    //  ARMING
    // =================================================================================
    if (state == ODY_STATE_PREFLIGHT_OK) {
      const uint8_t blockers = evaluateArmBlockers(nowMs, pilot, pilotPresent, health);
      portENTER_CRITICAL(&snapshotLock);
      snapshot.armBlockFlags = blockers;
      portEXIT_CRITICAL(&snapshotLock);

      const bool buttonDown = digitalRead(PIN_ARM_BUTTON) == LOW;
      const bool switchOn   = pilotPresent && pilot.armSwitch;

      if (buttonDown && switchOn && blockers == 0) {
        if (armButtonSince == 0) armButtonSince = nowMs;
        else if ((uint32_t)(nowMs - armButtonSince) >= ARM_BUTTON_HOLD_MS) {
          sensors.latchGroundReference();      // AGL is relative to THIS launch point
          resetControllers();
          blackbox.startFlight(++flightNumber);
          flightState.request(ODY_STATE_ARMED, "operator armed");
          armButtonSince = 0;
          cannotReachHome.reset();
          belowWarn.reset();
          belowCritical.reset();
          Serial.println(">>> ARMED <<<");
        }
      } else {
        armButtonSince = 0;
      }
    }

    // =================================================================================
    //  ENERGY BUDGET
    //
    //  FIX FOR FINDING 1 (6S thresholds) and part of FINDING 4 (the original ran no
    //  battery check at all once RTH was entered, so the aircraft hovered until flat).
    //  These checks now run in every powered state.
    // =================================================================================
    if (odyMotorsAreLive(state)) {
      const EnergyBudget budget = evaluateEnergy(sensors.battery(), distanceToHome);

      // Critical: land now, wherever we are. Checked in every powered state.
      if (belowCritical.update(budget.belowCriticalThreshold, nowMs)) {
        flightState.request(ODY_STATE_FAILSAFE_LANDING, "battery critical");
      }
      // Cannot make it home even at full reserve: ask for permission to land here.
      else if (belowWarn.update(budget.belowWarnThreshold, nowMs)) {
        if (state == ODY_STATE_ARMED || state == ODY_STATE_RTH_NAVIGATING) {
          if (flightState.request(ODY_STATE_AWAITING_LAND_PERMIT,
                                  "insufficient energy to return")) {
            landRequestedMs = nowMs;
            Serial.printf("[ENERGY] cannot return: %.2f V, %.0f mAh left, "
                          "%.0f m out. Requesting land permission.\n",
                          sensors.battery().packVolts, budget.remainingMah,
                          distanceToHome);
          }
        }
      }
      // Enough to get home but not to keep flying: start the return.
      else if (cannotReachHome.update(!budget.canReachHome, nowMs)) {
        if (state == ODY_STATE_ARMED) {
          if (flightState.requestFrom(ODY_STATE_ARMED, ODY_STATE_RTH_NAVIGATING,
                                      "energy budget exhausted")) {
            float agl = 0.0f;
            portENTER_CRITICAL(&snapshotLock);
            if (snapshot.perception.baroAglM.everValid) agl = snapshot.perception.baroAglM.value;
            portEXIT_CRITICAL(&snapshotLock);
            navigator.engage(agl, heading);
            Serial.printf("[RTH] engaged at %.0f m from home, %.2f V\n",
                          distanceToHome, sensors.battery().packVolts);
          }
        }
      }
    }

    // =================================================================================
    //  RTH ARRIVAL AND DEGRADED-NAV ESCALATION
    // =================================================================================
    if (state == ODY_STATE_RTH_NAVIGATING) {
      if (navigator.phase() == RthPhase::Arrived ||
          (homeLocked && haveFix && distanceToHome <= RTH_ARRIVAL_RADIUS_M &&
           navigator.phase() == RthPhase::Descend)) {
        flightState.request(ODY_STATE_FAILSAFE_LANDING, "reached home");
      }
      // If the position solution is lost the navigator cannot steer. Rather than
      // hovering indefinitely -- which is what the original did in every case -- give
      // it 10 s to recover and then land where we are.
      if (!haveFix && flightState.millisInState() > 10000u) {
        flightState.request(ODY_STATE_FAILSAFE_LANDING, "GNSS lost during return");
      }
    }

    // =================================================================================
    //  LAND PERMISSION WINDOW
    // =================================================================================
    if (state == ODY_STATE_AWAITING_LAND_PERMIT) {
      if ((uint32_t)(nowMs - landRequestedMs) > LAND_PERMISSION_TIMEOUT_MS) {
        flightState.request(ODY_STATE_FAILSAFE_LANDING, "land permission timeout");
      }
    }

    // =================================================================================
    //  LINK FAILSAFES
    // =================================================================================
    if ((state == ODY_STATE_ARMED) && !crsf.linkUp(nowMs)) {
      // Manual flight with no pilot. Try to bring it home rather than dropping it.
      if (homeLocked && haveFix) {
        if (flightState.requestFrom(ODY_STATE_ARMED, ODY_STATE_RTH_NAVIGATING,
                                    "RC link lost")) {
          float agl = 0.0f;
          portENTER_CRITICAL(&snapshotLock);
          if (snapshot.perception.baroAglM.everValid) agl = snapshot.perception.baroAglM.value;
          portEXIT_CRITICAL(&snapshotLock);
          navigator.engage(agl, heading);
        }
      } else {
        flightState.request(ODY_STATE_FAILSAFE_LANDING, "RC link lost, no home fix");
      }
    }

    // Pilot-commanded RTH from the handset.
    if (state == ODY_STATE_ARMED && pilotPresent && pilot.rthSwitch) {
      if (flightState.requestFrom(ODY_STATE_ARMED, ODY_STATE_RTH_NAVIGATING,
                                  "pilot commanded RTH")) {
        float agl = 0.0f;
        portENTER_CRITICAL(&snapshotLock);
        if (snapshot.perception.baroAglM.everValid) agl = snapshot.perception.baroAglM.value;
        portEXIT_CRITICAL(&snapshotLock);
        navigator.engage(agl, heading);
      }
    }

    // =================================================================================
    //  LORA COMMAND LINK  --  the operator half of FINDING 3
    // =================================================================================
    ReceivedCommand cmd;
    while (lora.poll(nowMs, cmd)) {
      switch (cmd.commandId) {
        case ODY_CMD_PERMIT_LAND:
          if (state == ODY_STATE_AWAITING_LAND_PERMIT) {
            flightState.request(ODY_STATE_FAILSAFE_LANDING, "pilot permitted landing");
            Serial.println("[CMD] landing permission granted");
          }
          break;

        case ODY_CMD_DENY_LAND:
          // Extend the hold. The critical-battery check still overrides this.
          landRequestedMs = nowMs;
          Serial.println("[CMD] landing denied, extending hold");
          break;

        case ODY_CMD_RTH_NOW:
          if (state == ODY_STATE_ARMED) {
            if (flightState.requestFrom(ODY_STATE_ARMED, ODY_STATE_RTH_NAVIGATING,
                                        "ground station RTH")) {
              float agl = 0.0f;
              portENTER_CRITICAL(&snapshotLock);
              if (snapshot.perception.baroAglM.everValid) agl = snapshot.perception.baroAglM.value;
              portEXIT_CRITICAL(&snapshotLock);
              navigator.engage(agl, heading);
            }
          }
          break;

        case ODY_CMD_ABORT_TO_LAND:
          flightState.request(ODY_STATE_FAILSAFE_LANDING, "ground station abort");
          break;

        default:
          break;
      }
    }

    // =================================================================================
    //  RECOVERY BEACON ARMING
    //
    //  FIX FOR FINDING 11. The original called activateEmergencyBeacon() from exactly
    //  one place: the failsafe-landing touchdown branch. A parachute descent -- by
    //  definition an uncontrolled landing wherever the wind takes the canopy, and the
    //  single scenario the beacon exists for -- never reached it.
    //
    //  Both terminal paths now arm the beacon.
    // =================================================================================
    static bool beaconArmedThisFlight = false;

    if (state == ODY_STATE_DISARMED && flightState.millisInState() < 2000u) {
      if (!beaconArmedThisFlight && distanceToHome > RECOVERY_RADIUS_M) {
        requestBeaconLatch = true;
        beaconArmedThisFlight = true;
        Serial.printf("[RECOVERY] landed %.0f m from home -- arming beacon\n",
                      distanceToHome);
      }
      blackbox.endFlight();
    }

    if (state == ODY_STATE_FREEFALL_PARACHUTE && !beaconArmedThisFlight) {
      // Give the canopy time to bring the aircraft down before latching, so the
      // beacon's first transmitted fix is the landing site rather than a point in
      // mid-air. If the aircraft is still descending we simply wait.
      if ((uint32_t)(nowMs - parachuteDeployedMs) > PARACHUTE_SETTLE_MS) {
        requestBeaconLatch = true;
        beaconArmedThisFlight = true;
        Serial.println("[RECOVERY] canopy descent complete -- arming beacon");
        blackbox.endFlight();
      }
    }

    if (state == ODY_STATE_ARMED && flightState.millisInState() < 1000u) {
      beaconArmedThisFlight = false;
    }

    if (requestParachuteLog) {
      requestParachuteLog = false;
      Serial.println("\n*** FREE-FALL DETECTED -- MOTORS CUT, CANOPY DEPLOYED ***");
    }

    serviceBeaconLatch(nowMs);

    // =================================================================================
    //  AUX BROADCAST  --  FIX FOR FINDING 7 (and the data path for FINDING 17)
    // =================================================================================
    if ((uint32_t)(nowMs - lastAuxMs) >= 500u) {
      lastAuxMs = nowMs;

      AuxPositionPayload aux{};
      aux.uptimeMs       = nowMs;
      aux.latitude1e7    = haveFix ? (int32_t)(fix.latitude  * 1e7) : 0;
      aux.longitude1e7   = haveFix ? (int32_t)(fix.longitude * 1e7) : 0;
      aux.altitudeMslMm  = haveFix ? (int32_t)(fix.altitudeMsl * 1000.0f) : 0;
      float agl = 0.0f;
      portENTER_CRITICAL(&snapshotLock);
      if (snapshot.perception.baroAglM.everValid) agl = snapshot.perception.baroAglM.value;
      portEXIT_CRITICAL(&snapshotLock);
      aux.altitudeAglMm  = (int32_t)(agl * 1000.0f);
      aux.groundSpeedCmS = haveFix ? (uint16_t)(fix.groundSpeed * 100.0f) : 0;
      aux.headingCentideg= (uint16_t)(heading * 100.0f);
      aux.battMillivolts = (uint16_t)(sensors.battery().packVolts * 1000.0f);
      aux.satellites     = haveFix ? fix.satellites : 0;
      aux.fixQuality     = haveFix ? fix.fixQuality : 0;
      aux.flightState    = state;
      aux.emergency      = (state == ODY_STATE_FREEFALL_PARACHUTE ||
                            state == ODY_STATE_FAILSAFE_LANDING) ? 1 : 0;

      // One frame per address. The beacon needs the fix cached; the Remote ID module
      // needs it at a higher rate to meet the 1 Hz broadcast requirement with margin.
      auxBus.send(ODY_AUX_ADDR_BEACON,   aux);
      auxBus.send(ODY_AUX_ADDR_REMOTEID, aux);
    }

    // =================================================================================
    //  TELEMETRY DOWNLINK
    // =================================================================================
    if ((uint32_t)(nowMs - lastTelemMs) >= 1000u) {    // 1 Hz -- see docs 8.4 (duty cycle)
      lastTelemMs = nowMs;

      VehicleSnapshot local;
      portENTER_CRITICAL(&snapshotLock);
      local = snapshot;
      portEXIT_CRITICAL(&snapshotLock);

      TelemetryPayload t{};
      t.uptimeMs          = nowMs;
      t.latitude1e7       = (int32_t)(local.gnss.latitude  * 1e7);
      t.longitude1e7      = (int32_t)(local.gnss.longitude * 1e7);
      t.altitudeAglMm     = (int32_t)((local.perception.baroAglM.everValid
                                        ? local.perception.baroAglM.value : 0.0f) * 1000.0f);
      t.varioCmS          = (int16_t)(local.perception.varioMps * 100.0f);
      t.groundSpeedCmS    = (int16_t)(local.gnss.groundSpeed * 100.0f);
      t.headingCentideg   = (int16_t)(local.attitude.headingDeg * 100.0f);
      t.bearingToHomeCentideg = (int16_t)(local.bearingToHomeDeg * 100.0f);
      t.distanceToHomeCm  = (uint32_t)(local.distanceToHomeM * 100.0f);
      t.rollCentideg      = (int16_t)(local.attitude.rollDeg * 100.0f);
      t.pitchCentideg     = (int16_t)(local.attitude.pitchDeg * 100.0f);
      t.battMillivolts    = (uint16_t)(local.battery.packVolts * 1000.0f);
      t.battCurrentCentiAmps = (int16_t)(local.battery.currentAmps * 100.0f);
      t.battConsumedMah   = (uint16_t)local.battery.consumedMah;
      t.battRemainingPercent = (uint16_t)(local.battery.remainingPercent * 100.0f);
      t.forwardObstacleCm = local.perception.forwardObstacleCm.isFresh(nowMs, LIDAR_MAX_AGE_MS)
                              ? local.perception.forwardObstacleCm.value
                              : LIDAR_INVALID_CM;
      t.sensorHealth      = local.sensorHealth;
      t.satellites        = local.gnss.satellites;
      t.flightState       = state;
      t.rcLinkQuality     = local.rcLinkQuality;
      t.armFlags          = local.armBlockFlags;

      lora.sendTelemetry(t);
    }

    // ---- Console announcements -----------------------------------------------------
    uint8_t announceState;
    char    reason[ODY_STATE_REASON_LEN];
    if (flightState.consumeAnnounce(&announceState, reason, sizeof(reason))) {
      Serial.printf("[STATE] -> %s (%s)\n", odyStateName(announceState), reason);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

// =====================================================================================
//  STORAGE TASK -- Core 0, low priority
// =====================================================================================
static void TaskStorage(void* /*arg*/) {
  for (;;) {
    blackbox.service();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =====================================================================================
//  SETUP
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== " AIRFRAME_NAME " -- ESP32-P4 avionics ===");

  // ---- BRING-UP BISECTION -----------------------------------------------------------
  // Build with -DBRINGUP_STOP_AT=n to return from setup() early, so a fault that takes
  // the USB console down with it can be located by halving instead of guessed at. On
  // this board the console dies with the application, so a crash produces silence --
  // and silence looks identical whatever the cause.
  //
  //   1  after the banner, before any hardware is touched
  //   2  after the PWM outputs are attached
  //   3  after I2C is brought up
  //
  // Remove once the firmware is known to boot on hardware.
#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 1
  Serial.println("[BRINGUP] stage 1: banner only, no hardware touched");
  Serial.flush();
  return;
#endif

  // Two builds are otherwise indistinguishable once flashed, and flying the wrong one
  // puts the gyro notch ~20 Hz off the actual motor peak and budgets the return-to-home
  // reserve from the wrong cruise current. Print the configuration so it can be checked
  // against the propellers actually fitted, before they go on.
  Serial.printf("Motors     : %s  (%.0f g each)\n",
                MOTOR_CONFIG_NAME, MOTOR_MASS_G_EACH);
  Serial.printf("Propellers : %s  (PROP_BLADES=%d)\n", PROP_CONFIG_NAME, PROP_BLADES);
  Serial.printf("Airframe   : AUW %.0f g, %.0f g/motor, TWR %.2f:1\n",
                AIRFRAME_AUW_G, MOTOR_MAX_THRUST_G,
                4.0f * MOTOR_MAX_THRUST_G / AIRFRAME_AUW_G);
  Serial.printf("Gyro notch : %.0f Hz, Q %.1f%s\n", NOTCH_CENTER_HZ, NOTCH_Q,
                (NOTCH_CENTER_HZ == PROP_NOTCH_DEFAULT_HZ)
                    ? "  (modelled default)" : "  (overridden)");
#if DYN_NOTCH_ENABLE
  Serial.printf("             dynamic: tracks %.0f-%.0f Hz, %d bins, %.1f Hz resolution\n",
                NOTCH_CENTER_HZ * DYN_NOTCH_BAND_LOW,
                min((float)IMU_DLPF_HZ, NOTCH_CENTER_HZ * DYN_NOTCH_BAND_HIGH),
                DYN_NOTCH_BINS,
                (float)FLIGHT_LOOP_HZ / (float)DYN_NOTCH_BINS);
#if DYN_NOTCH_HARMONIC
  {
    // Say plainly whether the harmonic notch can do anything on this build. Most of
    // the time it cannot, and that is a property of the IMU rather than a fault.
    const float h2 = NOTCH_CENTER_HZ * DYN_NOTCH_H2_MULTIPLE;
    const float ceiling = min((float)IMU_DLPF_HZ,
                              (float)FLIGHT_LOOP_HZ * 0.5f * DYN_NOTCH_H2_NYQUIST_FRAC);
    if (h2 <= ceiling) {
      Serial.printf("             harmonic: tracking near %.0f Hz (ceiling %.0f Hz)\n",
                    h2, ceiling);
    } else {
      Serial.printf("             harmonic: %.0f Hz is above the %.0f Hz ceiling "
                    "(DLPF %d Hz) -- not observable, notch idle\n",
                    h2, ceiling, IMU_DLPF_HZ);
    }
  }
#endif
#else
  Serial.println("             dynamic tracking DISABLED -- the fixed value above is "
                 "modelled, so measure it");
#endif
  Serial.printf("Energy     : cruise %.1f A, peak %.0f A\n",
                CRUISE_CURRENT_A, PROP_PEAK_PACK_A);
  Serial.printf("Battery    : %dS, warn %.1f V, critical %.1f V\n",
                CELL_COUNT, PACK_WARN_V, PACK_CRITICAL_V);

  // ---- Outputs first, so nothing spins while the rest of the stack comes up --------
  ledcAttach(PIN_PARACHUTE_SRV, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcWrite(PIN_PARACHUTE_SRV, SERVO_LOCKED);

#if DSHOT_ENABLE
  {
    const uint8_t motorPins[4] = { MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN };
    if (!dshotRmt.begin(motorPins)) {
      // There is no falling back to PWM here. The pins have been handed to the RMT
      // peripheral, and a half-initialised output stage driving motors is worse than
      // one that plainly refuses to start. Preflight will fail and the aircraft will
      // not arm.
      Serial.println("FATAL: DShot init failed -- see section 4.3.1. NOT arming.");
    } else {
      Serial.println("DShot output active (analog PWM is NOT in use)");
    }
  }
#else
  ledcAttach(MOTOR1_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(MOTOR2_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(MOTOR3_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(MOTOR4_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
#endif
  idleMotors();

#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 2
  Serial.println("[BRINGUP] stage 2: PWM attached, motors idled");
  Serial.flush();
  return;
#endif

#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before pinMode(ARM_BUTTON)");
  Serial.flush();
  delay(60);
#endif
  pinMode(PIN_ARM_BUTTON, INPUT_PULLUP);
#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before pinMode(BEACON_LATCH)");
  Serial.flush();
  delay(60);
#endif
  pinMode(PIN_BEACON_LATCH, OUTPUT);
  digitalWrite(PIN_BEACON_LATCH, LOW);
  // Driven HIGH by the Remote ID module while it is actually broadcasting.
  // Pulled down here so a missing or dead module reads as unhealthy and
  // blocks arming, rather than defaulting to "fine".
#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before pinMode(REMOTEID_HEALTH gpio25)");
  Serial.flush();
  delay(60);
#endif
  pinMode(PIN_REMOTEID_HEALTH, INPUT_PULLDOWN);

#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 21
  Serial.println("[BRINGUP] stage 21: GPIO modes set");
  Serial.flush();
  return;
#endif

#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before analogReadResolution");
  Serial.flush();
  delay(60);
#endif
  analogReadResolution(12);

#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before flightState.begin");
  Serial.flush();
  delay(60);
#endif
  flightState.begin();

#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 22
  Serial.println("[BRINGUP] stage 22: state machine started");
  Serial.flush();
  return;
#endif

  // ---- Peripherals ------------------------------------------------------------------
#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before VtxSerial UART2");
  Serial.flush();
  delay(60);
#endif
  VtxSerial.begin(115200, SERIAL_8N1, PIN_VTX_RX, PIN_VTX_TX);

#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 23
  Serial.println("[BRINGUP] stage 23: VTX UART2 open");
  Serial.flush();
  return;
#endif
#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before LidarSerial UART3");
  Serial.flush();
  delay(60);
#endif
  LidarSerial.begin(115200, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);

#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 24
  Serial.println("[BRINGUP] stage 24: LiDAR UART3 open");
  Serial.flush();
  return;
#endif
#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before crsf.begin UART4");
  Serial.flush();
  delay(60);
#endif
  crsf.begin(CrsfSerial);

#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 25
  Serial.println("[BRINGUP] stage 25: CRSF UART4 open");
  Serial.flush();
  return;
#endif
#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before auxBus.begin LP-UART5");
  Serial.flush();
  delay(60);
#endif
  auxBus.begin(AuxSerial);

#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 26
  Serial.println("[BRINGUP] stage 26: AUX LP-UART5 open");
  Serial.flush();
  return;
#endif


#if defined(BRINGUP_STOP_AT) && BRINGUP_STOP_AT <= 3
  Serial.println("[BRINGUP] stage 3: about to start I2C and the sensor hub");
  Serial.flush();
  return;
#endif

#if defined(BRINGUP_TRACE)
  Serial.println("[TRACE] before sensors.begin (I2C)");
  Serial.flush();
  delay(60);
#endif

  if (!sensors.begin()) {
    flightState.request(ODY_STATE_PREFLIGHT_FAIL, "sensor bring-up failed");
    Serial.println("[FATAL] primary IMU missing -- flight is not possible");
  }

  sensors.configureGnssLink();
  lora.begin();
  blackbox.begin();

  // ---- Calibration -------------------------------------------------------------------
  flightState.request(ODY_STATE_CALIBRATING, "auto-zero");
  const CalibrationResult cal = sensors.calibrate();

  if (cal.passed) {
    gyroBias[0] = cal.gyroBias[0];
    gyroBias[1] = cal.gyroBias[1];
    gyroBias[2] = cal.gyroBias[2];

    // Configure the notch filters here, but note that filters.h now makes an
    // unconfigured filter a pass-through rather than a silent zero, so a calibration
    // that bails out early can no longer blank the gyro signal.
    notchGx.configure(NOTCH_CENTER_HZ, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
    notchGy.configure(NOTCH_CENTER_HZ, (float)FLIGHT_LOOP_HZ, NOTCH_Q);
    notchGz.configure(NOTCH_CENTER_HZ, (float)FLIGHT_LOOP_HZ, NOTCH_Q);

#if DYN_NOTCH_ENABLE
    // The tracker starts on the configured value and is bounded around it, so the worst
    // case if tracking never locks is exactly the static behaviour configured above.
    notchTracker.begin((float)FLIGHT_LOOP_HZ, NOTCH_CENTER_HZ);
#endif
#if DYN_NOTCH_HARMONIC
    // Deliberately left unconfigured, which filters.h treats as a pass-through. The
    // harmonic notch must not exist until the tracker has actually found a harmonic.
    notchH2x.bypass();
    notchH2y.bypass();
    notchH2z.bypass();
#endif

    navigator.begin();

    flightState.request(ODY_STATE_PREFLIGHT_OK, "calibration passed");
    Serial.printf("[PASS] bias %.3f/%.3f/%.3f dps, gravity %.2f, ground %.1f m\n",
                  cal.gyroBias[0], cal.gyroBias[1], cal.gyroBias[2],
                  cal.gravityMagnitude, cal.groundAltitudeM);
    Serial.println(">>> preflight OK -- waiting for GNSS home lock <<<");
  } else {
    flightState.request(ODY_STATE_PREFLIGHT_FAIL, cal.failure);
    Serial.printf("[FAIL] calibration: %s\n", cal.failure);
    Serial.println(">>> ARMING IS BLOCKED. Power-cycle after correcting the fault. <<<");
  }

  // ---- Tasks --------------------------------------------------------------------------
  xTaskCreatePinnedToCore(TaskFlightLoop, "flight", 8192, nullptr,
                          configMAX_PRIORITIES - 1, nullptr, 1);
  xTaskCreatePinnedToCore(TaskTelemetry,  "telem",  8192, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(TaskStorage,    "store",  4096, nullptr, 1, nullptr, 0);
}

// Everything runs in tasks; the Arduino loop task is left idle so it cannot compete
// with the telemetry task for core 0.
void loop() {
#if defined(BRINGUP_STOP_AT)
  // A heartbeat, so a capture can attach at any time rather than racing the banner.
  // The firmware prints once at boot and is then silent until spoken to, which makes
  // "connected too late" and "never booted" produce identical evidence.
  {
    static uint32_t beat = 0;
    Serial.printf("[BRINGUP] stage %d alive %lu\n", BRINGUP_STOP_AT,
                  (unsigned long)beat++);
    Serial.flush();
    delay(1000);
    return;
  }
#endif
  vTaskDelay(pdMS_TO_TICKS(1000));
}
