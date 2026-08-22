// =====================================================================================
//  Odyssey-10 Pro -- Flight Controller Configuration
//  ------------------------------------------------------------------------------------
//  Every tunable constant lives here. Nothing in this file is derived at runtime, so
//  the static_asserts at the bottom catch an inconsistent airframe definition at
//  COMPILE time rather than at 400 m AGL.
//
//  The single most important thing in this file is CELL_COUNT. The original firmware
//  hard-coded 3S voltage thresholds (10.2 V warn / 9.9 V cut) into a 6S airframe, so
//  every battery failsafe was unreachable and the aircraft would have flown until the
//  pack collapsed (finding 1). Thresholds are now derived per-cell from CELL_COUNT.
// =====================================================================================

#ifndef ODY_CONFIG_H
#define ODY_CONFIG_H

#include <stdint.h>

// =====================================================================================
//  1. AIRFRAME AND PROPELLER CONFIGURATION
//
//  ------------------------------------------------------------------------------------
//  PROP_BLADES IS THE ONE SWITCH. Set it and everything downstream follows.
//  ------------------------------------------------------------------------------------
//
//  Blade count changes far more than the propellers. It moves the all-up weight, the
//  peak thrust, the hover point, the power loading, the cruise current that sizes the
//  return-to-home reserve, and -- most importantly -- the gyro notch frequency, because
//  a 2-blade propeller must spin about 21% faster than a 3-blade for the same thrust.
//
//  Before this switch existed those five constants had to be changed together by hand.
//  Changing four of them and forgetting the notch would leave the filter attenuating
//  empty spectrum while still adding phase lag in the control band, which is worse than
//  having no notch at all. That is exactly the kind of coupled edit a person gets wrong
//  once and never notices.
//
//  Which to choose is a real decision, not a default to accept blindly -- see docs
//  section 3.2. Briefly: two blades give ~10% more endurance and range, three give
//  ~13% more thrust and a lower hover point. The common "three blades for wind"
//  argument does not hold for this aircraft; the wind-limited mission is
//  endurance-limited, not thrust-limited.
//
//  Override from the build system with -DPROP_BLADES=3 rather than editing this file,
//  so a configuration change shows up in the build log.
// =====================================================================================
#define AIRFRAME_NAME             "Odyssey-10 Pro"

// =====================================================================================
//  THREE SWITCHES, IN DEPENDENCY ORDER
//
//      FRAME_SIZE_IN   7, 9 or 10        default 9
//      MOTOR_CLASS     must suit the frame; each frame has a default
//      PROP_BLADES     2 or 3            default 2
//
//  Everything else is derived. Ten combinations are characterised; anything outside
//  them stops the build.
//
//  ------------------------------------------------------------------------------------
//  CHARACTERISATION STATUS -- READ THIS BEFORE TRUSTING A NON-DEFAULT BUILD
//
//  The 9-inch / 2810 / 2-blade configuration is the one this document was written
//  around. Its mass budget is itemised against a real BOM, and every figure in the
//  specification traces to it.
//
//  The 7-inch and 10-inch parameter sets are MODELLED, scaled from that anchor by
//  momentum theory and disc loading. They are coherent and they are a sound starting
//  point, but no one has flown them. Treat every number in a non-default build as a
//  hypothesis to be checked on a thrust stand, and expect the PID gains in particular
//  to need tuning -- FRAME_GAIN_SCALE is a first-order guess from rotational inertia,
//  not a tune.
//
//  A 5-inch airframe was investigated and deliberately excluded: its hover fundamental
//  lands near 265 Hz, which needs a loop rate above the MPU-6050's 1 kHz output ceiling
//  to filter honestly. The hardware cannot do it, so pretending otherwise would be
//  worse than saying no.
// =====================================================================================

#ifndef FRAME_SIZE_IN
#define FRAME_SIZE_IN             9        // 7, 9 (default) or 10
#endif

// -------------------------------------------------------------------------------------
//  Motor identifiers. Mass and thrust factor come with each; the 900 KV winding is
//  common to the 9- and 10-inch motors, while the 7-inch 2807 is a 1300 KV part because
//  a smaller propeller wants more shaft speed.
// -------------------------------------------------------------------------------------
#define MOTOR_2807                2807
#define MOTOR_2810                2810
#define MOTOR_3110                3110
#define MOTOR_3115                3115

// =====================================================================================
//  FRAME
// =====================================================================================
#if FRAME_SIZE_IN == 7
  #define FRAME_WHEELBASE_MM      295
  #define PROP_DIAMETER_IN        7
  #define FRAME_BASE_DRY_G        440.0f   // no motors, propellers or battery
  #define BATTERY_MASS_G          470.0f   // 6S 3000 mAh 50C
  #define PACK_CAPACITY_MAH       3000.0f
  #define PACK_MIN_C_RATE         50       // peak draw is high on a 7-inch
  #define PAYLOAD_RESERVE_G       100.0f
  #define FRAME_DEFAULT_LOOP_HZ   1000     // 180 Hz fundamental needs the headroom
  #define FRAME_DEFAULT_DLPF_HZ   260      // must sit above the 180 Hz notch
  #define FRAME_GAIN_SCALE        1.29f    // ~9/7; smaller airframe, faster response
  #define FRAME_CRUISE_SPEED_MPS  14.0f
  #define FRAME_DEFAULT_MOTOR     MOTOR_2807
  #define FRAME_CONFIG_NAME       "7-inch (295 mm)"

#elif FRAME_SIZE_IN == 9
  #define FRAME_WHEELBASE_MM      387
  #define PROP_DIAMETER_IN        9
  #define FRAME_BASE_DRY_G        538.0f
  #define BATTERY_MASS_G          640.0f   // 6S 4500 mAh 20C
  #define PACK_CAPACITY_MAH       4500.0f
  #define PACK_MIN_C_RATE         20
  #define PAYLOAD_RESERVE_G       170.0f
  #define FRAME_DEFAULT_LOOP_HZ   500
  #define FRAME_DEFAULT_DLPF_HZ   184      // must sit above the 120 Hz notch
  #define FRAME_GAIN_SCALE        1.00f    // the reference airframe
  #define FRAME_CRUISE_SPEED_MPS  12.0f
  #define FRAME_DEFAULT_MOTOR     MOTOR_2810
  #define FRAME_CONFIG_NAME       "9-inch (387 mm)"

#elif FRAME_SIZE_IN == 10
  #define FRAME_WHEELBASE_MM      430
  #define PROP_DIAMETER_IN        10
  #define FRAME_BASE_DRY_G        594.0f
  #define BATTERY_MASS_G          640.0f   // 6S 4500 mAh 30C
  #define PACK_CAPACITY_MAH       4500.0f
  #define PACK_MIN_C_RATE         30
  #define PAYLOAD_RESERVE_G       200.0f
  #define FRAME_DEFAULT_LOOP_HZ   500
  #define FRAME_DEFAULT_DLPF_HZ   184      // must sit above the 105 Hz notch
  #define FRAME_GAIN_SCALE        0.90f    // ~9/10; larger airframe, slower response
  #define FRAME_CRUISE_SPEED_MPS  12.0f
  #define FRAME_DEFAULT_MOTOR     MOTOR_3115
  #define FRAME_CONFIG_NAME       "10-inch (430 mm)"

#else
  #error "FRAME_SIZE_IN must be 7, 9 or 10. A 5-inch needs a loop rate the MPU-6050 cannot feed."
#endif

#define PROP_PITCH_IN             5

// =====================================================================================
//  MOTOR
// =====================================================================================
#ifndef MOTOR_CLASS
#define MOTOR_CLASS               FRAME_DEFAULT_MOTOR
#endif

#if MOTOR_CLASS == MOTOR_2807
  #define MOTOR_MASS_G_EACH       45.0f
  #define MOTOR_STATOR_MM         28
  #define MOTOR_KV                1300
  #define MOTOR_THRUST_FACTOR     1.000f
  #define MOTOR_CONFIG_NAME       "2807 1300 KV"

#elif MOTOR_CLASS == MOTOR_2810
  #define MOTOR_MASS_G_EACH       52.0f
  #define MOTOR_STATOR_MM         28
  #define MOTOR_KV                900
  #define MOTOR_THRUST_FACTOR     1.000f   // reference
  #define MOTOR_CONFIG_NAME       "2810 900 KV"

#elif MOTOR_CLASS == MOTOR_3110
  #define MOTOR_MASS_G_EACH       68.0f
  #define MOTOR_STATOR_MM         31
  #define MOTOR_KV                900
  #define MOTOR_THRUST_FACTOR     1.033f
  #define MOTOR_CONFIG_NAME       "3110 900 KV"

#elif MOTOR_CLASS == MOTOR_3115
  #define MOTOR_MASS_G_EACH       78.0f
  #define MOTOR_STATOR_MM         31       // 31 mm diameter, 15 mm tall
  #define MOTOR_KV                900
  #define MOTOR_THRUST_FACTOR     1.070f
  #define MOTOR_CONFIG_NAME       "3115 900 KV"

#else
  #error "MOTOR_CLASS must be MOTOR_2807, MOTOR_2810, MOTOR_3110 or MOTOR_3115."
#endif

// -------------------------------------------------------------------------------------
//  Frame / motor compatibility.
//
//  A 2807 on a 10-inch frame is under-stator'd and a 3115 on a 7-inch is dead weight, so
//  the invalid pairings stop the build rather than producing plausible-looking numbers
//  for an aircraft nobody should build.
// -------------------------------------------------------------------------------------
#if FRAME_SIZE_IN == 7 && MOTOR_CLASS != MOTOR_2807
  #error "7-inch: only MOTOR_2807 is characterised."
#endif
#if FRAME_SIZE_IN == 9 && MOTOR_CLASS != MOTOR_2810 && MOTOR_CLASS != MOTOR_3110
  #error "9-inch: only MOTOR_2810 or MOTOR_3110 are characterised."
#endif
#if FRAME_SIZE_IN == 10 && MOTOR_CLASS != MOTOR_3110 && MOTOR_CLASS != MOTOR_3115
  #error "10-inch: only MOTOR_3110 or MOTOR_3115 are characterised."
#endif

// =====================================================================================
//  PROPELLER
//
//  Figures are quoted against each frame's DEFAULT motor and scaled by
//  MOTOR_THRUST_FACTOR below.
// =====================================================================================
#ifndef PROP_BLADES
#define PROP_BLADES               2        // 2 (default) or 3
#endif

#if PROP_BLADES != 2 && PROP_BLADES != 3
  #error "PROP_BLADES must be 2 or 3. Other blade counts are not characterised."
#endif

#if FRAME_SIZE_IN == 7
  #if PROP_BLADES == 2
    #define PROP_MASS_G_EACH      5.0f
    #define PROP_BASE_THRUST_G    1350.0f
    #define PROP_NOTCH_DEFAULT_HZ 180.0f
    #define PROP_POWER_LOADING_GW 7.92f
    #define PROP_BASE_PEAK_A      75.0f
  #else
    #define PROP_MASS_G_EACH      6.0f
    #define PROP_BASE_THRUST_G    1530.0f
    #define PROP_NOTCH_DEFAULT_HZ 150.0f
    #define PROP_POWER_LOADING_GW 7.17f
    #define PROP_BASE_PEAK_A      99.0f
  #endif

#elif FRAME_SIZE_IN == 9
  #if PROP_BLADES == 2
    #define PROP_MASS_G_EACH      7.0f
    #define PROP_BASE_THRUST_G    1230.0f
    #define PROP_NOTCH_DEFAULT_HZ 120.0f
    #define PROP_POWER_LOADING_GW 8.90f
    #define PROP_BASE_PEAK_A      51.0f
  #else
    #define PROP_MASS_G_EACH      9.0f
    #define PROP_BASE_THRUST_G    1400.0f
    #define PROP_NOTCH_DEFAULT_HZ 100.0f
    #define PROP_POWER_LOADING_GW 8.05f
    #define PROP_BASE_PEAK_A      67.0f
  #endif

#else   // 10 inch
  #if PROP_BLADES == 2
    #define PROP_MASS_G_EACH      10.0f
    #define PROP_BASE_THRUST_G    1500.0f
    #define PROP_NOTCH_DEFAULT_HZ 105.0f
    #define PROP_POWER_LOADING_GW 9.42f
    #define PROP_BASE_PEAK_A      65.0f
  #else
    #define PROP_MASS_G_EACH      12.0f
    #define PROP_BASE_THRUST_G    1750.0f
    #define PROP_NOTCH_DEFAULT_HZ 90.0f
    #define PROP_POWER_LOADING_GW 8.52f
    #define PROP_BASE_PEAK_A      89.0f
  #endif
#endif

#define PROP_CONFIG_NAME          (PROP_BLADES == 2 ? "2-blade" : "3-blade")

// =====================================================================================
//  DERIVED -- so ten combinations cannot disagree with each other
// =====================================================================================
#define AVIONICS_POWER_W          18.0f    // FC, VTX, camera, BEC losses
#define CRUISE_POWER_FACTOR       1.10f    // cruise costs ~10% more than hover
#define CONNECTOR_RATING_A        90.0f    // XT90-S continuous

#define PROP_AUW_DEFAULT_G        (FRAME_BASE_DRY_G                     \
                                 + 4.0f * MOTOR_MASS_G_EACH             \
                                 + 4.0f * PROP_MASS_G_EACH              \
                                 + BATTERY_MASS_G                       \
                                 + PAYLOAD_RESERVE_G)

#define MOTOR_MAX_THRUST_G        (PROP_BASE_THRUST_G * MOTOR_THRUST_FACTOR)
#define PROP_PEAK_PACK_A          (PROP_BASE_PEAK_A * MOTOR_THRUST_FACTOR)
#define PACK_MAX_DISCHARGE_A      (PACK_CAPACITY_MAH / 1000.0f * PACK_MIN_C_RATE)

// The notch is the one constant the documentation actively tells you to replace with a
// measurement, so it takes an override. PROP_BLADES only supplies the starting point.
//
//     -DNOTCH_CENTER_HZ=108.0f
//
// after reading the real peak off a BlackBox gyro trace at hover.
#ifndef NOTCH_CENTER_HZ
#define NOTCH_CENTER_HZ           PROP_NOTCH_DEFAULT_HZ
#endif

// All-up weight takes an override too, because payload changes it and the energy budget
// depends on it. PROP_BLADES supplies the empty-with-reserve figure.
#ifndef AIRFRAME_AUW_G
#define AIRFRAME_AUW_G            PROP_AUW_DEFAULT_G
#endif

// =====================================================================================
//  2. BATTERY  --  FIX FOR FINDING 1
//  -------------------------------------------------------------------------------
//  Derive everything from the cell count. A 6S pack running the old 9.9 V cutoff would
//  have to be at 1.65 V/cell before the failsafe fired -- long after the pack is
//  destroyed and the aircraft has fallen out of the sky.
// =====================================================================================
#define CELL_COUNT                6

#define CELL_FULL_V               4.20f
#define CELL_NOMINAL_V            3.70f
#define CELL_LAUNCH_MIN_V         3.85f   // refuse to arm below this
#define CELL_WARN_V               3.40f   // "cannot make it home" threshold
#define CELL_CRITICAL_V           3.30f   // land immediately, no negotiation
#define CELL_RESERVE_V            0.10f   // margin held back for the descent

#define PACK_FULL_V               (CELL_FULL_V      * CELL_COUNT)   // 25.20 V
#define PACK_NOMINAL_V            (CELL_NOMINAL_V   * CELL_COUNT)   // 22.20 V
#define PACK_LAUNCH_MIN_V         (CELL_LAUNCH_MIN_V* CELL_COUNT)   // 23.10 V
#define PACK_WARN_V               (CELL_WARN_V      * CELL_COUNT)   // 20.40 V
#define PACK_CRITICAL_V           (CELL_CRITICAL_V  * CELL_COUNT)   // 19.80 V
#define PACK_RESERVE_V            (CELL_RESERVE_V   * CELL_COUNT)   //  0.60 V

// Pack capacity and the fraction of it we are willing to spend in flight.
// PACK_CAPACITY_MAH and PACK_MIN_C_RATE are set by FRAME_SIZE_IN in section 1 --
// a 7-inch needs a 50C pack for its peak draw, a 9-inch only 20C.
#define PACK_USABLE_FRACTION      0.80f
#define PACK_USABLE_MAH           (PACK_CAPACITY_MAH * PACK_USABLE_FRACTION)

// Voltage sag under load makes an instantaneous reading a poor state-machine input.
// The pack voltage is filtered and a threshold must hold continuously for the debounce
// window before it latches an irreversible mode change.
#define BATT_FILTER_ALPHA         0.02f    // ~1 s time constant at the 50 Hz task rate
#define BATT_DEBOUNCE_MS          3000u
#define BATT_ADC_SAMPLES          8        // oversampling per read

// Voltage divider: 100k upper / 10k lower -> ratio 11.0. At PACK_FULL_V this presents
// 25.2 / 11 = 2.29 V to the ADC, comfortably inside the 3.3 V reference.
#define VOLTAGE_DIVIDER_RATIO     11.0f
#define VOLTAGE_DIVIDER_TRIM      1.0f     // per-airframe calibration; see docs 11.3

// =====================================================================================
//  3. FLIGHT CONTROL LOOP
// =====================================================================================
// -------------------------------------------------------------------------------------
//  LOOP RATE
//
//  The frame chooses a default; -DFLIGHT_LOOP_HZ=1000 overrides it. Three constants have
//  to move with it and they are derived here rather than left to be remembered:
//
//    IMU_DLPF_HZ     the anti-alias corner can rise once Nyquist does, and at 1000 Hz it
//                    goes to 260 Hz -- which is also what finally puts the second
//                    harmonic below the corner on the 9- and 10-inch. See section 8.3.3.
//    DYN_NOTCH_BINS  the SDFT's resolution is loop rate over bins, so the bin count has
//                    to double to hold 3.9 Hz.
//    every divider   the log, notch-update, accelerometer and barometer rates all divide
//                    the loop rate, and all are asserted exact.
//
//  Leaving any of them behind is the DLPF defect of section 8.3 all over again, which is
//  precisely why they are computed rather than configured.
//
//  1000 Hz IS NOT THE DEFAULT ON THE 9- AND 10-INCH, and section 8.3.4 says why: the
//  ESCs are driven by 400 Hz PWM, so 60% of motor writes are overwritten before they
//  reach one. A faster loop buys filtering resolution, not control authority. It becomes
//  the right default when the DShot driver of section 4.3.1 exists.
// -------------------------------------------------------------------------------------
#ifndef FLIGHT_LOOP_HZ
#define FLIGHT_LOOP_HZ            FRAME_DEFAULT_LOOP_HZ
#endif

#ifndef IMU_DLPF_HZ
  #if FLIGHT_LOOP_HZ >= 1000
    // Nyquist is 500 Hz, so the MPU-6050's widest setting is both legal and useful. It
    // also switches the part from a 1 kHz output rate to 8 kHz, which is what makes
    // sampling it at 1000 Hz honest rather than a beat between two free-running clocks.
    #define IMU_DLPF_HZ           260
  #else
    #define IMU_DLPF_HZ           FRAME_DEFAULT_DLPF_HZ
  #endif
#endif

// FLIGHT_LOOP_HZ was set by FRAME_SIZE_IN in section 1. A 7-inch runs at 1000 Hz
// because its hover fundamental is near 180 Hz, and a 500 Hz loop puts that
// uncomfortably close to Nyquist.
#define FLIGHT_LOOP_DT            (1.0f / (float)FLIGHT_LOOP_HZ)
#define FLIGHT_LOOP_PERIOD_US     (1000000u / FLIGHT_LOOP_HZ)
#define TELEM_TASK_HZ             50
#define BLACKBOX_LOG_HZ           100

// Dynamic bi-quad gyro notch.
//
// NOTCH_CENTER_HZ is set by PROP_BLADES in section 1, because blade count is what moves
// it: a 2-blade propeller spins about 21% faster than a 3-blade for the same thrust.
//
//   2-blade   momentum theory 124 Hz | scaling 121 Hz -> set 120 Hz
//   3-blade   momentum theory 104 Hz | scaling  ~-    -> set 100 Hz
//
// At Q = 4 the notch is -3 dB roughly 15 Hz either side, so a 120 Hz notch does
// essentially nothing for a 104 Hz peak. The propeller and the notch are a matched
// pair; that is why they are set together rather than separately.
//
// MEASURE IT. Take a BlackBox gyro trace at a stable hover, find the actual peak, and
// override NOTCH_CENTER_HZ from the data. None of the models account for the frame
// itself: the 6 mm arms on this airframe are less stiff than the 7-8 mm arms originally
// specified, which moves the structural resonance independently of the propellers.
#define NOTCH_Q                   4.0f

// -------------------------------------------------------------------------------------
//  DYNAMIC NOTCH
//
//  Tracks the real motor peak from the gyro spectrum instead of trusting the compiled
//  centre. See dynamic_notch.h for why -- briefly, the static value has been wrong or
//  stale at almost every point in this project's history, and it is a fixed number
//  standing in for a quantity that varies with mass, air density, pack voltage and
//  workload.
//
//  It is BOUNDED so it cannot do worse than the static value it replaces: the tracked
//  centre is clamped to a band around NOTCH_CENTER_HZ, a peak must be both tall enough
//  AND repeat in the same bin before it is believed, and the centre slews rather than
//  jumping. If tracking fails, the notch sits exactly where the static config put it.
//
//  DYN_NOTCH_MIN_CONFIDENCE is 8, not 4, because white noise across a ~30-bin band
//  already produces a peak-to-mean ratio near 4 about half the time -- a threshold set
//  there would believe noise. The bin-repeat test is what does the real rejecting.
//
//  Set DYN_NOTCH_ENABLE to 0 to fall back to the fixed notch entirely.
// -------------------------------------------------------------------------------------
#ifndef DYN_NOTCH_ENABLE
#define DYN_NOTCH_ENABLE          1
#endif

// Resolution is FLIGHT_LOOP_HZ / DYN_NOTCH_BINS, so the bin count follows the loop
// rate. 3.9 Hz either way; a 1000 Hz loop with 128 bins would give 7.8 Hz, which is
// coarse against a notch band only a few tens of hertz wide.
#ifndef DYN_NOTCH_BINS
  #if FLIGHT_LOOP_HZ >= 1000
    #define DYN_NOTCH_BINS        256
  #else
    #define DYN_NOTCH_BINS        128
  #endif
#endif
#define DYN_NOTCH_UPDATE_HZ       20       // spectrum re-evaluation rate
#define DYN_NOTCH_BAND_LOW        0.60f    // search from 0.6x the static centre
#define DYN_NOTCH_BAND_HIGH       1.60f    // ...to 1.6x, further capped by the DLPF
#define DYN_NOTCH_MIN_CONFIDENCE  8.0f     // peak must be 8x the band average...
#define DYN_NOTCH_PEAK_TOL_BINS   2        // ...and land in the same place twice running
#define DYN_NOTCH_SLEW_HZ_PER_S   40.0f    // rate limit on centre movement
#define DYN_NOTCH_RETUNE_HZ       1.5f     // ignore movement smaller than this

// -------------------------------------------------------------------------------------
//  SECOND-HARMONIC NOTCH
//
//  A propeller puts energy at 2x the shaft fundamental as well as at the fundamental
//  itself. Tracking and notching it is standard on modern flight controllers.
//
//  ON THIS HARDWARE IT USUALLY CANNOT BE SEEN. The MPU-6050's anti-alias filter sits at
//  IMU_DLPF_HZ, and 2*f0 is ABOVE that corner in eight of the ten build combinations --
//  including the 9-inch default, where 2*f0 is 240 Hz against a 184 Hz corner and also
//  96% of the 250 Hz Nyquist limit. The IMU has already attenuated it before the flight
//  controller receives a sample.
//
//  Notching a peak the DLPF has already removed is not free: it is the DLPF defect of
//  section 8.3 with the roles reversed. The filter would contribute phase lag in the
//  control band in exchange for attenuating spectrum that is already gone.
//
//  So observability is checked at RUNTIME, against the TRACKED fundamental rather than
//  the compiled one -- the whole point of the dynamic notch is that the real f0 is
//  measured, so whether 2*f0 clears the corner must be judged from the measurement too.
//  Where it does not clear, the harmonic notch never engages and costs nothing.
//
//  Raising IMU_DLPF_HZ is NOT the fix. The DLPF is the anti-alias filter; lifting it
//  toward Nyquist trades a visible harmonic for aliased content folding into the
//  control band. The real fix is a faster loop, which the MPU-6050's 1 kHz output
//  ceiling does not leave much room for. See section 8.3.3.
// -------------------------------------------------------------------------------------
#ifndef DYN_NOTCH_HARMONIC
#define DYN_NOTCH_HARMONIC        1        // 0 disables the second notch entirely
#endif

#define DYN_NOTCH_H2_MULTIPLE     2.0f     // which harmonic: the second
#define DYN_NOTCH_H2_SEARCH       0.12f    // search +/-12% around 2x the tracked f0
#define DYN_NOTCH_H2_NYQUIST_FRAC 0.80f    // and no closer than 80% of Nyquist
#define DYN_NOTCH_H2_MIN_CONF     8.0f     // same height gate as the fundamental

// Complementary filter weighting for the attitude estimate
#define ATT_COMP_ALPHA            0.992f

// Angle-mode outer loop
#define ANGLE_MAX_DEG             30.0f
#define ANGLE_P_GAIN              4.5f
#define YAW_RATE_MAX_DPS          150.0f

// =====================================================================================
//  4. MOTOR OUTPUT
//
//  Motor layout and rotation. THIS IS THE AUTHORITATIVE DEFINITION -- the section 4
//  diagram in the original document contradicted it on all four motors, which would
//  have inverted the yaw sign and produced an uncommanded flat spin (finding 2).
//
/*
 *                          FRONT
 *        M4 front-left CCW  \   /  M2 front-right CW
 *                            \ /
 *                            / \
 *        M3 rear-left  CW   /   \  M1 rear-right CCW
 *                           REAR
 *
 *  (Block comment deliberately: a `//` line ending in a backslash is a line
 *   continuation, so the diagonal would silently swallow the line beneath it.)
 */
//
//  Props-out (reversed rotation): the blade crossing the front of each motor travels
//  away from the airframe centreline. For the front-right motor that is clockwise
//  when viewed from above.
// =====================================================================================
#define PWM_FREQ_HZ               400

// -------------------------------------------------------------------------------------
//  DSHOT
//
//  DEFAULTS TO OFF, and that is deliberate. The frame arithmetic in dshot.h is fully
//  host-tested, but the RMT timing that puts those frames on a wire cannot be verified
//  without an oscilloscope. This code drives motors: a frame that misses a bit boundary
//  does not fail politely, it becomes a throttle value the ESC will act on.
//
//  Turn it on with -DDSHOT_ENABLE=1 once the waveform has been checked on a scope and
//  then on a thrust stand with the propellers OFF.
//
//  The reason to want it is not the throttle resolution. It is that bidirectional DShot
//  returns per-motor RPM, which makes shaft frequency a measurement instead of the
//  estimate the whole of section 8.3 is built around.
// -------------------------------------------------------------------------------------
#ifndef DSHOT_ENABLE
#define DSHOT_ENABLE              0
#endif
#ifndef DSHOT_BIDIRECTIONAL
#define DSHOT_BIDIRECTIONAL       1        // no point without the telemetry
#endif
#ifndef DSHOT_BITRATE_KHZ
#define DSHOT_BITRATE_KHZ         300      // DShot300: 3.33 us per bit
#endif
#ifndef DSHOT_TELEM_TIMEOUT_US
#define DSHOT_TELEM_TIMEOUT_US    200      // give up on a reply after this
#endif

// Pole pairs, for turning electrical RPM into shaft RPM. Every motor offered here is a
// 12N14P outrunner: 14 magnets, 7 pole pairs. Getting this wrong scales every derived
// frequency by a constant, which presents as a mis-tuned notch rather than as the units
// error it is -- so it is asserted rather than assumed.
#ifndef MOTOR_POLE_PAIRS
#define MOTOR_POLE_PAIRS          7
#endif
#define PWM_RES_BITS              12
#define PWM_PERIOD_COUNTS         (1u << PWM_RES_BITS)             // 4096
#define PWM_MIN                   1638      // 1000 us at 400 Hz / 12-bit
#define PWM_MAX                   3276      // 2000 us
#define PWM_ARM_IDLE              1750      // ~1068 us, props turning at idle
#define PWM_RANGE                 (PWM_MAX - PWM_MIN)

// Throttle authority reserved for attitude corrections at full stick. The mixer
// (see mixer.h) will steal from throttle before it ever clips a motor.
#define MIXER_HEADROOM_COUNTS     300

#define SERVO_FREQ_HZ             50
#define SERVO_RES_BITS            16
#define SERVO_LOCKED              3277      // 1000 us
#define SERVO_EJECT               6553      // 2000 us

// =====================================================================================
//  5. NAVIGATION AND RETURN-TO-HOME  --  FIX FOR FINDING 4
//
//  The original RTH state levelled the aircraft and held a fixed throttle. It never
//  computed a bearing, never yawed, never pitched forward, so the distance to home
//  never decreased and the exit condition (distance < 5 m) was unreachable. It also
//  ran no battery check at all once entered, so the aircraft hovered until the pack
//  was flat. The parameters below drive a real position controller.
// =====================================================================================
#define RTH_CRUISE_SPEED_MPS      FRAME_CRUISE_SPEED_MPS
#define RTH_APPROACH_SPEED_MPS    3.0f
#define RTH_SAFE_ALTITUDE_M       30.0f    // climb to this AGL before translating
#define RTH_ARRIVAL_RADIUS_M      5.0f
#define RTH_DESCENT_BUFFER_S      15.0f
#define RTH_MAX_PITCH_DEG         25.0f
#define RTH_POS_P                 0.50f    // metres of error -> m/s of demand
#define RTH_VEL_P                 2.00f    // m/s of error -> degrees of pitch
#define RTH_YAW_P                 2.50f    // degrees of heading error -> deg/s
#define RTH_ALT_P                 1.20f

// Beyond this distance from home the aircraft is "lost" for recovery purposes and the
// beacon is armed on touchdown.
#define RECOVERY_RADIUS_M         15.0f

// CRUISE_CURRENT_A is DERIVED, not tabulated.
//
// It is the current drawn flying home at cruise speed, and it budgets the charge the
// return leg will cost -- so it must be the cruise figure, not hover. Revisions 2.2 to
// 2.4 used hover power, about 10% lower, which made the return-to-home reserve
// optimistic.
//
// Deriving it from AIRFRAME_AUW_G means a payload override flows through automatically.
// The approximation is that PROP_POWER_LOADING_GW is treated as constant, when disc
// loading actually degrades it slightly as mass rises; at the masses involved that is
// about 2% optimistic, comfortably inside the 20% margin the energy budget already
// applies to the required mAh (see navigation.cpp).
//
// Override with -DCRUISE_CURRENT_A=... once you have measured it.
#ifndef CRUISE_CURRENT_A
#define CRUISE_CURRENT_A          (((AIRFRAME_AUW_G / PROP_POWER_LOADING_GW)   \
                                    + AVIONICS_POWER_W)                        \
                                   * CRUISE_POWER_FACTOR / PACK_NOMINAL_V)
#endif

// =====================================================================================
//  6. FAILSAFE AND LANDING
// =====================================================================================
#define CRSF_TIMEOUT_MS           500u     // ExpressLRS runs at >=50 Hz; 500 ms is 25 frames
#define LORA_CMD_TIMEOUT_MS       30000u   // command link is low rate; long timeout
#define LAND_PERMISSION_TIMEOUT_MS 15000u
#define FAILSAFE_LAND_TIMEOUT_MS  45000u   // hard ceiling on a descent
#define FAILSAFE_DESCENT_MPS      0.5f

// -------------------------------------------------------------------------------
//  Touchdown detection  --  FIX FOR FINDING 14
//  The original test was a single barometric comparison (relative altitude <= 0.05 m)
//  against a ground reference captured once at power-on. Ambient pressure drift over a
//  20-minute flight is easily a metre, so the test could pass in mid-air and cut all
//  four motors. Touchdown now requires agreement from independent evidence and is
//  vetoed outright if any altitude source says we are still up.
// -------------------------------------------------------------------------------
#define TOUCHDOWN_TOF_M           0.20f    // laser rangefinder says we are on the deck
#define TOUCHDOWN_BARO_M          0.60f    // fallback when the ToF is unhealthy
#define TOUCHDOWN_VARIO_MPS       0.25f    // vertical motion has stopped
#define TOUCHDOWN_HOLD_MS         400u     // ...and stayed stopped
#define TOUCHDOWN_VETO_AGL_M      1.50f    // any source above this blocks the disarm

// Precision landing flare (VL53L1X), previously specified but never implemented.
#define FLARE_ENGAGE_AGL_M        2.00f
#define FLARE_DESCENT_MPS         0.20f

// =====================================================================================
//  7. PARACHUTE  --  FIX FOR FINDING 12
//
//  Detection is enabled in every airborne state, not just ARMED and RTH. The original
//  gate excluded AWAITING_LAND_PERMIT (hovering, up to 15 s) and FAILSAFE_LANDING
//  (powered descent, up to 45 s) -- both states in which a shed propeller produces a
//  genuine free fall.
//
//  A minimum altitude gate is added because a canopy deployed below its inflation
//  height is worse than useless: it will not slow the aircraft and it will foul the
//  props. 8 m is the canister manufacturer's stated minimum for this AUW.
// =====================================================================================
#define FREEFALL_ACCEL_MPS2       1.50f    // ~0.15 g
#define FREEFALL_HOLD_MS          400u
#define PARACHUTE_MIN_AGL_M       8.0f
#define PARACHUTE_SETTLE_MS       20000u   // canopy descent before arming the beacon

// =====================================================================================
//  8. PERCEPTION  --  FIX FOR FINDING 15
//
//  Every perception reading carries a timestamp and is treated as invalid once stale.
//  The original code latched the last valid LiDAR distance forever: a sensor that died
//  while an obstacle was inside 3.5 m would clamp forward pitch for the rest of the
//  flight, and a sensor that never enumerated left the default 9999 in place so
//  avoidance was silently inactive.
// =====================================================================================
#define LIDAR_MAX_AGE_MS          200u
#define TOF_MAX_AGE_MS            200u
#define GNSS_MAX_AGE_MS           2000u
#define MAG_MAX_AGE_MS            500u
#define IMU_MAX_AGE_MS            50u
#define CURRENT_MAX_AGE_MS        500u

#define OBSTACLE_STOP_CM          350      // hard brake and step over
#define OBSTACLE_SLOW_CM          600      // proportional velocity scaling begins
#define OBSTACLE_BRAKE_PITCH_DEG  -5.0f
#define OBSTACLE_STEPOVER_M       5.0f     // commanded climb, previously undocumented in code
#define OBSTACLE_STEPOVER_RATE_MPS 1.5f
#define LIDAR_INVALID_CM          0xFFFFu

// =====================================================================================
//  9. ARMING  --  FIX FOR FINDING 5
//
//  The original arm path checked only PREFLIGHT_OK + home lock + button. currentRC
//  retained the last received throttle across flights, so arming with a raised stick
//  drove all four props to ~82% instantly.
// =====================================================================================
// -------------------------------------------------------------------------------
//  REMOTE ID POLICY
//
//  0 = Remote ID is OPTIONAL. Its health is still reported in telemetry and shown on
//      the ground station, but a missing, unconfigured or failed module does NOT block
//      arming. This is the default, and it is correct for the aircraft as specified:
//      a privately built airframe under 25 kg flown in the EU open category A3 is not
//      class-marked, and the Direct Remote ID obligation attaches to class-marked
//      C1/C2/C3 aircraft.
//
//  1 = Remote ID is REQUIRED to arm. Set this if any of the following apply:
//        * you fly in the SPECIFIC category (DRI is required there regardless of
//          weight or class);
//        * your aircraft is class-marked C1/C2/C3;
//        * you are in the United States, where 14 CFR Part 89 applies above 250 g
//          including to home-built aircraft;
//        * you are in Denmark after the proposed Trafikstyrelsen electronic-visibility
//          rules take effect (proposed 1 January 2027 -- confirm what was adopted);
//        * your national authority requires it for any other reason.
//
//  Getting this wrong in the permissive direction is a legal problem; getting it wrong
//  in the restrictive direction grounds a legal aircraft. Neither default is safe for
//  everyone, so the choice is explicit. See docs section 12.1.
// -------------------------------------------------------------------------------
#ifndef REQUIRE_REMOTE_ID_TO_ARM
#define REQUIRE_REMOTE_ID_TO_ARM  0
#endif

#define ARM_THROTTLE_MAX_US       1050     // stick must be at the bottom
#define ARM_MAX_TILT_DEG          8.0f     // aircraft must be roughly level
#define ARM_MIN_SATELLITES        6
#define ARM_BUTTON_DEBOUNCE_MS    50u
#define ARM_BUTTON_HOLD_MS        1000u    // deliberate press, not a brush

// =====================================================================================
//  10. GNSS  --  FIX FOR FINDING 16
//
//  Section 9 of the original document annotated the BN-220 at 115200 baud while the
//  firmware opened the port at 9600. Following the documentation left the aircraft
//  unable to acquire a fix and therefore unable to arm, with no diagnostic. The
//  firmware now probes both rates, then configures the module to the documented
//  115200 / 10 Hz and verifies the change took.
// =====================================================================================
#define GNSS_TARGET_BAUD          115200
#define GNSS_FALLBACK_BAUD        9600
#define GNSS_TARGET_RATE_HZ       10
#define GNSS_PROBE_TIMEOUT_MS     1500u

// =====================================================================================
//  11. PIN MAP
//
//  UART allocation on the ESP32-P4 (5 full UARTs + 1 LP-UART):
//    UART0   USB / debug console
//    UART1   BN-220 GNSS
//    UART2   VTX MSP OSD
//    UART3   TFmini-S forward LiDAR
//    UART4   ExpressLRS 2.4 GHz receiver (CRSF, 420 kbaud) -- new, fixes finding 3
//    LP-UART AUX broadcast to beacon + Remote ID     -- new, fixes findings 7 and 17
// =====================================================================================
#define PIN_LORA_SCK              12
#define PIN_LORA_MISO             13
#define PIN_LORA_MOSI             11
#define PIN_LORA_NSS              10
#define PIN_LORA_RST              9
#define PIN_LORA_DIO0             3

#define PIN_SD_CS                 33
#define PIN_SD_MOSI               34
#define PIN_SD_SCK                35
#define PIN_SD_MISO               36

#ifndef PIN_PARACHUTE_SRV
#define PIN_PARACHUTE_SRV         26
#endif
#ifndef PIN_GNSS_RX
#define PIN_GNSS_RX               17
#endif
#ifndef PIN_GNSS_TX
#define PIN_GNSS_TX               18
#endif
#ifndef PIN_VTX_TX
#define PIN_VTX_TX                19
#endif
#define PIN_VTX_RX                20
#define PIN_LIDAR_RX              22
#define PIN_LIDAR_TX              23
#ifndef PIN_CRSF_RX
#define PIN_CRSF_RX               27       // ExpressLRS -> FC
#endif
#define PIN_CRSF_TX               28       // FC -> ExpressLRS (telemetry to handset)
#ifndef PIN_AUX_BUS_TX
#define PIN_AUX_BUS_TX            24       // broadcast to beacon + Remote ID modules
#endif
#ifndef PIN_REMOTEID_HEALTH
#define PIN_REMOTEID_HEALTH       25       // driven HIGH by the Remote ID module
#endif
                                           // while it is broadcasting; INPUT_PULLDOWN
                                           // here, so a dead module blocks arming

#define PIN_I2C_SDA               7
#define PIN_I2C_SCL               8

#define PIN_BATT_ADC              1
#define PIN_ARM_BUTTON            2
#define PIN_BEACON_LATCH          21

#define MOTOR1_PIN                4        // rear-right,  CCW
#define MOTOR2_PIN                5        // front-right, CW
#define MOTOR3_PIN                6        // rear-left,   CW
#ifndef MOTOR4_PIN
#define MOTOR4_PIN                15       // front-left,  CCW
#endif

// I2C device addresses
#define I2C_ADDR_MPU6050          0x68
#define I2C_ADDR_ICM42688         0x69
#define I2C_ADDR_BMP280           0x76
#define I2C_ADDR_QMC5883L         0x0D
#define I2C_ADDR_VL53L1X          0x29
#define I2C_ADDR_INA226           0x40

#define I2C_BUS_HZ                400000

// -------------------------------------------------------------------------------------
//  PRIMARY IMU READ SPLIT
//
//  The Adafruit driver's getEvent() reads 14 bytes every call -- accelerometer,
//  temperature and gyroscope together. At 400 kHz that is 388 us, which is 19% of a
//  500 Hz loop period and would be 39% of a 1000 Hz one, on a bus shared with the backup
//  IMU, barometer, magnetometer and rangefinder.
//
//  The gyro genuinely needs the loop rate: it is integrated for attitude and it is the
//  signal the notch filters work on. The accelerometer does not. It contributes only the
//  slow correction term in the complementary filter -- ATT_COMP_ALPHA weights it at
//  1-alpha per iteration -- and the free-fall detector, which needs 400 ms of sustained
//  low magnitude and so sees a hundred samples at 250 Hz.
//
//  So the read is split. Gyro-only is 6 bytes and 208 us. See section 8.3.4 for the bus
//  arithmetic; the short version is that this makes a 1000 Hz loop cost about what the
//  500 Hz loop costs today, and it lowers bus pressure at 500 Hz in the meantime.
// -------------------------------------------------------------------------------------
#ifndef IMU_ACCEL_READ_HZ
#define IMU_ACCEL_READ_HZ         250
#endif

// MPU-6050 register map and the scale factors for the ranges set in initMpu6050().
// Reading registers directly means these are ours to get right rather than the
// driver's, so they are asserted against the configured ranges in the host tests.
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_ACCEL_LSB_PER_G   4096.0f    // +/-8 g range
#define MPU6050_GYRO_LSB_PER_DPS  65.5f      // +/-500 dps range
#define STANDARD_GRAVITY_MPS2     9.80665f
#define INA226_SHUNT_OHMS         0.001f
#define INA226_MAX_CURRENT_A      80.0f

// =====================================================================================
//  12. COMPILE-TIME CONSISTENCY CHECKS
//
//  These exist because finding 1 was a pure units mismatch that no amount of code
//  review of the flight logic would have caught -- but a two-line assertion does.
// =====================================================================================
#ifdef __cplusplus
static_assert(PACK_CRITICAL_V > 15.0f,
              "Critical cutoff looks like a 3S threshold on a 6S pack -- check CELL_COUNT");
static_assert(PACK_CRITICAL_V < PACK_WARN_V,
              "Critical cutoff must sit below the warning threshold");
static_assert(PACK_WARN_V < PACK_LAUNCH_MIN_V,
              "Launch minimum must sit above the warning threshold");
static_assert(PACK_LAUNCH_MIN_V < PACK_FULL_V,
              "Launch minimum cannot exceed a full pack");
static_assert(PACK_FULL_V / VOLTAGE_DIVIDER_RATIO < 3.10f,
              "Divider ratio too low: a full pack would saturate the 3.3 V ADC");
static_assert(PWM_MIN < PWM_ARM_IDLE && PWM_ARM_IDLE < PWM_MAX,
              "Idle throttle must sit between the endpoints");
static_assert(PWM_MAX < (int)PWM_PERIOD_COUNTS,
              "PWM_MAX exceeds the timer period for the configured resolution");
static_assert(OBSTACLE_STOP_CM < OBSTACLE_SLOW_CM,
              "Obstacle brake distance must be inside the slow-down distance");
static_assert(TOUCHDOWN_TOF_M < TOUCHDOWN_VETO_AGL_M,
              "Touchdown threshold must sit below the veto altitude");
static_assert(MOTOR_POLE_PAIRS >= 2 && MOTOR_POLE_PAIRS <= 14,
              "Pole pairs outside this range is a units error, not a motor");
static_assert(DSHOT_BITRATE_KHZ == 150 || DSHOT_BITRATE_KHZ == 300 ||
              DSHOT_BITRATE_KHZ == 600 || DSHOT_BITRATE_KHZ == 1200,
              "DShot runs at 150, 300, 600 or 1200 kbit/s and nothing else");
// A DShot frame is 16 bits. Sending one per loop iteration has to fit in the period,
// with room for the ESC's reply when bidirectional telemetry is enabled.
static_assert((16 * 1000 / DSHOT_BITRATE_KHZ) + DSHOT_TELEM_TIMEOUT_US
              < (1000000 / FLIGHT_LOOP_HZ),
              "A DShot frame plus its telemetry reply does not fit inside one flight "
              "loop period at this bitrate");
static_assert(FLIGHT_LOOP_HZ == 500 || FLIGHT_LOOP_HZ == 1000,
              "Only 500 and 1000 Hz are characterised. Both divide 1000 exactly, which "
              "the FreeRTOS tick period depends on, and both keep every derived divider "
              "whole.");
// The reason for a 1000 Hz loop is spectral, so check the spectrum actually improved.
static_assert(FLIGHT_LOOP_HZ < 1000 || IMU_DLPF_HZ >= 260,
              "A 1000 Hz loop that leaves the anti-alias corner where a 500 Hz loop put "
              "it gains nothing -- the corner is what hides the second harmonic");
static_assert(IMU_ACCEL_READ_HZ > 0 && FLIGHT_LOOP_HZ % IMU_ACCEL_READ_HZ == 0,
              "IMU_ACCEL_READ_HZ must divide FLIGHT_LOOP_HZ exactly, or the accelerometer "
              "divider drifts against the loop");
static_assert(IMU_ACCEL_READ_HZ <= FLIGHT_LOOP_HZ,
              "The accelerometer cannot be read faster than the loop that reads it");
// The free-fall detector needs enough samples inside its hold window to be trustworthy.
static_assert(IMU_ACCEL_READ_HZ * FREEFALL_HOLD_MS / 1000 >= 20,
              "Too few accelerometer samples inside the free-fall hold window");
static_assert(FLIGHT_LOOP_HZ % BLACKBOX_LOG_HZ == 0,
              "BlackBox rate must divide the flight loop rate evenly");

// ---- PROP_BLADES coherence -----------------------------------------------------------
// These exist because the whole point of the switch is that the constants move TOGETHER.
// If someone overrides one by hand and leaves the rest, the build should stop.
static_assert(FRAME_SIZE_IN == 7 || FRAME_SIZE_IN == 9 || FRAME_SIZE_IN == 10,
              "FRAME_SIZE_IN must be 7, 9 or 10");
static_assert(PROP_DIAMETER_IN == FRAME_SIZE_IN,
              "Propeller diameter must match the frame size");
static_assert(PROP_BLADES == 2 || PROP_BLADES == 3,
              "PROP_BLADES must be 2 or 3");
// The notch must be filterable at the configured loop rate. This is the assertion that
// excluded a 5-inch airframe: its ~265 Hz fundamental needs a loop the MPU-6050 cannot
// feed, and a notch above Nyquist filters an alias rather than the noise.
// The anti-alias filter must sit ABOVE the peak the notch is aimed at, or it removes
// that peak itself and the notch filters nothing while the DLPF's phase lag stays in
// the control band. A 30% margin keeps the notch centre off the DLPF's -3 dB knee.
//
// This assertion exists because the real thing happened: the DLPF was set to 94 Hz when
// the notch was 80 Hz, the notch then moved to 120 Hz across four revisions, and the
// DLPF was left behind. Nothing compared them until the build matrix check did.
static_assert(DYN_NOTCH_BAND_LOW > 0.0f && DYN_NOTCH_BAND_LOW < 1.0f,
              "Dynamic notch lower band must sit below the static centre");
static_assert(DYN_NOTCH_BAND_HIGH > 1.0f && DYN_NOTCH_BAND_HIGH < 3.0f,
              "Dynamic notch upper band is implausible");
static_assert(DYN_NOTCH_UPDATE_HZ > 0 && DYN_NOTCH_UPDATE_HZ <= 100,
              "Dynamic notch update rate is implausible");
static_assert(DYN_NOTCH_MIN_CONFIDENCE > 5.0f,
              "A confidence threshold this low is inside the noise floor of the search "
              "band -- see the note above");
static_assert(DYN_NOTCH_H2_MULTIPLE >= 2.0f && DYN_NOTCH_H2_MULTIPLE <= 4.0f,
              "The tracked harmonic must be a real overtone of the fundamental");
static_assert(DYN_NOTCH_H2_SEARCH > 0.0f && DYN_NOTCH_H2_SEARCH < 0.5f,
              "The harmonic search window is either empty or wide enough to catch the "
              "fundamental itself");
static_assert(DYN_NOTCH_H2_NYQUIST_FRAC > 0.0f && DYN_NOTCH_H2_NYQUIST_FRAC < 1.0f,
              "The harmonic ceiling must sit below Nyquist");
static_assert(DYN_NOTCH_PEAK_TOL_BINS >= 1 && DYN_NOTCH_PEAK_TOL_BINS <= 4,
              "Peak-repeat tolerance is either too strict to ever lock or too loose to "
              "reject a wandering noise peak");
static_assert((DYN_NOTCH_BINS & (DYN_NOTCH_BINS - 1)) == 0 && DYN_NOTCH_BINS >= 32,
              "Dynamic notch bin count must be a power of two and give usable resolution");
static_assert(IMU_DLPF_HZ > PROP_NOTCH_DEFAULT_HZ * 1.3f,
              "IMU DLPF must sit at least 30% above the notch centre");
static_assert(IMU_DLPF_HZ < FLIGHT_LOOP_HZ / 2.0f,
              "IMU DLPF must sit below Nyquist for the loop rate");
static_assert(PROP_NOTCH_DEFAULT_HZ < FLIGHT_LOOP_HZ / 2.0f,
              "Hover fundamental is above Nyquist for this loop rate -- raise "
              "FLIGHT_LOOP_HZ or the notch will chase an alias");
static_assert(PROP_PEAK_PACK_A <= PACK_MAX_DISCHARGE_A,
              "Peak draw exceeds what the specified pack can deliver -- raise "
              "PACK_MIN_C_RATE for this frame");
static_assert(MOTOR_CLASS == MOTOR_2807 || MOTOR_CLASS == MOTOR_2810
           || MOTOR_CLASS == MOTOR_3110 || MOTOR_CLASS == MOTOR_3115,
              "MOTOR_CLASS is not one of the characterised motors");
static_assert(MOTOR_MASS_G_EACH > 30.0f && MOTOR_MASS_G_EACH < 120.0f,
              "Motor mass is implausible for this airframe class");
static_assert(MOTOR_THRUST_FACTOR >= 1.0f && MOTOR_THRUST_FACTOR < 1.2f,
              "Motor thrust factor is outside the characterised range");
static_assert(NOTCH_CENTER_HZ < FLIGHT_LOOP_HZ / 2.0f,
              "Notch centre is above Nyquist for the flight loop rate");
// Sanity band only, NOT a tight coupling to PROP_BLADES. The documentation tells you to
// override this from a measurement, and a real hover peak could land anywhere sensible;
// an assertion that second-guessed the measurement would be worse than none.
static_assert(NOTCH_CENTER_HZ > 50.0f && NOTCH_CENTER_HZ < 200.0f,
              "Notch centre is outside any plausible motor-noise band for this airframe");
// Ordering between 2- and 3-blade is a comparison ACROSS builds, so it cannot be
// asserted from inside one. tools/check_consistency.py does it. What belongs here is a
// plausibility band for this frame.
static_assert(MOTOR_MAX_THRUST_G > 800.0f && MOTOR_MAX_THRUST_G < 2500.0f,
              "Peak thrust per motor is outside any plausible band for these frames");
static_assert(4.0f * MOTOR_MAX_THRUST_G / AIRFRAME_AUW_G > 2.0f,
              "Thrust-to-weight below 2:1 is not safely flyable -- check AIRFRAME_AUW_G "
              "if you have overridden it for payload");
static_assert(AIRFRAME_AUW_G > 1000.0f && AIRFRAME_AUW_G < 4000.0f,
              "All-up weight is implausible for this airframe");
static_assert(CRUISE_CURRENT_A > 0.0f && CRUISE_CURRENT_A < 30.0f,
              "Cruise current is implausible -- it budgets the return-to-home reserve");
#if !defined(ACCEPT_CONNECTOR_OVER_RATING)
static_assert(PROP_PEAK_PACK_A <= CONNECTOR_RATING_A,
              "Full-throttle draw exceeds the XT90-S 90 A continuous rating. This is a "
              "BURST condition, not continuous, and the connector will survive brief "
              "punch-outs -- but you should know about it. Either fit a higher-rated "
              "connector (AS150), drop to 2-blade propellers, or build with "
              "-DACCEPT_CONNECTOR_OVER_RATING=1 to acknowledge it deliberately.");
#endif
#endif

#endif // ODY_CONFIG_H
