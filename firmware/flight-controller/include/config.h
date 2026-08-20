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
//  1. AIRFRAME
// =====================================================================================
#define AIRFRAME_NAME             "Odyssey-10 Pro"
#define AIRFRAME_AUW_G            1632.0f   // all-up weight, grams (see docs section 3)
#define MOTOR_MAX_THRUST_G        1400.0f   // per motor, MODELLED at 6S / 9x5x3 -- verify

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
#define PACK_CAPACITY_MAH         4500.0f
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
#define FLIGHT_LOOP_HZ            500
#define FLIGHT_LOOP_DT            (1.0f / (float)FLIGHT_LOOP_HZ)
#define FLIGHT_LOOP_PERIOD_US     (1000000u / FLIGHT_LOOP_HZ)
#define TELEM_TASK_HZ             50
#define BLACKBOX_LOG_HZ           100

// Dynamic bi-quad gyro notch.
//
// Raised from 80 Hz when the airframe moved to the 387 mm 9-inch frame. A smaller disc
// needs more shaft speed for the same thrust, so the hover fundamental moved up.
//
// TWO INDEPENDENT ESTIMATES DISAGREE, which is worth knowing before trusting either:
//
//   scaling the 10-inch figure by (10/9)^2 ............. about  97 Hz
//   momentum theory from hover thrust and disc area .... about 105 Hz
//
// A 7% spread. 100 Hz is set as the midpoint, NOT because it is known to be right but
// because it is the least-wrong single number available without data. At Q = 4 the
// notch is -3 dB about 12 Hz either side, so 100 Hz covers roughly 94-106 Hz -- which
// spans both estimates, but only just.
//
// MEASURE IT. Take a BlackBox gyro trace at hover, find the actual peak, and set this
// from the data. The 6 mm arms on this frame are also less stiff than the 7-8 mm arms
// previously specified, which moves the structural resonance independently of the
// propellers, so neither model above accounts for it.
#define NOTCH_CENTER_HZ           100.0f
#define NOTCH_Q                   4.0f

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
#define RTH_CRUISE_SPEED_MPS      12.0f
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

// Nominal cruise current, used to budget the mAh required to fly home.
#define CRUISE_CURRENT_A          10.0f

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
#define REQUIRE_REMOTE_ID_TO_ARM  0

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

#define PIN_PARACHUTE_SRV         26
#define PIN_GNSS_RX               17
#define PIN_GNSS_TX               18
#define PIN_VTX_TX                19
#define PIN_VTX_RX                20
#define PIN_LIDAR_RX              22
#define PIN_LIDAR_TX              23
#define PIN_CRSF_RX               27       // ExpressLRS -> FC
#define PIN_CRSF_TX               28       // FC -> ExpressLRS (telemetry to handset)
#define PIN_AUX_BUS_TX            24       // broadcast to beacon + Remote ID modules
#define PIN_REMOTEID_HEALTH       25       // driven HIGH by the Remote ID module
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
#define MOTOR4_PIN                15       // front-left,  CCW

// I2C device addresses
#define I2C_ADDR_MPU6050          0x68
#define I2C_ADDR_ICM42688         0x69
#define I2C_ADDR_BMP280           0x76
#define I2C_ADDR_QMC5883L         0x0D
#define I2C_ADDR_VL53L1X          0x29
#define I2C_ADDR_INA226           0x40

#define I2C_BUS_HZ                400000
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
static_assert(FLIGHT_LOOP_HZ % BLACKBOX_LOG_HZ == 0,
              "BlackBox rate must divide the flight loop rate evenly");
#endif

#endif // ODY_CONFIG_H
