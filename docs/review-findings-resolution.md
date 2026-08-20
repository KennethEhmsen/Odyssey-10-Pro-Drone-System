# Review Findings — Resolution Record

Eighteen defects were confirmed in revision 1.0 of the Odyssey-10 Pro specification.
This document records what each one was, why it mattered, and exactly what changed.

Nine were flight-critical. Two could not be fixed in software and required hardware
additions. One (finding 17) described a capability that was not merely unimplemented but
impossible with the specified parts.

**Summary:** 18 findings, 18 resolved. Two hardware additions ($26.50), BOM total moved
from $512.50 to $540.50.

---

## Severity breakdown

| Severity | Count | Findings |
| --- | --- | --- |
| Critical | 7 | 1, 2, 3, 4, 5, 6, 7 |
| High | 9 | 8, 9, 10, 11, 12, 13, 14, 15, 17 |
| Medium | 1 | 16 |
| Low | 1 | 18 |

---

## Finding 1 — 3S voltage thresholds on a 6S pack

**Severity:** Critical
**Was:** `firmware section 10.1`, lines 439–440

```cpp
const float BATT_WARN_VOLTAGE     = 10.2f;
const float BATT_CRITICAL_CUTOFF  = 9.9f;
```

The airframe carries a 6S pack (22.2 V nominal, 25.2 V full), and the 100k/10k divider
was sized for 6S. Those thresholds correspond to 1.70 V and 1.65 V per cell. A 6S pack
never reaches them while the aircraft is flyable, so `voltageRequiredForRTH` and every
guard downstream of it was permanently false: no return-to-home, no land-permission
request, no critical cutoff. The aircraft would have flown until the pack collapsed.

The burn-rate constant in the same block (0.003 V/s) *was* a correct 6S figure, which is
how we know this was a transcription error rather than a different design intent.

**Now:** Every threshold derives per cell from `CELL_COUNT` in
`firmware/flight-controller/include/config.h`. Compile-time assertions reject a 3S value
on a 6S airframe:

```cpp
static_assert(PACK_CRITICAL_V > 15.0f,
              "Critical cutoff looks like a 3S threshold on a 6S pack -- check CELL_COUNT");
```

The energy budget also gained a second, independent currency — coulomb counting from the
INA226 — so the decision no longer rests on a voltage reading alone.

**Documented in:** specification §5.1, §5.2

---

## Finding 2 — Motor rotation diagram inverted on all four motors

**Severity:** Critical
**Was:** document §4, lines 138–139 and 158–160

The rotation diagram said M4 front-left CW, M2 front-right CCW, M3 rear-left CCW, M1
rear-right CW. The pinout (§9), the firmware comments (§10.1) and the commissioning
checklist (§11) all said the exact opposite. The diagram also contradicted its own
"Props-Out" heading: props-out requires the front-right motor to turn clockwise, and the
diagram drew it counter-clockwise.

The yaw mixer column (`m1 +yaw, m2 -yaw, m3 -yaw, m4 +yaw`) is correct only for the
pinout's assignment. A builder who set ESC rotation from the diagram would have inverted
the yaw sign, so the yaw PID would drive the heading error further from zero — positive
feedback, and an uncontrollable flat spin on the first armed take-off.

**Now:** One authoritative motor map, in specification §4.2 and duplicated as a comment
block in `config.h`, with the diagonal-pairing rule stated explicitly. The commissioning
checklist (§11.2) verifies each motor and each diagonal pair.

---

## Finding 3 — No manual control link existed anywhere

**Severity:** Critical
**Was:** §10.1 and §10.3

`RCCommandPacket` appeared six times in the document: the struct definition, a global
initialiser, a local copy in the flight loop, and the receive path. Nothing anywhere
constructed or transmitted one. The ground station sketch in §10.3 only called
`LoRa.parsePacket()` and wrote to serial.

Because `setup()` seeded `lastRcPacketTime = millis()`, the first thing an armed aircraft
did was fire `RC LINK LOSS TIMEOUT` 1.2 seconds later. The `PERMIT_LAND` branch that the
entire §5 failsafe flowchart depends on was unreachable code — the pilot had no way to
answer the aircraft's request.

This could not be fixed by writing a transmitter for the existing radio. At
SF7/BW125/CR4:5 an 18-byte frame occupies **51.5 ms** of airtime; a 20 Hz stick stream
needs 103% of the channel, and even 5 Hz sits at 26% duty against a 10% regulatory
ceiling.

**Now:** The control path is split by medium.

- **Manual control:** ExpressLRS 2.4 GHz receiver on UART4, CRSF at 420 kbaud, 50–500 Hz,
  with hardware failsafe reporting. *Hardware addition, $20.00, 3 g.*
- **Supervisory:** 433 MHz LoRa carrying 1 Hz telemetry down and operator commands up.
  The ground station firmware now transmits `PERMIT_LAND`, `DENY_LAND`, `RTH_NOW` and
  `ABORT_TO_LAND` from three physical buttons.

2.4 GHz was chosen over 900 MHz ExpressLRS deliberately: the second harmonic of the
433 MHz transmitter lands at 866 MHz, inside the 868 MHz ELRS band.

**Documented in:** specification §5.4, §8.4
**Implemented in:** `firmware/flight-controller/src/radio_link.cpp`,
`firmware/ground-station/src/main.cpp`

---

## Finding 4 — RTH neither navigated nor escalated on battery

**Severity:** Critical
**Was:** §10.1 flight loop, `STATE_RTH_NAVIGATING` branch

```cpp
float rollOut  = pidRoll.update((0.0f - rollEst) * 4.5f, gx, dt);
float pitchOut = pidPitch.update((0.0f - pitchEst) * 4.5f, gy, dt);
float yawOut   = pidYaw.update(0.0f, gz, dt);
int holdThrottle = PWM_ARM_IDLE + 450;
```

That levels the aircraft and holds a fixed throttle. No bearing, no yaw, no forward
pitch. `distHome` therefore never decreased, so the documented exit condition
(`distHome <= 5.0`) was unreachable — despite §5 promising "climb to safe AGL, fly direct
to origin".

Worse: the battery block was gated on `currentState == STATE_ARMED`, so once RTH was
entered **no voltage check ran at all**. With a healthy RC link the aircraft would hover
in place until the pack was flat.

**Now:** A three-phase position controller in
`firmware/flight-controller/src/navigation.cpp` — CLIMB to 30 m AGL, TRANSLATE under a
position→velocity→attitude cascade with yaw-to-bearing, DESCEND inside the arrival
radius. Heading comes from the QMC5883L, tilt-compensated.

Three behaviours the original lacked: translation is gated on heading error (no crabbing
sideways past a forward-only obstacle sensor), obstacle avoidance outranks the navigator,
and the energy checks run in **every** powered state. A lost GNSS fix gives the navigator
10 seconds to recover before the aircraft lands where it is.

**Documented in:** specification §5.3

---

## Finding 5 — No throttle-low check at arming

**Severity:** Critical
**Was:** §10.1 `loop()`, line 960

Arming required only `PREFLIGHT_OK`, a home lock and the button. `currentRC` was a global
updated by every received packet and never reset on disarm, so a stale throttle of 2000
survived across flights. Pressing the arm button applied
`map(2000, 1000, 2000, PWM_ARM_IDLE, PWM_MAX-300) = 2976` on the very next 2 ms
iteration — all four 10-inch propellers to roughly 82% throttle, with the aircraft
possibly in someone's hands.

**Now:** Eight conditions, each reported to the pilot as a bitmask in telemetry so a
blocked arm is never a silent mystery: sensor health, home lock, **throttle at minimum
from a live CRSF frame**, fresh RC, pack above launch minimum, calibration passed,
aircraft level within 8°, and both the physical button and the handset switch held for
one second.

**Documented in:** specification §5.8; bench-verified in §11.3

---

## Finding 6 — Cross-core state race could restart motors under a deployed canopy

**Severity:** Critical
**Was:** §10.1 `triggerFailsafe()`, line 625

`currentState` was a bare `volatile FlightState` written from both cores, and the
failsafe trigger did an unsynchronised read-modify-write. Core 1 could deploy the
parachute in the window between the guard and the store, leaving
`STATE_FREEFALL_PARACHUTE` overwritten with `STATE_FAILSAFE_LANDING`. The flight loop
would then take the powered-descent branch and spin all four motors up to
`PWM_ARM_IDLE + 350` underneath an already-deployed canopy, tangling the lines.

**Now:** Two changes, in `firmware/flight-controller/include/state_machine.h`.

1. Every transition is a single atomic test-and-set under a spinlock — no window.
2. Transitions are escalation-ordered. States are ranked by severity and a request is
   honoured only if it moves to an equal or higher rank.

`FAILSAFE_LANDING` (rank 7) therefore cannot displace `FREEFALL_PARACHUTE` (rank 8) no
matter how the cores interleave. The race is not narrowed; it is made harmless.

**Documented in:** specification §5.6

---

## Finding 7 — Beacon transmitted latitude 0 / longitude 0 forever

**Severity:** Critical
**Was:** §10.2, line 1076

`BeaconPacket packet;` was a file-scope global. The only fields ever written were
`header` (in `setup()`) and `battMillivolts`. The node had no GPS, and the only interface
from the flight controller was a single GPIO latch pulse — which carries no data.

After a crash outside the recovery radius, the beacon would chirp out
`lat=0.000000, lon=0.000000, sats=0` for its whole endurance. A searcher decoding the
packet would get a well-formed fix pointing at Null Island.

**Now:** A hardware change plus a protocol.

- The beacon node is powered during flight through a **Schottky diode-OR** from the
  aircraft's 5 V rail, with the 1S cell as the other input. It is awake for the whole
  flight.
- A new **AUX broadcast bus** (LP-UART, TX only, addressed, CRC-protected) feeds it the
  aircraft's position at 2 Hz. It caches the fix in RTC-backed memory, which survives the
  power handover and the light-sleep cycles.
- The transmitted packet now carries a **fix age** field, so a searcher can distinguish
  "one second before impact" from "four minutes stale".

**Documented in:** specification §7.2; wiring in §9.4; bench-verified in §11.3

---

## Finding 8 — Beacon endurance was out by roughly 5×

**Severity:** High
**Was:** §10.2 and §7

At SF11 / BW 62.5 kHz / CR 4:8 a symbol lasts 32.77 ms and the on-air frame needs 84.25
symbols: **2.76 s of airtime** per transmission. The original sent one every 2 s with no
sleep, so the radio ran at roughly 50% duty — five times the 10% ISM ceiling for 433 MHz
in ITU Region 1. Because `endPacket()` blocks for the whole transmission, the real cadence
was about 4 s rather than the specified 2 s. Average draw came to roughly 85 mA against
an 800 mAh cell: about **nine hours**, not the claimed 48+.

**Now:**

| Phase | Window | Interval | Duty | Average |
| --- | --- | --- | --- | --- |
| 1 | First 2 h | 45 s | 6.1% | ~7.4 mA |
| 2 | After 2 h | 180 s | 1.5% | ~1.8 mA |

Plus a chirp every 5 s (~1.0 mA) and light sleep between events (~0.8 mA). Phase 1 costs
about 21 mAh; phase 2 then runs for well over 100 hours on 680 mAh usable. **The claim is
revised to 72 hours**, which leaves margin for a cold cell and an aged pack.

`transmitBeacon()` also polices its own measured duty cycle at runtime and refuses a
transmission that would breach 10%, so a future schedule change cannot silently put the
aircraft outside the regulations.

**Documented in:** specification §7.3

---

## Finding 9 — MAVLink frames used hardcoded placeholder checksums

**Severity:** High
**Was:** §10.3, lines 1158–1159 and 1184–1185

```cpp
Serial.write(0x55); Serial.write(0xAA);   // heartbeat "checksum"
Serial.write(0x12); Serial.write(0x34);   // attitude "checksum"
```

MAVLink requires a CRC-16/MCRF4XX over the header and payload, seeded with a per-message
`CRC_EXTRA` byte. Every frame failed the receiver's CRC and was discarded, so
QGroundControl and Mission Planner showed nothing at all — despite the section heading
promising compatibility with both.

Two further defects in the same code: the start byte was `0xFE` (MAVLink v1) although the
heading claimed v2, and the sequence byte was a constant (0 for heartbeat, 1 for
attitude), so even a CRC-correct link would have reported 100% packet loss.

**Now:** `firmware/ground-station/src/mavlink_min.h` implements the canonical
`crc_accumulate`, proper v2 framing (0xFD, 10-byte header, 24-bit message id), a per-link
sequence counter, and seven message types instead of two — including `SYS_STATUS` with a
real sensor-health mask and `STATUSTEXT` alerts for state changes and arm blockers.

**Documented in:** specification §10.3

---

## Finding 10 — Ground station showed DISARMED while airborne

**Severity:** High
**Was:** §10.3, line 1205

```cpp
bool armed = (tIn.flightState == 3 || tIn.flightState == 4);
```

Ordinals 3 and 4 were `ARMED` and `RTH_NAVIGATING`. That omitted `AWAITING_LAND_PERMIT`
(5) and `FAILSAFE_LANDING` (6) — both states in which the aircraft is airborne with four
10-inch propellers turning. The operator saw a disarmed vehicle at exactly the moment §5
expects them to approve a landing, and might walk up to a live aircraft.

**Now:** The predicate lives in `shared/odyssey_link.h` as `odyMotorsAreLive()`, compiled
into both the aircraft and the ground station. The header states explicitly that no
caller may open-code an ordinal comparison against the enum.

---

## Finding 11 — Parachute descent never armed the recovery beacon

**Severity:** High
**Was:** §10.1, line 848

`activateEmergencyBeacon()` had exactly one call site: inside the failsafe-landing
touchdown branch. `ejectParachute()` set `STATE_FREEFALL_PARACHUTE`, which fell through
to the terminal `else { disarmMotors(); }` branch with no exit path and no beacon call.

A free-fall event — by definition an uncontrolled descent that lands the aircraft
wherever the wind takes the canopy — therefore ended with GPIO 21 never pulsed. The one
scenario the beacon exists for was the one that did not trigger it.

**Now:** Both terminal paths arm the beacon. The parachute path waits
`PARACHUTE_SETTLE_MS` (20 s) after deployment so the canopy descent completes first, and
the beacon's first transmitted fix is the landing site rather than a point in mid-air.

**Documented in:** specification §7.4

---

## Finding 12 — Free-fall detection disabled in two airborne states

**Severity:** High
**Was:** §10.1, line 757

```cpp
if (currentState == STATE_ARMED || currentState == STATE_RTH_NAVIGATING) {
```

That excluded `AWAITING_LAND_PERMIT` (hovering at altitude for up to 15 s) and
`FAILSAFE_LANDING` (powered descent from cruise altitude, up to 14 s) — precisely the
states in which a shed propeller or an ESC desync produces a genuine free fall, with the
parachute inhibited. `freefallDetectStart` was also not reset on state entry.

Related: the §11 bench test could not pass as written. A 10 cm drop gives
√(2×0.1/9.81) = **143 ms** of free fall against a 400 ms threshold, and inverting the
craft leaves the accelerometer magnitude at ~9.8 m/s², nowhere near the 1.5 m/s² trigger.
A tester following it would conclude the parachute was broken — or loosen the threshold
until the test passed.

**Now:** Detection runs in every state where the motors are live. Two guards added: a
minimum deployment altitude of 8 m AGL (a canopy below its inflation height will not slow
the aircraft and will foul the propellers), and a timer reset on every state entry.

The bench test is replaced with two workable procedures: a tethered drop from ≥ 1.5 m
(550 ms of free fall), or a **`SELFTEST FREEFALL` console command** that injects a
synthetic free-fall condition through the real detection path — proving the detector, the
altitude gate, the servo and the state machine without dropping anything. The injection
is refused unless the aircraft is on the ground.

**Documented in:** specification §5.9, §11.3

---

## Finding 13 — Mixer clamped each motor independently

**Severity:** High
**Was:** §10.1, lines 803–806 and `writeMotorPWM()`

Corrections were summed into each motor and each motor was then clamped in isolation.
That is correct only while nothing clips.

With base throttle at its ceiling of 2976 and a full roll correction of 350, motors 3
and 4 compute to 3326 and clamp to 3276 — losing 50 counts — while motors 1 and 2 drop
cleanly to 2626. The realised differential is 650 against a commanded 700. Stack roll,
pitch and yaw (up to 950 counts) and the shortfall grows large enough to reverse the net
moment on an axis: the aircraft rolls *away* from the correction.

**Now:** A two-stage mixer in `firmware/flight-controller/include/mixer.h`. Build the
correction-only mix; if its range exceeds the motor range, scale all corrections by one
common factor (which preserves the direction of the commanded moment — exactly the
property independent clamping destroys). Then fit the throttle offset into what remains.
When there is not enough range for all three axes, yaw is sacrificed before roll and
pitch.

The surrendered authority is recorded in the BlackBox on every sample and summarised by
`tools/blackbox_decode.py`, so a marginal airframe shows up in the log rather than only
in the crash.

**Documented in:** specification §4.4

---

## Finding 14 — Baro-only touchdown check could disarm mid-air

**Severity:** High
**Was:** §10.1, line 842

```cpp
if (millis() - failsafeStartTime > 14000 || currentAlt <= 0.05f) {
    disarmMotors();
    currentState = STATE_DISARMED;
```

`currentAlt` was barometric altitude relative to a ground reference captured once at
power-on. Ambient pressure drift over a 20-minute flight is easily a metre, and §6 of the
same document acknowledged ground-effect wash on the barometer. That comparison can pass
in mid-air, and the branch immediately cut all four motors.

§6 assigned this job to the VL53L1X below 2 m AGL with a −0.2 m/s regulated flare. The
VL53L1X, along with the INA226 and the QMC5883L, appeared only in the BOM and the prose —
none of the three was referenced anywhere in the firmware. So the precision flare, the
mAh energy tracking and the compass heading lock were all unimplemented.

**Now:** All three sensors are implemented in
`firmware/flight-controller/src/sensors.cpp`. Touchdown requires independent evidence to
agree — a ToF or barometric ground reading, plus vertical motion stopped, plus that state
held for 400 ms — and **any** healthy sensor reporting more than 1.5 m AGL vetoes the
whole thing. The barometric ground reference is re-latched at each arm, so AGL is
relative to this flight's launch point.

**Documented in:** specification §5.7

---

## Finding 15 — LiDAR reading never expired

**Severity:** High
**Was:** §10.1, line 669

`forwardLidarDistanceCm` was a plain `uint16_t`, written only on a valid frame and never
invalidated. Two failure modes:

- A sensor that **died while an obstacle was inside 3.5 m** latched that distance for the
  rest of the flight, clamping forward pitch to −5° on every stick input. The aircraft
  could not fly forward and — with no working RTH either — could not be brought home.
- A sensor that **never enumerated** left the 9999 initialiser in place, so avoidance was
  silently inactive with no warning.

§6 also specified a "+5.0 m vertical step-over climb" and the §11 checklist asked the
tester to verify it, but the firmware only implemented the pitch clamp. No climb was ever
commanded.

**Now:** `TimedValue<T>` in `firmware/flight-controller/include/types.h` — no sensor value
is stored as a bare number; every sample carries its timestamp and consumers must call
`isFresh()`. Per-sensor maximum ages are in `config.h`. A stale LiDAR **disables
avoidance and says so** rather than reusing the last distance. The step-over climb is
implemented, with a budget that is consumed during the escape and rearms once clear.

Sensor health is aggregated into a bitmask carried in telemetry and checked at arm time.

**Documented in:** specification §6

---

## Finding 16 — GNSS baud rate contradiction

**Severity:** Medium
**Was:** §9 line 361 vs §10.1 line 892

The pinout annotated the BN-220 at 115200 baud; the firmware opened the port at 9600. A
builder who followed the documentation and reconfigured the module got garbage into the
NMEA parser, so `homeLocked` was never set and the aircraft refused to arm — with nothing
on the console to explain it. The VTX and LiDAR ports were consistently 115200 in both
places; only the GNSS row was wrong.

**Now:** The firmware negotiates the link rather than assuming: probe 115200, fall back to
9600, then push the module to 115200 at 10 Hz with UBX `CFG-PRT` and `CFG-RATE`, and
verify. A total failure is reported on the console and keeps arming blocked.

**Documented in:** specification §9.1

---

## Finding 17 — Remote ID claimed but never implemented, and impossible as drawn

**Severity:** High
**Was:** §1

"Native Direct Remote ID (DRI) OpenDroneID Broadcaster" appeared exactly once in the
1258-line document and nothing implemented it. At 1773 g the aircraft is over the 250 g
threshold in every jurisdiction that has one, so this is not a missing feature — flying
without it is unlawful in the US (14 CFR Part 89), the EU (2019/945) and the UK.

There was also a hardware reason it could never have worked: **the ESP32-P4 has no radio
at all.** No Wi-Fi, no Bluetooth. No firmware on the flight controller could have
broadcast anything.

**Now:** An **ESP32-C6 module** on the AUX broadcast bus. *Hardware addition, $6.50, 4 g.*
It broadcasts ASTM F3411 messages over Bluetooth 5 Long Range (Coded PHY) and a Wi-Fi
beacon vendor IE — Location/Vector at 1 Hz, static messages every 3 s. Message encoding
uses the reference library `opendroneid/opendroneid-core-c` rather than a hand-rolled bit
layout, because the ASTM field packing is bit-exact and easy to get subtly wrong.

The module drives a health line back to the flight controller, asserted only while both
radios are advertising **and** the operator ID has been configured. Remote ID health is a
blocking arm condition, so an aircraft with an unconfigured or dead module will not arm.

**Documented in:** specification §12.1

---

## Finding 18 — BOM arithmetic

**Severity:** Low
**Was:** §2

Line items summed to $512.50 against a stated total of $514.50.

**Now:** `hardware/bom.csv` is the authoritative source and the totals in the
specification are computed from it. Per-line `qty × unit = total` is checked. The new
total is **$540.50**, including the ExpressLRS receiver ($20.00), the Remote ID module
($6.50) and $1.50 of additional passives for the beacon diode-OR and power-sense divider.

---

## Issues found during the rework, and also fixed

Not part of the original eighteen, but corrected while implementing them.

| Issue | Resolution |
| --- | --- |
| Telemetry at 2 Hz / BW 125 kHz would occupy 22.6% duty cycle | Moved to 1 Hz / BW 250 kHz → 5.6% |
| Both cores drove the I2C `Wire` bus with no mutex — a bus-corruption race | All I2C serialised behind a mutex; the 500 Hz loop touches exactly one device |
| `mpu.begin()`, `baro.begin()`, `SD.begin()`, `LoRa.begin()` return values discarded — a failed barometer still passed preflight | Every result checked and recorded in the health mask |
| Notch filters were configured inside calibration; an early bail-out left them zero-initialised, silently zeroing the gyro | An unconfigured filter is a pass-through, and calibration failure blocks arming |
| PID derivative taken on error, spiking on every setpoint step | Derivative on measurement, low-pass filtered, conditional anti-windup |
| Single unfiltered ADC sample drove irreversible mode changes; a punch-out sag could commit the aircraft to RTH | Oversampled 8×, filtered, slew-limited, debounced over 3 s |
| `activateEmergencyBeacon()` blocked the 500 Hz flight loop for ~700 ms and drove LoRa SPI concurrently with the other core | Non-blocking latch state machine on the telemetry core; the flight loop never calls `delay()`, `Serial` or the radio |
| BlackBox appended every flight to one `/blackbox.bin` with no magic, version or record size | Versioned header, one file per flight, ring-buffered writes on an isolated task |
| BlackBox logged raw pre-clamp mixer sums, hiding saturation | Post-clamp values plus an explicit saturation percentage |
| SD writes happened on the same thread as LoRa reception; a 100 ms card stall delayed command handling | Storage isolated on its own low-priority task |
| Ground station spun forever in `while(1)` on radio failure with no indication | Emits MAVLink `STATUSTEXT` so the failure is visible in the GCS |
| Beacon battery divider was on an undeclared pin with an undocumented ratio | Declared pin, documented 100k/100k divider |
| 433 MHz second harmonic at 866 MHz would desensitise an 868 MHz ExpressLRS receiver | RC link specified as 2.4 GHz; the interaction is documented |
| No endurance or range figures existed to calibrate the energy budget against | Specification §3.3, with assumptions stated |
| 433 MHz at +20 dBm is not legal in the US without an amateur licence, and exceeds typical EU ERP limits — unmentioned in revision 1.0 | Specification §12.2, with the pin-compatible 868/915 MHz alternative |
| Composite sensor-health mask was not passed to the arm gate, which would have blocked arming permanently | Single composite mask computed once and used by every consumer |

---

*Prepared as part of specification revision 2.0.*
