# Autonomous Long-Range Quadcopter Engineering Master Specification

**Platform Identification:** Odyssey-10 Pro

**Architecture:** 9-inch long-range airframe, ESP32-P4 dual-core RISC-V avionics, integrated perception, kinetic recovery and safety stack

**Document revision:** 2.2 — re-based on the 387 mm 9-inch airframe

---

## About revision 2.2 — the airframe changed

The build moved to an **RJXHOBBY Mark4 V2, 387 mm wheelbase, 6 mm arms** — a **9-inch**
frame, not the 10-inch 420–450 mm frame revisions 1.0 to 2.1 assumed.

That is not a parts substitution; it re-bases the whole propulsion chain. Everything
below has been recomputed:

| | Was (10-inch) | Now (9-inch) |
| --- | --- | --- |
| Frame | 420–450 mm, 7–8 mm arms, 286 g | 387 mm, 6 mm arms, 230 g |
| Propellers | 10x5x3, 48 g | 9x5x3, 36 g |
| All-up weight | 1773 g | **1705 g** |
| Thrust per motor | 1750 g | **1300 g** |
| Thrust-to-weight | 3.95:1 | **3.05:1** |
| Hover throttle | ~44% | **~51%** |
| Gyro notch centre | 80 Hz | **95 Hz** |

The notch change is the one that matters for safety. A smaller disc needs roughly
(10/9)² more shaft speed for the same thrust, moving the hover fundamental from about
78 Hz to about 97 Hz — and at Q = 4 an 80 Hz notch is already −3 dB by 90 Hz, so the old
setting would have passed the new motor-noise peak straight into the rate controller.

The reference images are in `hardware/`.

> **The name still says "10".** `Odyssey-10 Pro` is kept as the platform name so the
> repository, document and firmware identifiers stay stable. It no longer describes the
> propeller size.

---

## About this revision

Revision 1.0 of this document was reviewed and eighteen defects were confirmed. Nine of
them were flight-critical: the battery failsafe thresholds were 3S values on a 6S
airframe and therefore unreachable, the motor rotation diagram contradicted the pinout
on all four motors, the return-to-home mode had no navigation in it, and there was no
manual control link anywhere in the design.

This revision fixes all eighteen. Each fix is marked in place with the finding number,
and section 13 is a complete index mapping every finding to the change that resolves it
and the file that implements it.

Two of the fixes required hardware additions, because the defects could not be fixed in
software alone:

- an **ExpressLRS 2.4 GHz receiver**, because a 433 MHz LoRa link is physically
  incapable of carrying stick-rate control (section 5.4)
- an **ESP32-C6 Remote ID module**, because the ESP32-P4 has no radio of any kind and
  Remote ID is a legal requirement for this aircraft (section 12)

The firmware is no longer inlined in this document. It lives in the repository as a
buildable project; section 10 maps every subsystem described here to the file that
implements it. Inlining 700 lines of C++ into a specification is how the original's code
and prose drifted apart in the first place.

---

## 1. System Architecture and Theory of Operation

The **Odyssey-10 Pro** is built around the dual-core **ESP32-P4** (RISC-V, up to
400 MHz). The platform decouples hard real-time flight stabilisation from asynchronous
communication, logging and perception.

> **The ESP32-P4 has no Wi-Fi and no Bluetooth.** This is not an oversight in the part
> selection; the P4 is a deliberately radio-less applications processor. Every radio
> function in this aircraft therefore lives on a separate module. Revision 1.0 claimed a
> "Native Direct Remote ID broadcaster" on the P4, which was not merely unimplemented —
> it was impossible.

```
+---------------------------------------------------------------------------------------+
|                          ODYSSEY-10 PRO SYSTEM TOPOLOGY                               |
+---------------------------------------------------------------------------------------+
|                                                                                       |
|  [ CORE 1: 500 Hz DETERMINISTIC FLIGHT LOOP ]                                         |
|  * Primary IMU (MPU-6050) on a fast I2C bus -- the ONLY device this core touches      |
|  * 2nd-order bi-quad gyro notch filtering (centre 80 Hz, Q 4.0)                       |
|  * Ballistic free-fall detection (|a| < 0.15 g for >400 ms, above 8 m AGL)            |
|  * Cascaded angle and rate PID controllers, derivative on measurement                 |
|  * Quad-X mixer WITH DESATURATION -> 400 Hz LEDC PWM to 4x 3110 motors                |
|                                                                                       |
|  [ CORE 0: TELEMETRY, PERCEPTION, ENERGY AND SAFETY -- 50 Hz ]                        |
|  * Secondary IMU (ICM-42688-P) cross-check and failure voting                         |
|  * Beitian BN-220 multi-GNSS ingest (Galileo/GPS/GLONASS, 10 Hz, UART1)               |
|  * Benewake TFmini-S forward LiDAR ingest (UART3), readings expire after 200 ms       |
|  * Bosch BMP280 barometer and ST VL53L1X precision-landing ToF (I2C)                  |
|  * QMC5883L magnetometer -- tilt-compensated heading, required by the RTH controller  |
|  * INA226 current shunt -- coulomb counting for the energy budget                     |
|  * Dynamic energy and distance budgeting, and the RTH position controller             |
|  * MAVLink telemetry over 433 MHz SX1278 LoRa (SPI2), 1 Hz                            |
|  * MSP telemetry stream to the 5.8 GHz VTX (UART2)                                    |
|  * Solid-state power latch driver for the isolated 1S beacon (GPIO 21)                |
|  * AUX broadcast bus (LP-UART) feeding position to the beacon and Remote ID modules   |
|                                                                                       |
|  [ CORE 0: STORAGE -- low priority ]                                                  |
|  * 100 Hz binary BlackBox recorder (MicroSD via SPI1), isolated so a card stall       |
|    cannot delay a control deadline                                                    |
|                                                                                       |
+---------------------------------------------------------------------------------------+
                                        |
        +-------------------------------+-------------------------------+
        |                               |                               |
        v                               v                               v
+------------------+          +-------------------+          +--------------------+
| ExpressLRS 2.4G  |          | ESP32-C6          |          | ESP32-C3           |
| CRSF receiver    |          | Remote ID module  |          | Recovery beacon    |
| MANUAL CONTROL   |          | ASTM F3411 BLE5   |          | 433 MHz + audible  |
| 50-500 Hz, UART4 |          | + Wi-Fi, LP-UART  |          | + strobe, LP-UART  |
+------------------+          +-------------------+          +--------------------+
```

### 1.1 Why the control path is split across two radios

Revision 1.0 described the 433 MHz LoRa link as a "bi-directional MAVLink/RC link" and
defined an `RCCommandPacket` for it. That could never have worked, for a reason that is
arithmetic rather than a matter of implementation quality.

At SF7 / BW 125 kHz / CR 4:5, an 18-byte command frame occupies **51.5 ms of airtime**.
A 20 Hz stick stream would require 103% of the channel — more time than exists. Even
5 Hz would sit at 26% duty cycle, against a 10% regulatory ceiling for 433 MHz in ITU
Region 1.

The two jobs have completely different requirements, so they now use different media:

| Path | Medium | Rate | Purpose |
| --- | --- | --- | --- |
| Manual control | ExpressLRS 2.4 GHz, CRSF over UART4 | 50–500 Hz | Stick input, arm switch, RTH switch, hardware failsafe flag |
| Supervisory | 433 MHz LoRa, SX1278 | 1 Hz down, on demand up | Telemetry, land permission, abort, operator RTH |

---

## 2. Bill of Materials and Cost Breakdown

The authoritative machine-readable version is `hardware/bom.csv`. Totals below are
computed from that file, not maintained by hand — revision 1.0's stated total of
$514.50 did not match its own line items, which summed to $512.50.

| Category | Component | Key specification | Qty | Unit | Total | Mass |
| --- | --- | --- | --- | --- | --- | --- |
| Airframe | RJXHOBBY Mark4 V2 387 mm | 387 mm wheelbase, 3K carbon, 6 mm arms | 1 | $55.00 | $55.00 | 230 g |
| Propulsion | 3110 / 2812 brushless motor | 900 KV, 4S–6S, M5 shaft | 4 | $22.00 | $88.00 | 272 g |
| Propellers | HQProp / Gemfan 9x5x3 | 9 in, 2 CW + 2 CCW, props-out | 2 pr | $6.00 | $12.00 | 36 g |
| Drive | 4-in-1 ESC (BLHeli_32) | 50 A cont., 60 A burst, 6S | 1 | $45.00 | $45.00 | 34 g |
| Main battery | 6S LiPo pack | 22.2 V nom, 4500 mAh, 45C | 1 | $110.00 | $110.00 | 680 g |
| Main MCU | ESP32-P4 dev board | Dual-core 400 MHz, **no radio** | 1 | $14.00 | $14.00 | 9 g |
| Primary IMU | InvenSense MPU-6050 | ±500 °/s, ±8 g, I2C | 1 | $3.50 | $3.50 | 2 g |
| Backup IMU | ICM-42688-P | Low-noise 6-axis, voting | 1 | $4.00 | $4.00 | 2 g |
| Barometer | Bosch BMP280 | 0.16 m resolution, I2C | 1 | $2.50 | $2.50 | 1 g |
| GNSS | Beitian BN-220 | Multi-GNSS, 10 Hz, UART | 1 | $14.00 | $14.00 | 14 g |
| Compass | QMC5883L | 3-axis magnetometer, I2C | 1 | $3.50 | $3.50 | 3 g |
| Forward LiDAR | Benewake TFmini-S | 0.1–12 m, 100 Hz, UART | 1 | $22.00 | $22.00 | 5 g |
| Downward ToF | ST VL53L1X | 0–4 m laser rangefinder, I2C | 1 | $6.50 | $6.50 | 2 g |
| Current shunt | INA226 | High-side, 1 mΩ, I2C | 1 | $3.50 | $3.50 | 2 g |
| Telemetry | Semtech SX1278 (Ra-02) | 433 MHz LoRa, +20 dBm, SPI | 2 | $6.00 | $12.00 | 12 g |
| **RC receiver** | **ExpressLRS 2.4 GHz RX** | **CRSF, 50–500 Hz, HW failsafe** | **1** | **$20.00** | **$20.00** | **3 g** |
| **Remote ID** | **ESP32-C6-MINI-1** | **BLE 5 Long Range + Wi-Fi** | **1** | **$6.50** | **$6.50** | **4 g** |
| Video Tx | 5.8 GHz VTX | 48 ch, 7–26 V, MSP OSD | 1 | $38.00 | $38.00 | 32 g |
| FPV camera | RunCam Phoenix 2 Micro | 1/1.8 in starlight, 1200 TVL | 1 | $29.00 | $29.00 | 13 g |
| Power regs | Dual step-down BEC | 6S in, 5 V 3 A + 12 V 3 A | 1 | $8.00 | $8.00 | 14 g |
| BlackBox | SPI MicroSD + 32 GB card | Class 10 | 1 | $3.00 | $3.00 | 4 g |
| Parachute | Ejection canister + servo | Spring tube, 9 g MG servo | 1 | $9.00 | $9.00 | 55 g |
| Beacon node | ESP32-C3 standalone | 100 dB piezo, strobe LED | 1 | $7.00 | $7.00 | 6 g |
| Beacon battery | 1S LiPo cell | 3.7 V 800 mAh with PCM | 1 | $6.00 | $6.00 | 16 g |
| Power latch | SI2301DS P-FET + 2N3904 | Latching solid-state switch | 1 | $2.50 | $2.50 | 2 g |
| Wiring/passives | XT90-S, caps, TVS, Schottky | 1000 µF 35 V, SMBJ28A, BAT54C | 1 lot | $16.00 | $16.00 | 88 g |
| **Master total** | | | | | **$540.50** | |

**Rows added in this revision** are shown in bold. The wiring lot increased by $1.50 to
cover the Schottky diode-OR and the main-power-sense divider that the beacon redesign in
section 7 requires.

Airborne dry mass is 1535 g (the second SX1278 module is the ground station radio and
does not fly). See section 3.1.

---

## 3. Power, Propulsion and Sizing

### 3.1 Mass budget

All-up weight is now built bottom-up from the BOM rather than asserted. Revision 1.0
quoted 1850 g against a grouped estimate that did not reconcile with its own parts list.

| Group | Mass |
| --- | --- |
| Frame, standoffs, gel dampers, hardware | 230 g |
| 4× 3110 motors (68 g each installed) | 272 g |
| 4× 9x5x3 propellers | 36 g |
| 4-in-1 ESC | 34 g |
| 6S 4500 mAh LiPo pack | 680 g |
| Avionics stack (P4, IMUs, baro, SD) | 18 g |
| Navigation sensors (GNSS + mast, compass, LiDAR, ToF, INA226) | 26 g |
| Radios (LoRa air unit, ExpressLRS RX, Remote ID module) | 13 g |
| Video subsystem (VTX, camera, antennas) | 45 g |
| Power regulation (BEC) | 14 g |
| Parachute subsystem | 55 g |
| Beacon subsystem (node, 1S cell, latch) | 24 g |
| Wiring, connectors, capacitors, hardware | 88 g |
| **Dry mass** | **1535 g** |
| Payload reserve | 170 g |
| **All-up weight (AUW)** | **1705 g** |

### 3.2 Thrust and throttle response

Maximum static thrust per motor at 6S with a 9x5x3 propeller is **1300 g**, giving 5200 g
total and a **thrust-to-weight ratio of 3.05:1**.

The 3110 900 KV motor was selected for a 10-inch propeller, so on 9 inches it is
under-propped: it runs cooler and keeps headroom, at some cost in efficiency. Thrust is
roughly 74% of the 10-inch figure.

Revision 1.0 stated that hover occurred at 22% throttle. That figure was not physical:
propeller thrust scales with roughly the square of shaft speed, so 22% stick cannot
produce a quarter of maximum thrust on any real ESC curve.

The table below follows a T ∝ throttle^1.67 characteristic, representative of a
BLHeli_32 ESC with throttle linearisation. **These are modelled figures. Verify them on a
thrust stand before your first flight** — the hover point drives the energy budget in
section 5.2, and the whole table moved when the airframe changed.

| Throttle | Thrust/motor | Total | Note |
| --- | --- | --- | --- |
| 0% | 0 g | 0 g | |
| 20% | 88 g | 354 g | |
| 30% | 174 g | 696 g | |
| 40% | 281 g | 1126 g | |
| **51%** | **426 g** | **1705 g** | **hover equilibrium = AUW** |
| 60% | 554 g | 2216 g | |
| 75% | 804 g | 3216 g | fast cruise |
| 100% | 1300 g | 5200 g | emergency punch-out |

Hover now sits at roughly 51% rather than 44%, because the thrust ceiling fell further
than the mass did. A 3:1 thrust-to-weight ratio is still ample for a long-range platform
— it is a cruiser, not a racer — but there is less margin for a hard recovery, and the
mixer saturation figure in the BlackBox is worth watching on the first flights.

**Propeller clearance.** At a 387 mm wheelbase, adjacent motor hubs sit about 274 mm
apart. A 9-inch propeller (229 mm) leaves roughly 22 mm of tip clearance per side; a
10-inch propeller (254 mm) would leave under 10 mm, which is why this frame is sold as a
9-inch frame and why 10-inch propellers are not an option on it.

### 3.3 Endurance and range

Assumptions: hover power loading 7.8 g/W; avionics, VTX and BEC losses 18 W; usable pack
fraction 80% for LiPo and 85% for Li-ion.

The power loading fell from the 8.5 g/W used for the 10-inch airframe. Total disc area
dropped from 0.203 m² to 0.164 m², and induced power scales with the square root of disc
loading, so the smaller propellers cost efficiency even though the aircraft is lighter.
Hover power is essentially unchanged at about 237 W.

| Configuration | Hover | Cruise @ 12 m/s | One-way range | Practical radius |
| --- | --- | --- | --- | --- |
| 6S 4500 mAh LiPo (1705 g) | 20 min | 18 min | 13.3 km | ~5.3 km |
| 6S2P 8400 mAh Li-ion (1815 g) | 37 min | 34 min | 24.1 km | ~9.6 km |

Practical radius is roughly 40% of one-way range, because the return leg must be flown on
the same pack and the energy budget in section 5.2 holds a reserve back for the descent.
`CRUISE_CURRENT_A` in `config.h` is set to 10.7 A, matching the 237 W hover figure at
22.2 V; correct it after your own bench measurement.

## 4. Frame Dynamics, Motor Layout and Rotation

### 4.1 Props-out configuration

The platform uses a **props-out (reversed rotation)** Quad-X configuration. This keeps
grass and prop-wash debris off the camera lens and improves yaw authority in crosswinds.

Props-out means the propeller blade crossing the **front** of each motor disc travels
**away from the airframe centreline**. For a motor on the right-hand side, that blade
moves to the right, which is clockwise rotation viewed from above.

### 4.2 Motor map — authoritative

> **FINDING 2.** Revision 1.0's section 4 diagram assigned the opposite rotation to all
> four motors compared with its own pinout, its own firmware comments, and its own
> commissioning checklist. It also contradicted its own "props-out" label: it drew the
> front-right motor turning counter-clockwise, which is props-*in*.
>
> This mattered because the yaw mixer column is only correct for one of the two
> assignments. A builder who set ESC rotation from the diagram would have inverted the
> yaw sign, so the yaw PID would drive the heading error further from zero — positive
> feedback, and an uncontrollable flat spin on the first armed take-off.
>
> The table below is now the single source of truth. It is duplicated verbatim in
> `firmware/flight-controller/include/config.h` and checked by the commissioning
> procedure in section 11.

```
                                 FRONT (heading 0 deg)

              [M4: front-left]                            [M2: front-right]
              COUNTER-CLOCKWISE                              CLOCKWISE
                     CCW                                        CW
                 \        /                                 \        /
                  \      /                                   \      /
                   \    /                                     \    /
                    [M4]                                       [M2]
                      \                                         /
                       \      +--------------------+           /
                        \-----|     ESP32-P4       |----------/
                              |   AVIONICS BAY     |
                        /-----|  MPU-6050 / BMP280 |----------\
                       /      +--------------------+           \
                      /                                         \
                    [M3]                                       [M1]
                   /    \                                     /    \
                  /      \                                   /      \
                 /        \                                 /        \
                     CW                                         CCW
                 CLOCKWISE                               COUNTER-CLOCKWISE
              [M3: rear-left]                              [M1: rear-right]

                                       REAR (tail)
```

| Motor | Position | Rotation | GPIO | Diagonal pair |
| --- | --- | --- | --- | --- |
| M1 | Rear-right | Counter-clockwise | 4 | with M4 |
| M2 | Front-right | Clockwise | 5 | with M3 |
| M3 | Rear-left | Clockwise | 6 | with M2 |
| M4 | Front-left | Counter-clockwise | 15 | with M1 |

Diagonally opposite motors always share a rotation direction. This is what makes yaw
control work: spinning up one diagonal pair and slowing the other produces a net torque
about the yaw axis with no net change in thrust.

### 4.3 ESC protocol

The ESC is a BLHeli_32 4-in-1 capable of DShot300. The firmware drives it with **analog
PWM at 400 Hz, 12-bit resolution** via the ESP32 LEDC peripheral:

- 1000 µs = 1638 counts (disarmed / minimum)
- 1068 µs = 1750 counts (armed idle, propellers turning)
- 2000 µs = 3276 counts (maximum)

Revision 1.0 was internally inconsistent here — the BOM and placement diagram said
"DShot ESC" while the architecture and firmware used 400 Hz PWM. Analog PWM is retained
because it is what the code implements and it is adequate at a 500 Hz loop rate;
**configure the ESC for PWM input and calibrate its endpoints** during commissioning.
Migrating to DShot300 would remove the calibration step and add per-motor telemetry, and
is a reasonable future change.

### 4.4 Mixer and desaturation

> **FINDING 13.** Revision 1.0 summed throttle and the three axis corrections into each
> motor and then clamped each motor independently. That is correct only while nothing
> clips.
>
> With base throttle near its ceiling of 2976 counts and a full roll correction of 350,
> motors 3 and 4 compute to 3326 and are clamped to 3276 — losing 50 counts — while
> motors 1 and 2 drop cleanly to 2626. The realised roll differential is 650 instead of
> the commanded 700. Stack roll, pitch and yaw (up to 950 counts) and the shortfall
> grows large enough to reverse the net moment on an axis: the aircraft rolls *away*
> from the correction.

The mixer now runs in two stages (`firmware/flight-controller/include/mixer.h`):

1. **Build the correction-only mix**, with no throttle. If its peak-to-peak range
   exceeds the available motor range, scale all corrections by one common factor.
   Scaling uniformly preserves the *direction* of the commanded moment, which is
   precisely the property independent clamping destroys.
2. **Fit the throttle offset** into whatever range remains. Throttle is the term the
   aircraft gives up; attitude authority is not.

When there is not enough range for all three axes, **yaw is sacrificed before roll and
pitch**. Losing yaw authority costs heading; losing roll or pitch authority costs the
aircraft.

The mixer reports how much authority it had to surrender, and that figure is recorded in
the BlackBox on every sample. `tools/blackbox_decode.py` summarises it, so a marginal
airframe shows up in the log rather than only in the crash.

**How much margin the shipped gains leave.** Sweeping the whole reachable space of the
PID output limits (roll and pitch ±350, yaw ±250) gives a worst-case correction span of
**1400 counts against a motor range of 1638** — 15% headroom. At the shipped gains the
correction-scaling stage of the mixer is therefore unreachable; only the throttle stage
ever gives anything up. That margin disappears if the rate gains are raised, which is
why the scaling stage exists and is tested. The sweep is
`testMixerYawSacrifice` in `tools/host_tests/test_all.cpp`.

The mix matrix, in motor order [M1, M2, M3, M4]:

| Axis | M1 rear-right | M2 front-right | M3 rear-left | M4 front-left |
| --- | --- | --- | --- | --- |
| Roll (+ = right) | −1 | −1 | +1 | +1 |
| Pitch (+ = nose up) | −1 | +1 | −1 | +1 |
| Yaw (+ = nose right) | +1 | −1 | −1 | +1 |

---

## 5. Energy Budgeting, Return-to-Home and Failsafe Logic

### 5.1 Battery thresholds

> **FINDING 1 — the single most serious defect in revision 1.0.**
>
> Every battery threshold in the document and the firmware was a 3S value:
> `BATT_WARN_VOLTAGE = 10.2f`, `BATT_CRITICAL_CUTOFF = 9.9f`, and a flowchart built on
> `V_critical = 9.9 V`. The airframe carries a **6S** pack, and the 100k/10k divider was
> sized for 6S, so those thresholds correspond to 1.70 V and 1.65 V per cell.
>
> A 6S pack never reaches them while the aircraft is flyable. Every guard was therefore
> permanently false: no return-to-home, no land-permission request, no critical cutoff.
> The aircraft would have flown until the pack collapsed and it fell out of the sky.
>
> Tellingly, the burn-rate constant (0.003 V/s) *was* a correct 6S figure — which is how
> we know the thresholds were a transcription error rather than a different design.

Thresholds are now derived per cell from `CELL_COUNT`, so the same firmware is correct
on a 4S or 6S airframe by changing one number:

| Parameter | Per cell | 6S pack | Meaning |
| --- | --- | --- | --- |
| Full | 4.20 V | 25.20 V | |
| Nominal | 3.70 V | 22.20 V | |
| Launch minimum | 3.85 V | 23.10 V | Arming is refused below this |
| Warning | 3.40 V | 20.40 V | Cannot reach home — request landing |
| Critical | 3.30 V | 19.80 V | Land immediately, no negotiation |
| Reserve | 0.10 V | 0.60 V | Held back for the descent |

`config.h` carries compile-time assertions on these relationships. A future edit that
reintroduces a 3S threshold on a 6S airframe fails the build:

```cpp
static_assert(PACK_CRITICAL_V > 15.0f,
              "Critical cutoff looks like a 3S threshold on a 6S pack -- check CELL_COUNT");
static_assert(PACK_CRITICAL_V < PACK_WARN_V, ...);
static_assert(PACK_FULL_V / VOLTAGE_DIVIDER_RATIO < 3.10f, ...);
```

### 5.2 Dynamic energy and distance budgeting

The energy engine answers one question at 50 Hz: *can the aircraft still reach home from
where it is now?* It answers in two independent currencies and takes the pessimistic
result.

**Currency 1 — pack voltage.** Where the aircraft must still be when it lands, plus the
sag it will accumulate on the way, plus a reserve:

```
d_home   = 2R * asin(sqrt(sin^2(dPhi/2) + cos(phi1) * cos(phi2) * sin^2(dLambda/2)))
t_return = (d_home / v_cruise) + t_descent          v_cruise = 12 m/s, t_descent = 15 s
V_req    = V_critical + (t_return * V_burn) + V_reserve
           = 19.80 V  + (t_return * 0.003) + 0.60 V
```

**Currency 2 — charge.** The INA226 integrates current into consumed mAh. This is
load-independent, so unlike voltage it does not false-trigger on a punch-out:

```
mAh_req  = I_cruise * 1000 * (t_return / 3600)      I_cruise = 10.5 A
mAh_left = (pack_capacity * 0.80) - mAh_consumed
```

The aircraft is considered able to reach home only if the filtered voltage exceeds
`V_req` **and** at least 120% of `mAh_req` remains.

**Debouncing.** Revision 1.0 compared a single, unfiltered ADC sample against the
threshold, and the resulting state changes were irreversible. A momentary sag under
throttle could therefore commit the aircraft to a return it did not need. The pack
voltage is now oversampled 8×, filtered with a ~1 s time constant and slew-limited, and
a threshold must hold **continuously for 3 seconds** before it latches a mode change.

### 5.3 Return-to-home navigation

> **FINDING 4.** Revision 1.0's `STATE_RTH_NAVIGATING` branch levelled the aircraft and
> held a fixed throttle. It computed no bearing, never yawed and never pitched forward.
> The distance to home therefore never decreased, so the documented exit condition
> (`distHome <= 5.0`) was unreachable. Worse, the battery block was gated on
> `currentState == STATE_ARMED`, so **once RTH was entered no voltage check ran at all**
> — the aircraft hovered in place until the pack was flat.

RTH is now a real three-phase position controller
(`firmware/flight-controller/src/navigation.cpp`):

```
   ENGAGE
     |
     v
+-------------+   at altitude, or 20 s elapsed   +---------------+
|   CLIMB     |--------------------------------->|   TRANSLATE   |
| to 30 m AGL |                                  | yaw to bearing|
+-------------+                                  | pitch forward |
                                                 +-------+-------+
                                                         | d_home <= 5 m
                                                         v
                                                 +---------------+
                                                 |    DESCEND    |
                                                 +-------+-------+
                                                         | AGL <= 2 m
                                                         v
                                                 +---------------+
                                                 |    ARRIVED    | -> failsafe landing
                                                 +---------------+
```

The cascade in the TRANSLATE phase is:

| Stage | Input | Output | Gain |
| --- | --- | --- | --- |
| Position | Distance to home (m) | Ground speed demand (m/s) | 0.50, capped at 12 m/s |
| Velocity | Speed error (m/s) | Pitch demand (deg) | 2.00, capped at 25° |
| Heading | Bearing error (deg) | Yaw rate demand (deg/s) | 2.50, capped at 150 °/s |
| Altitude | Altitude error (m) | Climb rate demand (m/s) | 1.20, −2 to +3 m/s |

Heading comes from the QMC5883L, tilt-compensated with the current roll and pitch —
without that compensation a 20° bank swings the reported heading by tens of degrees and
the controller chases its own attitude.

Three behaviours the original lacked:

- **Translation is gated on heading.** If the nose is more than 45° off the bearing,
  forward pitch is cut to 20%. Crabbing sideways at 12 m/s with a forward-only obstacle
  sensor is how you fly into things.
- **Obstacle avoidance outranks the navigator.** Flying home is not worth flying into a
  tree on the way.
- **The energy checks run in every powered state**, so RTH escalates to a landing when
  the pack falls below the warning and then the critical threshold. If the GNSS fix is
  lost, the navigator has 10 seconds to recover before the aircraft lands where it is,
  rather than hovering indefinitely.

### 5.4 Control and command links

> **FINDING 3.** Revision 1.0 had no manual control link. `RCCommandPacket` was only ever
> *received* — nothing anywhere constructed or transmitted one, and the ground station
> sketch in section 10.3 was receive-only. Because `setup()` seeded
> `lastRcPacketTime = millis()`, the first thing an armed aircraft did was fire
> `RC LINK LOSS TIMEOUT` 1.2 seconds later. The `PERMIT_LAND` branch that the entire
> section 5 failsafe flowchart hinges on was unreachable code: the pilot had no way to
> answer the aircraft's request.

The manual path is an **ExpressLRS 2.4 GHz receiver** on UART4, speaking CRSF at
420 kbaud. It carries stick data at 50–500 Hz, reports link quality and RSSI, and raises
an explicit hardware failsafe flag when the link is genuinely gone. The flight controller
treats a CRSF gap of more than 500 ms as loss of pilot.

2.4 GHz is chosen deliberately over 900 MHz ExpressLRS: the second harmonic of a 433 MHz
transmitter lands at 866 MHz, squarely on the 868 MHz ExpressLRS band. See section 8.4.

The supervisory path is the 433 MHz LoRa link, carrying:

| Direction | Frame | Rate | Airtime | Duty cycle |
| --- | --- | --- | --- | --- |
| Aircraft → ground | Telemetry, 48-byte payload | 1 Hz | 56.4 ms | 5.6% |
| Ground → aircraft | Command, 8-byte payload, sent 3× | On demand | 25.7 ms each | negligible |

Both at SF7 / BW 250 kHz / CR 4:5. The bandwidth was raised from 125 kHz and the
telemetry rate lowered from 2 Hz specifically to bring the duty cycle under the 10%
regulatory ceiling; at the original settings, telemetry alone would have occupied 22.6%.

Commands available to the operator, via three physical buttons on the ground station:

| Command | Effect |
| --- | --- |
| `PERMIT_LAND` | Grants the aircraft's landing request; it lands where it is |
| `DENY_LAND` | Extends the hold. The critical-battery check still overrides this |
| `RTH_NOW` | Operator-commanded return to home |
| `ABORT_TO_LAND` | Immediate failsafe landing |

A button is the correct interface for a safety decision: it cannot be triggered by a GCS
software glitch, and the operator's hand is on it.

### 5.5 Command integrity

Every frame on both LoRa directions carries a header, a length, and a CRC-16/CCITT
(`shared/odyssey_link.h`). Commands additionally carry:

- a **session id**, randomised by the ground station at boot. The aircraft binds to the
  first station it hears and rejects every other, so a second transmitter cannot take
  over mid-flight without a power cycle;
- a **monotonic sequence number**. A command whose sequence does not strictly advance is
  rejected, which kills replays of a captured `ABORT` frame.

This is integrity, not authentication. It prevents a corrupted or duplicated frame from
being interpreted as a valid command; it does not stop a determined attacker with a
software-defined radio. For flights where command spoofing is a genuine threat, add a
shared-secret CMAC over the payload.

### 5.6 Failsafe state machine

> **FINDING 6.** Revision 1.0 kept the flight state in a bare `volatile FlightState`
> written from both cores, and `triggerFailsafe()` performed an unsynchronised
> read-modify-write:
>
> ```cpp
> if (currentState != FAILSAFE && currentState != DISARMED && ...) {
>     currentState = STATE_FAILSAFE_LANDING;      // separate store
> }
> ```
>
> Core 1 could deploy the parachute in the window between the test and the store. The
> result: `STATE_FREEFALL_PARACHUTE` silently overwritten with
> `STATE_FAILSAFE_LANDING`, the flight loop takes the powered-descent branch, and all
> four motors spin back up underneath an already-deployed canopy.

Two changes make that race harmless rather than merely narrower:

1. Every transition is a single **atomic test-and-set** under a spinlock. There is no
   window between the guard and the store.
2. Transitions are **escalation-ordered**. States are ranked by severity and a request is
   honoured only if it moves to an equal or higher rank.

| Rank | State | Motors live? |
| --- | --- | --- |
| 0 | `BOOT` | no |
| 1 | `CALIBRATING` | no |
| 2 | `PREFLIGHT_FAIL` | no |
| 3 | `PREFLIGHT_OK` | no |
| 4 | `ARMED` | **yes** |
| 5 | `RTH_NAVIGATING` | **yes** |
| 6 | `AWAITING_LAND_PERMIT` | **yes** |
| 7 | `FAILSAFE_LANDING` | **yes** |
| 8 | `FREEFALL_PARACHUTE` | no |
| 9 | `DISARMED` | no |

`FAILSAFE_LANDING` (7) therefore cannot displace `FREEFALL_PARACHUTE` (8) no matter how
the two cores interleave. `DISARMED` sits at the top because cutting the motors is always
permitted. Going back down the ladder to re-arm requires an explicit reset that is legal
only out of `DISARMED`.

The "motors live" column is exported as `odyMotorsAreLive()` in the shared header, and
both the aircraft and the ground station use it. Nothing open-codes an ordinal
comparison against this enum — see finding 10.

### 5.7 Landing and touchdown detection

> **FINDING 14.** Revision 1.0's touchdown test was one line:
>
> ```cpp
> if (millis() - failsafeStartTime > 14000 || currentAlt <= 0.05f)
> ```
>
> where `currentAlt` was a barometric reading relative to a ground reference captured
> once at power-on. Ambient pressure drift over a 20-minute flight is easily a metre, so
> that comparison can pass in mid-air — and the branch immediately cut all four motors.
>
> Section 6 of the same document assigned exactly this job to the VL53L1X below 2 m AGL
> with a −0.2 m/s regulated flare. The VL53L1X, along with the INA226 and the QMC5883L,
> appeared only in the BOM and the prose. None of the three was referenced anywhere in
> the firmware.

All three sensors are now implemented (`firmware/flight-controller/src/sensors.cpp`), and
touchdown requires independent evidence to agree:

```
TOUCHDOWN CONFIRMED requires ALL of:

  (a) ToF says we are on the deck            AGL_tof   <= 0.20 m
      or, if the ToF is unhealthy,
      the barometer says so                  AGL_baro  <= 0.60 m

  (b) vertical motion has stopped            |vario|   <  0.25 m/s

  (c) ...and stayed stopped                  for >= 400 ms

  (d) NO healthy sensor reports              AGL_any   >  1.50 m     <-- veto
      that we are still up
```

The barometric ground reference is re-latched at each arm, so AGL is relative to *this*
flight's launch point rather than to wherever the aircraft was powered on.

Below 2 m AGL the descent rate is regulated to **−0.2 m/s** by the VL53L1X, which is the
precision flare the original specified but never implemented. A hard 45 s ceiling on the
descent remains as a backstop, and is logged as the anomaly it is if it ever fires.

### 5.8 Arming

> **FINDING 5.** Revision 1.0 required only `PREFLIGHT_OK`, a home lock and the button.
> The global `currentRC` retained the last received throttle across flights, so arming
> with the stick up drove all four propellers to about 82% instantly.

Arming now requires **all** of the following. Any that fail are reported to the pilot as
a bitmask in telemetry, so a blocked arm is never a silent mystery in the field:

| Check | Blocker flag |
| --- | --- |
| Every required sensor healthy (IMU, baro, mag, GNSS, current, RC) | `SENSORS` |
| Remote ID healthy — **only if `REQUIRE_REMOTE_ID_TO_ARM` is set**; optional by default, see 12.1 | `SENSORS` |
| GNSS home position locked, ≥ 6 satellites | `NO_HOME` |
| **Throttle stick at minimum (≤ 1050 µs) from a live CRSF frame** | `THROTTLE` |
| Fresh CRSF frame within 500 ms | `RC_STALE` |
| Pack above the launch minimum (23.10 V) | `BATTERY` |
| Calibration passed | `CALIB` |
| Aircraft level within 8° | `ATTITUDE` |
| Physical button **and** handset arm switch, held for 1 s | — |

### 5.9 Free-fall detection and parachute

> **FINDING 12.** Revision 1.0 gated free-fall detection to `STATE_ARMED` and
> `STATE_RTH_NAVIGATING` only. That excluded `AWAITING_LAND_PERMIT` (hovering, up to
> 15 s) and `FAILSAFE_LANDING` (powered descent, up to 45 s) — precisely the states in
> which a shed propeller produces a genuine free fall, with the parachute inhibited.

Detection now runs in **every state in which the motors are live**. Two guards are added:

- **Minimum deployment altitude, 8 m AGL.** A canopy below its inflation height will not
  slow the aircraft and will foul the propellers. This is the canister manufacturer's
  figure for this AUW.
- **The timer resets on every state entry**, so a partially accumulated free-fall window
  cannot leak across a mode change.

Trigger condition: total acceleration magnitude below **1.50 m/s² (≈ 0.15 g)** held
continuously for **400 ms**. On trigger the servo moves to the eject position, the motors
go to minimum, and the state machine escalates to `FREEFALL_PARACHUTE` — which, per
section 5.6, nothing can subsequently override except a disarm.

---

## 6. Perception, Collision Avoidance and Precision Flare

> **FINDING 15.** Revision 1.0 stored the forward LiDAR distance in a plain `uint16_t`
> that was only ever written on a successful frame and never invalidated. Two failure
> modes followed:
>
> - A sensor that **died while an obstacle was inside 3.5 m** latched that distance for
>   the rest of the flight, clamping forward pitch to −5° on every stick input. The
>   aircraft could not fly forward and — with no RTH navigation either — could not be
>   brought home.
> - A sensor that **never enumerated** left the 9999 initialiser in place, so avoidance
>   was silently inactive with no warning to the pilot.
>
> The specification also called for a "+5.0 m vertical step-over climb", and the
> commissioning checklist asked the tester to verify it, but the firmware only ever
> implemented the pitch clamp. There was no climb command anywhere.

### 6.1 Every reading knows how old it is

No sensor value is stored as a bare number. `TimedValue<T>` carries the millisecond
timestamp at which the sample was produced, and consumers must call `isFresh()` before
using it:

| Sensor | Maximum age | Behaviour when stale |
| --- | --- | --- |
| Primary IMU | 50 ms | Hold last attitude, count the miss; sustained loss → failsafe landing |
| Forward LiDAR | 200 ms | **Avoidance disabled** and reported; never reuse the last distance |
| Downward ToF | 200 ms | Landing falls back to the barometer with a wider threshold |
| Magnetometer | 500 ms | RTH heading control degrades to hold-attitude |
| GNSS | 2000 ms | RTH escalates to land-in-place after 10 s |
| INA226 current | 500 ms | Energy budget falls back to voltage only |

Health is aggregated into a bitmask carried in telemetry and checked at arm time.

### 6.2 Forward collision bubble

TFmini-S on UART3, parsed by a byte-at-a-time state machine with checksum validation and
plausibility bounds (0.1–12 m, return strength ≥ 100):

```
  d > 6.0 m           full authority
  3.5 m < d <= 6.0 m  proportional velocity scaling,  scale = (d - 3.5) / 2.5
  d <= 3.5 m          hard brake: forward pitch clamped to -5 deg (active reverse)
                      AND a +5.0 m step-over climb commanded at 1.5 m/s
  sensor stale        avoidance INACTIVE, flagged in telemetry and the BlackBox
```

The step-over budget is consumed while the escape climb is commanded and rearms once the
path is clear, so a long wall produces one climb rather than a continuous ascent.

### 6.3 Downward precision flare

VL53L1X on I2C in short-distance mode, polled at 50 Hz. Active below 2.0 m AGL, where it
bypasses the barometric ground-effect wash. It regulates the final descent to −0.2 m/s
and provides the touchdown veto described in section 5.7.

---

## 7. Isolated Emergency Locator Beacon

### 7.1 What was wrong

> **FINDING 7.** The beacon transmitted latitude 0 / longitude 0 for its entire
> endurance. It had no GPS, and the only interface from the flight controller was a
> single GPIO latch pulse — which carries no information. `BeaconPacket` was a
> zero-initialised global whose only written field was `battMillivolts`. A searcher
> decoding the packet received a well-formed fix pointing at Null Island.
>
> **FINDING 8.** The 48-hour endurance claim was out by roughly 5×. At SF11 / BW 62.5 kHz
> / CR 4:8 a single transmission occupies **2.76 s of airtime**. The original sent one
> every 2 s with no sleep, so the radio ran at about 50% duty — five times the 10% ISM
> ceiling — and because `endPacket()` blocks for the whole transmission, the actual
> cadence was closer to 4 s than the specified 2 s. Average draw was roughly 85 mA
> against an 800 mAh cell: about nine hours.
>
> **FINDING 11.** `activateEmergencyBeacon()` was reachable from exactly one place: the
> failsafe-landing touchdown branch. A parachute descent — by definition an uncontrolled
> landing wherever the wind takes the canopy, and the single scenario the beacon exists
> for — never reached it.

### 7.2 Hardware: keep the beacon alive during flight

The fix for finding 7 is a hardware change plus a protocol. The beacon node is powered
during flight through a **Schottky diode-OR** from the aircraft's 5 V rail, with the 1S
cell as the other input:

```
                          MAIN FLIGHT CONTROLLER (ESP32-P4)
                          - GPIO 21 : latch pulse output
                          - LP-UART : AUX broadcast bus, TX only
                                    |
        5V rail                     | position frames @ 2 Hz
           |                        |
        [BAT54C]                    |
      Schottky diode-OR             |
           |     +------------------+---------------------+
           |     |                                        |
+----------v-----v-----------+              +-------------v--------------+
|  BEACON NODE (ESP32-C3)    |              |  REMOTE ID (ESP32-C6)      |
|  * awake for the whole     |              |  * ASTM F3411 broadcast    |
|    flight on aircraft power|              +----------------------------+
|  * caches the last fix in  |
|    RTC-backed memory       |<---- 1S 800 mAh cell via SI2301DS P-FET latch
|  * switches seamlessly to  |      - latches ON when GPIO 21 pulses
|    the 1S cell when main   |      - ALSO latches passively if main VBAT is cut
|    power disappears        |
+----------------------------+
```

Because the node is running throughout the flight, when the latch fires it already holds
a fix that is at most a second old. The RTC-backed cache survives both the power handover
and the light-sleep cycles.

The transmitted packet now carries a **fix age** field. A searcher can tell the
difference between "the aircraft was here one second before it went down" and "this
position is four minutes stale", which the original had no way to express because it had
no position at all.

### 7.3 Duty-cycle compliant transmission schedule

| Phase | Window | LoRa interval | Airtime | Duty | Average draw |
| --- | --- | --- | --- | --- | --- |
| 1 | First 2 hours | 45 s | 2.76 s | 6.1% | ~7.4 mA |
| 2 | After 2 hours | 180 s | 2.76 s | 1.5% | ~1.8 mA |

Plus an audible and visual chirp every 5 s throughout (~1.0 mA average) and ~0.8 mA in
light sleep between events.

Phase 1 costs about 21 mAh. With 680 mAh usable from an 800 mAh 1S cell down to 3.3 V,
phase 2 then runs for well over 100 hours at 20 °C. **The specification claims 72 hours**,
which leaves margin for a cold cell, an aged pack, and a buzzer drawing more than the
modelled figure.

The firmware also **polices its own measured duty cycle at runtime** and refuses a
transmission that would breach 10%, so a future schedule change cannot silently put the
aircraft outside the regulations.

Radio configuration: SF11, BW 62.5 kHz, CR 4:8, +20 dBm — the same extreme-range settings
as before. The change is entirely in the schedule, not the link budget.

### 7.4 Both terminal paths arm the beacon

Fixing finding 11:

| Path | Beacon armed? | When |
| --- | --- | --- |
| Confirmed touchdown after a failsafe landing, > 15 m from home | yes | Immediately on disarm |
| Parachute deployment | **yes** | 20 s after deployment, once the canopy descent has settled, so the first transmitted fix is the landing site rather than a point in mid-air |
| Normal landing within 15 m of home | no | Not needed — the aircraft is at your feet |

### 7.5 The latch pulse no longer blocks the flight loop

Revision 1.0 called `activateEmergencyBeacon()` from the 500 Hz flight task. It contained
a `delay(100)` and four `delay(150)` calls — roughly 700 ms of blocking inside a hard
real-time loop — and it drove the LoRa SPI bus concurrently with the telemetry task on
the other core, with no mutex.

The latch is now a non-blocking three-stage state machine on the telemetry core. The
flight loop raises a flag and returns; it never calls `delay()`, `Serial` or the radio.

---

## 8. Physical Placement, 3D Layout and RF Plan

```
                                   [+120 mm GNSS mast] (GNSS + compass)
                                            /|\
                                           / | \
                                          /  |  \
          [M4: front-left CCW]           /   |   \          [M2: front-right CW]
             (10 in prop)               /    |    \             (10 in prop)
                \===\                  /  [top battery]           /===/
                 \   \                /  [6S 4500 mAh]           /   /
                  (O)----------------+---------------------------+---(O)
                   |                 |      CENTRAL STACK        |    |
   [LiDAR + cam]---+                 | 1. ESP32-P4 controller    |    +---[5.8 GHz VTX]
   (0 deg nose)    |                 | 2. IMU stack (gel-damped) |    |   (RHCP pagoda)
                   |                 | 3. ESC + TVS diode        |    |
                   |                 | 4. Remote ID module       |    |
                   |                 |    (antenna forward,      |    |
                   |                 |     see 8.5 -- keep it    |    |
                   |                 |     away from the ELRS RX)|    |
                  (O)----------------+---------------------------+---(O)
                  /   /               \  [1S beacon + latch]     /   /
                 /   /                 \ [downward ToF belly]   /   /
                /===/                   \                      /===/
          [M3: rear-left CW]             \ [433 MHz LoRa dipole]
             (10 in prop)                 (vertical, downward whip)
                                   [ExpressLRS 2.4 GHz RX, tail-mounted,
                                    dipoles at 90 deg to each other]
```

### 8.1 Component placement directory

| Location | Coordinates | Contents |
| --- | --- | --- |
| Centre of gravity | (0, 0, 0) | ESP32-P4 on M3 rubber standoffs; MPU-6050, ICM-42688-P and BMP280 directly on the CoG, damped with polyurethane gel |
| Nose | (0, +145, +34) mm | TFmini-S forward LiDAR and 1200 TVL FPV camera |
| Elevated tail mast | (0, −105, +120) mm | BN-220 GNSS and QMC5883L compass, 120 mm above the power train to keep motor currents out of the magnetometer |
| Tail RF | (±20, −130) mm | 5.8 GHz pagoda pointing up (+Z); 433 MHz LoRa whip dropping down (−Z, 165 mm) |
| Tail, offset | (±35, −150, +20) mm | ExpressLRS receiver, dipoles orthogonal |
| Undercarriage belly | (0, −35, −4) mm | VL53L1X downward ToF through a carbon plate cut-out |
| Underside, aft | (0, −80, −10) mm | Beacon node, 1S cell and P-FET latch |

### 8.2 Magnetometer placement

The compass is now load-bearing: the RTH controller cannot steer without it. The 120 mm
mast is not decorative. Verify the hard-iron calibration described in section 11 with the
motors spinning at hover throttle, not just on a bench.

### 8.3 Vibration isolation

The IMU stack sits on polyurethane gel. The notch centre was **raised from 80 Hz to
95 Hz** when the airframe moved to the 387 mm 9-inch frame.

At 900 KV on 6S, hover shaft speed on a 10-inch propeller is roughly 4700 rpm, giving a
fundamental near 78 Hz. A 9-inch propeller needs roughly (10/9)² more shaft speed for the
same thrust — about 5800 rpm, or a fundamental near 97 Hz, with the blade-pass peak near
194 Hz.

That matters because at Q = 4 an 80 Hz notch is already −3 dB by 90 Hz. The old setting
would have left the new motor-noise peak essentially unattenuated, feeding it into the
rate controller.

**Both figures are modelled.** Confirm the actual peak from a BlackBox gyro trace and
retune `NOTCH_CENTER_HZ` for your build. The 6 mm arms on this frame are also less stiff
than the 7–8 mm arms previously specified, which moves the structural resonance as well,
so this is not a setting to take on trust.

### 8.4 Radio frequency plan

This aircraft carries four transmitters. Two interactions matter.

**The 433 MHz second harmonic.** A 433 MHz transmitter's second harmonic falls at
866 MHz, which sits inside the 868 MHz ExpressLRS band. A +20 dBm LoRa transmission
40 cm from an 868 MHz receiver will desensitise it even with a well-filtered module.
**This is why the RC link is specified as 2.4 GHz rather than 900 MHz.** If you must use
900 MHz ExpressLRS, move the telemetry link to 868 MHz as well and coordinate the two, or
fit a harmonic filter on the LoRa output.

**Duty cycle.** Both LoRa users are now budgeted rather than assumed:

| Transmitter | Band | Power | Duty cycle |
| --- | --- | --- | --- |
| Telemetry (SX1278) | 433 MHz | +20 dBm | 5.6% |
| Recovery beacon (SX1278) | 433 MHz | +20 dBm | 6.1% falling to 1.5% |
| ExpressLRS RX | 2.4 GHz | +20 dBm typical | FHSS |
| Video (VTX) | 5.8 GHz | 800 mW–1.6 W | continuous |
| Remote ID (ESP32-C6) | 2.4 GHz | +9 dBm | BLE + beacon |

**Legality of 433 MHz at +20 dBm is your responsibility and it is not universal.**
See section 12.5.

### 8.5 The 2.4 GHz problem this design created for itself

Revision 2.0 moved the RC link to 2.4 GHz ExpressLRS specifically to escape the 433 MHz
second harmonic at 866 MHz. That was the right call for *that* problem, but it created a
second one that revision 2.0 did not address: **the Remote ID module also transmits on
2.4 GHz**, and revision 2.0's placement diagram put it in the central stack, close to
everything.

Three 2.4 GHz emitters now share the airframe:

| Emitter | Duty | Consequence of interference |
| --- | --- | --- |
| ExpressLRS receiver (RX, and TX for handset telemetry) | continuous, FHSS | **Loss of manual control** |
| Remote ID BLE 5 advertising | ~1–4 packets/s | Missed Remote ID broadcast |
| Remote ID Wi-Fi beacon | 1 Hz | Missed Remote ID broadcast |

These are not equally important. **ExpressLRS packet reception is mission-critical and
Remote ID is not.** A Remote ID advertisement that collides with an ELRS packet costs a
Remote ID frame; the reverse costs the aircraft. The mitigation must therefore be
asymmetric — protect the receiver, and let Remote ID take the loss.

**Placement rules, in priority order:**

1. **Separation.** The Remote ID antenna and the ExpressLRS antennas sit at opposite
   ends of the airframe — Remote ID forward on the central stack, ExpressLRS at the tail.
   Aim for ≥ 150 mm; a +9 dBm transmitter 30 mm from a receiver front-end will desensitise
   it regardless of channel.
2. **Orthogonal polarisation.** The ExpressLRS dipoles are already mounted at 90° to each
   other for diversity; mount the Remote ID antenna orthogonal to both.
3. **Never bundle the Remote ID coax with the ExpressLRS coax**, and do not route either
   along the ESC power leads.
4. **Ground plane.** Keep the carbon top plate between the Remote ID module and the
   ExpressLRS receiver where the layout allows it — carbon fibre is conductive and gives
   useful isolation almost for free.

**Verification, not assumption.** Section 11.4 adds a bench check: with the aircraft
powered and Remote ID broadcasting, confirm the ExpressLRS link quality reported in
telemetry does not drop when Remote ID transmits. If it does, the antennas are too close.
This is measurable in the BlackBox — `rcLinkQuality` is logged — so it does not require
test equipment.

**If the coexistence cannot be made to work** on your airframe, the escape route is to
move the *telemetry* link rather than the RC link: fit an Ra-02H (868/915 MHz) as
section 12.5 describes, which frees 2.4 GHz for RC and Remote ID alone and removes the
433 MHz harmonic issue at the same time. That is the cleanest RF plan overall and is the
recommended configuration for a new build.

---

## 9. Pinout and Hardware Interconnect Mapping

### 9.1 UART allocation

The ESP32-P4 provides five full UARTs plus a low-power UART. All six are used:

| Port | Peripheral | Baud | Notes |
| --- | --- | --- | --- |
| UART0 | USB / debug console | 115200 | |
| UART1 | Beitian BN-220 GNSS | **115200** | Auto-negotiated — see below |
| UART2 | VTX MSP OSD | 115200 | |
| UART3 | TFmini-S forward LiDAR | 115200 | |
| UART4 | ExpressLRS receiver (CRSF) | 420000 | **New** — the manual control link |
| LP-UART | AUX broadcast bus | 115200 | **New** — TX only, one wire to two modules |

> **FINDING 16.** Revision 1.0's section 9 documented the BN-220 at 115200 baud while the
> firmware opened the port at 9600. A builder who followed the documentation and
> reconfigured the module got garbage into the NMEA parser, so `homeLocked` was never
> set and the aircraft refused to arm — with nothing on the console to explain it. The
> VTX and LiDAR ports were consistently 115200 in both places; only the GNSS row was
> wrong.

The firmware now **negotiates the link** rather than assuming a rate: probe 115200,
fall back to 9600, then push the module to 115200 at 10 Hz with UBX `CFG-PRT` and
`CFG-RATE` and verify the change took. If neither rate answers, the failure is reported
on the console and arming stays blocked with the `SENSORS` flag set.

### 9.2 GPIO map

```
ESP32-P4 MASTER AVIONICS PINOUT
-------------------------------------------------------------------------------
GPIO 1   --> Battery voltage divider (100k/10k, 12-bit ADC, 8x oversampled)
GPIO 2   --> Physical pre-arm safety push-button (active LOW to GND)
GPIO 3   --> LoRa SX1278 DIO0 (interrupt line)
GPIO 4   --> Motor 1 ESC PWM  (rear-right,  CCW)
GPIO 5   --> Motor 2 ESC PWM  (front-right, CW)
GPIO 6   --> Motor 3 ESC PWM  (rear-left,   CW)
GPIO 7   --> I2C SDA (MPU-6050, ICM-42688-P, BMP280, QMC5883L, VL53L1X, INA226)
GPIO 8   --> I2C SCL (as above), 400 kHz
GPIO 9   --> LoRa SX1278 RST
GPIO 10  --> LoRa SX1278 NSS  (SPI2 chip select)
GPIO 11  --> LoRa SX1278 MOSI (SPI2)
GPIO 12  --> LoRa SX1278 SCK  (SPI2)
GPIO 13  --> LoRa SX1278 MISO (SPI2)
GPIO 15  --> Motor 4 ESC PWM  (front-left,  CCW)
GPIO 17  --> GNSS BN-220 RX1  (ESP32 RX <- GNSS TX @ 115200)
GPIO 18  --> GNSS BN-220 TX1  (ESP32 TX -> GNSS RX @ 115200)
GPIO 19  --> VTX MSP OSD TX2  (ESP32 TX -> VTX RX @ 115200)
GPIO 20  --> VTX MSP OSD RX2  (ESP32 RX <- VTX TX @ 115200)
GPIO 21  --> Emergency beacon solid-state latch trigger (active HIGH, 100 ms pulse)
GPIO 22  --> TFmini-S LiDAR RX3 (ESP32 RX <- LiDAR TX @ 115200)
GPIO 23  --> TFmini-S LiDAR TX3 (ESP32 TX -> LiDAR RX @ 115200)
GPIO 24  --> AUX broadcast bus TX  [NEW] -> beacon node RX and Remote ID module RX
GPIO 25  --> Remote ID health line [NEW] (INPUT_PULLDOWN; module drives HIGH while
             broadcasting -- a dead or unconfigured module blocks arming)
GPIO 26  --> Ballistic parachute servo (LEDC 50 Hz, 16-bit)
GPIO 27  --> ExpressLRS CRSF RX     [NEW] (ESP32 RX <- receiver TX @ 420000)
GPIO 28  --> ExpressLRS CRSF TX     [NEW] (ESP32 TX -> receiver RX, handset telemetry)
GPIO 33  --> MicroSD BlackBox CS   (SPI1)
GPIO 34  --> MicroSD BlackBox MOSI (SPI1)
GPIO 35  --> MicroSD BlackBox SCK  (SPI1)
GPIO 36  --> MicroSD BlackBox MISO (SPI1)
```

### 9.3 I2C device addresses

| Device | Address | Polled by | Rate |
| --- | --- | --- | --- |
| MPU-6050 (primary IMU) | 0x68 | Core 1 flight loop | 500 Hz |
| ICM-42688-P (backup IMU) | 0x69 | Core 0 telemetry | 50 Hz |
| QMC5883L (compass) | 0x0D | Core 0 telemetry | 50 Hz |
| VL53L1X (downward ToF) | 0x29 | Core 0 telemetry | 50 Hz |
| INA226 (current shunt) | 0x40 | Core 0 telemetry | 50 Hz |
| BMP280 (barometer) | 0x76 | Core 0 telemetry | 50 Hz |

All I2C traffic is serialised behind a mutex. The 500 Hz flight loop touches exactly one
device; every other transaction happens on core 0. Revision 1.0 had both cores driving
`Wire` with no mutex, which is a bus-corruption race.

### 9.4 AUX broadcast bus wiring

One wire from GPIO 24 to the RX pins of both the beacon node and the Remote ID module, in
parallel. Frames are addressed, CRC-protected and transmit-only — the flight controller
never listens on this bus, so a faulty auxiliary module cannot feed data back into the
avionics or jam the line.

---

## 10. Software Architecture

The firmware is a buildable PlatformIO project, not a listing in this document. Four
images share one protocol header.

### 10.1 Repository layout

```
Odyssey-10-Pro-Drone-System/
+-- shared/
|   +-- odyssey_link.h              Protocol, framing, CRC, flight-state enum
+-- firmware/
|   +-- flight-controller/          ESP32-P4
|   |   +-- include/
|   |   |   +-- config.h            Every tunable + compile-time assertions
|   |   |   +-- types.h             TimedValue<T>, snapshot, BlackBox record
|   |   |   +-- filters.h           Bi-quad notch, low-pass, debounce
|   |   |   +-- pid.h               PID with derivative-on-measurement
|   |   |   +-- mixer.h             Desaturating Quad-X mixer      [finding 13]
|   |   |   +-- state_machine.h     Atomic escalate-only states    [finding 6]
|   |   |   +-- sensors.h           Sensor hub with staleness      [findings 14, 15, 16]
|   |   |   +-- navigation.h        RTH + energy budget            [findings 1, 4]
|   |   |   +-- radio_link.h        CRSF + LoRa + AUX bus          [findings 3, 7]
|   |   |   +-- blackbox.h          Ring-buffered SD recorder
|   |   +-- src/
|   |       +-- main.cpp            Tasks, control, arming         [findings 5, 11, 12]
|   |       +-- sensors.cpp
|   |       +-- navigation.cpp
|   |       +-- radio_link.cpp
|   |       +-- blackbox.cpp
|   +-- beacon-node/                ESP32-C3                       [findings 7, 8]
|   +-- remote-id/                  ESP32-C6                       [finding 17]
|   +-- ground-station/             ESP32 + SX1278                 [findings 3, 9, 10]
+-- hardware/bom.csv
+-- tools/
|   +-- blackbox_decode.py
|   +-- md2docx.py
|   +-- host_tests/               Compiles the real headers on a PC   [98 assertions]
|       +-- test_all.cpp
|       +-- arduino_shim.h
|       +-- run_tests.sh
+-- docs/
```

### 10.2 Task structure

| Core | Task | Rate | Priority | Responsibility |
| --- | --- | --- | --- | --- |
| 1 | `TaskFlightLoop` | 500 Hz | MAX−1 | IMU, attitude, free-fall, rate control, mixing, ESC output |
| 0 | `TaskTelemetry` | 50 Hz | 3 | Sensors, GNSS, arming, energy, state, radios, console |
| 0 | `TaskStorage` | 20 Hz | 1 | Drains the BlackBox ring to the card |

Storage is isolated on its own task specifically so an SD write stall — which can exceed
100 ms on a cheap card — cannot delay a control deadline or a radio poll. Revision 1.0
wrote to the card directly from `loop()`, on the same thread as LoRa reception.

Cross-core data moves through exactly two mechanisms: a spinlock-protected snapshot
struct, and the atomic state machine. No global is read without one of them.

### 10.3 MAVLink bridge

> **FINDING 9.** Revision 1.0's bridge wrote constant bytes where the checksum belongs —
> `0x55, 0xAA` for heartbeat and `0x12, 0x34` for attitude. MAVLink requires a
> CRC-16/MCRF4XX over the header and payload, seeded with a per-message `CRC_EXTRA`.
> Every frame failed the receiver's CRC and was discarded, so QGroundControl and Mission
> Planner showed nothing at all — despite the section heading promising compatibility
> with both. The start byte was also `0xFE` (MAVLink v1) although the heading claimed v2,
> and the sequence byte was a constant, so even a CRC-correct link would have reported
> 100% packet loss.
>
> **FINDING 10.** The armed flag was `(flightState == 3 || flightState == 4)`, which
> omitted `AWAITING_LAND_PERMIT` and `FAILSAFE_LANDING` — states in which the aircraft is
> airborne with all four propellers turning. The operator saw "disarmed" at exactly
> the moment section 5 expects them to approve a landing.

Both are fixed in `firmware/ground-station/src/mavlink_min.h`. The bridge now emits
proper MAVLink v2 frames with real CRCs and a per-link sequence counter, and publishes
six message types rather than two:

| Message | ID | CRC_EXTRA | Carries |
| --- | --- | --- | --- |
| `HEARTBEAT` | 0 | 50 | Armed state, system status |
| `SYS_STATUS` | 1 | 124 | Sensor health mask, battery, comm drop rate |
| `GPS_RAW_INT` | 24 | 24 | Fix type, satellites |
| `ATTITUDE` | 30 | 39 | Roll, pitch, heading |
| `GLOBAL_POSITION_INT` | 33 | 104 | Position, altitude, velocity |
| `VFR_HUD` | 74 | 20 | Speed, altitude, climb |
| `STATUSTEXT` | 253 | 83 | State changes and arm blockers as GCS alerts |

The armed predicate is now `odyMotorsAreLive()` from the shared header, so the aircraft
and the ground station cannot drift apart and no caller open-codes an ordinal comparison.

### 10.4 BlackBox format

48-byte packed records at 100 Hz, preceded by a versioned header carrying the magic,
format version, record size, log rate and airframe name. One file per flight
(`/flight_NNNN.ody`).

Motor fields hold the **post-clamp values actually written to the ESCs**, and each record
carries the mixer's saturation percentage — so a loss of control authority is visible in
the log, which is exactly when you most want to see it.

`tools/blackbox_decode.py` reads the record layout from the file's own header, exports
CSV, and prints a summary including the state timeline, per-cell voltage range, sample
loss, mixer saturation and per-sensor availability.

### 10.5 Host verification

The algorithms that carry the safety argument — the mixer, the link framing, the state
machine, the filters — are pure computation and are tested on the host against the
**real firmware headers**, with no ESP32 and no hardware:

```bash
sh tools/host_tests/run_tests.sh
```

98 assertions across 10 groups, each naming the finding it guards. Several reproduce the
original defect alongside the fix so the difference is demonstrated rather than asserted:
the mixer test shows the original losing 7.1% of a commanded roll differential in the
exact scenario finding 13 describes, then sweeps 1189 throttle × roll combinations
confirming the fixed mixer never attenuates or reverses a commanded moment. The framing
test confirms all 144 single-bit corruptions of a command frame are rejected.

This is not a substitute for the flight testing in section 11. It is what stops a
regression in these specific algorithms from reaching a flight test at all.

### 10.6 Building

```bash
cd firmware/flight-controller && pio run -t upload
```

Each of the four firmware directories is an independent PlatformIO project. Build them
in any order; they share `shared/odyssey_link.h` by include path, so a protocol change
rebuilds all four.

---

## 11. Pre-Flight and Field Commissioning Checklist

### 11.1 Bench

- [ ] **Power rail isolation.** Power the avionics from a current-limited bench supply at
      1.0 A. Confirm clean 5.0 V and 12.0 V rails.
- [ ] **Sensor enumeration.** On boot the console prints
      `[SENSORS] IMU:1 BACKUP:1 BARO:1 MAG:1 TOF:1 INA:1`. Every one must read 1. A zero
      is a wiring or address fault, not something to fly past.
- [ ] **GNSS link negotiation.** Console shows `[GNSS] module answered at N baud` and,
      if N was 9600, `[GNSS] switching to 115200 baud`. A `[GNSS] FAILED` line means
      arming will stay blocked.
- [ ] **Voltage divider calibration.** Compare the reported pack voltage against a
      multimeter and set `VOLTAGE_DIVIDER_TRIM` in `config.h`. A 2% error here shifts the
      critical threshold by 0.4 V.
- [ ] **Current shunt zero.** With the pack connected and motors idle, confirm the INA226
      reads within ±0.2 A of zero.
- [ ] **IMU polarity.** Nose-down tilt must produce positive pitch; right-wing-down must
      produce positive roll. If either is inverted, the aircraft will diverge on the
      first take-off.
- [ ] **Compass calibration.** Rotate the airframe through all axes to collect hard-iron
      offsets. Then **repeat with the motors spinning at hover throttle** and confirm the
      heading does not shift more than 5° — if it does, the mast is too short or the
      power leads need twisting.

### 11.2 Propulsion

- [ ] **ESC endpoint calibration** for 400 Hz PWM input (1000–2000 µs).
- [ ] **Motor direction (props-out), propellers removed.** Against section 4.2:
      - Motor 1 (rear-right) rotates **counter-clockwise**
      - Motor 2 (front-right) rotates **clockwise**
      - Motor 3 (rear-left) rotates **clockwise**
      - Motor 4 (front-left) rotates **counter-clockwise**
      Confirm each diagonal pair matches: M1 with M4, M2 with M3.
- [ ] **Thrust stand.** Measure per-motor thrust at 20 / 40 / 60 / 80 / 100% and correct
      the table in section 3.2 and `CRUISE_CURRENT_A` in `config.h`. The energy budget
      depends on these numbers being real.

### 11.3 Safety systems

- [ ] **Arm blocker sweep.** With propellers removed, verify each condition in section
      5.8 blocks arming and reports the right flag: raise the throttle stick, power off
      the handset, cover the GNSS, disconnect the compass.
- [ ] **Throttle-low arm interlock.** Specifically confirm that arming is refused with
      the throttle stick raised. This was finding 5, and it is the one that hurts people.
- [ ] **Free-fall parachute test.** Do **not** use the revision 1.0 procedure — a 10 cm
      drop gives only 143 ms of free fall against a 400 ms threshold, and inverting the
      craft leaves the accelerometer magnitude at 9.8 m/s², nowhere near the 1.5 m/s²
      trigger. That test could not have passed. Instead, either:
      - **tethered drop rig**, releasing from ≥ 1.5 m onto a crash mat, giving 550 ms of
        free fall (propellers removed, canister unloaded); or
      - **software self-test**: send `SELFTEST FREEFALL` on the debug console with the
        aircraft on the bench, which injects a synthetic free-fall condition and exercises
        the full detection and deployment path without dropping anything.
      Verify the servo reaches the 2000 µs eject position and that the state machine
      reports `FREEFALL_PARACHUTE`.
- [ ] **Beacon position feed.** With the aircraft powered and a GNSS fix acquired, confirm
      the beacon console logs cached coordinates. Then disconnect main power and confirm
      it enters beacon mode and transmits **the real coordinates**, not zeros. This is
      finding 7 and it is only testable end to end.
- [ ] **Beacon latch.** Confirm GPIO 21 latches the 1S cell and the buzzer and strobe
      begin their 5 s chirp cycle.
- [ ] **LiDAR intervention.** Place a hand inside 3.5 m of the TFmini-S. Verify forward
      pitch clamps **and** that the step-over climb is commanded (visible as increased
      throttle on the bench, and in the BlackBox).
- [ ] **LiDAR staleness.** Unplug the TFmini-S mid-test. Verify that avoidance goes
      *inactive* and is flagged, rather than latching the last distance. This is
      finding 15.
- [ ] **BlackBox.** Power on, wait 10 s, power down. Run
      `python tools/blackbox_decode.py flight_0001.ody` and confirm a valid header and a
      plausible record count.

### 11.4 Links

- [ ] **ExpressLRS binding**, and set the failsafe behaviour on the receiver to
      "no pulses" so the flight controller sees a genuine link loss.
- [ ] **Handset channel map** against section 5.4: roll, pitch, throttle, yaw on 1–4, arm
      on 5, RTH on 6.
- [ ] **RC failsafe.** Power off the handset in flight-ready state on the bench. Confirm
      the aircraft enters RTH (with a home lock) or failsafe landing (without one).
- [ ] **Ground station.** Confirm QGroundControl or Mission Planner shows the vehicle,
      with a moving attitude indicator and a plausible battery reading. If the GCS shows
      nothing, the MAVLink CRC is wrong — that was finding 9.
- [ ] **Land permission round trip.** Force `AWAITING_LAND_PERMIT` on the bench, confirm
      the GCS raises the alert, press PERMIT on the ground station and confirm the
      aircraft transitions. This path was unreachable in revision 1.0.
- [ ] **Remote ID — only if you are fitting it.** Confirm a Remote ID receiver app sees
      the aircraft, with **your** registered operator ID and serial number. Until both
      are valid the module broadcasts nothing, which is deliberate: a malformed
      identifier is worse than silence. Whether an unhealthy module blocks arming
      depends on `REQUIRE_REMOTE_ID_TO_ARM`, which is **0** by default — see 12.1.
- [ ] **2.4 GHz coexistence (see 8.5).** With the aircraft powered and Remote ID
      broadcasting, watch the ExpressLRS link quality in telemetry for 60 s. It must
      not dip in step with the Remote ID transmissions. `rcLinkQuality` is recorded
      in the BlackBox, so this is measurable without test equipment. A visible dip
      means the antennas are too close — move them apart before flying.

### 11.5 Field

- [ ] **GNSS home lock.** Confirm ≥ 6 satellites and a `HOME LOCKED` console line before
      arming.
- [ ] **First flight, low and close.** Hover at 3 m for 60 s. Land, pull the log, and
      check the mixer saturation figure. Anything sustained above 30% means the airframe
      is out of control authority and should not be flown further out.
- [ ] **RTH proving flight.** At 100 m out and 30 m up, with plenty of pack remaining,
      command RTH from the handset and confirm the aircraft climbs, yaws to the bearing,
      flies home and lands. Do not discover in an emergency that this works.

---

## 12. Regulatory Compliance

> **CORRECTION TO REVISION 2.0.** Revision 2.0 stated that at 1773 g "flying it without
> Remote ID is not a missing feature — it is unlawful" in the US, EU and UK. **That was
> an over-claim for the EU and it is withdrawn.** Section 12.1 sets out the actual
> position for a privately built aircraft. The engineering is unchanged; the legal
> framing was wrong.

### 12.1 Do you actually need Direct Remote ID?

For a **privately built** aircraft in the EU **open category**, the answer is not
automatically yes, and revision 2.0 got this wrong.

Under EU 2019/945 the Direct Remote ID obligation attaches to **class-marked** aircraft —
C1, C2, C3. EASA separately permits **privately built** UAS in the open category: A1
below 250 g, and A3 below 25 kg. A privately built aircraft carries no class mark, so
the C-class DRI requirement does not reach it by that route.

| Aircraft | Operation | Direct Remote ID |
| --- | --- | --- |
| Privately built, < 250 g, < 19 m/s | A1 | Not required merely because it is DIY |
| Privately built, < 25 kg | A3 | Not required merely because it is DIY |
| Class-marked C1 / C2 / C3 | A1 / A2 / A3 | **Required** |
| Any aircraft | **Specific category** | **Required** |

This aircraft is 1773 g and privately built, so flown in **A3** in the EU it plausibly
falls in the second row. Flown in the **specific category** — under an operational
authorisation, an STS, or a SORA — Remote ID is required regardless.

**This is a summary, not advice.** Confirm the position for your own operation and
jurisdiction. The distinction that matters is *class-marked versus privately built*, and
it is easy to miss because most published guidance is written for shop-bought drones.

**What the firmware does about it.** Because the requirement does not apply universally,
the firmware does not assume it. `REQUIRE_REMOTE_ID_TO_ARM` in `config.h` selects the
policy:

| Setting | Behaviour | Use when |
| --- | --- | --- |
| **`0`** (default) | Remote ID health is reported in telemetry and on the ground station, but a missing, unconfigured or failed module does **not** block arming | Privately built, open category, A1 or A3 — this airframe as specified |
| `1` | Remote ID must be healthy before the aircraft will arm | Specific category; class-marked C1/C2/C3; the United States under 14 CFR Part 89; or wherever your authority requires it |

Revision 2.0 hard-wired the restrictive behaviour, which would have grounded a legal
aircraft. Getting this wrong in the permissive direction is a legal problem; getting it
wrong in the restrictive direction stops you flying something you are entitled to fly.
Neither default is safe for everyone, so the choice is explicit and documented rather
than assumed.

The module itself stays honest either way: if the CTA serial is not structurally valid
it **broadcasts nothing at all** rather than transmitting a malformed identifier, and it
holds its health line low so the condition is visible in telemetry whether or not it
blocks arming.

### 12.2 Denmark

Trafikstyrelsen's current guidance requires Remote ID for the relevant C-marked drones,
and — regardless of weight or class — when flying in the **specific category**.

There is a change in progress. Trafikstyrelsen has proposed national rules broadening
electronic-visibility requirements. The consultation closed **21 August 2026** and the
proposed changes are expected to take effect **1 January 2027**.

> As of this revision those are **proposed** rules, not final law. If you are reading
> this after 1 January 2027, check what was actually adopted rather than trusting this
> paragraph.

Because that change is coming, the Remote ID module is specified and implemented here
even though it may not be required for A3 operation today. The alternative — discovering
in January 2027 that the airframe has no room, no spare UART and no power budget for it
— is worse.

### 12.3 The two identifiers

Direct Remote ID involves two completely different identifiers. Conflating them is a
common and consequential mistake, and revision 2.0 partly did.

**Which identifier to broadcast.** ASTM F3411 defines four UAS ID types, and the Basic
ID message declares which one it carries. Two matter here:

| ID type | What it is | Who needs it |
| --- | --- | --- |
| **`CAA_REGISTRATION_ID`** (default) | The registration your civil aviation authority issued | **A home build.** No ICAO involvement whatsoever |
| `SERIAL_NUMBER` | A CTA-2063-A serial, under an ICAO manufacturer code | Manufacturers — or a bought Remote ID module, which already carries its maker's serial |

> Revision 2.1 hard-coded the serial-number type and instructed the builder to apply to
> ICAO at `OPSInbox@icao.int`. **That is not necessary for a privately built aircraft.**
> ICAO issues manufacturer codes to manufacturers; building one aircraft for yourself
> does not make you one. `UAS_ID_TYPE` in `firmware/remote-id/src/main.cpp` now selects
> the route and defaults to the CAA registration.

**CTA-2063-A serial number — identifies the HARDWARE**

```
    K7E3F000000000000001
    ├──┘│└────────────┘
    │   │      └── manufacturer serial, up to 15 characters
    │   └───────── length code: 1-9 = 1..9 characters, A-F = 10..15
    └───────────── 4-character ICAO manufacturer code
```

Permitted characters are 0–9 and A–Z **excluding I and O**, so a serial cannot be
misread as containing 1 or 0. A 4 + 1 + 15 = 20 character serial fills OpenDroneID's
20-byte UAS ID field exactly, which is why the design uses the full length.

Only relevant if you are manufacturing. Manufacturer codes come from ICAO at
`OPSInbox@icao.int` — but see the note above before assuming you need one.

> Revision 2.0 shipped `ODY1P0000000000000000` as the placeholder. It was malformed in
> two ways — `P` is not a valid length code, and the serial that followed ran to 16
> characters against a maximum of 15 — but it looked plausible enough to ship, and would
> have broadcast a structurally invalid UAS ID. The placeholder is now deliberately
> obvious (`SET-YOUR-CTA-SERIAL`) and `odyValidateCtaSerial()` rejects it, holding the
> module's health line low so the aircraft will not arm.

**Operator registration number — identifies the PERSON**

```
    DNK87astrdge12k8-xyz
    ├─┘├──────────┘│ └─┘
    │  │           │  └── 3 SECRET characters  <-- never publish these
    │  │           └───── Luhn mod 36 check character
    │  └───────────────── 12 characters
    └──────────────────── ICAO country code (DNK for Denmark)
```

The **16 characters before the hyphen are public** and go on the airframe label. The
**three characters after it are secret.** EASA is explicit that they must not be shared;
Trafikstyrelsen calls the Danish equivalent a *security code*.

> **The secret is therefore never compiled into this firmware.** There is no `#define`
> for it. It is provisioned at runtime into NVS over the serial console
> (`SETOPERATOR DNKxxxxxxxxxxxxx-yyy`), only the public 16 characters are ever copied
> into the broadcast structure, and `tools/check_consistency.py` fails the build if
> anything resembling a full operator ID appears in a committed source file. A secret
> committed once stays in git history even after it is "removed".

**A note on the checksum.** `odyOperatorChecksum()` implements Luhn mod 36 and
reproduces EASA's published example (`FIN87astrdge12k8`). That is **one** test vector; a
sweep of sixteen plausible algorithm variants found two that reproduce it, so a single
example cannot distinguish between them. The implementation is therefore **advisory**:
strict validation is opt-in and is not used anywhere that could block a flight, because
a false rejection would ground a legitimate operator over an unverified algorithm.
Confirm it against ASD-STAN prEN 4709-002 before relying on it.

### 12.4 What this module is, and is not

The module broadcasts correctly encoded OpenDroneID messages over Bluetooth 5 Long Range
(Coded PHY) and a Wi-Fi beacon vendor IE — Location/Vector at 1 Hz, static messages every
3 s — using the reference encoder `opendroneid/opendroneid-core-c` rather than a
hand-rolled bit layout.

That is **not** the same as being a compliant Direct Remote Identification add-on. EU
2019/945 Part 6 additionally requires:

- operator-ID upload with validity and consistency checking;
- a CTA-2063-A serial physically associated with the module;
- continuous periodic broadcast of the defined aircraft information;
- resistance to tampering;
- manufacturer instructions covering installation and protocol.

And if the module were **placed on the EU market**, product-conformity obligations apply
on top of that. Linking the reference encoder gets the wire format right. It does not
confer compliance, and this document does not claim it does.

### 12.5 Radio spectrum

The specified 433 MHz LoRa link at +20 dBm (100 mW) is **not universally legal**, and
revision 1.0 did not mention this at all.

| Region | 433 MHz status | Recommendation |
| --- | --- | --- |
| United States | 70 cm amateur band. +20 dBm requires an FCC amateur licence under Part 97. Part 15 unlicensed operation at 433.92 MHz is limited to field strengths far below this. | Hold an amateur licence, **or** substitute a 915 MHz module (Ra-02H / SX1276) and operate under Part 15.247 |
| EU / UK | 433.05–434.79 MHz SRD. Typical limit is 10 mW ERP with ≤10% duty under EN 300 220 — the specified +20 dBm exceeds it | Reduce to +10 dBm, **or** substitute an 868 MHz module and observe that band's duty limits |

The SX1278 is a 433 MHz part; the pin-compatible **Ra-02H (SX1276)** covers 868/915 MHz
and is a drop-in swap at the same price. Only the `LoRa.begin()` frequency constant
changes in firmware. **Choose the band for your region before you build.**

Duty cycles are budgeted in section 8.4 and enforced at runtime by the beacon firmware.

### 12.6 Video transmitter

The 800 mW–1.6 W 5.8 GHz VTX exceeds unlicensed limits in most jurisdictions. In the US
it requires an amateur licence; in the EU, 25 mW EIRP is the general limit. Set the
output power accordingly — most VTX modules support 25 mW.

### 12.7 Weight and category

At 1773 g the aircraft falls into EU open category **A3**, requiring 150 m separation
from residential, commercial and industrial areas, and an A1/A3 remote pilot
certificate. As a privately built aircraft it carries **no class mark** — see 12.1, and
note that the Remote ID firmware therefore declares
`ODID_CLASS_EU_CLASS_UNDECLARED` rather than asserting a C-class it has not been
assessed against.

In the US it requires FAA registration and, for anything beyond recreational flying
under the exception, a Part 107 certificate. US Remote ID rules (14 CFR Part 89) are
structured differently from the EU's and do apply above 250 g including to home-built
aircraft, with a broadcast module being the standard route.

---

## 13. Review Findings Resolution Index

All eighteen findings from the revision 1.0 review, with the change that resolves each.

| # | Finding | Severity | Resolution | Where |
| --- | --- | --- | --- | --- |
| 1 | 3S voltage thresholds (10.2 V / 9.9 V) on a 6S pack — every battery failsafe unreachable | Critical | Thresholds derived per cell from `CELL_COUNT`; compile-time assertions reject a 3S value on a 6S airframe | §5.1, `config.h` |
| 2 | §4 rotation diagram inverted all four motors vs. pinout, code and checklist; also contradicted its own props-out label | Critical | Single authoritative motor map, duplicated in `config.h` and verified in commissioning | §4.2, §11.2 |
| 3 | No manual control link existed anywhere; ground station was receive-only; `PERMIT_LAND` unreachable | Critical | ExpressLRS 2.4 GHz receiver added for stick control; LoRa command uplink implemented with three operator buttons | §5.4, `radio_link.*`, ground station |
| 4 | RTH neither navigated nor escalated on battery — hovered until the pack was flat | Critical | Three-phase position controller (climb / translate / descend); energy checks run in every powered state | §5.3, `navigation.cpp` |
| 5 | No throttle-low check at arming; stale RC throttle applied instantly | Critical | Eight-condition arm gate including throttle-at-minimum from a live frame, reported as blocker flags | §5.8, `main.cpp` |
| 6 | Cross-core state race could restart motors under a deployed canopy | Critical | Atomic test-and-set under a spinlock, plus escalation-ordered states so the race is harmless | §5.6, `state_machine.h` |
| 7 | Beacon transmitted latitude 0 / longitude 0 forever — no data path from the flight controller | Critical | AUX broadcast bus feeds live position; beacon kept alive on aircraft power via diode-OR, caches in RTC memory | §7.2, beacon firmware |
| 8 | Beacon endurance ~9 h not 48 h; ~50% radio duty cycle | High | 45 s / 180 s schedule at 6.1% / 1.5% duty, light sleep between events, runtime duty policing; claim revised to 72 h | §7.3 |
| 9 | MAVLink checksums were hardcoded placeholders; v1 magic despite a v2 heading; constant sequence | High | Real CRC-16/MCRF4XX with per-message `CRC_EXTRA`, proper v2 framing, per-link sequence, six message types | §10.3, `mavlink_min.h` |
| 10 | Ground station showed DISARMED while the aircraft was airborne with props turning | High | Shared `odyMotorsAreLive()` predicate; no caller open-codes an ordinal comparison | §5.6, `odyssey_link.h` |
| 11 | Parachute descent never armed the recovery beacon | High | Both terminal paths arm the beacon; parachute path waits 20 s for the canopy to settle first | §7.4 |
| 12 | Free-fall detection disabled during land-permit hold and failsafe descent; documented bench test impossible | High | Detection in every motors-live state, 8 m minimum deployment altitude, timer reset on state entry; corrected test procedure | §5.9, §11.3 |
| 13 | Mixer clamped each motor independently — control authority could invert at high throttle | High | Two-stage desaturating mixer; yaw sacrificed before roll and pitch; saturation logged every sample | §4.4, `mixer.h` |
| 14 | Baro-only touchdown check could disarm mid-air; VL53L1X, INA226 and QMC5883L never implemented | High | Four-condition touchdown with an altitude veto; all three sensors implemented; ground reference re-latched at arm | §5.7, `sensors.cpp` |
| 15 | LiDAR reading never expired — stale value latched the pitch clamp; step-over climb never implemented | High | `TimedValue<T>` on every sensor with per-sensor max age; stale sensor disables avoidance and is flagged; step-over climb implemented | §6, `types.h` |
| 16 | GNSS baud contradiction: pinout said 115200, firmware opened 9600 | Medium | Firmware negotiates the rate, reconfigures the module to 115200 / 10 Hz and verifies; documentation matches | §9.1, `sensors.cpp` |
| 17 | Remote ID claimed in §1 but never implemented — and impossible, since the ESP32-P4 has no radio | High | ESP32-C6 module added; ASTM F3411 broadcast over BLE 5 Long Range and Wi-Fi; Remote ID health blocks arming | §12.1, remote-id firmware |
| 18 | BOM summed to $512.50 against a stated $514.50 | Low | Totals computed from `hardware/bom.csv`; new total $540.50 including the two added modules | §2 |

### 13.1 Issues found while fixing these, and also corrected

These were not in the original eighteen but surfaced during the rework:

| Issue | Resolution |
| --- | --- |
| Telemetry at 2 Hz / BW 125 kHz would have occupied 22.6% duty cycle | Moved to 1 Hz / BW 250 kHz → 5.6% |
| Both cores drove the I2C `Wire` bus with no mutex | All I2C traffic serialised; flight loop touches one device |
| `mpu.begin()`, `baro.begin()`, `SD.begin()` and `LoRa.begin()` return values were discarded | Every result checked and recorded in the health mask |
| Notch filters configured inside calibration; an early bail-out left them zero-initialised, silently zeroing the gyro | An unconfigured filter is now a pass-through, and calibration failure blocks arming |
| PID derivative taken on error, spiking on every setpoint step | Derivative on measurement, low-pass filtered, with conditional anti-windup |
| Single unfiltered ADC sample drove irreversible mode changes | Oversampled, filtered, slew-limited and debounced over 3 s |
| BlackBox appended every flight to one file with no header or version | Versioned header, one file per flight, ring-buffered writes on an isolated task |
| BlackBox logged raw pre-clamp mixer sums, hiding saturation | Post-clamp values plus an explicit saturation percentage |
| Ground station spun forever in `while(1)` on radio failure, with no indication | Emits MAVLink status text so the failure is visible in the GCS |
| Beacon battery divider was on an undeclared pin with an undocumented ratio | Declared pin, documented 100k/100k divider |
| 433 MHz second harmonic at 866 MHz would desensitise an 868 MHz ExpressLRS receiver | RC link specified as 2.4 GHz; interaction documented |
| No endurance or range figures existed to calibrate the energy budget against | Section 3.3, with assumptions stated |

### 13.2 Corrected in revision 2.1

Revision 2.0 introduced or left standing the following, all corrected here.

| Issue | Resolution |
| --- | --- |
| §12 asserted that flying above 250 g without Remote ID is unlawful in the EU. **Over-claim.** The DRI obligation attaches to class-marked C1/C2/C3 aircraft; EASA separately permits privately built UAS in A1 (<250 g) and A3 (<25 kg), and a home build carries no class mark | §12.1 rewritten with the class-marked versus privately built distinction, and a table of when DRI actually applies |
| No Denmark-specific position, despite the aircraft being built there | §12.2 added, including Trafikstyrelsen's proposed electronic-visibility rules (consultation closed 21 August 2026, proposed effect 1 January 2027) flagged as proposed rather than adopted |
| The CTA-2063-A placeholder `ODY1P0000000000000000` was malformed — `P` is not a valid length code and the serial ran to 16 characters against a maximum of 15 — but looked plausible enough to ship | Placeholder made obviously invalid; `odyValidateCtaSerial()` added and the module holds its health line low until a real serial is set |
| `OPERATOR_ID` was a compiled-in constant, so a user following the instructions would have committed the three SECRET characters of their registration into git history | Operator ID moved to runtime NVS provisioning; no `#define` exists; only the public 16 characters reach the broadcast structure; `check_consistency.py` fails if a full operator ID appears in a source file |
| Remote ID declared `ODID_CLASS_EU_CLASS_3`, asserting a conformity assessment a home build has not been through | Changed to `ODID_CLASS_EU_CLASS_UNDECLARED` |
| Moving the RC link to 2.4 GHz solved the 433 MHz harmonic problem but created an unaddressed 2.4 GHz coexistence problem with the Remote ID module, which the placement diagram put in the central stack | §8.5 added: asymmetric mitigation protecting the mission-critical ELRS receiver, placement rules, and a measurable bench check in §11.4 |
| §12 implied that broadcasting OpenDroneID messages constitutes compliance | §12.4 added, listing what EU 2019/945 Part 6 additionally requires |
| Nothing checked that the specification and the firmware still agreed with each other — the failure mode behind findings 1, 2, 16 and 18 | `tools/check_consistency.py` added: 12 automated checks with `--fix` for the mechanically correctable ones, wired into the pre-push hook and CI |
| Remote ID was a **blocking** arm condition, imposing on a privately built A3 aircraft a requirement that attaches to class-marked C1/C2/C3 aircraft — it would have grounded a legal airframe | `REQUIRE_REMOTE_ID_TO_ARM` added to `config.h`, defaulting to 0. Remote ID is optional unless the operator opts in; §12.1 documents when to |
| Airframe changed to a 387 mm 9-inch frame after revision 2.1, invalidating the thrust, AUW, TWR, endurance and gyro-notch chain | Recomputed throughout in revision 2.2. The notch moved 80 Hz → 95 Hz, without which the new ~97 Hz motor peak would have passed into the rate controller |
| The Basic ID message hard-coded `SERIAL_NUMBER`, forcing the CTA-2063-A route and an ICAO manufacturer code application that a privately built aircraft does not need | `UAS_ID_TYPE` added, defaulting to `CAA_REGISTRATION_ID`. `odyValidateCaaRegistration()` and 10 host assertions added; §12.3 documents both routes |

---

*End of specification, revision 2.0.*
