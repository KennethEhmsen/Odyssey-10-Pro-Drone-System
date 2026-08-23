# Autonomous Long-Range Quadcopter Engineering Master Specification

**Platform Identification:** Odyssey-10 Pro

**Architecture:** 9-inch long-range airframe, ESP32-P4 dual-core RISC-V avionics, integrated perception, kinetic recovery and safety stack

**Document revision:** 4.9 — the power rails are assigned, and the 3.3 V budget is stated

---

## About revision 2.2 — the airframe changed

The build moved to an **RJXHOBBY Mark4 V2, 387 mm wheelbase, 6 mm arms** — a **9-inch**
frame, not the 10-inch 420–450 mm frame revisions 1.0 to 2.1 assumed.

That is not a parts substitution; it re-bases the whole propulsion chain. Everything
below has been recomputed:

| | Was (10-inch) | Now (9-inch) |
| --- | --- | --- |
| Frame | 420–450 mm, 7–8 mm arms, 286 g | 387 mm, 6 mm arms, 230 g |
| Propellers | 10x5x3, 48 g | **9x5x2, 28 g** (`PROP_BLADES` switch, see 3.2) |
| All-up weight | 1773 g | **1584 g** |
| Thrust per motor | 1750 g | **1230 g** |
| Thrust-to-weight | 3.95:1 | **3.11:1** |
| Hover throttle | ~44% | **~51%** |
| Motors | 3110, 68 g each | **2810, 52 g each** (`MOTOR_CLASS` switch) |
| ESC | 50 A/ch, 34 g | **40 A/ch, 25 g** |
| Pack | 4500 mAh 45C, 680 g | **4500 mAh 20C, 640 g** |
| Gyro notch centre | 80 Hz | **120 Hz** |

The notch change is the one that matters for safety. A smaller disc needs more shaft
speed for the same thrust, moving the hover fundamental up out of the old notch — at
Q = 4 an 80 Hz notch is already −3 dB by 90 Hz, so the original setting would have passed
the new motor-noise peak straight into the rate controller. See section 8.3, including
why the new figure carries more uncertainty than a single number suggests.

The motors changed with the frame. A 3110 is a 31 mm stator — a 10–11 inch motor. The
28 mm 2810 class is the 8–9 inch class, and it is the **stator** that was wrong, not the
900 KV winding, which is already correct for a 9-inch propeller on 6S.

The ESC followed the motors. Peak draw is **16.9 A per motor**, so the 50 A per-channel
rating carried over from the 10-inch build was nearly three times what the aircraft can
actually pull. 40 A per channel keeps a 2.4× margin and saves 9 g.

The propellers and pack followed the mission. Three blades buy thrust per unit diameter;
two blades buy efficiency — and with thrust-to-weight already above 3:1 there was thrust
to trade for endurance. The pack was rated 45C against a **15C** peak demand, so 40 g was
being carried for capability the aircraft cannot use. Together those give **24 minutes of
hover and a 16 km one-way range**, up from 22 minutes and 14.1 km.

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
|  * Quad-X mixer WITH DESATURATION -> 400 Hz LEDC PWM to 4x 2810 motors                |
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
| Propulsion | 2810 / 2812 brushless motor | 900 KV, 28 mm stator, 4S–6S | 4 | $22.00 | $88.00 | 208 g |
| Propellers | HQProp / Gemfan 9x5x2 | 9 in **two-blade**, 2 CW + 2 CCW, props-out | 2 pr | $6.00 | $12.00 | 28 g |
| Drive | 4-in-1 ESC (BLHeli_32 / AM32) | 40 A cont., 50 A burst per ch., 6S | 1 | $38.00 | $38.00 | 25 g |
| Main battery | 6S LiPo pack | 22.2 V nom, 4500 mAh, **20C min** | 1 | $110.00 | $110.00 | 640 g |
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
| **Master total** | | | | | **$560.50** | |

> **Every price here is an estimate.** None of this has been purchased. The figures are researched typical prices, not invoices, and they will move with supplier, region and time — the ESP32-P4 in particular is a recent part whose board-level pricing is still settling. Treat the master total as an order of magnitude for budgeting, not as a quotation.
>
> **The masses are a different matter.** They are estimates too, but they are not merely informational: they sum into `FRAME_BASE_DRY_G` and `AIRFRAME_AUW_G`, which size thrust-to-weight, hover power and the return-to-home reserve. A price that is 50% out costs money; a mass that is 50% out costs endurance and margin. **Weigh the parts as they arrive** and correct §2 — the `bom-mass` check will then reconcile `config.h` against what you actually have.

**Rows added in this revision** are shown in bold. The wiring lot increased by $1.50 to
cover the Schottky diode-OR and the main-power-sense divider that the beacon redesign in
section 7 requires.

Airborne dry mass is 1414 g (the second SX1278 module is the ground station radio and
does not fly). See section 3.1.

---



### 2.2 The flight controller board is not decided

The BOM's `Main MCU` line now carries a **real price** — 285 DKK, about $41, quoted for
an **ESP32-P4-Function-EV-Board v3.2**. It still carries a **modelled mass** of 9 g, and
those two figures describe different objects.

The Function-EV-Board is Espressif's full-size evaluation board. It is the right thing to
run §4.3.2's bench procedure on: every GPIO broken out, USB debug, and a known-good
reference design to blame the firmware against rather than the wiring. It is the wrong
thing to fly. A full-size evaluation board weighs many times 9 g, and on a 1584 g
airframe that is not a rounding error — it comes straight off the payload reserve and out
of the endurance the energy budget in §5 is written around.

The 9 g assumes an **ESP32-P4 module on a compact carrier**, which in this project means
the PCB at build-order step 11. Until that exists, or until an off-the-shelf compact P4
board is chosen and weighed, the flight-controller mass is the least-supported number in
§2 — and unlike a price, it propagates:

```
board mass  ->  FRAME_BASE_DRY_G  ->  AIRFRAME_AUW_G  ->  thrust-to-weight
                                                      ->  hover power
                                                      ->  CRUISE_CURRENT_A
                                                      ->  the return-to-home reserve
```

**So: bench on the EV board, and weigh whatever eventually flies.** §11.1 now asks for
that explicitly. The `bom-mass` check reconciles `config.h` against §2 for all ten
builds, so correcting the mass there is the only edit needed — everything downstream is
computed.

### 2.1 The other nine builds

`hardware/bom.csv` is the **9-inch / 2810 / 2-blade reference**. `config.h` supports ten
combinations, and until revision 3.9 nine of them compiled, passed every test, and had
no parts list — anyone building one was on their own for frame, motors, propellers,
battery and ESC.

`hardware/bom-variants.csv` closes that. It lists only what **differs** by build;
everything else — every sensor, both radios, the flight controller, the recovery system,
308 g of shared avionics — is identical across all ten, and repeating it three times
would be three places for it to drift.

| Frame | Airframe | Motors | Battery | Dry mass |
| --- | --- | --- | --- | --- |
| 7-inch | 295 mm, 5 mm arms, 132 g | 2807, 45 g each | 3000 mAh **50C** | 440 g |
| **9-inch** | **387 mm, 6 mm arms, 230 g** | **2810 or 3110** | **4500 mAh 20C** | **538 g** |
| 10-inch | 430 mm, 6 mm arms, 286 g | 3110 or 3115, 68–78 g | 4500 mAh **30C** | 594 g |

The 7-inch takes a smaller pack at a much higher C rating, because it peaks at 99 A on
three blades — 33C from 3000 mAh, where the 9-inch's peak is only 15C from 4500.

**Three builds need a bigger ESC than the reference.** The 40 A/channel part is sized on
the 9-inch 3-blade figure of 16.9 A per motor, a 2.4× margin. The 7-inch 3-blade and both
10-inch 3-blade builds peak at 23–25 A per motor, which the same ESC would meet at only
1.6–1.7× — below the 2× §4.3 argues for on a hover-heavy profile where thermal soak
matters more than burst rating. Those three list a 60 A/channel ESC instead, at +$14 and
+9 g.

Those same three also exceed the XT90-S's 90 A rating and list an AS150 in its place. The
build already refuses to compile without either that part or an explicit
`-DACCEPT_CONNECTOR_OVER_RATING=1`; now the parts list says what to buy instead of only
what to acknowledge.

> **Mass that does not fly.** The BOM buys **two** SX1278 radios and only one leaves the
> ground — the second is the ground station. A naive sum of the mass column therefore
> overstates the airframe by 6 g. That distinction used to live in a prose note inside a
> CSV cell, where nothing could check it; there is now an explicit `Airborne Mass g`
> column, and the `bom-mass` check reconciles it against `FRAME_BASE_DRY_G`,
> `MOTOR_MASS_G_EACH`, `PROP_MASS_G_EACH`, `BATTERY_MASS_G` and `AIRFRAME_AUW_G` for
> every one of the ten builds.
>
> `check_bom()` had verified BOM *arithmetic* and the *price* quoted here since the
> original review. Nothing verified **mass** against `config.h` — so `AIRFRAME_AUW_G`,
> which sizes thrust-to-weight, hover power and the entire energy budget, was free to
> drift away from the parts that produce it.

---

## 3. Power, Propulsion and Sizing

### 3.1 Mass budget

All-up weight is now built bottom-up from the BOM rather than asserted. Revision 1.0
quoted 1850 g against a grouped estimate that did not reconcile with its own parts list.

| Group | Mass |
| --- | --- |
| Frame, standoffs, gel dampers, hardware | 230 g |
| 4× 2810 motors (52 g each installed) | 208 g |
| 4× 9x5x2 propellers | 28 g |
| 4-in-1 ESC | 25 g |
| 6S 4500 mAh LiPo pack | 640 g |
| Avionics stack (P4, IMUs, baro, SD) | 18 g |
| Navigation sensors (GNSS + mast, compass, LiDAR, ToF, INA226) | 26 g |
| Radios (LoRa air unit, ExpressLRS RX, Remote ID module) | 13 g |
| Video subsystem (VTX, camera, antennas) | 45 g |
| Power regulation (BEC) | 14 g |
| Parachute subsystem | 55 g |
| Beacon subsystem (node, 1S cell, latch) | 24 g |
| Wiring, connectors, capacitors, hardware | 88 g |
| **Dry mass** | **1414 g** |
| Payload reserve | 170 g |
| **All-up weight (AUW)** | **1584 g** |

### 3.2 Thrust and throttle response

Maximum static thrust per motor at 6S with a 9x5x2 propeller is **1230 g**, giving 4920 g
total and a **thrust-to-weight ratio of 3.11:1**.

**Why the motor changed.** Revisions up to 2.2 specified a 3110 — a 31 mm stator, which
is a 10–11 inch motor. On a 387 mm 9-inch frame that is over-sized: it carries mass the
airframe does not need. The 2810 class (28 mm stator, about 82% of the volume) is the
8–9 inch class and is the correct match.

The 900 KV winding is unchanged, and deliberately so — it is the **stator** that was
wrong, not the KV. 900 KV on 6S is already right for a 9-inch propeller; dropping the
stator size while keeping the winding gives a lighter motor at the same shaft speed.

> **Correction to revision 2.3.** That revision quoted 1300 g for the 3110 and 1400 g for
> the 2810 on the same propeller — which had the *smaller* stator producing *more*
> thrust. That is backwards. The 1300 g came from scaling the 10-inch figure down for a
> smaller propeller; the 1400 g came from asserting the 2810 was "properly matched". Only
> the first was derived from anything.
>
> The physical picture: at 900 KV on 6S a 9-inch propeller is **prop-limited**, not
> motor-limited, so both stators reach similar thrust. The 3110 holds RPM slightly better
> under load — about 3% more thrust — and mostly buys **thermal headroom**. It does
> not buy performance, and it costs 64 g.

So the two motors are close to a wash on thrust-to-weight, and the real choice is between
mass and thermal margin:

| | 2810 | 3110 |
| --- | --- | --- |
| Stator | 28 mm | 31 mm |
| Mass each | **52 g** | 68 g |
| Peak thrust (2-blade) | 1230 g | **1271 g** |
| All-up weight | **1584 g** | 1648 g |
| Thrust-to-weight | **3.11:1** | 3.08:1 |
| Cruise current | **9.7 A** | 10.1 A |
| One-way range | **16.0 km** | 15.4 km |
| Thermal headroom | adequate | **better** |

The 2810 is the default. Choose the 3110 if you fly long hovering missions in warm
weather, or if a thrust-stand run shows the 2810 running hot — 0.6 km of range is a fair
price for margin you have measured a need for.

Revision 1.0 stated that hover occurred at 22% throttle. That figure was not physical:
propeller thrust scales with roughly the square of shaft speed, so 22% stick cannot
produce a quarter of maximum thrust on any real ESC curve.

The table below follows a T ∝ throttle^1.67 characteristic, representative of a
BLHeli_32 ESC with throttle linearisation. **These are modelled figures. Verify them on a
thrust stand before your first flight** — the hover point drives the energy budget in
section 5.2, and this table has now moved twice.

| Throttle | Thrust/motor | Total | Note |
| --- | --- | --- | --- |
| 0% | 0 g | 0 g | |
| 20% | 84 g | 335 g | |
| 30% | 165 g | 659 g | |
| 40% | 266 g | 1065 g | |
| **51%** | **396 g** | **1584 g** | **hover equilibrium = AUW** |
| 60% | 524 g | 2096 g | |
| 75% | 761 g | 3043 g | fast cruise |
| 100% | 1230 g | 4920 g | emergency punch-out |

**Two blades or three?** Both are fully supported and the choice is genuine. The
default is two, but the reasoning matters more than the default.

| | 9x5x2 (specified) | 9x5x3 |
| --- | --- | --- |
| All-up weight | 1584 g | 1592 g |
| Peak thrust per motor | 1230 g | 1400 g |
| Thrust-to-weight | 3.11:1 | **3.52:1** |
| Hover throttle | 51% | **47%** |
| Hover power | **196 W** | 215 W |
| Power loading | **8.90 g/W** | 8.07 g/W |
| Cruise endurance | **22 min** | 20 min |
| One-way range | **16.0 km** | 14.6 km |
| Peak pack current | **51 A** | 67 A |
| **Gyro notch centre** | **120 Hz** | **100 Hz** |

**The common argument for three blades — wind — does not survive checking.** Holding
position in a 20 m/s wind requires about 1.35× hover thrust, which is 43% of maximum on
two blades and 38% on three. Neither is close to thrust-limited. And on a 5 km
out-and-back into an 8 m/s headwind, the round trip needs 25 minutes of cruise against
22.3 minutes available on two blades and 20.3 on three — so in wind the **two-blade
configuration has more margin, not less**, because that mission is endurance-limited
rather than thrust-limited.

**Where three blades genuinely win** is attitude authority: faster control response,
more punch-out margin, and a hover point 4% lower, which leaves more throttle range
before the mixer starts trading attitude authority for climb. If you fly this aircraft
manually and aggressively, or in gusty conditions close to obstacles, three blades are
the better choice and the 1.4 km of range is a fair price.

**Three build switches.** Frame size, motor class and blade count each change several
coupled constants, so they are compile-time switches rather than sets of edits:

```bash
idf.py build                                                # 9-inch, 2810, 2-blade
idf.py build -DFRAME_SIZE_IN=7
idf.py build -DFRAME_SIZE_IN=10 -DMOTOR_CLASS=MOTOR_3115 -DPROP_BLADES=3
```

**These are cached, unlike the `pio run` form they replace.** `idf.py -D` sets a CMake
cache variable, so a switch passed once keeps applying to every later `idf.py build`
until it is overridden or cleared with `idf.py fullclean`. The configure step prints
each switch it applied (`-- odyssey: build switch FRAME_SIZE_IN=7`) and the firmware
prints the resulting configuration at boot, because two builds are otherwise
indistinguishable once flashed.

| Frame | Motors | Default | Wheelbase | Loop | Notch (2b / 3b) |
| --- | --- | --- | --- | --- | --- |
| 7-inch | 2807 | 2807 | 295 mm | 1000 Hz | 180 / 150 Hz |
| **9-inch** | 2810, 3110 | **2810** | 387 mm | 500 Hz | **120** / 100 Hz |
| 10-inch | 3110, 3115 | 3115 | 430 mm | 500 Hz | 105 / 90 Hz |

> **The flight controller builds with ESP-IDF, not PlatformIO, and this section was
> rewritten only after a build succeeded.** PlatformIO's stock `espressif32` platform
> lists 242 boards and `esp32-p4-function-ev-board` — the board
> `firmware/flight-controller/platformio.ini` names — is not among them, so
> `pio run -e odyssey-fc` fails resolving the board before it reaches a compiler. The
> ESP32-P4 is simply newer than the stock platform.
>
> Of the two routes — **pioarduino**, a community fork carrying a current Arduino-ESP32,
> or **ESP-IDF 5.5**, official and supporting the P4 directly — ESP-IDF was chosen, with
> Arduino pulled in as a managed component.
>
> **The other three images are still PlatformIO projects**, and correctly so: the beacon
> node, Remote ID module and ground station target the ESP32-C3, C6 and S3, which the
> stock platform has supported for years. Only the P4 needed moving. See §10.6.
>
> An earlier revision of this note promised the instructions would be rewritten "when a
> build succeeds, not before". They have been, and the switches above were verified by
> building a non-default combination and confirming the binary changed — the 9-inch
> image embeds `2810 900 KV`, the `-DFRAME_SIZE_IN=7` image embeds `2807 1300 KV`.

Ten combinations, all building and passing the host tests. Invalid pairings stop the
build with a specific message — a 2807 on a 10-inch frame is under-stator'd and a 3115 on
a 7-inch is dead weight, so neither is offered.

> **A 5-inch airframe was investigated and deliberately excluded.** Its hover fundamental
> lands near 265 Hz, which needs a loop rate above the MPU-6050's 1 kHz output ceiling to
> filter honestly. The hardware cannot do it, and shipping a configuration whose notch
> would chase an alias is worse than declining.

> **Characterisation status.** Only the 9-inch / 2810 / 2-blade configuration is backed by
> an itemised BOM and the analysis in this document. The 7-inch and 10-inch parameter sets
> are **modelled**, scaled from that anchor by momentum theory and disc loading. They are
> coherent and they are a reasonable starting point, but nobody has flown them. Treat
> every figure in a non-default build as a hypothesis for the thrust stand, and expect the
> PID gains to need tuning — `FRAME_GAIN_SCALE` is a first-order inertia argument, not a
> tune.

**Three configurations exceed the XT90-S 90 A continuous rating** on a full-throttle burst
— 7-inch 3-blade at 99 A, and both 10-inch 3-blade builds at 92 and 95 A. That is a burst
condition rather than continuous and the connector will survive brief punch-outs, but the
build stops until you either fit an AS150, drop to 2-blade, or acknowledge it with
`-DACCEPT_CONNECTOR_OVER_RATING=1`.

`PROP_BLADES` supplies the per-propeller figures — mass, base thrust, notch centre,
power loading. `MOTOR_CLASS` supplies the per-motor figures — mass and thrust factor.
All-up weight, peak thrust, peak current and cruise current are then **derived** from
both:

```c
AIRFRAME_AUW_G     = base dry + 4 x motor mass + 4 x prop mass + payload reserve
MOTOR_MAX_THRUST_G = prop base thrust x motor thrust factor
CRUISE_CURRENT_A   = (AUW / power loading + avionics W) x cruise factor / pack V
```

Tabulating two motors by two propellers would be four sets of numbers and four chances to
mistype one. Deriving them also means an `AIRFRAME_AUW_G` override for payload flows
through to the cruise current automatically, which matters because that constant sizes
the return-to-home reserve.

This is a switch rather than documentation because the constants are **coupled**, and
the coupling is not obvious. Changing thrust and weight but forgetting the notch would
leave the filter attenuating empty spectrum while still adding phase lag in the control
band — worse than having no notch at all, and invisible until the aircraft flies badly
for no apparent reason. Section 8.3 explains why a 120 Hz notch does essentially nothing
for the 104 Hz peak three blades produce.

Two constants accept an override, because measurement should beat a model:

```bash
-DNOTCH_CENTER_HZ=108.0f     # after reading the real peak off a BlackBox trace
-DAIRFRAME_AUW_G=1750.0f     # after adding payload
```

Compile-time assertions catch an incoherent combination — a thrust-to-weight ratio below
2:1, a notch outside any plausible motor-noise band, an implausible all-up weight or motor
mass. `tools/check_consistency.py` goes further: it resolves all four builds through the
real C preprocessor and checks that each constant actually varies on the axis it should,
in the right direction, and that the default is still 2810 + 2-blade. It also rejects a
3110 modelled at more than 10% extra thrust, because that would mean the revision 2.3
error had crept back in.

**The firmware prints its configuration at boot**, because two builds are otherwise
indistinguishable once flashed:

```
Frame      : 9-inch (387 mm)
Motors     : 2810 900 KV  (52 g each)
Propellers : 9x5x2 two-blade  (PROP_BLADES=2)
Airframe   : AUW 1584 g, 1230 g/motor, TWR 3.11:1
Gyro notch : 120 Hz, Q 4.0  (modelled default -- measure and override)
Energy     : cruise 9.7 A, peak 51 A
```

Check that against the propellers actually fitted before they go on.

**Peak current, which sizes the ESC.** Momentum theory with a figure of merit of 0.55
and a combined motor/ESC efficiency of 0.78 puts full-throttle draw at about 374 W per
motor on 3-blade propellers — **16.9 A per motor at 22.2 V, or 67 A for the aircraft**.
Two-blade propellers draw less, about 51 A for the aircraft. The ESC in section 4.3 is
sized on the higher figure so that either propeller can be fitted.

**If you have already bought 3110s**, fly them. They will work on 9-inch propellers — the
aircraft is 64 g heavier (AUW 1648 g) and the thrust-to-weight lands near 2.99:1
instead of 3.11:1,
which is still ample for a cruiser. Set `AIRFRAME_AUW_G` to 1648 and
`MOTOR_MAX_THRUST_G` to 1230 in `config.h` and the rest of the chain follows.

**Propeller clearance.** At a 387 mm wheelbase, adjacent motor hubs sit about 274 mm
apart. A 9-inch propeller (229 mm) leaves roughly 22 mm of tip clearance per side; a
10-inch propeller (254 mm) would leave under 10 mm, which is why this frame is sold as a
9-inch frame and why 10-inch propellers are not an option on it.

### 3.3 Endurance and range

Assumptions: hover power loading 8.90 g/W; avionics, VTX and BEC losses 18 W; usable pack
fraction 80% for LiPo and 85% for Li-ion.

| Configuration | Mass | Hover | Cruise @ 12 m/s | One-way | Radius | TWR |
| --- | --- | --- | --- | --- | --- | --- |
| **6S 4500 mAh LiPo (specified)** | 640 g | **24 min** | **22 min** | **16.0 km** | **~6.4 km** | 3.11:1 |
| 6S2P 8400 mAh Li-ion | 870 g | 40 min | 37 min | 26.4 km | ~10.6 km | 2.71:1 |

Practical radius is roughly 40% of one-way range, because the return leg must be flown on
the same pack and the energy budget in section 5.2 holds a reserve back for the descent.

**On the pack C-rating.** Peak draw is 51 A on two-blade propellers, or 67 A on three —
**15C on a 4500 mAh pack**. The 45C previously specified was rated at 202 A, three times
anything the aircraft can pull, and cost 40 g for the privilege. A 20C pack is ample and
widens the choice considerably. C-rate is not the constraint on this platform; energy per
gram is.

**On the Li-ion option.** It nearly doubles the range and is genuinely attractive for a
long-range mission, but read the two caveats first. Thrust-to-weight falls to **2.71:1**,
which noticeably reduces authority in wind and in a recovery. And a Molicel P42A 2P pack
derated to a realistic 30 A per cell supplies about 60 A, so a 51 A peak sits at **85%
utilisation** — acceptable for brief punch-outs, but it is not a pack to fly aggressively,
and cold weather makes it worse.

`CRUISE_CURRENT_A` in `config.h` is set to **9.7 A**, which is the cruise figure, not the
8.8 A hover figure. That distinction matters: the constant budgets the charge needed to
fly home at cruise speed, so using hover current would make the return-to-home reserve
optimistic. Revisions 2.2 to 2.4 had it wrong; see section 5.2.

Correct all of these after your own bench measurement.

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

### 4.3 ESC selection and protocol

**Sizing.** Peak draw is 16.9 A per motor at full throttle (section 3.2), and hover sits
near 2.7 A. The specified ESC is **40 A continuous, 50 A burst, per channel**:

| Rating | Margin on 16.9 A peak | |
| --- | --- | --- |
| 30 A/ch | 1.8× | thin once ESC ratings are derated for still air |
| 35 A/ch | 2.1× | workable |
| **40 A/ch** | **2.4×** | **specified** |
| 50 A/ch | 3.0× | what revisions up to 2.3 carried over from the 10-inch build |

Note that 4-in-1 ratings are **per channel**, not for the aircraft. The 2.4× margin is
not timidity: published ESC ratings are typically measured with forced airflow over short
durations, and this is a hover-heavy long-range profile where thermal soak matters more
than burst capability.

At full throttle the pack sees about 67 A on three-blade propellers, or 51 A on the
specified two-blade. The XT90-S is rated 90 A continuous, and a 6S 4500 mAh 20C pack
delivers 90 A:

| | 2-blade (51 A) | 3-blade (67 A) |
| --- | --- | --- |
| XT90-S, 90 A | 57% | 74% |
| Pack, 20C = 90 A | 57% | 74% |
| ESC, 4 x 40 A = 160 A | 32% | 42% |

Dropping the pack from 45C to 20C moved it from a third utilised to roughly two thirds at
peak on three-blade propellers. That is still comfortable headroom for brief punch-outs,
but it is no longer the irrelevance it was — if you fit three-blade propellers and fly
aggressively in the cold, specify 30C instead.

**Protocol.** The firmware drives the ESC with **analog PWM at 400 Hz, 12-bit** via the
ESP32 LEDC peripheral:

- 1000 µs = 1638 counts (disarmed / minimum)
- 1068 µs = 1750 counts (armed idle, propellers turning)
- 2000 µs = 3276 counts (maximum)

Revision 1.0 was internally inconsistent here — the BOM and placement diagram said
"DShot ESC" while the architecture and firmware used 400 Hz PWM. Analog PWM is retained
because it is what the code implements and it is adequate at a 500 Hz loop rate.
**Configure the ESC for PWM input and calibrate its endpoints** during commissioning.

**Why DShot is worth doing later.** The specified ESC supports DShot300 and, on AM32 or
recent BLHeli_32 builds, **bidirectional DShot**, which returns per-motor RPM telemetry.
That is not merely tidier — it would resolve the single largest open uncertainty in this
document. Section 8.3 sets the gyro notch to 100 Hz from two models that disagree by 7%,
because there is no measurement to settle it. With RPM telemetry the notch could **track
the actual shaft speed** rather than sitting at a fixed estimate, which is how modern
flight controllers handle it and which removes the failure mode entirely.

**Revision 3.4 implements the protocol.** `firmware/flight-controller/include/dshot.h`
carries frame construction, both checksum conventions, the GCR telemetry decode, the
eRPM-to-shaft-Hz conversion, and a four-motor aggregator that turns per-motor RPM into
one notch frequency. All of it is pure integer arithmetic and all of it is host-tested —
51 assertions, including a full round trip of every value from 0 to 2047 in both checksum
conventions and of every notch frequency in the build matrix through a wire-level
telemetry word.

**Revision 3.6 added the RMT driver, and 3.7 wired it into the flight loop.** Its
timing arithmetic and symbol encoding are host-tested; the ESP-IDF peripheral calls have
never been compiled, because there is no IDF in the environment they were written in.
§4.3.1 splits the two apart, lists the four assumptions the hardware has to confirm, and
gives the bring-up order. **Nothing here has driven a motor.**

`DSHOT_ENABLE` defaults to **0**, and analog PWM remains the default output. The
`unverified-defaults` check keeps it that way, so enabling it is a deliberate,
reviewable change rather than a line left over from debugging.

#### 4.3.1 The RMT driver

Written in revision 3.6, and split along the line that actually matters — what can be
tested here, and what cannot.

**Tested (`dshot_rmt.h`, 41 assertions).** All of it integer arithmetic, and all of it
the kind of code where the bugs in this subsystem have actually been:

| | |
| --- | --- |
| Clock resolution | 10 MHz, giving 33 ticks per DShot300 bit — within 1% of nominal. 80 MHz divides by 8 exactly, so the divider is integral on any P4 clock source |
| Bit shaping | a `1` holds 25 of 33 ticks, a `0` holds 12. Asserted, along with the fact that a `1` is about twice a `0` — which is the entire discrimination an ESC performs |
| Bitrate range | 150/300/600 all land within 5% at this resolution. **DShot1200 does not** — it leaves under 4 ticks between a `1` and a `0`, so the resolution has to rise with the bitrate. Asserted, so it cannot be selected by accident |
| Symbols | every symbol's halves sum to exactly one bit period, MSB goes first, and a symbol list reconstructs the frame it came from with its checksum intact |
| Inversion | flips levels and nothing else — durations are identical |
| Reply decode | captured run-lengths back to eRPM, tolerant of ±4% ESC clock skew, and rejecting glitches, overruns, truncation and a zero bit period rather than padding them into a plausible RPM |

**Not tested (`dshot_rmt.cpp`).** ESP-IDF API usage and peripheral behaviour. It has not
been compiled — there is no IDF in the environment it was written in. It carries four
numbered assumptions in its own comments, and this is the order to check them:

| # | Assumption | How it fails |
| --- | --- | --- |
| 1 | A TX and an RX channel can share one GPIO | `rmt_new_rx_channel()` returns an error. **This one means the approach is wrong rather than mis-tuned** — bidirectional telemetry would need a different capture mechanism, not different settings |
| 2 | The RX callback signature and its ISR-context restriction are as documented | a crash inside the ISR |
| 3 | The copy encoder emits symbols back to back with no gap | motors do not arm; a gap inside a frame is a missed bit boundary and the ESC rejects the checksum |
| 4 | RX can be armed while TX is driving the same pin, and the ESC's reply is separable from our own echo | telemetry never validates, while throttle works fine |

##### Bring-up, in order

1. **Scope the line with the propellers off and the motors unplugged.** Confirm the bit
   period is 3.3 µs ±5%, that a `1` is visibly about twice the high time of a `0`, and —
   for bidirectional — that the line **idles high**. An inverted-wrong signal is not a
   glitchy one; it is one an ESC ignores completely, which looks identical to a dead pin.
2. **Plug in one ESC, still no propeller.** Confirm it arms and beeps. Step throttle
   from idle to 20% and back.
3. **Check telemetry validates before trusting any number in it.** A reply that fails
   GCR or its checksum is rejected by design, so the symptom of assumption 4 failing is
   silence, not wrong data.
4. **Thrust stand, propellers off**, comparing commanded throttle against reported RPM
   across the range. This is also where `MOTOR_POLE_PAIRS` is confirmed: a wrong pole
   count scales every derived frequency by a constant and presents as a mis-tuned notch
   rather than as the units error it is.
5. **Only then** anything that could spin a blade.

##### Wired into the flight loop in revision 3.7

With `DSHOT_ENABLE=1`, four things change and nothing else does:

| | |
| --- | --- |
| Output | `setMotors()` maps the mixer's PWM counts through `dshotFromPwmCounts()`. **The mixer is untouched** — it still works in the count domain, so §4.2's desaturation arithmetic and every test of it are unaffected |
| Disarm | `idleMotors()` sends `DSHOT_CMD_DISARM`, not minimum throttle. On PWM those are the same 1000 µs signal; on DShot they are 0 and 48 and mean different things. Sending 48 there would leave four motors turning on a disarmed aircraft |
| Telemetry | polled each loop, aggregated across the four motors, and fed to the notch |
| Init failure | **does not fall back to PWM.** The pins have been handed to the RMT peripheral, and a half-initialised output stage driving motors is worse than one that plainly refuses to start |

**The notch now prefers a measurement to a search.** When all four ESCs report recently
and agree closely enough for one notch to cover them, `applyMeasured()` takes the
frequency directly and the sliding DFT is not consulted. Under a hard roll the diagonals
diverge, coherence is lost, and the search takes over — which is the right way round,
because the DFT sees the blend of all four shafts and does not care that they differ.

Gyro samples keep being pushed into the DFT even while telemetry is in use. The harmonic
search still runs on the spectrum, and the transform has to stay primed against a
telemetry dropout mid-flight.

A measured centre is **still clamped to the band and still slewed.** Not because the
measurement is doubted, but because the conversion from it is not: `MOTOR_POLE_PAIRS`
scales every derived frequency, so a wrong pole count yields a confident, precise, wrong
number. Landing far outside the band the models predict is exactly that error's symptom,
and clamping turns it into a bounded error rather than a notch in the wrong place.

The log records which source was used, in `notch_measured`. That needed no format bump:
the bit was always zero in v4, and zero is the *truthful* reading for those logs rather
than merely a safe one, because DShot telemetry did not exist when they were written.

`DSHOT_ENABLE` defaults to **0**, and the `unverified-defaults` consistency check keeps
it there — flipping it has to be a deliberate, reviewable change rather than a line left
over from debugging. With it off, `dshot_rmt.cpp` compiles to an empty translation unit
and the aircraft flies on analog PWM exactly as before.

**Revision 2.9 took the other route to the same goal.** Rather than measuring shaft speed
at the ESC, the flight controller now finds the motor peak in the gyro spectrum directly
and tracks it — see §8.3.1. That removes the fixed-estimate failure mode without new
hardware or new timing-critical code, and it could be tested on the bench. RPM telemetry
would still be better, because it gives the shaft frequency directly rather than inferring
it from the noise, but it is no longer the only thing standing between this design and a
notch that knows where the peak actually is.


#### 4.3.2 Bench procedure: what to wire and what to buy

§4.3.1 lists four assumptions the hardware has to confirm. They are not equally expensive
to test, and they are not in the order you might expect — **the one that could invalidate
the design needs no instrument at all.**

##### Step 0 — the serial console, and nothing else

**Parts: none.** The flight controller, a USB cable, and a terminal.

Build with DShot enabled and read what it says at boot. `DSHOT_ENABLE` is force-set in
`firmware/flight-controller/CMakeLists.txt` rather than passed on the command line, so
that the `unverified-defaults` check can keep it at 0 in `config.h` — an unproven
feature must not become the default by being convenient:

```bash
idf.py set-target esp32p4          # first time only; sdkconfig.defaults does not set it
idf.py build
idf.py -p COM12 flash monitor      # your port
```

`DShotRmt::begin()` reports each failure separately, on purpose. Assumptions 1 to 3 are
answered here:

| What you see | What it means |
| --- | --- |
| `DShot output active (analog PWM is NOT in use)` | assumptions 1–3 hold; go to step 1 |
| `RX channel on GPIO n failed` | **assumption 1 is false.** A pin cannot carry both a TX and an RX channel. Bidirectional telemetry needs a different capture mechanism, not different settings — stop and change the design |
| `TX channel on GPIO n failed` | the resolution or clock source was rejected. `DSHOT_RMT_RESOLUTION_HZ` needs to change, and §4.3.1's table says what it must still satisfy |
| `copy encoder allocation failed` | out of RMT memory; reduce `mem_block_symbols` |

> **STEP 0 IS DONE. Assumptions 1, 2 and 3 hold**, on an ESP32-P4-Function-EV-Board on
> 2026-08-23:
>
> ```
> I (378) dshot: DShot300 on 10 MHz RMT: 33 ticks/bit (9999 ppm error), bidirectional
> DShot output active (analog PWM is NOT in use)
> ```
>
> Four TX and four RX channels allocated on shared GPIOs. **Assumption 1 — that a TX and
> an RX channel can share one pin — is confirmed**, which was the one that could have
> invalidated the bidirectional design rather than merely needed tuning. The 9999 ppm bit
> error is 1%, exactly what the host tests compute from 33 ticks at 10 MHz, and the first
> agreement between this document's arithmetic and the silicon.
>
> **Assumption 4 remains untested.** It needs an ESC, and it is step 2.
>
> Two things had to be fixed to get here, and neither was a DShot problem — see findings
> 42 and 43 below. A third was found before them: `mem_block_symbols` was 64 against a
> chip whose RMT channels hold 48 words, so the first two motors consumed all four TX
> candidates and motor 2 failed with `no free tx channels`. It is now derived from
> `SOC_RMT_MEM_WORDS_PER_CHANNEL`. Note that four TX plus four RX uses the **entire** RMT
> peripheral, 8 of 8: nothing else in this firmware may ever claim an RMT channel.

> **Expect compile errors before you see any of this.** `dshot_rmt.cpp` has never been
> through a compiler. The ESP-IDF RMT API changed between IDF 4 and 5, and the code is
> written against 5. Getting it to build is part of step 0, not a sign anything is wrong.

##### The EV board's on-board ESP32-C6 takes four of these pins

The ESP32-P4 has no radio, so the Function-EV-Board carries an **ESP32-C6-MINI-1** as its
Wi-Fi and Bluetooth co-processor. The two talk over **SDIO**, and that bus is wired to
fixed pins:

| P4 GPIO | SDIO function | This project wants it for |
| --- | --- | --- |
| 14 | D0 | — |
| **15** | **D1** | **`MOTOR4_PIN`** |
| 16 | D2 | — |
| **17** | **D3** | **`PIN_GNSS_RX`** |
| **18** | **CLK** | **`PIN_GNSS_TX`** |
| **19** | **CMD** | **`PIN_VTX_TX`** |
| 54 | C6 reset | — |

**This is a development-board constraint, not a design defect.** The aircraft's flight
controller is an ESP32-P4 module on a carrier with no C6 on SDIO — the project's own C6
is the Remote ID module in §12, attached over UART, not SDIO. On that board GPIO 15, 17,
18 and 19 are free and the pinout in §9 stands.

But on *this* board those four are spoken for, and the consequences for bench work are:

| | |
| --- | --- |
| **Motors 1, 2 and 3** — GPIO 4, 5, 6 | free on header J1, use these |
| **Motor 4** — GPIO 15 | **unavailable.** §4.3.2 only needs one channel, so this does not block the DShot bring-up, but all four cannot be exercised on this board |
| **GNSS** — GPIO 17, 18 | unavailable; remap if bench-testing the GNSS |
| **VTX** — GPIO 19 | unavailable |

If you want all four motor channels on this board, remap `MOTOR4_PIN` to a free J1 pin
for the bench only — do **not** change §9's pinout, which describes the aircraft.

> **A useful side effect.** The board gives you a real ESP32-C6 to experiment with, and
> the C6 is exactly the part §12 specifies for Remote ID. It is wired here as an
> SDIO co-processor rather than the UART-attached module the design uses, so it is not a
> drop-in — but it is a C6 with an antenna, which is enough to develop and range-test the
> ASTM F3411 broadcast against the Android receiver in `android/` long before the real
> module arrives.

##### Step 1 — the signal

**Parts: a USB logic analyser, about $12.** Eight channels at 24 MSa/s is ample — that is
41 ns of resolution against a 3.3 µs bit, so about 79 samples per bit. PulseView (free)
reads it. A scope also works and shows edge quality a logic analyser cannot, but for
*timing* the analyser is the better instrument and a twentieth of the price.

Wire it with **no ESC, no motor and no battery**:

```
  ESP32-P4  GPIO 4  (MOTOR1_PIN) ---------> logic analyser CH0
  ESP32-P4  GND     ----------------------> logic analyser GND
```

One ground, shared. That is the single most common reason a capture looks like noise.

Motors 2 and 3 are **GPIO 5 and 6**, both free. **Motor 4 is GPIO 15, which this board wires to the C6's SDIO bus** — see above. One channel is all step 1 needs.

What you should see, with the aircraft disarmed and `DSHOT_BIDIRECTIONAL` on:

| Measurement | Expected | Why it matters |
| --- | --- | --- |
| Idle level between frames | **HIGH** | bidirectional DShot idles high so the ESC can pull it down to reply. Idling low means the inversion is wrong, and the symptom is an ESC that ignores everything — indistinguishable from a dead pin |
| Bit period | **3.30 µs** ±5% | 33 ticks at 10 MHz |
| A `1` bit | low for **2.50 µs**, high for 0.80 µs | inverted, so the *pulse* is low |
| A `0` bit | low for **1.20 µs**, high for 2.10 µs | a `1` is about twice a `0`; that ratio is the whole discrimination |
| Frame length | **52.8 µs**, 16 bits | |
| Frame interval | **2000 µs** (500 Hz), or 1000 µs on a 7-inch | 2.6% duty |
| Frame content, disarmed | value 0 = `DSHOT_CMD_DISARM` | not 48. If you decode 48 here, `idleMotors()` is sending minimum throttle and motors would turn on a disarmed aircraft |

##### Step 2 — one ESC, no propeller

**Parts you likely already have:** the 4-in-1 ESC and one motor. **Parts worth buying:**

| Item | Approx. | Why |
| --- | --- | --- |
| Bench PSU, 24 V / 5 A, current-limited | $60 | Set the limit to 1 A. A current limit turns a wiring mistake into a shutdown instead of a fire. Far safer than a LiPo for a first power-on |
| *or* an XT60 smoke stopper | $10 | If you would rather use the 6S pack |
| Dupont jumpers, ESC signal lead | $5 | |

```
  ESP32-P4  GPIO 4  --------+-------------> ESC channel 1 signal
                            +-------------> logic analyser CH0
  ESP32-P4  GND     --------+-------------> ESC GND ---> analyser GND
  Bench PSU 24 V ------------------------> ESC power  (current limit 1 A)
  ESC channel 1 ------------------------> motor, NO PROPELLER
```

Tap the signal line, do not break it. Analyser and ESC both see it.

Then:

1. The ESC should arm and beep. If it does not, decode the frame — a checksum the ESC
   rejects usually means the copy encoder inserted a gap, which is assumption 3.
2. Step throttle from idle to about 20%. The motor should turn smoothly.
3. **Look for the reply**, roughly 30 µs after each frame ends: 21 bits at 2.60 µs each,
   about 55 µs long, on the same wire. If the frame is there and the reply never is,
   **assumption 4 is false** — RX cannot be armed while TX drives the pin, and the
   capture needs restructuring.
4. Check the console. A reply that fails GCR or its checksum is rejected by design, so
   the symptom of a marginal capture is *silence*, not wrong numbers.

##### Starting from nothing: the order list

Grouped by which step each item unlocks, because **step 0 needs only the first two
lines** and it is the step that can invalidate the design. If `rmt_new_rx_channel()`
fails there, nothing below the first group matters yet.

| For | Item | What to look for |
| --- | --- | --- |
| **Step 0** | ESP32-P4 development board | **ESP32-P4-Function-EV-Board v3.2** is confirmed available at 285 DKK (~$41) and is the right choice here: every GPIO broken out, USB debug, and a reference design to blame the firmware against rather than the wiring. It is a bench board, **not** a flight board — see §2.2 |
| **Step 0** | USB-C data cable | A charge-only cable is the classic wasted evening |
| **Step 1** | USB logic analyser, 8 channels, 24 MSa/s | The commodity "Saleae-compatible" type. 24 MSa/s is 41 ns against a 3.3 µs bit; more is not useful here. Works with PulseView, which is free |
| **Step 1** | Female-to-female jumper leads | Usually bundled with the analyser |
| **Step 2** | 4-in-1 ESC, 40 A/channel, 6S, BLHeli\_32 or AM32 | The BOM part. Use channel 1 only on the bench. A single 40 A ESC is cheaper if you want bench-only |
| **Step 2** | One motor | 2810 900 KV for the 9-inch. You need four eventually; one is enough here |
| **Step 2** | Bench PSU, 24 V, 3–5 A, adjustable current limit | Set the limit to 1 A. This is the item that turns a wiring mistake into a shutdown rather than a fire, and it stays useful long after this |
| **Step 2** | XT60 pigtails, silicone wire | |

**Not yet:** the flight pack. A 6S LiPo also needs a balance charger and somewhere safe
to store it, and none of that helps on a bench where a current-limited supply is both
safer and more informative.

**Not needed:** a level shifter. The ESP32-P4 drives 3.3 V logic and ESC signal inputs
accept it directly.

**Also required, and free:** **ESP-IDF 5.x**. Not PlatformIO — its stock platform cannot
resolve the P4 at all (§3.2). `dshot_rmt.cpp` is written against the IDF 5 RMT API, which
is also why the port was cheap: the code already targeted the toolchain it moved to.

##### Step 3 — thrust stand, still no propeller

This is where `MOTOR_POLE_PAIRS` is confirmed. Compare commanded throttle against
reported RPM across the range. A wrong pole count scales every derived frequency by a
constant, which presents as a mis-tuned notch rather than as the units error it is.

**Only after all of that does a propeller go on anything.**

##### What to send back

The useful artefact is a PulseView capture — or just the decoded numbers — of one frame
and the gap after it, plus whatever the console printed. The four assumptions are
answerable from that, and the driver can be corrected against what the peripheral
actually did rather than against what its documentation says it should.

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
mAh_req  = I_cruise * 1000 * (t_return / 3600)      I_cruise = 9.7 A
mAh_left = (pack_capacity * 0.80) - mAh_consumed
```

The aircraft is considered able to reach home only if the filtered voltage exceeds
`V_req` **and** at least 120% of `mAh_req` remains.

> **Corrected in revision 2.5.** `CRUISE_CURRENT_A` had been set from *hover* power
> since revision 2.2. The return leg is flown at cruise speed, which costs about 10%
> more, so the aircraft was budgeting slightly less charge for the trip home than it
> actually needs. Now set from the cruise figure.

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

The IMU stack sits on polyurethane gel. The notch centre is **120 Hz**, raised from the
original 80 Hz in two steps as the airframe and then the propellers changed.

**Blade count moves this more than the frame did.** A 2-blade propeller must spin about
21% faster than a 3-blade for the same thrust, because it has less blade area to work
with. That shifts the hover fundamental proportionally:

| Configuration | Hover shaft speed | Fundamental |
| --- | --- | --- |
| 10-inch, 3-blade (original) | ~5300 rpm | ~88 Hz |
| 9-inch, 3-blade | ~6260 rpm | ~104 Hz |
| **9-inch, 2-blade (specified)** | **~7470 rpm** | **~124 Hz** |

Two independent methods now agree to within 3% — momentum theory gives 124 Hz and
scaling the 3-blade figure by √(Ct₃/Ct₂) gives 121 Hz. That is better than the 3-blade
case, where two methods differed by 7%. **120 Hz** is set from them.

**If you fit 3-blade propellers, move this back to about 100 Hz.** At Q = 4 the notch is
−3 dB roughly 15 Hz either side, so a 120 Hz notch does essentially nothing for a 104 Hz
peak. The propeller and the notch are a matched pair; changing one without the other
leaves the filter attenuating empty spectrum.

> **A defect found while building the frame switch, and worth recording.** The
> MPU-6050's anti-alias filter was set to 94 Hz back when the notch was at 80 Hz — correct
> at the time. The notch then moved to 95, 100 and finally 120 Hz as the airframe changed,
> and **the DLPF never moved with it**.
>
> A 94 Hz low-pass sitting *below* a 120 Hz notch is the worst of both: the DLPF
> attenuates the motor peak itself, so the notch filters spectrum that is already gone,
> while the DLPF's phase lag stays in the control band where it costs the D term.
>
> It survived four revisions because nothing compared the two numbers. The build-matrix
> check does now, and the corrected values are 260 Hz for the 7-inch and 184 Hz for the
> 9- and 10-inch — at least 30% above the notch, and below Nyquist for the loop rate.
> This is precisely the coupled-constant failure the switches exist to prevent, which is
> a fair argument that they should have existed sooner.

> **FINDING 34. The measurement procedure this section gave could never have worked.**
> Revisions 1.0 through 2.9 all said: take a BlackBox gyro trace at a stable hover, find
> the peak, set `NOTCH_CENTER_HZ` from it. That instruction is impossible to carry out,
> for two independent reasons, and both have been there since the log format was defined.
>
> **The logged gyro is post-notch.** `BlackBoxRecord.gyroX` is filled from the filtered
> signal, downstream of the very notch the procedure is trying to tune. The peak has
> already been attenuated by the time it reaches the card, so the trace shows the filter
> working, not where the motors are.
>
> **The log rate is below the peak.** BlackBox records at `BLACKBOX_LOG_HZ` = 100 Hz, so
> its Nyquist limit is 50 Hz. Every notch frequency in the build matrix — 88 to 180 Hz —
> is *above* that. The peak does not appear at its own frequency; it aliases:
>
> | Real peak | Appears in a 100 Hz log at |
> | --- | --- |
> | 88 Hz | 12 Hz |
> | 105 Hz | 5 Hz |
> | **120 Hz (default)** | **20 Hz** |
> | 150 Hz | 50 Hz |
> | 180 Hz | 20 Hz |
>
> No post-processing recovers this. The information is destroyed at the sampling step,
> before anything is written. Raising the log rate is not a fix either: covering the
> 7-inch's 180 Hz peak honestly would need better than 400 Hz of three-axis gyro on the
> SD card, and the notch would still be tuned from a trace rather than in flight.
>
> The instruction survived because it *sounds* like standard practice — it is what you do
> on a flight controller that logs raw gyro at 2–8 kHz. This one does not, and nothing
> compared the two numbers.

**Measure it — here is how it actually works.** The aircraft analyses the spectrum on
board, at the full flight-loop rate where the peak is genuinely visible, and records its
conclusion. See §8.3.1 for the tracker and §8.3.2 for the procedure. None of the models above account for the frame
itself: the 6 mm arms on this airframe are less stiff than the 7–8 mm arms originally
specified, which moves the structural resonance independently of the propellers.
Blade-pass energy sits at `PROP_BLADES` times the fundamental — 240 Hz on the 2-blade
default, 300 Hz on a 3-blade — which the notch does not target and, as §8.3.3 shows,
mostly cannot.

#### 8.3.1 The notch now measures itself

Everything above describes how to *estimate* the notch frequency. Revision 2.9 stops
relying on the estimate.

The static value was the one number in this specification where a modelled figure was
actively risky rather than merely approximate, because a notch in the wrong place is
worse than no notch at all — it adds phase lag in the control band while attenuating
nothing. And it was demonstrably fragile: it moved four times as the airframe evolved,
two models of it disagreed by 7%, it spans 88–180 Hz across the ten build combinations,
and when it last moved the IMU anti-alias filter was left behind, producing the defect
recorded above.

Every one of those failures has the same shape — a fixed constant standing in for a
quantity that is not fixed. Hover shaft speed varies with mass, air density, pack
voltage, payload and how hard the aircraft is working. A number chosen at compile time is
wrong the moment any of those changes.

**A sliding DFT over the raw gyro signal finds the peak, and the notch follows it.**

| | |
| --- | --- |
| Transform | Sliding DFT, 128 bins, 64 usable |
| Resolution | 3.9 Hz at a 500 Hz loop, 7.8 Hz at 1000 Hz |
| Update rate | 20 Hz |
| Cost | ~0.5 MFLOP/s, ~3 KB — under 1% of one P4 core |
| Input | **raw** gyro, roll axis, upstream of the filter it tunes |

A sliding DFT rather than an FFT because an FFT does a burst of work every N samples,
which is exactly the periodic spike that ruins a hard real-time loop's worst case. The
SDFT updates every bin by a constant amount on every sample, so the worst case equals the
average case.

The tracker is fed the **raw** gyro deliberately. Feeding it the filtered signal would be
self-defeating: the notch removes the very peak the tracker exists to find, so it would
watch the peak vanish, conclude there was nothing there, and wander off.

One tracker drives all three axes. The peak being hunted is the four motor shafts, and
they are the same four shafts whichever axis you look down.

**It cannot do worse than the constant it replaces.** That is the condition for enabling
it by default on hardware nobody has flown yet, and it is enforced four ways:

- The tracked centre is **clamped** to 0.6×–1.6× the compiled `NOTCH_CENTER_HZ`, further
  capped at the DLPF corner and below Nyquist. A peak outside that band cannot pull the
  notch out with it.
- A peak must be **both** tall enough *and* land in the same bin twice running. The height
  test alone is not sufficient: across a ~30-bin search band, white noise produces a
  peak-to-mean ratio near 4 about half the time. What separates a motor from noise is not
  height but **stability** — a motor peak stays put because shaft speed changes on the
  timescale of the aircraft's mass, while a noise peak wanders. The bin-repeat test is
  what does the real rejecting.
- The centre **slews** at 40 Hz/s rather than jumping, and retunes the biquad without
  clearing its delay line, so a moving peak does not punch a transient through the rate
  loop.
- If tracking does not lock, or the gyro is quiet, the notch sits at **exactly** the
  static value. Analysis only runs while the motors are live, so a spurious bench lock
  cannot be carried into the air.

Disable it with `-DDYN_NOTCH_ENABLE=0` to fall back to the fixed notch.

**What is verified, and what is not.** The tracker is covered by host tests that drive it
with synthetic gyro signals — tone plus broadband noise — at every notch frequency in the
build matrix. It locks to within about 1 Hz at each, resolves frequencies that fall
between bins, refuses to engage on pure noise or silence, and stays inside its band under
out-of-band tones and deliberately absurd input. What that does **not** establish is
behaviour on a real airframe: real gyro noise is not a tone plus white noise, it has
structural resonances, blade-pass energy near twice the fundamental, and frame modes that
none of the models above account for. Section 11.2 still calls for a BlackBox trace, and
`NOTCH_CENTER_HZ` should still be set from it — the tracker narrows the consequences of
getting that number wrong, it does not make it unnecessary.

Bidirectional DShot (§4.3) remains the better long-term answer, because RPM telemetry
gives the shaft frequency directly instead of inferring it from the noise it produces.
This is the version that could be built and tested without hardware.

#### 8.3.2 Reading the measurement off a flight

The tracker's verdict is written to the BlackBox log, so the number it found is
recoverable on the ground. **The verdict is logged, not the spectrum** — that distinction
is what makes it work at all, given the aliasing above. A slowly-moving centre frequency
samples perfectly well at 100 Hz; the waveform it was derived from does not.

Three fields were added in log format **v3**:

| Field | Meaning |
| --- | --- |
| `notch_hz` | the tracked centre, 0.1 Hz resolution |
| `notch_confidence` | peak-to-mean ratio at the moment of the lock |
| `notch_tracking` / `notch_dynamic` | whether it had a confident lock, and whether tracking was compiled in |

The procedure:

1. Hover steadily for 30 seconds or more, in as little wind as you can find. The tracker
   needs the motors doing steady work, not a hover fighting gusts.
2. Pull `flight_NNNN.ody` off the card and run
   `python tools/blackbox_decode.py flight_NNNN.ody`.
3. Read the **Gyro notch** section of the summary. It reports the locked percentage, the
   range and mean of the tracked centre, and the confidence.
4. Set `NOTCH_CENTER_HZ` to the reported mean.

That value is a measurement of your airframe with your propellers at your mass — not a
model. It is what every previous revision of this section was asking for and could not
deliver.

Two things the summary will tell you that are worth acting on:

- **If the tracked centre moved by more than 20 Hz across the flight**, no single fixed
  value covers it. Leave dynamic tracking enabled; that spread is the argument for it.
- **If the tracker was unlocked for most of the flight**, either the real peak is outside
  the 0.6–1.6× search band — in which case the compiled value is badly wrong and worth
  re-deriving from §8.3 — or the airframe is quieter than the confidence threshold
  expects, which is a good problem to have.

Setting `NOTCH_CENTER_HZ` from the measurement still matters even with tracking enabled,
because the compiled value defines the centre of the search band and the fallback when
tracking does not lock.

> **Log format v3 and older logs.** The record grew by four bytes. `tools/blackbox_decode.py`
> reads both v2 and v3, so logs recorded before this change remain readable — a decoder
> that only reads the newest format would silently make every earlier flight unreadable.
> The C struct and the Python format string are cross-checked field by field against the
> real compiler by the `blackbox` consistency check, because a field at the wrong offset
> does not fail loudly, it produces plausible numbers that are wrong.


#### 8.3.3 The second harmonic, and why it usually cannot be notched

A propeller puts energy at twice the shaft fundamental as well as at the fundamental
itself. Tracking that overtone and notching it too is standard on modern flight
controllers, and revision 3.1 implements it. **On this hardware it can only engage in
two of the ten build combinations**, and the reason is worth stating carefully, because
the natural assumption is that a harmonic notch works everywhere.

The gyro reaches the flight controller through the MPU-6050's anti-alias filter at
`IMU_DLPF_HZ`. Twice the fundamental lands **above that corner** in eight of the ten
builds:

| Build | f₀ | 2·f₀ | Ceiling | Observable? |
| --- | --- | --- | --- | --- |
| 7-inch 2-blade | 180 Hz | 360 Hz | 260 Hz | no — 38% above the DLPF |
| 7-inch 3-blade | 150 Hz | 300 Hz | 260 Hz | no — 15% above the DLPF |
| **9-inch 2-blade (default)** | **120 Hz** | **240 Hz** | **184 Hz** | **no — 30% above the DLPF, and 96% of Nyquist** |
| 9-inch 3-blade | 100 Hz | 200 Hz | 184 Hz | no — 9% above the DLPF |
| 10-inch 2-blade | 105 Hz | 210 Hz | 184 Hz | no — 14% above the DLPF |
| **10-inch 3-blade** | **90 Hz** | **180 Hz** | **184 Hz** | **yes** |

The ceiling is the lower of the DLPF corner and 80% of Nyquist for the loop rate.

**Notching an unobservable harmonic would be actively harmful**, not merely useless. It
is the defect of §8.3 with the roles reversed: the DLPF has already attenuated the peak,
so a notch placed there filters spectrum that is largely gone while contributing its own
phase lag in the control band. That is a straight loss to the D term in exchange for
nothing.

So the harmonic notch checks observability **at runtime, against the tracked
fundamental**, not against the compiled one. If the real f₀ turns out lower than the
model assumed, the harmonic may become visible when the compiled value said it would not
— and the reverse. Where it does not clear the ceiling the second notch is never
configured, which `filters.h` treats as a pass-through, so it costs nothing but three
function calls per loop. The firmware says which case a given build is in at boot, and
the log records it per flight in the `harmonic_observable` flag, so a log with no
harmonic notch reads as *"the IMU could not see it"* rather than *"the tracker failed"*.

When it does engage, it searches a ±12% window around 2·f₀ rather than notching at
exactly twice the fundamental. A real overtone is not an exact integer multiple once
blade flex and frame modes are involved, and placing a notch by arithmetic rather than
by measurement is precisely the mistake this subsystem exists to stop repeating.

**Raising `IMU_DLPF_HZ` is not the fix.** The DLPF *is* the anti-alias filter; lifting it
toward Nyquist trades a visible harmonic for aliased content folding into the control
band, which is a worse problem than the one it solves. The real fix is a faster loop —
1000 Hz on the 9- and 10-inch would put Nyquist at 500 Hz and allow a 260 Hz corner,
making every build's harmonic visible. **§8.3.4 costs that change.** The short version is
that it is affordable but wrongly sequenced: the actuator would still run at 400 Hz, and
bidirectional DShot would locate the harmonic exactly rather than making it searchable.

> Blade-pass frequency is `PROP_BLADES × f₀`, so it coincides with the second harmonic
> only on a 2-blade propeller. On the 3-blade builds blade-pass is 3·f₀ — 300 Hz on the
> 9-inch — which is further above the corner still. Earlier revisions of §8.3 described
> blade-pass as "near twice the fundamental", which is true only for the 2-blade case.


#### 8.3.4 Costing a 1000 Hz loop on the 9- and 10-inch

§8.3.3 records a faster loop as the route to making the second harmonic observable.
This is that route costed. **The recommendation is not to take it yet**, and the reason
is not any of the costs below — it is that the loop rate is the wrong lever.

**What it would buy.** At 1000 Hz, Nyquist moves to 500 Hz and the anti-alias corner can
go to 260 Hz, which puts 2·f₀ inside the observable band on every build: 240 Hz on the
9-inch 2-blade, 200 on the 9-inch 3-blade, 210 and 180 on the 10-inch. All ten
configurations would gain a working harmonic notch instead of two.

**What it would not buy.** The ESCs are driven by 400 Hz PWM. A 1000 Hz loop writes
motor values 2.5 times per PWM period, so **60% of them are overwritten before they ever
reach an ESC**. Actuator bandwidth stays at 400 Hz. The faster loop buys *filtering and
estimation* resolution — which is exactly what the harmonic needs — but it buys nothing
for control response, and it must not be described as if it did.

##### The MPU-6050's output rate is not the obstacle

Earlier revisions of this document said the 1 kHz output ceiling left "no margin". That
was imprecise. The output rate is a function of the DLPF setting, not an independent
limit:

| `DLPF_CFG` | Gyro bandwidth | Output rate |
| --- | --- | --- |
| 0 | 256 Hz | **8 kHz** |
| 1 | 188 Hz | 1 kHz |

The 9-inch runs `DLPF_CFG` = 1 today, so the part does output at 1 kHz — and sampling a
free-running 1 kHz source with a free-running 1 kHz loop would beat, which is a real
objection to raising the loop rate *alone*. But a 1000 Hz loop requires moving the DLPF
to 260 Hz anyway, to keep the corner above the notch, and that switches the part to
8 kHz. The two changes have to move together, and once they do the ceiling does not bind.

##### The I²C bus is the real constraint

`mpu.getEvent()` reads 14 bytes — accel, temperature and gyro — every call. At 400 kHz
each byte costs 22.5 µs and each transaction carries 72.5 µs of addressing overhead:

| Loop | Primary IMU read | Per loop | Bus utilisation |
| --- | --- | --- | --- |
| 500 Hz | full, 14 bytes | 388 µs of 2000 µs | 39% |
| **1000 Hz** | **full, 14 bytes** | **388 µs of 1000 µs** | **59%** |
| 1000 Hz | gyro only, 6 bytes | 208 µs of 1000 µs | 41% |

At 59% the bus is not saturated, but the flight loop would spend 39% of every period
blocked on a shared, mutex-guarded bus that also carries the backup IMU, barometer,
magnetometer and ToF sensor. That is the number that would need care.

**The fix is not a faster bus** — 400 kHz is the MPU-6050's maximum. It is to stop
reading the accelerometer at gyro rate. The complementary filter uses accel only for slow
attitude correction; 200–250 Hz is ample. Splitting the read puts a 1000 Hz loop at
41% bus utilisation, essentially the same as the 500 Hz loop costs today.

> **Done in revision 3.3.** `SensorHub::readPrimaryImu()` no longer calls the driver's
> `getEvent()`. It reads the 6-byte gyro block at 0x43 every loop and the 6-byte
> accelerometer block at 0x3B at `IMU_ACCEL_READ_HZ`, default 250 Hz. **Bus utilisation
> at 500 Hz drops from 39% to 30%**, and the precondition for a 1000 Hz loop is in place.
>
> Reading registers directly means the scale factors are ours rather than the driver's,
> so they are asserted against the ranges `initMpu6050()` actually sets — ±8 g at
> 4096 LSB/g and ±500 dps at 65.5 LSB/dps — along with the big-endian two's-complement
> decode, which if reversed would produce plausible noise rather than an obvious failure.
>
> The accelerometer is now a `TimedValue` like every other slower-than-loop sensor, so
> its age is visible rather than implied. Before the first sample arrives the read
> reports failure rather than returning a zero vector, which free-fall detection would
> otherwise read as 0 m/s² — a spurious parachute deployment on the bench.
>
> `IMU_ACCEL_READ_HZ` must divide `FLIGHT_LOOP_HZ` exactly and must leave at least 20
> samples inside `FREEFALL_HOLD_MS`. Both are `static_assert`s: 30 Hz and 300 Hz are
> rejected at compile time, which stops anyone buying bus time by quietly making
> parachute deployment depend on eight samples.

##### Everything else is cheap

| Item | Cost |
| --- | --- |
| SDFT | `DYN_NOTCH_BINS` must double to 256 to hold 3.9 Hz resolution. 0.13 MFLOP/s and 3 KB — still ~1% of a core |
| PID | `dt` halves; gains carry over in principle because the D term is low-passed in seconds, not samples. Needs re-validation, not re-derivation |
| Dividers | 1000 divides exactly by 100 (log), 20 (notch) and 50 (baro) |
| Power | negligible; the core is already running |

##### Scheduling has no slack left, and a dependency nobody declared

The loop period is `pdMS_TO_TICKS(1000 / FLIGHT_LOOP_HZ)`. At 1000 Hz that is **one
FreeRTOS tick**. Any overrun misses the deadline outright rather than eating into margin.

> **FINDING 36, found while costing this and affecting the *existing* 7-inch build.**
> `pdMS_TO_TICKS(ms)` expands to `(ms * configTICK_RATE_HZ) / 1000`. At the ESP-IDF
> default tick rate of 100 Hz, a 1 ms period evaluates to **zero ticks**, and
> `vTaskDelayUntil()` with a zero period does not delay — the flight loop would spin and
> starve everything else pinned to that core.
>
> The 7-inch build has run at 1000 Hz since revision 2.8. It works only because
> Arduino-ESP32 sets the tick to 1000 Hz, and **nothing in this repository said so**.
> A `static_assert` on `configTICK_RATE_HZ` now does, along with one requiring
> `FLIGHT_LOOP_HZ` to divide 1000 exactly — the division is integer, so a rate that does
> not divide cleanly would be silently rounded to a different period than the one `dt`,
> the notch and every divider were computed from.

##### Recommendation

**Do not raise the loop rate for the harmonic alone.** The costs are all manageable, but
the sequencing is wrong:

1. **Bidirectional DShot first** (§4.3). Protocol and telemetry decode landed in
   revision 3.4 and are tested; the RMT driver (§4.3.1) still needs hardware. Until it
   drives motors, the actuator argument for a faster loop does not hold.
2. ~~**Split the IMU read** — gyro at loop rate, accel at 250 Hz.~~ **Done in
   revision 3.3.** Bus utilisation at 500 Hz fell from 39% to 30%.
3. **Then reconsider 1000 Hz**, at which point the actuator can use it and the bus can
   afford it.

##### Available in revision 3.5, but not as a default

`FLIGHT_LOOP_HZ` is now a build switch. `-DFLIGHT_LOOP_HZ=1000` is characterised and
tested on all eight 9- and 10-inch configurations, and three constants move with it
automatically rather than being left to be remembered:

| Constant | 500 Hz | 1000 Hz | Why it has to move |
| --- | --- | --- | --- |
| `IMU_DLPF_HZ` | 184 Hz | **260 Hz** | Nyquist doubles, so the corner can rise — and this is the change that uncovers the harmonic |
| `DYN_NOTCH_BINS` | 128 | **256** | Resolution is loop ÷ bins; without this it would fall from 3.9 Hz to 7.8 Hz |
| every divider | — | — | log, notch-update, accelerometer and barometer rates all stay whole |

**The result is the one §8.3.3 was missing.** At 1000 Hz the second harmonic becomes
observable on **all six** 9- and 10-inch configurations, against two at 500 Hz:

| | 500 Hz | 1000 Hz |
| --- | --- | --- |
| 9-inch, 2·f₀ = 240 Hz | above the 184 Hz corner | **below the 260 Hz corner** |
| 10-inch 2-blade, 210 Hz | above | **below** |
| 10-inch 3-blade, 180 Hz | below | below |

The 7-inch is the exception and stays unobservable at any loop rate: its 2·f₀ is 360 Hz
and the MPU-6050's widest DLPF setting is 260 Hz, so the part itself cannot pass it. That
is a sensor limit, not a configuration one.

**It is still not the default**, for the reason §8.3.4 opens with: the ESCs are driven at
400 Hz, so 60% of motor writes are overwritten before reaching one. Until the DShot
driver of §4.3.1 exists, a 1000 Hz loop buys filtering resolution and pays CPU, bus and
scheduling slack for it. That is a reasonable trade to make deliberately and a poor one
to make by default.

```bash
idf.py build -DFLIGHT_LOOP_HZ=1000                    # 9-inch, faster loop
```

Doing it in the other order buys a harmonic notch on two more airframes, at the cost of
59% bus utilisation, a doubled SDFT, a PID re-validation and zero scheduling slack —
to filter a peak that RPM telemetry would have located exactly.

> **FINDING 41. GPIO 24-27 are the USB Serial/JTAG data lines, and the pinout used
> all four.** The ESP32-P4 routes USB Serial/JTAG to GPIO 24/26 (D−) and 25/27 (D+).
> Revisions up to 4.6 assigned the AUX broadcast bus, the Remote ID health line, the
> parachute servo and CRSF RX to exactly those pins.
>
> One line was enough to make the aircraft unbootable:
>
> ```
> pinMode(PIN_REMOTEID_HEALTH, INPUT_PULLDOWN);   // GPIO 25 = USB D+
> ```
>
> The host loses the USB device, the chip takes `rst:0x17 CHIP_USB_UART_RESET`, and it
> boot-loops forever. Espressif documents this: reconfiguring these pins makes the device
> disappear from the system, recoverable only by forcing download mode by hand.
>
> **This is not a development-board constraint.** It costs the finished aircraft its
> flashing port and its console. Found on hardware day, and only after a long detour —
> a port that opens and returns nothing looks identical to a crashed board, a mis-routed
> console or a bad reader, and all three were investigated first. What settled it was a
> flushed trace marker before each call in `setup()`, which named the offender in one run.
>
> Corrected: the four moved to GPIO 39–42, and the `devboard-pins` check now refuses any
> build that puts a function back on 24–27.

> **FINDING 42. The battery ADC was on a pin that has no ADC.** `PIN_BATT_ADC` was
> GPIO 1. The ESP32-P4's ADC reaches GPIO 16–23 (ADC1) and 49–54 (ADC2) and nothing else.
>
> `analogReadMilliVolts()` on any other pin does not return a poor reading — it takes a
> Load access fault:
>
> ```
> Guru Meditation Error: Core 0 panic'ed (Load access fault)
>   __analogReadMilliVolts  <- SensorHub::serviceSlowSensors  <- TaskTelemetry
> ```
>
> Pack voltage drives the return-to-home threshold and the landing decision, so this
> crashed the aircraft's judgement rather than a displayed number. Moved to **GPIO 51**
> on ADC2, because every free ADC1 pin collides with the GNSS, the VTX, the LiDAR or the
> development board's SDIO bus. The `devboard-pins` check now refuses any build whose
> `PIN_BATT_ADC` cannot reach an ADC.
>
> Found on the first DShot-enabled boot — not by DShot, but because that was the first
> boot that ran far enough to reach the telemetry task.

> **FINDING 43, and it was mine.** Moving `PIN_AUX_BUS_TX` to GPIO 39 to clear the USB
> data lines (finding 41) broke the AUX bus, because `AuxSerial` was `HardwareSerial(5)`
> — the LP-UART, which can only attach to LP-IO pins, GPIO 0–15 on this chip:
>
> ```
> lp_uart_config_io(): Failed to initialize LP_IO 39
> ```
>
> That whole range is already spoken for by the ADC input, the arm button, the motors,
> I²C and the LoRa SPI bus, so there was nowhere to put it. `AuxSerial` moved to
> **UART1** instead. The AUX bus is a one-way broadcast to the beacon and Remote ID
> module while the aircraft is powered, so it gains nothing from a low-power UART, and
> UART1 was free once the console moved to USB.
>
> The lesson is narrower than "check the pins": a pin move is not a local edit when the
> peripheral behind the pin has its own constraints. The check added for finding 41
> verified that 24–27 stayed clear; it did not know the LP-UART existed. It does now.

> **FINDING 44, and it is the direct consequence of finding 43's fix.** Moving
> `AuxSerial` off the LP-UART put it on `HardwareSerial(1)` — and `GnssSerial` has been
> `HardwareSerial(1)` since revision 1.0. Two peripherals, one port.
>
> **Nothing fails when you do this.** There is no error and no console output. The
> second `begin()` simply reconfigures the peripheral, and `sensors.cpp` calls
> `GnssSerial.begin()` *after* `setup()` calls `auxBus.begin()`, so UART1 ends up on the
> GNSS pins and the AUX bus stops existing.
>
> The cost is not the AUX bus itself. It is what hangs off it: the beacon would keep
> broadcasting its last known position instead of the current one — **finding 7 all over
> again** — and the Remote ID module would never be told to broadcast, so
> `PIN_REMOTEID_HEALTH` stays LOW and arming is blocked with `REQUIRE_REMOTE_ID_TO_ARM`
> set, with nothing to explain why.
>
> Moved to **UART0**, which is genuinely free: `ARDUINO_USB_CDC_ON_BOOT=1` puts the
> console on USB Serial/JTAG and nothing references `Serial0`.
>
> The comment I wrote at the time said "UART1 was unused — the console is on USB, which
> frees UART0 and UART1." Half right, stated with the same confidence as the half that
> was wrong. The `uart-allocation` check now compares every `HardwareSerial(n)` in the
> firmware against §9.1's table, so a port cannot be claimed twice or drift from the
> document again.
>
> Found while drawing the schematic in §9.5 — the collision is invisible in a pin list,
> because both objects have their own pins. It is only visible when you draw the
> peripheral behind them.

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

The ESP32-P4 provides five full UARTs plus a low-power UART. Five are used:

| Port | Peripheral | Baud | Notes |
| --- | --- | --- | --- |
| UART0 | AUX broadcast bus | 115200 | TX only, one wire to two modules. Free because the console is USB Serial/JTAG |
| UART1 | Beitian BN-220 GNSS | **115200** | Auto-negotiated — see below |
| UART2 | VTX MSP OSD | 115200 | |
| UART3 | TFmini-S forward LiDAR | 115200 | |
| UART4 | ExpressLRS receiver (CRSF) | 420000 | The manual control link |
| LP-UART | *unused* | — | Reaches only LP-IO pins, GPIO 0–15, all of which are allocated — see finding 43 |

The console is not a UART on this build. `ARDUINO_USB_CDC_ON_BOOT=1` binds Arduino's
`Serial` to USB Serial/JTAG, which is what frees UART0 for the AUX bus. The ROM
bootloader still logs on GPIO 37/38 before `setup()` runs, but GPIO 39 is undriven until
the firmware claims it, so the beacon and Remote ID module never see that traffic.

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
GPIO 28  --> ExpressLRS CRSF TX     [NEW] (ESP32 TX -> receiver RX, handset telemetry)
GPIO 39  --> AUX broadcast bus TX  -> beacon node RX and Remote ID module RX
GPIO 40  --> Remote ID health line (INPUT_PULLDOWN; module drives HIGH while
             broadcasting -- a dead or unconfigured module blocks arming)
GPIO 41  --> Ballistic parachute servo (LEDC 50 Hz, 16-bit)
GPIO 42  --> ExpressLRS CRSF RX     (ESP32 RX <- receiver TX @ 420000)
GPIO 51  --> Battery voltage divider (100k/10k, ADC2_CH2, 12-bit, 8x oversampled)

GPIO 24-27  RESERVED -- USB Serial/JTAG D-/D+. Do not use. See below.
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

### 9.5 Power rails

The BOM's dual step-down BEC gives **5 V at 3 A and 12 V at 3 A**. It does **not** give
3.3 V, and every sensor on this aircraft except the LiDAR wants 3.3 V. That gap is the
whole of this section: 3.3 V has to be made on the flight controller carrier, and how
much of it that carrier must supply is a specification, not a detail.

| Rail | Source | Feeds |
| --- | --- | --- |
| **6S pack** | XT90-S, 22.2 V nominal / 25.2 V full | 4-in-1 ESC only |
| **12 V, 3 A** | BEC | 5.8 GHz VTX, FPV camera |
| **5 V, 3 A** | BEC | FC carrier input, TFmini-S LiDAR, ExpressLRS receiver, BN-220 GNSS, parachute servo |
| **3.3 V** | *the carrier's own regulator* | ESP32-P4, the six I²C sensors, SX1278 LoRa, MicroSD, Remote ID module |
| **1S cell** | Beacon's own battery, diode-OR with 5 V | Beacon node — deliberately survives the main pack |

The split follows the parts, not preference. The VTX and camera are the only things that
want more than 5 V and the only things that radiate; keeping them on their own rail keeps
video switching noise away from the sensors. Everything on 5 V is there because it cannot
run on 3.3 V. Everything else is 3.3 V because the ESP32-P4's GPIO are 3.3 V and are
**not 5 V tolerant**.

#### Four rules that are not negotiable

**1. The entire I²C bus is 3.3 V, including the pull-ups.** Six devices share SDA 7 and
SCL 8. Several of the common breakouts — GY-521 for the MPU-6050, GY-271 for the QMC5883L
— carry their own regulator and will happily accept 5 V, and if you give it to them their
pull-ups sit at 5 V and drive 5 V into the P4's GPIO through the bus. One device powered
from the wrong rail contaminates the whole bus.

**2. The SX1278 is 3.3 V only, on power and on every signal line.** It is not 5 V
tolerant anywhere. There is nothing else on SPI2, so no level shifting is needed — but
there is also nothing to protect it if the rail is wrong.

**3. Anything powered from 5 V that drives a line *into* the P4 must output 3.3 V
logic.** The TFmini-S (LVTTL), the ExpressLRS CRSF line and the BN-220's TX all do. The
VTX's MSP TX line is the one to verify with a meter before connecting it, because MSP
telemetry is optional and a 5 V line here damages a pin that is not optional.

**4. The parachute servo gets its own pair of wires from the BEC.** Not daisy-chained
through the flight controller. A stalled 9 g servo can pull most of an amp, and the
parachute fires at precisely the moment the flight controller must stay alive.

#### What the carrier's 3.3 V regulator has to supply

§2.2 specifies "an ESP32-P4 module on a compact carrier" without naming one. This is the
number that carrier has to meet, beyond whatever the P4 itself draws:

| On 3.3 V | Continuous | Worst-case peak | Note |
| --- | --- | --- | --- |
| SX1278 LoRa | 12 mA (RX) | **120 mA** | +20 dBm TX through PA_BOOST |
| MicroSD | ~5 mA idle | **~100 mA** | Write bursts; cards vary widely |
| VL53L1X | ~20 mA | ~40 mA | While ranging |
| MPU-6050 | 3.9 mA | — | |
| ICM-42688-P | ~0.9 mA | — | |
| BMP280, QMC5883L, INA226 | <2 mA combined | — | |
| **Total** | **~45 mA** | **~270 mA** | Peak assumes LoRa TX, an SD write and a ranging cycle coincide, which nothing prevents |

**Specify the carrier's 3.3 V regulator with at least 300 mA of headroom above the P4's
own demand.** The peak is not hypothetical: the blackbox writes at 100 Hz, the LiDAR
ranges continuously and telemetry transmits on its own schedule. Nothing sequences them.

**Bulk capacitance at the SX1278 is required, not optional** — 470 µF electrolytic plus
100 nF ceramic at the module's own pins. A 120 mA transmit pulse on a shared 3.3 V rail
is the classic way to brown out an MCU, and the failure looks like a random reset rather
than a power problem. The passives are already in the BOM's wiring line.

#### Still unmeasured

The current figures above are datasheet and typical-application values, not measurements
of this aircraft. `CRUISE_CURRENT_A` — which sizes the return-to-home reserve — is still
modelled rather than measured, and that remains the thrust stand's job in §4.3.2 step 3.
The INA226 on the pack measures total draw, not per-rail, so the 5 V and 12 V budgets
here cannot be confirmed in flight without instrumenting the BEC outputs separately.


## 10. Software Architecture

The firmware is a set of buildable projects, not a listing in this document. Four images
share one protocol header. The flight controller builds with **ESP-IDF** because
PlatformIO cannot target the ESP32-P4 (§3.2); the other three remain **PlatformIO**
projects, which suits the C3, C6 and S3 they run on.

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
|   |   |   +-- dynamic_notch.h     Sliding DFT + bounded peak tracker  [8.3.1]
|   |   |   +-- dshot.h             DShot frames, GCR telemetry, eRPM   [4.3]
|   |   |   +-- dshot_rmt.h         RMT bit timing and symbol encoding  [4.3.1]
|   |   |   +-- pid.h               PID with derivative-on-measurement
|   |   |   +-- mixer.h             Desaturating Quad-X mixer      [finding 13]
|   |   |   +-- state_machine.h     Atomic escalate-only states    [finding 6]
|   |   |   +-- sensors.h           Sensor hub with staleness      [findings 14, 15, 16]
|   |   |   +-- navigation.h        RTH + energy budget            [findings 1, 4]
|   |   |   +-- radio_link.h        CRSF + LoRa + AUX bus          [findings 3, 7]
|   |   |   +-- blackbox.h          Ring-buffered SD recorder
|   |   +-- CMakeLists.txt          ESP-IDF build; PlatformIO cannot target the P4
|   |   +-- sdkconfig.defaults      CONFIG_FREERTOS_HZ=1000 -- the loop depends on it
|   |   +-- main/CMakeLists.txt     component wiring
|   |   +-- main/idf_component.yml  pulls arduino-esp32
|   |   +-- components/README.md    vendored Arduino libraries, pinned
|   |   +-- src/
|   |       +-- main.cpp            Tasks, control, arming         [findings 5, 11, 12]
|   |       +-- sensors.cpp         Split IMU read: gyro at loop rate, accel slower
|   |       +-- navigation.cpp
|   |       +-- radio_link.cpp
|   |       +-- blackbox.cpp
|   |       +-- dshot_rmt.cpp       ESP-IDF RMT driver -- NEVER COMPILED, see 4.3.1
|   +-- beacon-node/                ESP32-C3                       [findings 7, 8]
|   +-- remote-id/                  ESP32-C6                       [finding 17]
|   |   +-- src/                    identity.*, odid_transport.*, main.cpp
|   +-- ground-station/             ESP32 + SX1278                 [findings 3, 9, 10]
|       +-- src/                    main.cpp, mavlink_min.h
+-- android/                        Remote ID receiver, for verifying the broadcast
|   +-- app/src/main/java/dk/odyssey/ridtest/
|   |   +-- odid/                   Decoder with NO Android imports, so it host-tests
|   |   +-- scan/                   BLE and Wi-Fi scanners
|   +-- app/src/test/java/...       OdidParserTest
+-- hardware/
|   +-- bom.csv                     9-inch reference build
|   +-- bom-variants.csv            what differs in the other nine
+-- tools/
|   +-- check_consistency.py        The specification-vs-code gate     [27 checks]
|   +-- blackbox_decode.py          Flight log decoder, formats v2-v4
|   +-- test_blackbox_decode.py     ...and its tests
|   +-- patchfile.py                Line-ending-safe in-place edits
|   +-- test_patchfile.py           ...and its tests
|   +-- md2docx.py                  Regenerates the Word documents, reproducibly
|   +-- fetch_arduino_libs.py       Vendors the Arduino libraries as IDF components
|   +-- run_android_parser_tests.sh Decoder tests with plain javac
|   +-- git-hooks/pre-push          Runs the whole gate before every push
|   +-- host_tests/                 Compiles the real headers on a PC
|       +-- test_all.cpp
|       +-- arduino_shim.h
|       +-- Arduino.h
|       +-- sd_shim.h                 Fake MicroSD, so the real blackbox ring runs here
|       +-- SD.h, SPI.h, FS.h         Resolve blackbox.cpp's includes to sd_shim.h
|       +-- run_tests.sh
+-- .github/workflows/host-tests.yml
+-- docs/
```

### 10.2 Task structure

| Core | Task | Rate | Priority | Responsibility |
| --- | --- | --- | --- | --- |
| 1 | `TaskFlightLoop` | 500 Hz¹ | MAX−1 | IMU, attitude, free-fall, rate control, mixing, ESC output |
| 0 | `TaskTelemetry` | 50 Hz | 3 | Sensors, GNSS, arming, energy, state, radios, console |
| 0 | `TaskStorage` | 20 Hz | 1 | Drains the BlackBox ring to the card |

¹ The flight-loop rate follows `FRAME_SIZE_IN` and can be overridden: the 7-inch runs at
1000 Hz by default, the 9- and 10-inch at 500 Hz, and `-DFLIGHT_LOOP_HZ=1000` is
characterised on both. `IMU_DLPF_HZ` and `DYN_NOTCH_BINS` move with it automatically —
see §8.3.4.

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

**Flight controller — ESP-IDF.** A fresh clone or a new git worktree needs two setup
steps first, because `sdkconfig` and the vendored Arduino components are both
gitignored. Skipping them produces two failures in a row that name neither cause: an
unresolved `Adafruit_BusIO`, then a missing `xtensa-esp32-elf-gcc` — the default target,
not the P4.

```bash
cd firmware/flight-controller
python ../../tools/fetch_arduino_libs.py   # vendors the 6 Arduino libraries
idf.py set-target esp32p4                  # sdkconfig.defaults does NOT set the target
idf.py build
idf.py -p COM12 flash monitor              # your port
```

`idf.py set-target` implies a `fullclean` and refuses to run if `build/` exists but is
not a valid CMake tree, so delete `build/` by hand if an earlier failed configure left
one behind.

**Beacon node, Remote ID and ground station — PlatformIO.** These target the ESP32-C3,
C6 and S3, all of which the stock `espressif32` platform has supported for years:

```bash
cd firmware/beacon-node   && pio run -t upload
cd firmware/remote-id     && pio run -t upload
cd firmware/ground-station && pio run -t upload
```

Build them in any order; all four share `shared/odyssey_link.h` by include path, so a
protocol change rebuilds every image.

---


### 10.4 What the host tests do not cover

> **FINDING 40, and the largest gap this project has had.** Eight of the nine `.cpp`
> files in this repository had **never been compiled by anything** — not by CI, not by
> the host tests, not by a developer — until the ESP-IDF toolchain was installed on
> hardware day.

The host suite compiles the real *headers* against a small Arduino shim, and most of this
project's logic genuinely lives in headers: the mixer, the PID, the filters, the state
machine, the sliding DFT, the DShot frame arithmetic, every compile-time assertion in
`config.h`. That is why 387 assertions felt like broad coverage.

But it links exactly one implementation file, `identity.cpp`, because that one has no
Arduino dependencies. Everything else — `main.cpp`, `sensors.cpp`, `navigation.cpp`,
`radio_link.cpp`, `blackbox.cpp`, `dshot_rmt.cpp`, and both Remote ID sources — needs
`Wire`, `Serial`, FreeRTOS and the ESP-IDF drivers, none of which the shim provides. So
they were never built.

**The first real compile found a defect in the first file it reached.** `sensors.cpp`
used `SlewLimitedEma` for the battery voltage filter without including `filters.h`, where
it is declared. Not a subtle error — the kind any compiler catches instantly, sitting
undisturbed in a file that had passed every review and every check this project runs.

| | |
| --- | --- |
| Covered by host tests | headers: `config.h`, `types.h`, `filters.h`, `pid.h`, `mixer.h`, `state_machine.h`, `dynamic_notch.h`, `dshot.h`, `dshot_rmt.h`, plus `identity.cpp` |
| **Not covered** | `main.cpp`, `sensors.cpp`, `navigation.cpp`, `radio_link.cpp`, `blackbox.cpp`, `dshot_rmt.cpp`, `remote-id/main.cpp`, `odid_transport.cpp` |
| Covered now by | `idf.py build`, which is the only thing that compiles them |

**So a green host-test run does not mean the firmware compiles.** It never did. It means
the algorithms are right, which is worth having and is not the same claim. The ESP-IDF
build is now the compile check for the other eight files, and §11.1 asks for it before
any flashing.

This is the same shape as every other finding here — a verification that looked
comprehensive with an unexamined hole in it — but it is the one that had been open
longest and covered the most code.

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

- [ ] **`idf.py build` completes.** This is the only thing that compiles `main.cpp`, `sensors.cpp`, `navigation.cpp`, `radio_link.cpp`, `blackbox.cpp` and `dshot_rmt.cpp` — the host tests do not, and never did. See §10.4.
- [ ] **Weigh the flight-controller board and correct §2.** The BOM carries 9 g, which assumes a module on a compact carrier rather than a development board. This is the least-supported mass in the BOM and it propagates into `AIRFRAME_AUW_G`, the hover point and the return-to-home reserve — see §2.2.
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
- [ ] **Gyro notch measurement.** Hover for 30 s or more in still air, then decode the
      log and read the **Gyro notch** section:

      python tools/blackbox_decode.py flight_0001.ody

      Set `NOTCH_CENTER_HZ` in `config.h` to the reported mean. Do **not** attempt to
      find the peak by spectral analysis of the logged gyro — that trace is post-notch
      and sampled at 100 Hz, well below the peak. See the finding in section 8.3.

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

The original eighteen findings from the revision 1.0 review, with the change that
resolves each. **Eighteen is not the total.** Reworking these surfaced more, and later
work surfaced more again — §13.1 and §13.2 carry those, and several of them were worse
than anything in the original list. The DLPF corner left behind by a moving notch, a
measurement procedure that could never have been carried out, and a flight loop whose
period silently depended on an undeclared FreeRTOS tick rate were all found *after* this
table was complete.

`docs/review-findings-resolution.md` records the original eighteen in detail. The later
ones live here.

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
| 18 | BOM summed to $512.50 against a stated $514.50 | Low | Totals are now computed from `hardware/bom.csv` and checked against §2 on every run, so the two cannot drift apart again. The figure quoted here in revision 2.0 was $540.50; it has moved since as parts were right-sized, which is the point — §2 carries the live total | §2 |

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
| `CRUISE_CURRENT_A` was set from HOVER power since revision 2.2, but it budgets the charge to fly home at CRUISE speed — about 10% more. The RTH energy reserve was therefore optimistic | Corrected to 9.7 A, and §3.3 now states which figure it is and why |
| The pack was rated 45C against a 15C peak demand, carrying 40 g for capability the aircraft cannot use | Specified as 20C minimum. §3.3 explains that energy per gram, not C-rate, is the constraint on this platform |
| 3-blade propellers on a long-range platform with thrust to spare | Changed to 9x5x2. About 10% better hover efficiency for ~12% of peak thrust: 24 min hover and 16 km one-way, up from 22 min and 14.1 km |
| `initVl53l1x()` read the downward rangefinder's model ID through `i2cRead(..., 0x010F, ...)`, whose register parameter is a `uint8_t`. The VL53L1X uses 16-bit register addressing, so 0x010F truncated to 0x0F and addressed the wrong register; the correct two-byte fallback beneath it ran only if that malformed transaction happened to fail. The landing flare and touchdown veto depend on this sensor | Corrected to two-byte addressing, matching `readVl53l1x()`. Found by the compiler on the first build of the file — "changes value from 271 to 15" |
| Eight of the nine `.cpp` files in the repository had never been compiled by anything. The host suite builds the headers, where most of the logic lives, but links only `identity.cpp` — every other implementation file needs Arduino and ESP-IDF, so none was ever built. The first real compile found `sensors.cpp` using `SlewLimitedEma` without including `filters.h` | Recorded in §10.4. `idf.py build` is now the compile check for those files and §11.1 requires it before flashing. A green host run means the algorithms are right, not that the firmware compiles |
| The build command given throughout the specification, `pio run -e odyssey-fc`, cannot work. PlatformIO's stock `espressif32` platform has no `esp32-p4-function-ev-board` — the board `platformio.ini` names — so the build fails resolving the board before reaching a compiler. Every `pio run` line in the document was written without ever being run | **Closed.** ESP-IDF 5.5 chosen over the pioarduino community fork, and every `pio run` line for the flight controller rewritten in §3.2, §4.3.2, §6 and §10.6 — after a build succeeded, as that note promised. `idf.py -D` sets CMake cache variables rather than compile definitions, so `CMakeLists.txt` now forwards the documented switches; verified by building a non-default combination and confirming the binary changed. The other three images are still PlatformIO, correctly |
| `PIN_BATT_ADC` was GPIO 1, which has no ADC on the ESP32-P4 — ADC1 is 16–23 and ADC2 is 49–54. `analogReadMilliVolts()` there takes a Load access fault rather than returning a poor reading, in the telemetry task, and pack voltage drives the return-to-home threshold and the landing decision | Moved to GPIO 51 on ADC2; `devboard-pins` now refuses a `PIN_BATT_ADC` that cannot reach an ADC. §8.3, finding 42 |
| Moving `PIN_AUX_BUS_TX` off the USB data lines put it on GPIO 39, but `AuxSerial` was the LP-UART, which can only attach to LP-IO pins — GPIO 0–15. The AUX broadcast to the beacon and Remote ID module failed to start. Introduced by the fix for finding 41 | `AuxSerial` moved to UART1, free since the console went to USB; the LP-IO constraint is now checked. §8.3, finding 43 |
| The blackbox ring buffer used `volatile` for `head_`, `tail_` and `dropped_` across two CPU cores — `push()` in the flight loop on core 1, `service()` in the storage task on core 0. `volatile` emits no barrier and does not order the non-volatile store of `ring_[head_]` against the volatile store to `head_`, so the consumer could observe an advanced index while the slot still held the previous record. `BlackBoxRecord` is 54 bytes, so the failure is a torn record written silently into the one file that exists to explain a crash | `std::atomic` with paired acquire/release, and a `static_assert` that the atomic is lock-free so no mutex enters the 500 Hz flight loop. `endFlight()` also drained *before* clearing its open flag, stranding every record pushed in between — the last samples before landing — and `service()` had two callers on tasks that preempt each other, sharing one batch buffer. §10.4 |
| Nine of the ten build combinations had no parts list. The BOM covered the 9-inch reference only, while `config.h` happily built 7-inch and 10-inch aircraft whose frame, motors, propellers, battery and ESC were nowhere specified | `hardware/bom-variants.csv` covers all ten, and the `bom-mass` check reconciles every build's parts against the mass model in `config.h`. §2.1 |
| `check_bom()` verified BOM arithmetic and the price quoted in §2, but nothing compared BOM **mass** against `config.h` — `AIRFRAME_AUW_G` sizes thrust-to-weight, hover power and the energy budget, and was free to drift from the parts that produce it | Reconciled for all ten builds, including an explicit `Airborne Mass g` column so the ground-station radio the BOM also buys is not counted as flying mass |
| The bidirectional-DShot GCR decode read its four 5-bit groups from the wrong bit offsets, and most-significant group first. Every legal group still decoded to a legal nibble, so a corrupted RPM would have passed its checksum and been fed to the notch as a measurement | Corrected to read groups low-first at offsets 0/5/10/15. Caught by a full encode/decode round trip, which is why the test builds the wire format rather than trusting the decoder against itself |
| The flight loop's period is `pdMS_TO_TICKS(1000 / FLIGHT_LOOP_HZ)`, which at the ESP-IDF default 100 Hz tick rounds to ZERO ticks — `vTaskDelayUntil` would not delay and the loop would spin, starving its core. The 1000 Hz 7-inch build has depended on Arduino-ESP32's 1000 Hz tick since revision 2.8 with nothing declaring it | `static_assert` on `configTICK_RATE_HZ`, plus one requiring `FLIGHT_LOOP_HZ` to divide 1000 exactly. Host tests check every divider derived from the loop rate. §8.3.4 |
| Second-harmonic notching was proposed on the assumption that the SDFT already computes those bins, so tracking the overtone would be a modest addition. The bins exist, but in 8 of 10 builds 2·f₀ lands above the MPU-6050 anti-alias corner, so the IMU has already attenuated it | Implemented, but gated on observability computed at runtime from the tracked fundamental. Where 2·f₀ does not clear the corner the notch never engages, because notching an already-filtered peak buys phase lag for nothing. §8.3.3 |
| Every revision instructed the reader to find the motor peak in a BlackBox gyro trace. That was impossible twice over: the logged gyro is post-notch, and the 100 Hz log rate puts its Nyquist limit at 50 Hz while the peak is 88–180 Hz, so a 120 Hz peak aliases to 20 Hz | Instruction withdrawn. The spectrum is analysed on board at the full loop rate and the tracker's verdict is logged in format v3, so the measurement is recoverable on the ground. §8.3, §8.3.2 |
| The gyro notch was a compile-time constant standing in for a quantity that varies with mass, air density, pack voltage and workload — it moved four times, two models of it disagreed by 7%, and when it last moved the IMU DLPF was left behind | A sliding DFT over the raw gyro now tracks the real peak at 20 Hz, bounded to 0.6–1.6× the compiled value so it cannot do worse than the constant it replaced. §8.3.1 |
| The IMU anti-alias filter was left at 94 Hz while the gyro notch moved from 80 Hz to 120 Hz across four revisions, so the DLPF sat BELOW the notch — attenuating the peak the notch was aimed at, while keeping its phase lag in the control band | Corrected to 260 Hz (7-inch) and 184 Hz (9- and 10-inch), with an assertion requiring 30% clearance above the notch. Found by the build-matrix check in §3.2 |
| The ESC was rated 50 A per channel against a 16.9 A peak — nearly 3× margin, carried over from the 10-inch build along with 9 g | Right-sized to 40 A/ch (2.4×). §4.3 now shows the sizing arithmetic and records bidirectional DShot as the route to an RPM-tracked notch |
| Motors were a 3110 (31 mm stator, a 10-11 inch motor) on a 9-inch frame — over-sized and carrying 64 g the airframe did not need | Changed to the 2810 class (28 mm stator, 8-9 inch). The 900 KV winding was already correct, so only the stator changed. AUW 1705 → 1641 g, TWR 3.05 → 3.41:1 |
| Airframe changed to a 387 mm 9-inch frame after revision 2.1, invalidating the thrust, AUW, TWR, endurance and gyro-notch chain | Recomputed throughout in revision 2.2. The notch moved 80 Hz → 95 Hz, without which the new ~97 Hz motor peak would have passed into the rate controller |
| The Basic ID message hard-coded `SERIAL_NUMBER`, forcing the CTA-2063-A route and an ICAO manufacturer code application that a privately built aircraft does not need | `UAS_ID_TYPE` added, defaulting to `CAA_REGISTRATION_ID`. `odyValidateCaaRegistration()` and 10 host assertions added; §12.3 documents both routes |

---

*End of specification, revision 2.0.*
