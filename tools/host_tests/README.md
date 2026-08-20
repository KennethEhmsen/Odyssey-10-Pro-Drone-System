# Host verification tests

The safety-critical algorithms in this project are pure computation: the desaturating
mixer, the link framing and CRC, the escalation-ordered state machine, the notch filter,
the PID, the threshold debounce. None of them needs an ESP32 to verify, and "it compiles
for the target" is not evidence that any of them is correct.

These tests compile the **real firmware headers** — not copies — against a small Arduino
shim and exercise them on the host.

```bash
./run_tests.sh
```

Requires a C++17 compiler. Nothing else.

## What is covered

Each test names the review finding it guards, and several of them reproduce the original
defect alongside the fix so the difference is visible rather than asserted:

| Test | Guards |
| --- | --- |
| `testBatteryThresholds` | Finding 1 — thresholds are 6S values and are actually reachable |
| `testMixerGeometry` | Finding 2 — diagonal yaw pairs, side/front-rear pairing, columns sum to zero |
| `testMixerDesaturation` | Finding 13 — reproduces the original authority loss, then sweeps 1189 throttle × roll combinations proving no commanded moment is ever attenuated or reversed |
| `testMixerYawSacrifice` | Finding 13 — uniform scaling preserves the roll:pitch ratio under overload |
| `testStateEscalation` | Finding 6 — a late `FAILSAFE_LANDING` cannot displace `FREEFALL_PARACHUTE` |
| `testMotorsLivePredicate` | Finding 10 — every airborne state is covered; the original predicate is shown to miss three |
| `testLinkFraming` | Findings 3, 5 — all 144 single-bit corruptions rejected, plus truncation, bad magic, version and replay |
| `testNotchFilter` | An unconfigured filter passes the gyro through instead of returning zero |
| `testPid` | Derivative-on-measurement (no step spike), anti-windup, pathological `dt` |
| `testDebounce` | A transient sag cannot latch an irreversible mode change |

## The shim

`arduino_shim.h` provides just enough of the Arduino and FreeRTOS surface for the
firmware headers to compile under g++. It is a test fixture; nothing in `firmware/`
includes it. `millis()` is driven explicitly by the tests rather than by a real clock,
so timing behaviour is deterministic.

The spinlock is a no-op on the host. That is deliberate and worth being clear about:
these tests verify the escalation **ordering**, which is the property that makes the
cross-core race harmless. They do not and cannot verify the locking primitive itself.
