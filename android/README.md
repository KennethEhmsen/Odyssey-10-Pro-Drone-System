# Odyssey Remote ID test receiver

An Android app that receives ASTM F3411 Remote ID broadcasts and tells you whether the
aircraft is transmitting **what you configured, at the rate the standard requires**.

It is a bench instrument, not a viewer. Seeing an aircraft appear on a map proves
something is transmitting; it does not prove the identifiers are right, the rate is met,
or the position is plausible. Those are the things worth checking before a first flight.

```
Odyssey RID Test
[Expected UAS ID          ]
[Expected operator ID     ]
[ Start scan ][ Self-test ][ Clear ][ Share ]

================================================================
DNK87astrdge12k8  [BLUETOOTH AA:BB:CC:DD:EE:FF]
RSSI -54 dBm   seen for 47 s
ID type: CAA registration
Operator: DNK87astrdge12k8
Status: airborne
Position: 55.676098, 12.568337  (<10 m)
Height: 42.5 m above take-off
Speed: 12.0 m/s  heading 90 deg
Counts: basic 16, location 47, system 16, operator 16

-- conformance ------------------------------------------------
  Location rate >= 1 Hz         PASS    worst gap 1.1 s over 47 s
  Static msgs <= 3 s            PASS    worst gap 3.0 s
  Basic ID present              PASS    16 received
  Location present              PASS    47 received
  System present                PASS    16 received
  Operator ID present           PASS    16 received
  UAS ID matches                PASS    DNK87astrdge12k8
  Operator secret withheld      PASS    no secret suffix in the broadcast
  Position plausible            PASS    55.67610, 12.56834
  EU class declaration          PASS    undeclared -- correct for a privately built aircraft
  Decode errors                 PASS    none
  Message counter advances      PASS    counter incrementing

  ALL OBSERVABLE CHECKS PASSED
```

---

## What it checks

| Check | Why it matters |
| --- | --- |
| Location at ≥ 1 Hz | The standard's minimum for the dynamic message |
| Static messages ≤ 3 s | Basic ID, System and Operator ID cadence |
| All four message types present | A broadcast missing Operator ID is incomplete |
| UAS ID matches what you configured | Catches a stale flash or the wrong module |
| **Operator secret withheld** | The registration's last 3 characters are secret. If a hyphen appears in the broadcast, the firmware is transmitting them |
| Position plausible | Rejects 0,0 — the exact failure that was finding 7 |
| EU class declaration | A privately built aircraft should declare `undeclared`, not a C-class it was never assessed against |
| Message counter advances | A stuck counter hides packet loss |

These are **receiver-observable** checks. They are not an EU 2019/945 Part 6 conformity
assessment, which requires tamper resistance and product conformity that no receiver can
see. See specification section 12.4.

---

## Building

Needs the Android SDK and Gradle. Both ship with Android Studio.

```bash
cd android && gradle assembleDebug
```

The APK lands in `app/build/outputs/apk/debug/app-debug.apk`. Install with
`adb install -r app/build/outputs/apk/debug/app-debug.apk`.

There are **no external dependencies** — not AndroidX, not a chart library, nothing. For
a diagnostic tool that is deliberate: it has to still build years from now on whatever
machine is to hand, and every dependency is a future reason it will not.

## Testing the decoder without any of that

```bash
sh tools/run_android_parser_tests.sh
```

80 assertions, about a second, needs only a JDK. The decoder is dependency-free Java in
`odid/` with no Android imports precisely so this is possible; it runs in the pre-push
hook and in CI.

---

## Requirements on the handset

| Requirement | Why |
| --- | --- |
| **Android 8.0 (API 26) minimum** | Extended advertising and Coded PHY scanning |
| **Bluetooth 5 Long Range support** | The aircraft transmits on the Coded PHY. A handset that cannot scan extended advertisements **will never see it**, and that looks identical to the aircraft being switched off — the app warns you when it detects this |
| Android 11 (API 30) for Wi-Fi | Reading vendor information elements needs `getInformationElements()` |
| Location permission | Required for BLE scanning below API 31, and for Wi-Fi scan results always |

**Wi-Fi is the weaker path.** Android throttles Wi-Fi scans to a few per minute, so a
1 Hz broadcast cannot be sampled anywhere near its true rate — **judge the rate checks
over Bluetooth**. Wi-Fi is included because some regulator-supplied receivers use it, so
confirming presence there still has value.

---

## Run the self-test first

The Odyssey firmware **broadcasts nothing at all** when its identifiers are invalid,
deliberately: an untraceable identifier in a receiver's log is worse than silence.

That makes "I see nothing" ambiguous — silent aircraft, denied permission, unsupported
handset, or broken decoder. The **Self-test** button runs a known vector through the
decoder and rules out the last of those before you start chasing the others.

---

## On trusting the decoder

The aircraft encodes with `opendroneid/opendroneid-core-c`, the reference
implementation, because the ASTM field packing is bit-exact and easy to get subtly
wrong. This decoder is hand-written from the documented structure, so it carries the
opposite risk.

That is the point. The encoder is trusted; the decoder is not. **When this app decodes a
real broadcast from the aircraft and the identifiers and position come out matching what
was configured, both sides are cross-validated** — an independent implementation agreeing
with the reference one is far stronger evidence than either alone.

Until that has happened on real hardware, treat a decode failure as "the decoder might be
wrong" rather than "the aircraft is wrong".

---

## Layout

```
android/
+-- settings.gradle, build.gradle, gradle.properties
+-- app/
    +-- build.gradle
    +-- src/main/
    |   +-- AndroidManifest.xml
    |   +-- res/                     layout and strings, framework widgets only
    |   +-- java/dk/odyssey/ridtest/
    |       +-- odid/                NO ANDROID IMPORTS -- runs under a plain JVM
    |       |   +-- OdidMessage.java      decoded message types
    |       |   +-- OdidParser.java       the decoder
    |       |   +-- OdidEncoder.java      for round-trip tests and the self-test
    |       |   +-- OdidConformance.java  rate and identifier observations
    |       |   +-- OdidSelfTest.java     on-device known-vector check
    |       +-- scan/
    |       |   +-- BleScanner.java       extended advertising, Coded PHY
    |       |   +-- WifiScanner.java      vendor IE, API 30+
    |       +-- AircraftStore.java        accumulation and report rendering
    |       +-- MainActivity.java         one screen, plain widgets
    +-- src/test/java/.../OdidParserTest.java    80 assertions, plain main()
```
