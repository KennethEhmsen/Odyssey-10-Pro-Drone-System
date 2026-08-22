# Odyssey-10 Pro

[![host-tests](https://github.com/KennethEhmsen/Odyssey-10-Pro-Drone-System/actions/workflows/host-tests.yml/badge.svg?branch=main)](https://github.com/KennethEhmsen/Odyssey-10-Pro-Drone-System/actions/workflows/host-tests.yml)
[![assertions](https://img.shields.io/badge/host_assertions-387-blue)](tools/host_tests/)
[![consistency](https://img.shields.io/badge/consistency_checks-25-blue)](tools/check_consistency.py)
[![decoder](https://img.shields.io/badge/RID_decoder_tests-80-blue)](android/)
[![license](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![platform](https://img.shields.io/badge/platform-ESP32--P4%20%7C%20C3%20%7C%20C6-lightgrey)](firmware/)

Autonomous long-range 9-inch quadcopter. ESP32-P4 dual-core RISC-V avionics, integrated
perception, kinetic recovery and safety stack.

**Status:** revision 4.4 — DShot bench procedure ready; the driver awaits its first contact with hardware. All eighteen defects found in the
revision 1.0 review are fixed — and eighteen was not the end of it. Reworking them
surfaced more, and the work since has surfaced more again, several worse than
anything in the original list. See
[`docs/review-findings-resolution.md`](docs/review-findings-resolution.md) for the
original eighteen, or sections 13.1 and 13.2 of the specification for everything
found afterwards.

> The badge reflects `tools/check_consistency.py` plus the host test suite, which compile
> the real firmware headers on a PC. It does **not** mean the aircraft has flown. Nothing
> here has been on hardware; see [Safety](#safety).

---

## What is here

| Path | Contents |
| --- | --- |
| `docs/Odyssey-10-Pro-Drone-System.md` | The engineering master specification |
| `docs/Odyssey-10-Pro-Drone-System.docx` | The same document as Word, generated from the Markdown |
| `docs/review-findings-resolution.md` | Every review finding mapped to its fix |
| `docs/remote-id-regulatory-notes.md` | Direct Remote ID reference notes for a privately built aircraft in Denmark/EU (`.docx` alongside) |
| `docs/Odyssey-10-Pro-Drone-System.ORIGINAL.md` | Revision 1.0, kept for reference |
| `shared/odyssey_link.h` | Wire protocol shared by all four firmware images |
| `firmware/flight-controller/` | ESP32-P4 flight controller |
| `firmware/beacon-node/` | ESP32-C3 emergency locator beacon |
| `firmware/remote-id/` | ESP32-C6 ASTM F3411 Remote ID broadcaster |
| `firmware/ground-station/` | ESP32 + SX1278 telemetry bridge and command transmitter |
| `hardware/bom.csv` | Bill of materials — the authoritative source for costs and masses |
| `tools/blackbox_decode.py` | Decodes flight logs to CSV, prints a flight summary |
| `tools/md2docx.py` | Regenerates the Word documents from the Markdown, byte-reproducibly |
| `tools/test_blackbox_decode.py` | Verifies the BlackBox decoder scales and flags every field correctly |
| `hardware/bom-variants.csv` | Parts that differ across the ten build combinations |
| `tools/patchfile.py` | In-place source edits that preserve line endings, refuse ambiguous anchors and reject shell-mangled content |
| `tools/test_patchfile.py` | Tests for the above, including the whole-file anchor bug that motivated it |
| `tools/host_tests/` | Compiles the real firmware headers on a PC and verifies the safety-critical algorithms |
| `tools/check_consistency.py` | Checks the specification, firmware and BOM still agree; `--fix` repairs the mechanical ones |
| `android/` | Remote ID test receiver for Android — verifies the aircraft broadcasts what you configured, at the required rate |

---

## Building the firmware

Each firmware directory is an independent [PlatformIO](https://platformio.org) project.
They share `shared/odyssey_link.h` by include path, so a protocol change rebuilds all
four.

```bash
cd firmware/flight-controller && pio run -t upload
```

```bash
cd firmware/beacon-node && pio run -t upload
```

```bash
cd firmware/remote-id && pio run -t upload
```

```bash
cd firmware/ground-station && pio run -t upload
```

### Before you build

Three things to settle before your first flight:

1. **`firmware/flight-controller/include/config.h`** — decide whether you need Remote ID.

   `REQUIRE_REMOTE_ID_TO_ARM` defaults to **0**, meaning Remote ID is optional and a
   missing or unconfigured module will **not** stop the aircraft arming. That default is
   correct for this airframe as specified: a privately built aircraft under 25 kg flown
   in the EU open category A3 is not class-marked, and the Direct Remote ID obligation
   attaches to class-marked C1/C2/C3 aircraft.

   Set it to **1** if you fly in the **specific category**, your aircraft is
   class-marked, you are in the **United States** (14 CFR Part 89 applies above 250 g
   including to home builds), or your authority requires it for any other reason. See
   specification section 12.1.

2. **`firmware/flight-controller/include/config.h`** — set `FRAME_SIZE_IN`,
   `MOTOR_CLASS` and `PROP_BLADES` to match the hardware you are fitting. They default to
   **9-inch + 2810 + 2-blade** and together drive every coupled constant: mass, thrust,
   loop rate, IMU filter corner, gyro notch, PID gain scale, pack capacity, and the
   cruise current that sizes the return-to-home reserve.

   ```bash
   pio run -- -DFRAME_SIZE_IN=10 -DMOTOR_CLASS=MOTOR_3115 -DPROP_BLADES=3
   ```

   Ten combinations across 7, 9 and 10 inch are characterised and tested; anything else
   fails the build with a specific message. The firmware prints which one it is at boot.

   Only the 9-inch default is backed by a real BOM and the document's analysis — the
   others are modelled starting points. See specification section 3.2.

   Then confirm `CELL_COUNT` matches your
   pack. Every battery threshold derives from it. Getting this wrong is what finding 1
   was, and it is the difference between a working failsafe and an aircraft that flies
   until the pack collapses.

3. **If you are fitting the Remote ID module**, set `UAS_CAA_REGISTRATION` in
   `firmware/remote-id/src/main.cpp` to the registration issued by your civil aviation
   authority, then provision your operator registration at runtime over the serial
   console:

   ```
   SETOPERATOR DNKxxxxxxxxxxxxx-yyy
   ```

   **You do not need an ICAO manufacturer code.** `UAS_ID_TYPE` defaults to
   `ODY_UAS_ID_CAA_REGISTRATION`, which broadcasts your CAA registration and involves
   ICAO not at all. The alternative, `ODY_UAS_ID_CTA_SERIAL`, requires a 4-character
   code that ICAO issues to *manufacturers* — building one aircraft for yourself is not
   that, and a bought Remote ID module already carries its own serial from whoever made
   it.

   The operator ID is **not** compiled in, deliberately: the three characters after the
   hyphen are secret, and anything committed to a repository stays in its history. Until
   both identifiers are valid the module broadcasts nothing at all — an untraceable
   identifier in a receiver's log is worse than silence.

Also check `VOLTAGE_DIVIDER_TRIM` against a multimeter and `CRUISE_CURRENT_A` against a
real thrust-stand measurement, both described in section 11 of the specification.

---

## Reading a flight log

Pull `/flight_NNNN.ody` off the MicroSD card and run:

```bash
python tools/blackbox_decode.py flight_0001.ody
```

That prints a summary — state timeline, per-cell voltage range, sample loss, mixer
saturation, per-sensor availability — and writes a CSV alongside the log.

The mixer saturation figure is worth reading every time. Anything sustained above 30%
means the airframe ran out of control authority in flight.

---

## Running the tests

The mixer, link protocol, state machine, filters and PID are pure computation and are
verified on the host against the **real firmware headers** — no ESP32, no hardware:

```bash
sh tools/host_tests/run_tests.sh
```

98 assertions, each naming the review finding it guards. Several reproduce the original
defect next to the fix, so the difference is demonstrated rather than claimed.

### Automatically, before every push

```bash
git config core.hooksPath tools/git-hooks
```

The `pre-push` hook runs the suite and aborts the push if anything fails, but only when
something under `firmware/`, `shared/` or `tools/host_tests/` actually changed. This
costs nothing and needs no network. Bypass a single push with `git push --no-verify`.

### Remote ID decoder

```bash
sh tools/run_android_parser_tests.sh
```

80 assertions in about a second, needing only a JDK — no Gradle, no Android SDK, no
network. The decoder in `android/app/src/main/java/dk/odyssey/ridtest/odid/` is
dependency-free Java with no Android imports specifically so this is possible.

### Consistency

Most defects in this project's review were *disagreements* — the specification saying
115200 baud while the firmware opened 9600, the motor diagram contradicting the pinout,
the BOM total not matching its own line items. Each was obvious alone and invisible
together, because nothing checked that the parts still agreed.

```bash
python tools/check_consistency.py --fix
```

> **Editing these files programmatically?** Write the script to a file and run it by
> path. A script piped to `python` through a shell heredoc loses one level of backslash
> escaping before the shell sees it, and quoting does not help — even a `<<'EOF'`
> heredoc, which the shell guarantees is literal, arrives mangled. A `\\b` intended as
> backslash-b becomes `\b`, which Python reads as a **backspace character**, and that
> gets written into the source invisibly. It happened to `check_consistency.py` and
> needed a restore from git. `tools/patchfile.py` now refuses control characters, and
> the `line-endings` check scans every tracked file for them.

25 checks. `--fix` repairs whitespace, BOM totals and any specification constant that has
drifted from `config.h` — the code is the source of truth, and the fixer rewrites the
documentation to match, never the reverse. It runs in the pre-push hook on every push,
including documentation-only ones.

Two of the checks cover different failure modes. `spec-constants` catches values in table
cells; `prose-constants` catches values asserted in sentences and formula blocks, which is
where a real defect hid for three revisions — `CRUISE_CURRENT_A` was set from hover power
instead of cruise power, making the return-to-home reserve optimistic, while the prose
kept quoting a figure that no longer matched the constant.

### In CI

`.github/workflows/host-tests.yml` runs the consistency check and the host suite on
every push and pull request.

This repository is public, so GitHub-hosted standard runners are **free and unlimited** —
there is no quota to exhaust and no billing to configure. The workflow keeps its cost
controls anyway (Linux-only runner, no schedule, no matrix, cancel-in-progress, path
filters, a 5-minute timeout, and a guard step that fails if any of those are removed),
because they cost nothing to keep and would matter again if the repository were ever
made private.

Note that `docs/` and `hardware/` are in the trigger paths. The consistency check
compares the specification and the BOM against the firmware, so a documentation edit
genuinely can break the build — which is the entire point of it.

---

## Regenerating the Word document

```bash
python tools/md2docx.py docs/Odyssey-10-Pro-Drone-System.md docs/Odyssey-10-Pro-Drone-System.docx --subtitle "Engineering Master Specification"
```

Requires `python-docx`. The Markdown is the source of truth; do not edit the `.docx`
directly, because the next regeneration will overwrite it.

---

## Remote ID test receiver

`android/` holds an Android app that receives the aircraft's Remote ID broadcast and
checks it against what you configured — identifiers, 1 Hz location rate, 3 s static
message cadence, position plausibility, and whether the operator secret is leaking on
air. Build it with `cd android && gradle assembleDebug`, or read
[android/README.md](android/README.md).

Needs a handset that can scan **Bluetooth 5 extended advertisements on the Coded PHY**.
One that cannot will never see the aircraft, and that looks exactly like the aircraft
being switched off — the app detects and warns about this.

---

## Architecture at a glance

```
                        ExpressLRS 2.4 GHz  ----> manual control (CRSF, UART4)
                                                          |
   433 MHz LoRa  <----> telemetry 1 Hz / commands  --->  ESP32-P4
                                                          |  core 1: 500 Hz flight loop
                                                          |  core 0: 50 Hz telemetry
                                                          |  core 0: SD storage
                                                          |
                                       AUX broadcast bus (LP-UART, TX only)
                                                          |
                                         +----------------+----------------+
                                         |                                 |
                                  ESP32-C3 beacon                  ESP32-C6 Remote ID
                                  (position cached,                (ASTM F3411 over
                                   1S cell, 72 h)                   BLE 5 + Wi-Fi)
```

Three radios, three jobs, because no single link does all three well. The reasoning is
in section 5.4 of the specification — briefly, a 433 MHz LoRa frame costs 51.5 ms of
airtime, so the "LoRa RC link" the original design described would have needed more than
100% of the channel.

---

## Safety

This is a 1.58 kg aircraft with four 9-inch propellers and a 6S pack. It can hurt
someone.

- Complete the commissioning checklist in section 11 before the first flight. It is not
  decorative; several items exist specifically because the corresponding failure was
  found in review.
- Never spin the motors with propellers fitted during bench testing.
- Verify Remote ID is broadcasting your real registration before flying.
- Check your local rules for 433 MHz at +20 dBm — see section 12.2. It is not legal
  everywhere, and the pin-compatible Ra-02H (868/915 MHz) may be the right part for your
  region.

---

## License

MIT. See [LICENSE](LICENSE).
