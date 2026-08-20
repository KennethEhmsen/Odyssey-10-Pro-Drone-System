# Odyssey-10 Pro

Autonomous long-range 10-inch quadcopter. ESP32-P4 dual-core RISC-V avionics, integrated
perception, kinetic recovery and safety stack.

**Status:** revision 2.0 — engineering review applied. All eighteen defects found in the
revision 1.0 review are fixed. See [`docs/review-findings-resolution.md`](docs/review-findings-resolution.md)
for the finding-by-finding index, or section 13 of the specification.

---

## What is here

| Path | Contents |
| --- | --- |
| `docs/Odyssey-10-Pro-Drone-System.md` | The engineering master specification |
| `docs/Odyssey-10-Pro-Drone-System.docx` | The same document as Word, generated from the Markdown |
| `docs/review-findings-resolution.md` | Every review finding mapped to its fix |
| `docs/Odyssey-10-Pro-Drone-System.ORIGINAL.md` | Revision 1.0, kept for reference |
| `shared/odyssey_link.h` | Wire protocol shared by all four firmware images |
| `firmware/flight-controller/` | ESP32-P4 flight controller |
| `firmware/beacon-node/` | ESP32-C3 emergency locator beacon |
| `firmware/remote-id/` | ESP32-C6 ASTM F3411 Remote ID broadcaster |
| `firmware/ground-station/` | ESP32 + SX1278 telemetry bridge and command transmitter |
| `hardware/bom.csv` | Bill of materials — the authoritative source for costs and masses |
| `tools/blackbox_decode.py` | Decodes flight logs to CSV, prints a flight summary |
| `tools/md2docx.py` | Regenerates the Word document from the Markdown |
| `tools/host_tests/` | Compiles the real firmware headers on a PC and verifies the safety-critical algorithms |

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

Two things must be set for your own aircraft, and the firmware will not let you fly
without them:

1. **`firmware/remote-id/src/main.cpp`** — set `UAS_SERIAL_NUMBER` and `OPERATOR_ID` to
   the values registered with your civil aviation authority. Until you do, the module
   holds its health line low and the flight controller refuses to arm. Broadcasting a
   placeholder Remote ID is itself a regulatory violation.

2. **`firmware/flight-controller/include/config.h`** — confirm `CELL_COUNT` matches your
   pack. Every battery threshold derives from it. Getting this wrong is what finding 1
   was, and it is the difference between a working failsafe and an aircraft that flies
   until the pack collapses.

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

---

## Regenerating the Word document

```bash
python tools/md2docx.py docs/Odyssey-10-Pro-Drone-System.md docs/Odyssey-10-Pro-Drone-System.docx --subtitle "Engineering Master Specification"
```

Requires `python-docx`. The Markdown is the source of truth; do not edit the `.docx`
directly, because the next regeneration will overwrite it.

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

This is a 1.77 kg aircraft with four 10-inch propellers and a 6S pack. It can hurt
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
