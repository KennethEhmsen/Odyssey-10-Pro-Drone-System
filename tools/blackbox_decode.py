#!/usr/bin/env python3
"""
Odyssey-10 Pro -- BlackBox log decoder.

Reads a /flight_NNNN.ody binary log from the aircraft's MicroSD card and writes CSV,
plus a short flight summary. The record layout comes from the file's own header, so
this tool does not have to be kept in lockstep with the firmware by hand -- which was
a problem with the original format, where bare structs were appended to a single
/blackbox.bin with no magic, no version and no record size.

Usage:
    python blackbox_decode.py flight_0001.ody
    python blackbox_decode.py flight_0001.ody --csv out.csv
    python blackbox_decode.py flight_0001.ody --summary-only
"""

import argparse
import csv
import struct
import sys
from pathlib import Path

# Must match types.h
BLACKBOX_MAGIC = 0x4F445931          # "ODY1"
SUPPORTED_VERSIONS = (2,)

HEADER_FMT = "<IHHII24s"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

RECORD_FMT = "<I3h3h3h4H H h H h h h H H B B"
RECORD_SIZE = struct.calcsize(RECORD_FMT)

FIELDS = [
    "timestamp_ms",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "accel_x_mps2", "accel_y_mps2", "accel_z_mps2",
    "roll_deg", "pitch_deg", "heading_deg",
    "m1_pwm", "m2_pwm", "m3_pwm", "m4_pwm",
    "batt_v", "current_a", "consumed_mah",
    "baro_agl_m", "tof_agl_m", "vario_mps",
    "obstacle_cm", "sensor_health", "flight_state", "mixer_sat_pct",
]

STATE_NAMES = {
    0: "BOOT", 1: "CALIBRATING", 2: "PREFLIGHT_FAIL", 3: "PREFLIGHT_OK",
    4: "ARMED", 5: "RTH_NAVIGATING", 6: "AWAITING_LAND_PERMIT",
    7: "FAILSAFE_LANDING", 8: "FREEFALL_PARACHUTE", 9: "DISARMED",
}

SENSOR_BITS = [
    (1 << 0, "imu_primary"), (1 << 1, "imu_backup"), (1 << 2, "baro"),
    (1 << 3, "mag"), (1 << 4, "gnss"), (1 << 5, "lidar_fwd"),
    (1 << 6, "tof_down"), (1 << 7, "current"), (1 << 8, "sdcard"),
    (1 << 9, "lora"), (1 << 10, "crsf_rc"), (1 << 11, "aux_bus"),
    (1 << 12, "remote_id"),
]

LIDAR_INVALID = 0xFFFF


def read_header(fh):
    raw = fh.read(HEADER_SIZE)
    if len(raw) < HEADER_SIZE:
        raise SystemExit("file is too short to contain a header")
    magic, version, rec_bytes, rate_hz, boot_ms, airframe = struct.unpack(HEADER_FMT, raw)
    if magic != BLACKBOX_MAGIC:
        raise SystemExit(f"bad magic 0x{magic:08X} -- this is not an Odyssey BlackBox log")
    if version not in SUPPORTED_VERSIONS:
        raise SystemExit(f"log format v{version} is not supported by this decoder "
                         f"(supported: {SUPPORTED_VERSIONS})")
    if rec_bytes != RECORD_SIZE:
        raise SystemExit(f"record size mismatch: file says {rec_bytes} bytes, "
                         f"decoder expects {RECORD_SIZE}")
    return {
        "version": version,
        "record_bytes": rec_bytes,
        "rate_hz": rate_hz,
        "boot_ms": boot_ms,
        "airframe": airframe.rstrip(b"\x00").decode("ascii", "replace"),
    }


def decode_record(raw):
    v = struct.unpack(RECORD_FMT, raw)
    return {
        "timestamp_ms": v[0],
        "gyro_x_dps": v[1] / 10.0, "gyro_y_dps": v[2] / 10.0, "gyro_z_dps": v[3] / 10.0,
        "accel_x_mps2": v[4] / 100.0, "accel_y_mps2": v[5] / 100.0,
        "accel_z_mps2": v[6] / 100.0,
        "roll_deg": v[7] / 100.0, "pitch_deg": v[8] / 100.0, "heading_deg": v[9] / 100.0,
        "m1_pwm": v[10], "m2_pwm": v[11], "m3_pwm": v[12], "m4_pwm": v[13],
        "batt_v": v[14] / 1000.0,
        "current_a": v[15] / 100.0,
        "consumed_mah": v[16],
        "baro_agl_m": v[17] / 100.0,
        # -1 is the firmware's marker for "the ToF was stale for this sample".
        "tof_agl_m": None if v[18] < 0 else v[18] / 100.0,
        "vario_mps": v[19] / 100.0,
        "obstacle_cm": None if v[20] == LIDAR_INVALID else v[20],
        "sensor_health": v[21],
        "flight_state": v[22],
        "mixer_sat_pct": v[23],
    }


def summarise(header, records):
    if not records:
        print("no records in the log")
        return

    t0 = records[0]["timestamp_ms"]
    t1 = records[-1]["timestamp_ms"]
    duration = (t1 - t0) / 1000.0

    print(f"Airframe      : {header['airframe']}")
    print(f"Format        : v{header['version']}, {header['rate_hz']} Hz, "
          f"{header['record_bytes']} B/record")
    print(f"Records       : {len(records)}")
    print(f"Duration      : {duration:.1f} s")

    # N samples span N-1 intervals, so the expected count is duration*rate + 1.
    # Without the +1 a perfect log reports negative loss.
    expected = int(round(duration * header["rate_hz"])) + 1
    if expected > 1:
        missing = max(0, expected - len(records))
        pct = 100.0 * missing / expected
        note = "  <-- SD card could not keep up" if pct > 1.0 else ""
        print(f"Sample loss   : {missing} of {expected} ({pct:.2f}%){note}")

    # State timeline
    print("\nState timeline:")
    prev = None
    for r in records:
        if r["flight_state"] != prev:
            prev = r["flight_state"]
            print(f"  {(r['timestamp_ms'] - t0) / 1000.0:8.2f} s  "
                  f"{STATE_NAMES.get(prev, f'STATE_{prev}')}")

    # Battery
    volts = [r["batt_v"] for r in records if r["batt_v"] > 1.0]
    if volts:
        print(f"\nPack voltage  : {max(volts):.2f} V -> {min(volts):.2f} V")
        cells = round(max(volts) / 4.2)
        if cells:
            print(f"                {max(volts)/cells:.2f} -> {min(volts)/cells:.2f} "
                  f"V/cell across {cells}S")
    mah = [r["consumed_mah"] for r in records]
    if mah:
        print(f"Consumed      : {max(mah)} mAh")

    # Altitude
    baro = [r["baro_agl_m"] for r in records]
    if baro:
        print(f"Max altitude  : {max(baro):.1f} m AGL (barometric)")

    # Mixer saturation -- the thing finding 13 is about. Any sustained non-zero value
    # means the aircraft was giving up attitude authority to keep throttle.
    sat = [r["mixer_sat_pct"] for r in records]
    if sat and max(sat) > 0:
        n = sum(1 for s in sat if s > 0)
        print(f"\nMixer saturation: peak {max(sat)}%, present in {n} samples "
              f"({100.0*n/len(sat):.1f}% of the flight)")
        if max(sat) > 30:
            print("  WARNING: sustained saturation above 30% means the aircraft ran out")
            print("           of control authority. Reduce the PID output limits or")
            print("           check for a heavy/overloaded airframe.")
    else:
        print("\nMixer saturation: none -- full control authority throughout")

    # Sensor dropouts
    print("\nSensor availability:")
    for bit, name in SENSOR_BITS:
        have = sum(1 for r in records if r["sensor_health"] & bit)
        pct = 100.0 * have / len(records)
        flag = "" if pct > 99.0 else ("  <-- intermittent" if pct > 1.0 else "  <-- never seen")
        print(f"  {name:<12} {pct:6.1f}%{flag}")


def main():
    ap = argparse.ArgumentParser(description="Decode an Odyssey-10 Pro BlackBox log")
    ap.add_argument("logfile", type=Path)
    ap.add_argument("--csv", type=Path, help="write decoded records to this CSV file")
    ap.add_argument("--summary-only", action="store_true")
    args = ap.parse_args()

    if not args.logfile.exists():
        raise SystemExit(f"no such file: {args.logfile}")

    with args.logfile.open("rb") as fh:
        header = read_header(fh)
        records = []
        truncated = 0
        while True:
            raw = fh.read(RECORD_SIZE)
            if not raw:
                break
            if len(raw) < RECORD_SIZE:
                # A log that ends mid-record means the aircraft lost power without a
                # clean close. Report it rather than silently discarding the tail.
                truncated = len(raw)
                break
            records.append(decode_record(raw))

    summarise(header, records)
    if truncated:
        print(f"\nNOTE: final {truncated} bytes are a partial record -- the log was not")
        print("      closed cleanly (power loss or hard reset).")

    out = args.csv
    if out is None and not args.summary_only:
        out = args.logfile.with_suffix(".csv")
    if out:
        with out.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            w.writerows(records)
        print(f"\nwrote {len(records)} rows to {out}")


if __name__ == "__main__":
    sys.exit(main())
