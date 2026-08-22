#!/usr/bin/env python3
"""
Odyssey-10 Pro -- BlackBox decoder tests.

The consistency check proves the C struct and the Python format string agree on layout.
It does NOT prove the decoder turns those bytes back into the right numbers -- a field
can sit at the correct offset and still be scaled, signed or flagged wrongly.

This exercises the decoder end to end: build a log in memory with known values, decode
it, and check what comes back. It runs in CI alongside the host tests.

    python3 tools/test_blackbox_decode.py
"""

import io
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import blackbox_decode as bb

_pass = 0
_fail = 0


def check(cond, what):
    global _pass, _fail
    if cond:
        _pass += 1
        print(f"  PASS  {what}")
    else:
        _fail += 1
        print(f"  FAIL  {what}")


def near(got, want, tol, what):
    check(got is not None and abs(got - want) <= tol,
          f"{what}  (got {got}, want {want})")


def section(title):
    print(f"\n{title}\n{'-' * len(title)}")


def build_header(version, rate_hz=100, boot_ms=1234, airframe=b"Odyssey-10 Pro"):
    return struct.pack(bb.HEADER_FMT, bb.BLACKBOX_MAGIC, version,
                       struct.calcsize(bb.RECORD_FMTS[version]), rate_hz, boot_ms,
                       airframe.ljust(24, b"\x00"))


def build_record(version, **kw):
    """A record with sane defaults, overridable field by field."""
    v = [
        kw.get("timestamp_ms", 1000),
        kw.get("gyro_x", 0), kw.get("gyro_y", 0), kw.get("gyro_z", 0),
        kw.get("accel_x", 0), kw.get("accel_y", 0), kw.get("accel_z", 981),
        kw.get("roll", 0), kw.get("pitch", 0), kw.get("heading", 0),
        kw.get("m1", 1500), kw.get("m2", 1500), kw.get("m3", 1500), kw.get("m4", 1500),
        kw.get("batt_mv", 22200),
        kw.get("current_ca", 970),
        kw.get("consumed_mah", 0),
        kw.get("baro_cm", 0),
        kw.get("tof_cm", 0),
        kw.get("vario_cms", 0),
        kw.get("obstacle_cm", 500),
        kw.get("sensor_health", 0xFFFF),
        kw.get("flight_state", 4),
        kw.get("mixer_sat", 0),
    ]
    if version >= 3:
        v += [kw.get("notch_decihz", 1200), kw.get("notch_conf", 40),
              kw.get("notch_flags", 3)]
    if version >= 4:
        v += [kw.get("harmonic_decihz", 0)]
    return struct.pack(bb.RECORD_FMTS[version], *v)


def decode_all(version, records):
    """Runs the real read path over an in-memory log."""
    blob = build_header(version) + b"".join(records)
    fh = io.BytesIO(blob)
    header = bb.read_header(fh)
    size = struct.calcsize(bb.RECORD_FMTS[header["version"]])
    out = []
    while True:
        raw = fh.read(size)
        if len(raw) < size:
            break
        out.append(bb.decode_record(raw, header["version"]))
    return header, out


def test_scaling():
    section("v4 record: values survive the round trip")
    _, recs = decode_all(4, [build_record(
        4, gyro_x=1234, accel_z=981, roll=-4500, batt_mv=22200, current_ca=970,
        baro_cm=1550, vario_cms=-250, notch_decihz=1187, notch_conf=42, notch_flags=15,
        harmonic_decihz=1760)])
    r = recs[0]
    near(r["gyro_x_dps"], 123.4, 0.01, "gyro is decoded in 0.1 deg/s units")
    near(r["accel_z_mps2"], 9.81, 0.01, "accel is decoded in 0.01 m/s^2 units")
    near(r["roll_deg"], -45.0, 0.01, "roll is signed and in centidegrees")
    near(r["batt_v"], 22.2, 0.01, "pack voltage is decoded in millivolts")
    near(r["current_a"], 9.7, 0.01, "current is decoded in centiamps")
    near(r["baro_agl_m"], 15.5, 0.01, "barometric altitude is decoded in cm")
    near(r["vario_mps"], -2.5, 0.01, "vertical speed is signed")
    near(r["notch_hz"], 118.7, 0.01, "notch centre is decoded in 0.1 Hz units")
    check(r["notch_confidence"] == 42, "notch confidence passes through unscaled")
    check(r["notch_tracking"] is True, "the tracking flag is decoded")
    check(r["notch_dynamic"] is True, "the dynamic-enabled flag is decoded")
    near(r["harmonic_hz"], 176.0, 0.01, "the harmonic centre is decoded in 0.1 Hz units")
    check(r["harmonic_tracking"] is True, "the harmonic tracking flag is decoded")
    check(r["harmonic_observable"] is True, "the harmonic observability flag is decoded")


def test_sentinels():
    section("Sentinel values are not decoded as real readings")
    _, recs = decode_all(4, [build_record(4, tof_cm=-1, obstacle_cm=bb.LIDAR_INVALID)])
    r = recs[0]
    # -1 and 0xFFFF are "no reading", not "the ground is 1 cm below" or "655 m clear".
    check(r["tof_agl_m"] is None, "a stale ToF reading decodes as None, not -0.01 m")
    check(r["obstacle_cm"] is None, "an invalid LiDAR reading decodes as None")

    _, recs = decode_all(4, [build_record(4, tof_cm=250, obstacle_cm=300)])
    near(recs[0]["tof_agl_m"], 2.5, 0.01, "a valid ToF reading still decodes")
    check(recs[0]["obstacle_cm"] == 300, "a valid LiDAR reading still decodes")


def test_notch_flags():
    section("Notch flags are four independent bits")
    for flags in range(16):
        _, recs = decode_all(4, [build_record(4, notch_flags=flags)])
        r = recs[0]
        ok = (r["notch_tracking"] is bool(flags & 1) and
              r["notch_dynamic"] is bool(flags & 2) and
              r["harmonic_tracking"] is bool(flags & 4) and
              r["harmonic_observable"] is bool(flags & 8))
        check(ok, f"flags 0b{flags:04b} decode to four independent booleans")

    # The distinction the whole harmonic feature turns on. "Observable but nothing
    # found" means the propellers are clean; "not observable" means the IMU could
    # never have shown it. Reading a log without telling those apart would credit the
    # tracker with a success or blame it for a failure that was never its to have.
    _, recs = decode_all(4, [build_record(4, notch_flags=8, harmonic_decihz=0)])
    check(recs[0]["harmonic_observable"] and not recs[0]["harmonic_tracking"],
          "observable-but-not-found is distinguishable from not-observable")

    # Bit 4 records whether the centre was MEASURED from RPM or searched for. It needs
    # no format bump: it was always zero in v4, and zero is the truthful reading there.
    _, recs = decode_all(4, [build_record(4, notch_flags=0b10011)])
    check(recs[0]["notch_measured"], "the measured-source flag is decoded")
    _, recs = decode_all(4, [build_record(4, notch_flags=0b00011)])
    check(not recs[0]["notch_measured"],
          "and reads False when the centre came from the spectrum search")
    _, recs = decode_all(4, [build_record(4, notch_flags=0)])
    check(not recs[0]["harmonic_observable"] and not recs[0]["harmonic_tracking"],
          "not-observable is its own state")


def test_v2_still_readable():
    section("Older logs remain readable")
    # A flight recorder whose decoder only reads the newest format silently makes every
    # previous flight unreadable. v2 logs predate the notch fields and must still work.
    header, recs = decode_all(2, [build_record(2, gyro_x=500, batt_mv=21000)])
    check(header["version"] == 2, "a v2 header is accepted")
    near(recs[0]["gyro_x_dps"], 50.0, 0.01, "v2 gyro decodes correctly")
    near(recs[0]["batt_v"], 21.0, 0.01, "v2 voltage decodes correctly")
    check("notch_hz" not in recs[0],
          "v2 records do not invent notch fields that were never recorded")
    check(bb.FIELDS_BY_VERSION[2] != bb.FIELDS_BY_VERSION[3],
          "the CSV column set differs by version")

    # v3 predates the harmonic and must still decode, without inventing its fields.
    header, recs = decode_all(3, [build_record(3, notch_decihz=1187)])
    check(header["version"] == 3, "a v3 header is accepted")
    near(recs[0]["notch_hz"], 118.7, 0.01, "v3 notch centre decodes correctly")
    check("harmonic_hz" not in recs[0],
          "v3 records do not invent harmonic fields that were never recorded")
    check(len(bb.SUPPORTED_VERSIONS) == 3,
          "three log formats are readable, so no recorded flight became unreadable")


def test_bad_files_are_rejected():
    section("Corrupt and foreign files are rejected, not guessed at")
    def expect_exit(blob, what):
        try:
            bb.read_header(io.BytesIO(blob))
        except SystemExit:
            check(True, what)
            return
        check(False, what)

    expect_exit(b"", "an empty file is rejected")
    expect_exit(b"\x00" * 8, "a file too short for a header is rejected")
    bad_magic = struct.pack(bb.HEADER_FMT, 0xDEADBEEF, 4, 54, 100, 0, b"x".ljust(24, b"\0"))
    expect_exit(bad_magic, "a file with the wrong magic is rejected")
    bad_ver = struct.pack(bb.HEADER_FMT, bb.BLACKBOX_MAGIC, 99, 52, 100, 0,
                          b"x".ljust(24, b"\0"))
    expect_exit(bad_ver, "an unsupported format version is rejected")
    # The size field disagreeing with the format is how a silent mis-decode would start.
    bad_size = struct.pack(bb.HEADER_FMT, bb.BLACKBOX_MAGIC, 4, 999, 100, 0,
                           b"x".ljust(24, b"\0"))
    expect_exit(bad_size, "a record-size mismatch is rejected rather than mis-decoded")


def test_summary_runs():
    section("The summary survives real-shaped input")
    # A short flight: unlocked at first, then locked onto 118.7 Hz.
    recs = [build_record(4, timestamp_ms=i * 10, notch_flags=2, notch_decihz=1200,
                         flight_state=4 if i > 20 else 3)
            for i in range(30)]
    recs += [build_record(4, timestamp_ms=(30 + i) * 10, notch_flags=3,
                          notch_decihz=1187 + (i % 5), notch_conf=40, flight_state=4)
             for i in range(170)]
    header, decoded = decode_all(4, recs)

    buf = io.StringIO()
    stdout, sys.stdout = sys.stdout, buf
    try:
        bb.summarise(header, decoded)
    finally:
        sys.stdout = stdout
    text = buf.getvalue()

    check("Gyro notch:" in text, "the summary reports a gyro notch section")
    check("tracking locked" in text, "it reports how much of the flight was locked")
    check("118." in text, "it reports the tracked centre frequency")
    check("Set NOTCH_CENTER_HZ" in text,
          "it tells the reader what to do with the measurement")

    # And with tracking off, it must say so rather than reporting a fake measurement.
    header, decoded = decode_all(4, [build_record(4, notch_flags=0) for _ in range(50)])
    buf = io.StringIO()
    stdout, sys.stdout = sys.stdout, buf
    try:
        bb.summarise(header, decoded)
    finally:
        sys.stdout = stdout
    text = buf.getvalue()
    check("DISABLED" in text,
          "a fixed-notch flight is reported as disabled, not as a measurement")


def main():
    print("=" * 69)
    print(" Odyssey-10 Pro -- BlackBox decoder tests")
    print("=" * 69)
    test_scaling()
    test_sentinels()
    test_notch_flags()
    test_v2_still_readable()
    test_bad_files_are_rejected()
    test_summary_runs()
    print("\n" + "=" * 69)
    print(f" {_pass} passed, {_fail} failed")
    print("=" * 69)
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
