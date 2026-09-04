#!/usr/bin/env python3
"""Parse a Baja Sensor ECU log file (LOGnnnn.BIN) into physical units.

The binary layout is the contract in src/record.h. If that header changes,
LOG_VERSION is bumped there and this parser must be updated to match --
it refuses to parse a version it was not written for.

Usage:
    python parse_log.py LOG0001.BIN            # summary only
    python parse_log.py LOG0001.BIN out.csv    # also write a CSV

Version 2 records hold raw MPU6050 counts; scale factors come from the
full-scale ranges stored in the file header:
    accel_ms2 = raw * (accel_fs_g * 9.80665 / 32768)
    gyro_dps  = raw * (gyro_fs_dps / 32768)
    temp_c    = raw / 340 + 36.53
"""

import csv
import struct
import sys

MAGIC = 0x424A4C31  # "BJL1"
VERSION = 2

HEADER_FMT = "<IHHIIIHBB"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 24

SAMPLE_FMT = "<I3h3hhBBiiHHBB"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)  # 34

FLAG_FIX_VALID = 0x01
FLAG_NEW_GPS = 0x02
FLAG_IMU_STALE = 0x04

GRAVITY = 9.80665

COLUMNS = [
    "t_us", "t_s", "seq",
    "ax_ms2", "ay_ms2", "az_ms2",
    "gx_dps", "gy_dps", "gz_dps",
    "temp_c",
    "imu_stale", "fix_valid", "new_gps",
    "lat", "lon", "speed_ms", "course_deg", "sats", "hdop",
]


def parse_header(buf):
    (magic, version, record_bytes, boot_id, gps_utc, t_us_at_fix,
     gyro_fs_dps, accel_fs_g, _reserved) = struct.unpack_from(HEADER_FMT, buf)
    if magic != MAGIC:
        raise SystemExit(f"not a BJL1 log (magic 0x{magic:08X})")
    if version != VERSION:
        raise SystemExit(
            f"log version {version}, parser only knows {VERSION}. "
            "Update parse_log.py to match src/record.h."
        )
    if record_bytes != SAMPLE_SIZE:
        raise SystemExit(
            f"record size {record_bytes} != expected {SAMPLE_SIZE}"
        )
    return {
        "boot_id": boot_id,
        "gps_utc": gps_utc,
        "t_us_at_fix": t_us_at_fix,
        "accel_fs_g": accel_fs_g,
        "gyro_fs_dps": gyro_fs_dps,
    }


def parse_sample(buf, offset, accel_scale, gyro_scale):
    (t_us, ax, ay, az, gx, gy, gz, temp, seq, flags,
     lat_e7, lon_e7, speed_cms, course_cdeg, sats, hdop_tenths) = (
        struct.unpack_from(SAMPLE_FMT, buf, offset)
    )
    return {
        "t_us": t_us,
        "t_s": 0.0,  # filled by unwrap_times
        "seq": seq,
        "ax_ms2": ax * accel_scale,
        "ay_ms2": ay * accel_scale,
        "az_ms2": az * accel_scale,
        "gx_dps": gx * gyro_scale,
        "gy_dps": gy * gyro_scale,
        "gz_dps": gz * gyro_scale,
        "temp_c": temp / 340.0 + 36.53,
        "imu_stale": int(bool(flags & FLAG_IMU_STALE)),
        "fix_valid": int(bool(flags & FLAG_FIX_VALID)),
        "new_gps": int(bool(flags & FLAG_NEW_GPS)),
        "lat": lat_e7 / 1e7,
        "lon": lon_e7 / 1e7,
        "speed_ms": speed_cms / 100.0,
        "course_deg": course_cdeg / 100.0,
        "sats": sats,
        "hdop": hdop_tenths / 10.0,
    }


def unwrap_times(samples):
    """32-bit micros() wraps at ~71 min; accumulate deltas pairwise so
    t_s stays monotonic across a four-hour run."""
    total = 0
    prev = samples[0]["t_us"]
    for s in samples:
        total += (s["t_us"] - prev) & 0xFFFFFFFF
        prev = s["t_us"]
        s["t_s"] = total / 1e6


def count_seq_gaps(samples):
    """Dropped records show as jumps in the wrapping sequence byte."""
    lost = 0
    for a, b in zip(samples, samples[1:]):
        lost += (b["seq"] - a["seq"] - 1) & 0xFF
    return lost


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    path = sys.argv[1]

    with open(path, "rb") as f:
        data = f.read()
    if len(data) < HEADER_SIZE:
        raise SystemExit("file shorter than a header")

    header = parse_header(data)
    accel_scale = header["accel_fs_g"] * GRAVITY / 32768.0
    gyro_scale = header["gyro_fs_dps"] / 32768.0

    n = (len(data) - HEADER_SIZE) // SAMPLE_SIZE
    samples = [
        parse_sample(data, HEADER_SIZE + i * SAMPLE_SIZE,
                     accel_scale, gyro_scale)
        for i in range(n)
    ]
    if samples:
        unwrap_times(samples)

    # ---- summary ------------------------------------------------------
    print(f"boot_id        0x{header['boot_id']:08X}")
    print(f"records        {n}")
    print(f"accel fs       +/-{header['accel_fs_g']} g")
    print(f"gyro fs        +/-{header['gyro_fs_dps']} deg/s")
    if header["gps_utc"]:
        print(f"gps_utc        {header['gps_utc']} (unix, at first fix)")
        print(f"t_us_at_fix    {header['t_us_at_fix']}")
    else:
        print("gps_utc        never fixed")
    if n >= 2:
        dur = samples[-1]["t_s"]
        print(f"duration       {dur:.1f} s")
        print(f"mean rate      {(n - 1) / dur:.2f} Hz")
        gaps = [
            samples[i + 1]["t_s"] - samples[i]["t_s"] for i in range(n - 1)
        ]
        print(f"worst gap      {max(gaps) * 1000:.2f} ms")
        print(f"seq losses     {count_seq_gaps(samples)} records")
        print(f"stale imu      {sum(s['imu_stale'] for s in samples)}")
        print(f"gps fixes      {sum(s['new_gps'] for s in samples)}")

    # ---- optional CSV -------------------------------------------------
    if len(sys.argv) >= 3:
        with open(sys.argv[2], "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
            w.writeheader()
            w.writerows(samples)
        print(f"wrote {sys.argv[2]}")


if __name__ == "__main__":
    main()
