# Log record format

`src/record.h` is the authoritative contract between the firmware and
`analysis/parse_log.py`. This file records the history of that contract.
When the layout changes: bump `LOG_VERSION`, update the parser, add a row
here.

| Version | Date | Change |
|---|---|---|
| 1 | 2026-09 | Initial format: 20-byte header, 40-byte samples (BNO055 quat/lin-accel/gyro counts, calib byte, GPS fields). Never used on hardware. |
| 2 | 2026-09 | IMU swapped to MPU6050: samples carry raw accel/gyro/temp counts + wrapping `seq` byte; header gains the configured full-scale ranges. 24-byte header, 34-byte samples. |

## Layout, version 2

All little endian, packed.

**File header (24 bytes):** `magic` u32 = `0x424A4C31` ("BJL1"), `version`
u16, `record_bytes` u16, `boot_id` u32 (random per power cycle),
`gps_utc` u32 (Unix seconds at first fix, 0 if never fixed),
`t_us_at_fix` u32 (`micros()` at that instant — pairs the relative clock
with wall time), `gyro_fs_dps` u16, `accel_fs_g` u8, `reserved` u8.

**Sample (34 bytes, 100 Hz):** `t_us` u32, `accel[3]` i16 (raw counts),
`gyro[3]` i16 (raw counts), `temp` i16 (°C = raw/340 + 36.53), `seq` u8
(wraps at 256; a jump = dropped records), `flags` u8 (bit0 fix valid,
bit1 new GPS this sample, bit2 IMU stale), `lat_e7` i32, `lon_e7` i32
(deg × 1e7), `speed_cms` u16, `course_cdeg` u16, `sats` u8,
`hdop_tenths` u8.

Count scaling uses the header ranges: `accel_ms2 = raw × accel_fs_g ×
9.80665 / 32768`, `gyro_dps = raw × gyro_fs_dps / 32768`.

Raw counts are stored, never floats: smaller, lossless, and the scale
factors are re-derivable. Conversion happens only in Python.
