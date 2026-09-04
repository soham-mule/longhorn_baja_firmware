#pragma once

// Log record format -- the contract between this firmware and
// analysis/parse_log.py. If either struct changes, bump LOG_VERSION,
// update the parser, and note the change in docs/record_format.md.
//
// Version 2: MPU6050 raw counts (accel/gyro/temp) replace the BNO055
// quaternion/linear-accel/calib fields of version 1; the header carries
// the configured full-scale ranges so the parser can scale counts.

#include <stdint.h>

#define LOG_MAGIC   0x424A4C31u  // "BJL1"
#define LOG_VERSION 2

typedef struct __attribute__((packed)) {
    uint32_t magic;         // LOG_MAGIC
    uint16_t version;       // LOG_VERSION
    uint16_t record_bytes;  // sizeof(sample_t)
    uint32_t boot_id;       // random per power cycle, ties files together
    uint32_t gps_utc;       // UTC seconds (Unix) at first fix, 0 if never fixed
    uint32_t t_us_at_fix;   // micros() at that same instant
    uint16_t gyro_fs_dps;   // gyro full scale, deg/s  (counts = +/-32768)
    uint8_t  accel_fs_g;    // accel full scale, g     (counts = +/-32768)
    uint8_t  reserved;
} file_header_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;          // micros() at the instant of sampling
    int16_t  accel[3];      // x, y, z raw counts, big-endian on the wire
    int16_t  gyro[3];       // x, y, z raw counts
    int16_t  temp;          // raw; degC = raw/340 + 36.53
    uint8_t  seq;           // wraps at 256; a gap = dropped record(s)
    uint8_t  flags;         // see SAMPLE_FLAG_*
    int32_t  lat_e7;        // degrees x 1e7
    int32_t  lon_e7;        // degrees x 1e7
    uint16_t speed_cms;     // cm/s
    uint16_t course_cdeg;   // centidegrees
    uint8_t  sats;
    uint8_t  hdop_tenths;
} sample_t;

#define SAMPLE_FLAG_FIX_VALID 0x01u  // GPS location valid and recent
#define SAMPLE_FLAG_NEW_GPS   0x02u  // fresh fix landed on this sample
#define SAMPLE_FLAG_IMU_STALE 0x04u  // IMU read failed; fields are stale

static_assert(sizeof(file_header_t) == 24, "header layout changed");
static_assert(sizeof(sample_t) == 34, "record layout changed");
