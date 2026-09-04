#pragma once

// MPU6050 driver, written from the register map (RM-MPU-6000A). Raw
// int16 counts only -- scaling to physical units happens offline in
// Python using the full-scale values recorded in the log header.
//
// Vehicle full-scale settings (CLAUDE.md known issue 4: +/-2 g clips on
// a Baja car silently): +/-16 g accel, +/-1000 deg/s gyro, 44 Hz DLPF.

#include <Arduino.h>
#include <Wire.h>

struct Mpu6050Sample {
    int16_t accel[3];  // x, y, z raw counts
    int16_t temp;      // raw; degC = raw/340 + 36.53 (offline)
    int16_t gyro[3];   // x, y, z raw counts
};

// WHO_AM_I check, reset, wake (PLL clock), 100 Hz sample rate, 44 Hz
// low-pass, vehicle full-scale ranges. Blocking (~200 ms); call before
// the watchdog is armed.
bool mpu6050_init(TwoWire& wire, uint8_t addr);

// One 14-byte burst read of accel + temp + gyro (registers 0x3B..0x48,
// big-endian on the wire). Returns false on bus error.
bool mpu6050_read_sample(Mpu6050Sample& out);

// Re-run configuration after a run-time bus fault (skips the reset wait).
bool mpu6050_reinit();

// Configured full scale, recorded in the log file header so the offline
// parser can scale raw counts without guessing.
uint8_t mpu6050_accel_fs_g();
uint16_t mpu6050_gyro_fs_dps();
