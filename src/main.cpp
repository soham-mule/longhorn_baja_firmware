// Baja Sensor ECU datalogger -- Longhorn Baja Racing, car 185.
// Samples an MPU6050 at a fixed 100 Hz, drains GPS NMEA continuously,
// timestamps both on the micros() clock, and writes fixed-size binary
// records to microSD. No fusion, no filtering on target: raw counts are
// logged here, filtering happens offline (analysis/parse_log.py).
//
// Pin map (docs/pinmap.md):
//   MPU6050 SDA -> GP0 (I2C0 SDA)   SCL -> GP1 (I2C0 SCL)   addr 0x68
//     The MPU6050 breakout sits on the ECU board's 1x6 IMU header, which
//     routes I2C0 to GP0/GP1. Do NOT use the old breadboard wiring
//     (GP4/GP5): those pins are the SD card's MISO/CS on this board.
//   microSD SCK -> GP2, MOSI -> GP3, MISO -> GP4, CS -> GP5 (SPI0)
//   GPS     module TX -> GP13 (UART1 RX), module RX -> GP12 (UART1 TX)
//     Module: GT-U7 (u-blox NEO-6M compatible), NMEA at 9600 baud.
//   Status LED: onboard GP25
//
// Startup behaviour, decided and documented: a missing SD card or
// missing IMU HALTS with a distinct blink pattern. A run that logs
// nothing is worse than a run that does not start, and the driver must
// see the fault before rolling out.
//   normal logging : 1 Hz heartbeat blink
//   SD failure     : fast continuous blink (~8 Hz)
//   IMU failure    : double-flash, pause, repeat

#include <Arduino.h>
#include <Wire.h>
#include <hardware/watchdog.h>
#include <pico/rand.h>

#include "mpu6050.h"
#include "gps.h"
#include "logger.h"
#include "record.h"

static const uint8_t kPinSda = 0;
static const uint8_t kPinScl = 1;
static const uint8_t kMpuAddr = 0x68;  // 0x69 if AD0 is tied high
static const uint32_t kI2cHz = 400000;

static const uint32_t kSamplePeriodUs = 10000;  // 100 Hz
static const uint32_t kStatusPeriodUs = 1000000;
static const uint32_t kWatchdogMs = 4000;

static const uint8_t kPinLed = LED_BUILTIN;

static uint32_t next_us = 0;
static uint32_t status_next_us = 0;
static uint32_t sample_count = 0;
static uint32_t imu_fail_count = 0;
static uint32_t imu_fail_streak = 0;
static uint8_t seq = 0;
static Mpu6050Sample imu;  // last good reading, logged stale on failure

// Blocking halt with a blink pattern. Only reachable before the watchdog
// is armed, so the pattern persists until someone power cycles.
static void halt_blink(uint16_t on_ms, uint16_t off_ms, uint8_t pulses,
                       uint16_t gap_ms)
{
    pinMode(kPinLed, OUTPUT);
    while (true) {
        for (uint8_t i = 0; i < pulses; i++) {
            digitalWrite(kPinLed, HIGH);
            delay(on_ms);
            digitalWrite(kPinLed, LOW);
            delay(off_ms);
        }
        delay(gap_ms);
    }
}

static void take_sample()
{
    const uint32_t t_us = micros();

    bool stale = false;
    if (mpu6050_read_sample(imu)) {
        imu_fail_streak = 0;
    } else {
        stale = true;
        imu_fail_count++;
        imu_fail_streak++;
        // One second of solid failures: the chip likely browned out or
        // the bus wedged. Reconfigure and carry on; records keep flowing
        // (flagged stale) either way.
        if (imu_fail_streak >= 100) {
            imu_fail_streak = 0;
            mpu6050_reinit();
        }
    }

    const GpsState gps = gps_take();
    if (gps.new_this_sample && gps.fix_valid && gps.have_utc) {
        logger_note_first_fix(gps.utc_seconds, t_us);
    }

    sample_t s;
    s.t_us = t_us;
    for (int i = 0; i < 3; i++) s.accel[i] = imu.accel[i];
    for (int i = 0; i < 3; i++) s.gyro[i] = imu.gyro[i];
    s.temp = imu.temp;
    // seq increments per sample *taken*; a record dropped by a full ring
    // buffer shows up on the card as a jump in seq.
    s.seq = seq++;
    s.flags = (gps.fix_valid ? SAMPLE_FLAG_FIX_VALID : 0) |
              (gps.new_this_sample ? SAMPLE_FLAG_NEW_GPS : 0) |
              (stale ? SAMPLE_FLAG_IMU_STALE : 0);
    s.lat_e7 = gps.lat_e7;
    s.lon_e7 = gps.lon_e7;
    s.speed_cms = gps.speed_cms;
    s.course_cdeg = gps.course_cdeg;
    s.sats = gps.sats;
    s.hdop_tenths = gps.hdop_tenths;

    logger_push(s);
    sample_count++;
}

static void print_status()
{
    char line[128];
    const int len = snprintf(
        line, sizeof(line),
        "n=%lu drop=%lu imu_err=%lu fix=%u sats=%u sd=%s\r\n",
        (unsigned long)sample_count, (unsigned long)logger_dropped(),
        (unsigned long)imu_fail_count,
        (unsigned)(gps_peek().fix_valid ? 1 : 0), (unsigned)gps_peek().sats,
        logger_ok() ? "ok" : "FAIL");
    // Lowest priority output: drop the line rather than ever blocking.
    if (len > 0 && Serial.availableForWrite() >= len) {
        Serial.write((const uint8_t*)line, (size_t)len);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for USB, but never block when running headless on the car.
    }
    Serial.println();
    Serial.println("Baja Sensor ECU datalogger (MPU6050 + GT-U7)");
    if (watchdog_caused_reboot()) {
        Serial.println("NOTE: recovering from a watchdog reset");
    }

    pinMode(kPinLed, OUTPUT);

    Wire.setSDA(kPinSda);
    Wire.setSCL(kPinScl);
    Wire.begin();
    Wire.setClock(kI2cHz);

    if (!mpu6050_init(Wire, kMpuAddr)) {
        Serial.println("FATAL: MPU6050 not responding (WHO_AM_I != 0x68). "
                       "Check wiring/address/pull-ups. Halting.");
        halt_blink(100, 100, 2, 800);  // double-flash pattern
    }
    Serial.println("MPU6050 up: +/-16 g, +/-1000 deg/s, 44 Hz DLPF");

    gps_begin();

    // boot_id: hardware random, ties this power cycle's file(s) together.
    const uint32_t boot_id = get_rand_32();
    if (!logger_begin(boot_id, mpu6050_accel_fs_g(), mpu6050_gyro_fs_dps())) {
        Serial.println("FATAL: no usable SD card. Halting.");
        halt_blink(60, 60, 1, 0);  // fast continuous blink
    }
    Serial.print("logging, boot_id=");
    Serial.println(boot_id, HEX);

    // Armed only after init: the blocking bring-up above must not race a
    // 4 s timeout. Fed in exactly one place, the top of loop().
    rp2040.wdt_begin(kWatchdogMs);

    next_us = micros() + kSamplePeriodUs;
    status_next_us = next_us;
}

void loop()
{
    rp2040.wdt_reset();

    // GPS bytes drain every pass so the UART FIFO never overflows.
    gps_poll();

    // Deadline pattern, wrap-safe: signed test on the unsigned difference
    // works across the 71-minute micros() rollover; a direct >= compare
    // does not -- the bug that survives bench tests and then kills an
    // endurance run.
    if ((int32_t)(micros() - next_us) >= 0) {
        next_us += kSamplePeriodUs;
        take_sample();
    }

    logger_poll();

    if ((int32_t)(micros() - status_next_us) >= 0) {
        status_next_us += kStatusPeriodUs;
        // Heartbeat: steady 0.5 s toggle says the loop is alive.
        digitalWrite(kPinLed, (millis() / 500) % 2 ? HIGH : LOW);
        print_status();
    }
}
