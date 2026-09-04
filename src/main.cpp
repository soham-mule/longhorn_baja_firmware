// Vehicle firmware entry point: samples the MPU6050, runs the error-state
// KF, prints summary telemetry. Owns setup()/loop() -- no other file in
// the `pico` env may define them.
//
// Wiring (I2C0, same as mpu6050_test.cpp):
//   MPU6050 VCC -> 3V3(OUT)  physical pin 36  (never VBUS)
//   MPU6050 GND -> GND       physical pin 38
//   MPU6050 SDA -> GP4       physical pin 6
//   MPU6050 SCL -> GP5       physical pin 7
//   AD0 low -> address 0x68
//
// Mounting / frames: the filter runs in NED (Z down, gravity +9.80665).
// The MPU6050 axes are remapped to a front-right-down body frame assuming
// the breakout is mounted component-side up with its +X axis pointing
// forward:  x_b = +x_imu, y_b = -y_imu, z_b = -z_imu.
// If the board is mounted differently, change remap_to_body() only.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <hardware/watchdog.h>

#include "ekf.h"
#include "ekf_predict.h"

// ---------------------------------------------------------------- config

static const uint8_t kPinSda = 4;
static const uint8_t kPinScl = 5;
static const uint8_t kMpuAddr = 0x68;

static const uint32_t kSamplePeriodUs = 10000;   // 100 Hz IMU + predict
static const uint32_t kZuptPeriodUs   = 100000;  // 10 Hz while stationary
static const uint32_t kTelemPeriodUs  = 200000;  // 5 Hz status line
static const uint32_t kInitRetryUs    = 500000;  // IMU re-probe when absent

static const uint16_t kCalibSamples = 200;       // 2 s of standstill
// Motion guards during calibration: restart if exceeded.
static const float kCalibGyroSpread = 0.05f;     // rad/s per-axis max-min
static const float kCalibAccelTol   = 1.0f;      // m/s^2 from |g|

// Standstill detector for ZUPT (bias-corrected thresholds).
static const float kStillGyroMax  = 0.03f;       // rad/s
static const float kStillAccelTol = 0.3f;        // m/s^2 from |g|
static const uint16_t kStillCount = 50;          // consecutive samples (0.5 s)

static const uint32_t kWatchdogMs = 4000;

// Arduino's RAD_TO_DEG is a double literal; using it would promote every
// product to double math (no FPU -- see CLAUDE.md).
static const float kRadToDeg = 57.29577951f;

// ---------------------------------------------------------------- state

static Adafruit_MPU6050 mpu;
static ekf::Filter filter;

enum class Mode : uint8_t { kImuProbe, kCalibrating, kRunning };
static Mode mode = Mode::kImuProbe;
static bool imu_configured = false;

static uint32_t last_sample_us = 0;
static uint32_t last_zupt_us = 0;
static uint32_t last_telem_us = 0;
static uint32_t last_probe_us = 0;
static uint32_t prev_read_us = 0;
static bool have_prev_read = false;

// Calibration accumulators.
static uint16_t calib_n = 0;
static ekf::Vec3 calib_sum_w = ekf::Vec3::Zero();
static ekf::Vec3 calib_sum_f = ekf::Vec3::Zero();
static ekf::Vec3 calib_min_w = ekf::Vec3::Zero();
static ekf::Vec3 calib_max_w = ekf::Vec3::Zero();

// Diagnostics.
static uint16_t still_counter = 0;
static uint32_t read_failures = 0;
static uint32_t filter_resets = 0;
static uint32_t dt_rejects = 0;
static uint32_t predict_us_max = 0;
static uint32_t predict_us_sum = 0;
static uint32_t predict_count = 0;

// ---------------------------------------------------------------- helpers

// Sensor frame -> FRD body frame (see mounting note in the header).
static ekf::Vec3 remap_to_body(float x, float y, float z)
{
    return ekf::Vec3(x, -y, -z);
}

static void start_calibration()
{
    calib_n = 0;
    calib_sum_w.setZero();
    calib_sum_f.setZero();
    still_counter = 0;
    mode = Mode::kCalibrating;
    Serial.println("calibrating: hold still ~2 s");
}

// Try to bring the IMU up. Called from setup() once and then re-tried
// from loop() -- deployed firmware degrades and retries, never halts.
static bool probe_imu()
{
    if (!mpu.begin(kMpuAddr, &Wire)) return false;

    // Vehicle ranges: a Baja car clips +/-2 g instantly and the filter
    // would ingest the clipped data with full confidence.
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);  // < Nyquist of 100 Hz
    imu_configured = true;
    return true;
}

// One IMU read, remapped to the body frame. Returns false on bus failure.
static bool read_imu(ekf::Vec3& f_b, ekf::Vec3& w_b, uint32_t& t_us)
{
    sensors_event_t accel, gyro, temp;
    if (!mpu.getEvent(&accel, &gyro, &temp)) return false;
    t_us = micros();
    // Unified Sensor API already reports m/s^2 and rad/s -- fed to the
    // filter unconverted (SI internally, always).
    f_b = remap_to_body(accel.acceleration.x, accel.acceleration.y,
                        accel.acceleration.z);
    w_b = remap_to_body(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    return true;
}

static void calibration_step(const ekf::Vec3& f_b, const ekf::Vec3& w_b)
{
    if (calib_n == 0) {
        calib_min_w = w_b;
        calib_max_w = w_b;
    }
    calib_min_w = calib_min_w.cwiseMin(w_b);
    calib_max_w = calib_max_w.cwiseMax(w_b);
    calib_sum_w += w_b;
    calib_sum_f += f_b;
    calib_n++;

    // Motion guard: a calibration taken while moving poisons the bias
    // estimate, so restart rather than average it in.
    const bool moving =
        (calib_max_w - calib_min_w).maxCoeff() > kCalibGyroSpread ||
        fabsf(f_b.norm() - ekf::kGravity) > kCalibAccelTol;
    if (moving) {
        start_calibration();
        return;
    }

    if (calib_n >= kCalibSamples) {
        const float inv_n = 1.0f / (float)calib_n;
        const ekf::Vec3 bg = calib_sum_w * inv_n;
        const ekf::Vec3 f_mean = calib_sum_f * inv_n;

        filter.reset();
        filter.set_gyro_bias(bg);
        filter.init_attitude_from_accel(f_mean);

        have_prev_read = false;  // first dt after calibration is invalid
        mode = Mode::kRunning;
        Serial.println("calibration done");
    }
}

static void running_step(const ekf::Vec3& f_b, const ekf::Vec3& w_b,
                         uint32_t t_us)
{
    // dt from measured timestamps, never the nominal rate. Unsigned
    // subtraction survives the ~71.6 min micros() rollover.
    if (!have_prev_read) {
        prev_read_us = t_us;
        have_prev_read = true;
        return;
    }
    const float dt = (float)(t_us - prev_read_us) * 1e-6f;
    prev_read_us = t_us;

    const uint32_t t0 = micros();
    if (!filter.predict(f_b, w_b, dt)) {
        dt_rejects++;
        return;
    }
    const uint32_t cost = micros() - t0;
    predict_us_sum += cost;
    predict_count++;
    if (cost > predict_us_max) predict_us_max = cost;

    // Gravity attitude update every 4th sample (25 Hz); gated inside.
    static uint8_t decim = 0;
    if (++decim >= 4) {
        decim = 0;
        filter.update_accel(f_b);
    }

    // Standstill detector (bias-corrected gyro) feeding ZUPT.
    const ekf::Vec3 w_corr = w_b - filter.state().bg;
    const bool still_now = w_corr.norm() < kStillGyroMax &&
                           fabsf(f_b.norm() - ekf::kGravity) < kStillAccelTol;
    if (still_now) {
        if (still_counter < kStillCount) still_counter++;
    } else {
        still_counter = 0;
    }
    if (still_counter >= kStillCount &&
        (uint32_t)(t_us - last_zupt_us) >= kZuptPeriodUs) {
        last_zupt_us = t_us;
        filter.update_zupt();
    }

    // NaN is contagious and permanent: dump the filter and recalibrate.
    if (!filter.healthy()) {
        filter_resets++;
        Serial.println("filter NaN -- resetting");
        start_calibration();
    }
}

static void print_telemetry()
{
    // Degrees only at this boundary -- everything upstream is rad.
    // Casts to double are explicit: varargs would promote anyway and
    // -Wdouble-promotion flags the implicit version.
    const ekf::Vec3 rpy = filter.rpy() * kRadToDeg;
    const ekf::Vec3 bg = filter.state().bg * kRadToDeg;
    const ekf::Covariance& P = filter.covariance();
    const float att_sd_deg =
        sqrtf(P(ekf::kAtt, ekf::kAtt) + P(ekf::kAtt + 1, ekf::kAtt + 1)) *
        kRadToDeg;
    const uint32_t avg_us =
        predict_count ? predict_us_sum / predict_count : 0;

    char line[224];
    const int len = snprintf(
        line, sizeof(line),
        "rpy %7.2f %7.2f %7.2f deg | bg %6.2f %6.2f %6.2f deg/s | "
        "sd %5.2f | pred %lu/%lu us | acc %lu/%lu | %s | "
        "rdfail %lu dtrej %lu rst %lu\r\n",
        (double)rpy.x(), (double)rpy.y(), (double)rpy.z(),
        (double)bg.x(), (double)bg.y(), (double)bg.z(),
        (double)att_sd_deg,
        (unsigned long)avg_us, (unsigned long)predict_us_max,
        (unsigned long)filter.accel_updates_applied,
        (unsigned long)filter.accel_updates_gated,
        still_counter >= kStillCount ? "still" : "moving",
        (unsigned long)read_failures, (unsigned long)dt_rejects,
        (unsigned long)filter_resets);

    predict_us_sum = 0;
    predict_us_max = 0;
    predict_count = 0;

    // Telemetry is lowest priority: drop the line rather than block.
    if (len > 0 && Serial.availableForWrite() >= len) {
        Serial.write((const uint8_t*)line, (size_t)len);
    }
}

// ---------------------------------------------------------------- arduino

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        // Wait for the USB CDC port, but never block when untethered.
    }

    Serial.println();
    Serial.println("longhorn baja firmware -- ESKF");
    if (watchdog_caused_reboot()) {
        Serial.println("WARNING: last reset was the watchdog");
    }

#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
    Wire.setSDA(kPinSda);
    Wire.setSCL(kPinScl);
#endif
    Wire.begin();
    Wire.setClock(400000);

    if (probe_imu()) {
        start_calibration();
    } else {
        Serial.println("MPU6050 not found -- retrying in background");
        mode = Mode::kImuProbe;
    }

    // Fed in exactly one place: the top of loop().
    rp2040.wdt_begin(kWatchdogMs);
}

void loop()
{
    rp2040.wdt_reset();

    const uint32_t now_us = micros();

    if (mode == Mode::kImuProbe) {
        if (now_us - last_probe_us >= kInitRetryUs) {
            last_probe_us += kInitRetryUs;
            if (probe_imu()) start_calibration();
        }
        return;
    }

    if (now_us - last_sample_us >= kSamplePeriodUs) {
        last_sample_us += kSamplePeriodUs;  // preserves the timebase

        ekf::Vec3 f_b, w_b;
        uint32_t t_us;
        if (!read_imu(f_b, w_b, t_us)) {
            read_failures++;
            if (read_failures % 100 == 0) {
                imu_configured = false;
                mode = Mode::kImuProbe;  // bus is gone; re-probe
            }
            return;
        }

        if (mode == Mode::kCalibrating) {
            calibration_step(f_b, w_b);
        } else {
            running_step(f_b, w_b, t_us);
        }
    }

    if (mode == Mode::kRunning && now_us - last_telem_us >= kTelemPeriodUs) {
        last_telem_us += kTelemPeriodUs;
        print_telemetry();
    }
}
