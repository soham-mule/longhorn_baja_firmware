# longhorn_baja_firmware

> ⚠️ **Superseded in part (2026-09).** The authoritative design is the
> *Baja Sensor ECU: Firmware Specification and Handoff* in this directory,
> with one hardware amendment: the IMU on the ECU's I2C header is the
> **MPU6050** (0x68 on **GP0/GP1**, ±16 g / ±1000 °/s, raw counts), not the
> BNO055 the spec describes — see docs/pinmap.md. Everything else stands:
> microSD on SPI0 GP2–GP5, GT-U7 GPS on UART1 GP12/GP13, and **no fusion on
> target** (log raw, filter offline in Python; record contract v2 in
> `record.h`). The default `pico` env builds that datalogger (`main.cpp`,
> `mpu6050.*`, `gps.*`, `logger.*`, `record.h`). The old GP4/GP5 IMU wiring
> below is bench-only (`pico_imu_test`) — those pins are SD lines on the
> ECU board. The EKF material applies to the offline filter and the
> `pico_ekf` bench env. The coding constraints (floats, allocation,
> blocking, timing, types, ISRs) remain in force everywhere.

Data acquisition firmware for a Baja SAE vehicle. IMU → sensor fusion → logging → telemetry.

Longer background, rationale and teaching material live in `EMBEDDED_GUIDE.md`. This file is the operational contract.

---

## Hardware

| | |
|---|---|
| MCU | RP2040 (Raspberry Pi Pico) — dual Cortex-M0+, 125 MHz, 264 KB SRAM, 2 MB flash |
| **FPU** | **None.** Software float via bootrom routines |
| IMU | MPU6050, I2C address `0x68` (`0x69` if AD0 high) |
| Bus | I2C0 at 400 kHz — SDA on **GP4** (physical pin 6), SCL on **GP5** (physical pin 7) |
| Sensor power | `3V3(OUT)`, physical pin 36. **Never `VBUS`** |
| Logic level | 3.3 V, **not 5 V tolerant** |

---

## Build

PlatformIO, environment `pico`, Arduino framework on the **earlephilhower** RP2040 core (not mbed).

```
Build            → pio run
Upload + Monitor → pio run -t upload -t monitor
Serial           → 115200 (USB CDC; rate is nominal)
```

Pin reassignment (`Wire.setSDA/setSCL`) exists only on the earlephilhower core. Guard it:

```cpp
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
```

---

## Hard constraints — these override normal C++ habits

**No FPU.**
- `float` only. **Never `double`.**
- Every float literal ends in `f`: `0.5f`, not `0.5`
- Every math call uses the float variant: `sinf`, `cosf`, `atan2f`, `sqrtf`, `fabsf`
- `-Wdouble-promotion` is enabled — treat those warnings as errors

**No dynamic allocation.**
- No `malloc`, `free`, `new`, `delete`
- No Arduino `String` — use fixed `char` buffers and `snprintf`
- Eigen: **fixed-size types only** (`Matrix<float,15,15>`, never `MatrixXf`). `EIGEN_NO_MALLOC` is set, but it's a runtime assertion that `EIGEN_NO_DEBUG` disables — the guarantee is discipline, not the flag
- Large matrices are members or `static`, never function locals (stack)

**Nothing blocks.**
- No `delay()` in `loop()` or in any task
- Non-blocking scheduling only: `if (now - last >= period) { last += period; ... }`
- Use `last += period`, **not** `last = now` — preserves the timebase
- Check `availableForWrite()` before writing to a UART

**Timing.**
- `micros()` rolls over at ~71.6 min, `millis()` at ~49.7 days
- Always compare by subtraction: `now - last >= period`
- Compute `dt` from measured time, never from the nominal rate
- Sanity-check `dt` before using it: reject `<= 0` or `> 0.1f`

**Types.**
- Fixed-width everywhere: `uint8_t`, `int16_t`, `uint32_t`. Never bare `int`/`short`
- Raw IMU samples are `int16_t` (signed, big-endian on the wire)

**ISRs.**
- Set a flag or push to a ring buffer, then return. Nothing else
- No `delay()`, no printing, no allocation
- Every variable shared with an ISR must be `volatile`

---

## Conventions

**Frame: NED (North-East-Down).**
⚠️ **Z points down, so gravity is `[0, 0, +9.80665]`.** Sign errors here are the most common way this filter breaks.

**Units: SI internally, always.**
- Acceleration m/s², angular rate **rad/s**, angles rad, time s
- The Adafruit Unified Sensor API already gives m/s² and rad/s — feed those to the filter unconverted
- Convert to degrees **only** at a print or telemetry boundary, never upstream

**Naming.** `snake_case` for variables and functions, `kCamelCase` for enum constants (matching `ekf_state.h`), `UPPER_SNAKE` for macros. Filter code lives in `namespace ekf`.

**Every file that touches hardware documents its wiring in a header comment**, with both GP numbers and physical pin numbers.

---

## Layout

```
src/
  mpu6050_test.cpp   working IMU test sketch — owns setup()/loop()
  ekf_state.h        COMPLETE: types, 15-state layout, skew/deltaQ/inject
  ekf.h              EMPTY — needs the filter interface
  ekf.cpp            EMPTY — needs predict/update
  ekf_predict.h      EMPTY
include/             (unused)
lib/eigen/           only a stray CMakeLists.txt — Eigen is NOT vendored here
test/                empty
```

### The filter's state layout (`ekf_state.h`)

Error-state Kalman filter (ESKF). Two states, deliberately:

- **Nominal** — 16 params: `p`(3) `v`(3) `q`(quaternion, 4) `ba`(3) `bg`(3). Propagated by the IMU. Never appears in `dx`
- **Error `dx`** — 15 elements, attitude as a 3-element rotation vector

```
kPos  = 0    position error       (m)
kVel  = 3    velocity error       (m/s)
kAtt  = 6    attitude error       (rotation vector, rad)
kBAcc = 9    accelerometer bias   (m/s²)
kBGyr = 12   gyroscope bias       (rad/s)
kN    = 15
```

Cycle: propagate nominal → propagate `P` with the error Jacobian → compute `dx` from a measurement → `inject(s, dx)` folds it in and zeroes `dx`.

Position, velocity and bias errors are **additive**. Attitude error is **multiplicative** — `q = q * deltaQ(att(dx))`, then normalize.

---

## ⚠️ Known issues — read before building

**1. `platformio.ini` Eigen include path is broken.**
```ini
-I C:/dev/baja_pico/lib/eigen/eigen_src
```
Absolute path on another developer's machine, and Eigen is not vendored in this repo (`lib/eigen/` holds only a stray ESP-IDF `CMakeLists.txt`). Nothing includes `ekf_state.h` yet, so the build currently passes. **The first file that includes it will fail with `Eigen/Dense: No such file or directory`.**

Fix: vendor Eigen to `lib/eigen/eigen_src/` and change the flag to the relative `-I lib/eigen/eigen_src`. Coordinate with the repo owner — this is shared build config.

**2. `src/` compiles every `.cpp`.**
`ekf.cpp` is empty, so `mpu6050_test.cpp` currently owns the only `setup()`/`loop()`. **Adding a second `setup()` or `loop()` anywhere in `src/` breaks the link** with `multiple definition of 'setup'`.

Target structure: one `src/main.cpp` owns `setup()`/`loop()`; everything else is a module exposing functions. `ekf.cpp` must define `ekf::predict(...)` etc., never `setup()`. Move standalone test sketches out of `src/` or behind a separate environment with `build_src_filter`.

**3. The EKF is unimplemented.** `ekf.h`, `ekf.cpp`, `ekf_predict.h` are 0 bytes. `ekf_state.h` is the spec.

**4. Test-sketch settings are not vehicle settings.**
`mpu6050_test.cpp` uses ±2 g and ±250 °/s — **these will clip on a Baja car, silently.** Clipped data enters the filter with full confidence. Use ±8 g or ±16 g and ±500 or ±1000 °/s for vehicle runs. It also uses `delay(100)` (10 Hz, jittery) and halts forever on IMU init failure — both wrong for deployed firmware, which should retry and degrade.

---

## ⚠️ Observability — do not skip this

With **only the MPU6050** (6-axis), most of the 15 states are **not observable**. This is a property of the sensor set, not a bug to fix in code:

| State | Observable? |
|---|---|
| Roll, pitch | ✅ Yes — gravity is an absolute reference |
| Yaw | ❌ **No** — nothing distinguishes headings |
| Gyro bias x,y | ✅ Yes |
| Gyro bias z | ❌ No |
| Accel bias | ⚠️ Weakly, only under changing attitude |
| Velocity | ❌ **No** — error grows linearly, unbounded |
| Position | ❌ **No** — error grows **quadratically** |

Running the full 15-state filter IMU-only will diverge in position/velocity within about a minute. Expected, not a bug.

**Preferred path:** implement an attitude-only filter first (roll, pitch, gyro bias — fully observable, useful today), structured so aiding sensors are added as new `update*()` functions rather than a rewrite. Highest-value aiding sensor for a Baja car is a **wheel-speed sensor** (makes velocity observable, cheap, robust); then GPS (position/velocity/heading); magnetometer only with on-vehicle hard/soft-iron calibration. ZUPT (zero-velocity updates when stationary) is free and worth adding regardless.

---

## Numerical practice for the filter

- **Joseph form** for the covariance update: `P = (I-KH)P(I-KH)ᵀ + KRKᵀ`. The simple form loses symmetry in float32 and diverges
- Force symmetry periodically: `P = 0.5f * (P + P.transpose())`
- Renormalize the quaternion every predict step
- **Gate the accelerometer attitude update** — reject or inflate `R` when `abs(a.norm() - 9.80665f)` is large; the accelerometer only indicates "down" when not accelerating
- Innovation gating (NIS check) before applying any update
- Check for NaN and reinitialize — NaN is contagious and permanent
- Log `P`'s diagonal during development; covariance collapse is a classic silent failure
- ⚠️ Watch Eigen aliasing: prefer `A = (A * B).eval()` for matrix products

**Cost.** Covariance propagation `Φ·P·Φᵀ` is ~6,750 float multiply-accumulates per predict step. At 200 Hz on a 125 MHz core with no FPU this is a real budget question — **measure it with `micros()` before designing around a rate.** If too slow: lower the rate, exploit `F`'s block sparsity, exploit `P`'s symmetry, or reduce the state (6-state attitude filter is 15× cheaper than 15-state).

---

## Development approach

**Build in verifiable stages.** Do not write the whole filter and then test it.

1. Stable timestamped sampling — verify `dt` spread before anything else
2. Gyro bias calibration at startup (guard against calibrating while moving)
3. Complementary filter — 4 lines, works, and becomes the reference to check the EKF against
4. Nominal propagation only, no covariance
5. Add covariance propagation — verify `P` grows, stays symmetric and positive
6. Add the accelerometer attitude update
7. Gating and tuning

**Test the filter on the desktop.** It's pure math with no hardware dependency. Log raw IMU data to a file, replay it through the same code on a laptop with plots and a debugger, then compile the identical code for the Pico. Far faster than iterating on hardware.

**Debugging.** `scanBus()` in `mpu6050_test.cpp` splits electrical faults from software faults — run it first, always. Nothing found → wiring, power, or pull-ups. Found at `0x68` → hardware is fine. A second Pico as a picoprobe gives real SWD breakpoints and is worth the ~$12.

---

## Logging and telemetry

- **Binary records, not CSV.** Float formatting is expensive with no FPU; convert offline
- **Log raw `int16_t` counts**, not scaled floats — smaller, lossless, and re-derivable. Store the configured range in a file header
- Timestamp every record with `micros()`; include a wrapping sequence byte so dropped records are visible
- Buffer to 512-byte blocks; never write per-sample
- Ring buffer between sampler and writer — SD cards stall unpredictably for 100 ms+
- `SdFat` over the stock `SD` library; `preAllocate()` the file
- `file.sync()` every few seconds — power loss on a vehicle is guaranteed eventually
- Don't overwrite `log.bin`; scan for the next free index
- **Telemetry is a summary, not a subset.** Link bandwidth is 10–100× below the log rate. Send derived values (attitude, peak accel, health, drop count) at a few Hz, framed with sync bytes + length + CRC
- Telemetry is the lowest-priority task; drop packets before dropping samples

---

## Vehicle context

Vibration, ignition noise, brownouts during cranking, heat, and dust. When something works on the bench and fails on the car, **suspect power and mechanical connections before firmware.** Mechanically isolate the IMU — vibration aliasing (7.4 in the guide) is a hardware problem no filter fixes. Enable the watchdog, feed it in exactly one place at the top level, and record when a watchdog reset occurred.
