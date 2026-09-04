# Baja Sensor ECU: Firmware Specification and Handoff

Longhorn Baja Racing, car 185. RP2040 telemetry logger.
Written as a knowledge transfer document. Assumes no prior context on this project.

---

## 1. What this is

A datalogger for the Baja SAE vehicle. It samples a BNO055 inertial measurement unit and a GPS receiver, timestamps both onto one clock, and writes fixed size binary records to a microSD card. Analysis happens offline in Python.

The board is a custom two layer PCB built around a Raspberry Pi Pico. Design files live in `BAJASENSORS.PrjPcb` on branch `main` of the `LBR-Wiring-Diagram` repository, at commit `0c6c1ae3`.

**Scope note.** The original design carried seven channels: two Hall effect wheel speed sensors, brake pressure, CVT temperature, an IMU, a driver LCD and a cooling fan driver. A budget reduction in July 2026 cut the built board to IMU and SD logging only. The removed channels remain drawn, conditioned and routed in version control, so they can be populated on a future board without redesign.

---

## 2. Hardware baseline

### 2.1 Power architecture, as modified

The board as fabricated generated its 5 V rail from a resistor divider buffered by a TLV2372 op amp. That arrangement could not supply an SD card's write current and has been replaced.

Current architecture:

```
12 V vehicle supply
   |
  fuse
   |
  LDO, 12 V to 5 V
   |
   +-- Pico VSYS
   |
   +-- BNO055 breakout VIN
   |
  Pico internal regulator
   |
  3.3 V
   |
   +-- RP2040 core, and the GPIO logic levels
   |
   +-- microSD card
```

Decoupling and I2C pull ups have been added. The TLV2372 path is no longer in use.

**Thermal note for whoever maintains this.** A linear regulator dropping 12 V to 5 V dissipates the difference as heat. Expected load is roughly 30 mA for the Pico core, 12 mA for the BNO055, and up to 100 mA reflected from the SD card through the Pico's regulator, so call it 110 mA peak. That gives:

```
P = (12 - 5) x 0.110 = 0.77 W
```

A TO-220 package without a heatsink has a junction to ambient thermal resistance around 50 to 65 degC per watt, so expect a 40 to 50 degC rise above ambient. Fit at least a small heatsink, and measure the case temperature after a sustained logging run. If it runs too hot in the engine bay, replace the LDO with a switching buck converter, which does the same job at roughly 90 percent efficiency instead of 42 percent.

**SD card supply note.** The card is powered from the Pico's own 3.3 V output rather than a dedicated regulator. The Pico's regulator can supply this comfortably, but the card and the microcontroller now share a rail, so a large enough current burst from the card can disturb the MCU. The mitigation is bulk capacitance on 3.3 V physically close to the card. Confirm a bulk electrolytic of a few hundred microfarads is fitted there, plus a 100 nF ceramic. If the Pico resets during heavy logging, this is the first thing to suspect.

### 2.2 Pin map

Confirmed from `Pico.SchDoc`. Every net sits on its native RP2040 peripheral function.

| Net | Pico pin | Peripheral | Connects to |
|---|---|---|---|
| `SDA0` | **GP0** | I2C0 SDA | BNO055 SDA |
| `SCL0` | **GP1** | I2C0 SCL | BNO055 SCL |
| `GPIO9` | **GP9** | GPIO | BNO055 INT or RST |
| `SPISCK0` | **GP2** | SPI0 SCK | microSD SCK |
| `MOSI_GP3` | **GP3** | SPI0 TX | microSD MOSI / DI |
| `MISO_GP4` | **GP4** | SPI0 RX | microSD MISO / DO |
| `SPICS0` | **GP5** | SPI0 CSn | microSD CS |
| GPS TX | **GP13** | UART1 RX | GPS module TX |
| GPS RX | **GP12** | UART1 TX | GPS module RX |

**Why UART1 rather than UART0.** I2C0 occupies GP0 and GP1, which are also the default UART0 pins. UART0 is therefore unavailable. Of the UART1 pin options, GP4/GP5 collide with SPI and GP8/GP9 collides with the IMU interrupt line, which leaves GP12/GP13 free.

GPS is not on the PCB. It connects on flying leads to the Pico header. A four pin GPS header belongs on the next board revision.

### 2.3 Header pinouts

**IMU header, TSW-106-07-S-S, 1x6, 2.54 mm**

| Pin | Net |
|---|---|
| 1 | `GPIO9` |
| 2 | `SDA0` |
| 3 | `SCL0` |
| 4 | `GND` |
| 5 | not connected |
| 6 | `5V+` |

**SD header, 1x6**

| Pin | Net |
|---|---|
| 1 | `GND` |
| 2 | `5V+`, now unused, card runs from Pico 3.3 V |
| 3 | `MOSI_GP3` |
| 4 | `MISO_GP4` |
| 5 | `SPISCK0` |
| 6 | `SPICS0` |

### 2.4 Free pins

GP6, GP7, GP8, GP10, GP11, GP14 through GP22, and GP26 through GP28 for analog. Available for the channels that were descoped, or for status LEDs.

---

## 3. Toolchain

### 3.1 Setup

VS Code with the **PlatformIO IDE** extension. PlatformIO pulls its own compiler toolchain, so nothing else needs installing. Do not also install the Arduino IDE extension, the two contend for serial ports.

### 3.2 platformio.ini

```ini
[env:pico]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = pico
framework = arduino
board_build.core = earlephilhower

monitor_speed = 115200
upload_protocol = picotool

build_flags =
    -Wall
    -Wextra
    -DPICO_STDIO_USB=1

lib_deps =
    greiman/SdFat @ ^2.2.2
    mikalhart/TinyGPSPlus @ ^1.0.3
```

**Why the platform is a URL.** The official PlatformIO `raspberrypi` platform has historically shipped only the Arduino Mbed core, which has weaker dual core and SPI support. Maxgerhardt's fork provides the earlephilhower `arduino-pico` core, which is the better one. Check whether official support has caught up before assuming this line is still needed.

**Why no BNO055 library.** The driver is written in this project. See section 6.

**First upload.** Hold BOOTSEL while connecting USB. Subsequent uploads reset automatically.

---

## 4. Architecture

### 4.1 The central decision: log raw, filter offline

The firmware performs **no sensor fusion and no filtering**. It timestamps raw sensor output and writes it to the card. All filtering, including the Kalman filter, runs in Python on a laptop against the logged files.

Reasons, in order of importance:

1. **Iteration speed.** Retuning a filter on target means a reflash and another vehicle run. Offline it is a rerun of a script against the same data.
2. **Data safety.** A filter bug on target corrupts the log and destroys the raw data that would have let you diagnose it.
3. **Comparability.** Filter variants can be run against identical input, which is impossible on target.
4. **Raw data has independent value.** The chassis and suspension teams want raw accelerations, not a fused position estimate.

Fusion belongs on target only when something on the vehicle must react to the estimate in real time. A datalogger does not.

### 4.2 What the BNO055 already does

The BNO055 contains a Cortex-M0 running Bosch's sensor fusion. In fusion mode it outputs, over I2C, already fused:

- Quaternion, absolute orientation
- Euler angles
- Linear acceleration with gravity removed
- Gravity vector

**No attitude filter is required in firmware or in post processing.** The remaining filtering problem is fusing GPS position with the BNO's linear acceleration to estimate position and velocity, which is a different and much smaller problem.

### 4.3 Operating mode: IMUPLUS, not NDOF

Use **IMUPLUS**, register value `0x08`. This is six degree of freedom fusion using accelerometer and gyroscope only, with the magnetometer disabled.

NDOF, the nine degree of freedom mode, uses the magnetometer for absolute heading. On this vehicle the magnetometer sits inside a steel space frame, adjacent to an engine ignition system and DC current paths. Magnetic heading in that environment is noise.

Heading comes from **GPS course over ground** instead. Relative heading from the gyro is still available and drifts slowly, which GPS corrects.

### 4.4 Task split across the two cores

The RP2040 has two cores. Use both.

- **Core 0** runs the sample loop at a fixed 100 Hz. It reads the BNO055, drains any waiting GPS bytes, assembles a record, and pushes it into a ring buffer. It never touches the SD card.
- **Core 1** drains the ring buffer to the SD card whenever the card will accept data.

This matters because SD cards stall unpredictably. See section 8.

The `arduino-pico` core exposes the second core as `setup1()` and `loop1()`. The core also ships FreeRTOS support if a task and queue structure is preferred, which is a reasonable alternative and closer to how this would be written on a professional RTOS.

**Build the single core version first.** Get it working, measure the dropped sample rate, then move to dual core and measure the improvement. That comparison is worth having.

---

## 5. Record format

### 5.1 The struct

```c
#define LOG_MAGIC    0x424A4C31u   // "BJL1"
#define LOG_VERSION  1

typedef struct __attribute__((packed)) {
    uint32_t magic;        // LOG_MAGIC
    uint16_t version;      // LOG_VERSION
    uint16_t record_bytes; // sizeof(sample_t)
    uint32_t boot_id;      // random per power cycle, ties files together
    uint32_t gps_utc;      // UTC seconds at first fix, 0 if never fixed
    uint32_t t_us_at_fix;  // micros() at that same instant
} file_header_t;

typedef struct __attribute__((packed)) {
    uint32_t t_us;          // micros() at the instant of sampling
    int16_t  quat[4];       // w, x, y, z. raw counts, divide by 16384
    int16_t  lin_accel[3];  // x, y, z. raw counts, divide by 100 for m/s^2
    int16_t  gyro[3];       // x, y, z. raw counts, divide by 16 for deg/s
    uint8_t  calib;         // packed, 2 bits each: sys, gyr, acc, mag
    uint8_t  flags;         // bit 0: GPS fix valid, bit 1: new GPS this sample
    int32_t  lat_e7;        // degrees x 1e7
    int32_t  lon_e7;        // degrees x 1e7
    uint16_t speed_cms;     // cm/s
    uint16_t course_cdeg;   // centidegrees
    uint8_t  sats;
    uint8_t  hdop_tenths;
} sample_t;                 // 40 bytes
```

### 5.2 Why it is shaped this way

- **Raw sensor counts, not floats.** Smaller, faster, and lossless. Convert in Python using the scale factors in section 6.4.
- **Packed and fixed size.** The Python parser can index directly to record N without scanning.
- **Magic number and version in the header.** The struct will change. Without a version field, a future parser cannot tell which layout it is reading, and a dataset becomes unreadable.
- **`boot_id`** is a random value generated at startup. It lets you tell whether two files came from the same power cycle.
- **GPS time paired with a `micros()` value.** `micros()` gives precise relative timing but resets on power cycle and has no absolute reference. Recording GPS UTC alongside the `micros()` value at that instant lets the entire log be converted to wall clock time offline, which is how you align telemetry with video or with another team's data.

### 5.3 Data rate

```
40 bytes x 100 Hz = 4.0 kB/s
             = 14.4 MB per hour
             = 58 MB over a four hour endurance run
```

A 64 GB card holds roughly 4,400 hours. Capacity is not a constraint.

The card is rated V30, meaning 30 MB/s sustained sequential write. The requirement is 4 kB/s, which is **0.013 percent of the card's capability**. Throughput is not the design problem. Latency is.

---

## 6. Module: BNO055 driver

Write this from the datasheet rather than using a library. It is a contained problem, the datasheet is good, and it is where the peripheral level understanding is.

### 6.1 Bus configuration

- I2C0, SDA on GP0, SCL on GP1
- Address **0x28**. It is 0x29 if the breakout pulls its ADR pin high. Scan the bus if unsure.
- Start at **100 kHz**. The BNO055 has a documented clock stretching behaviour that is out of I2C specification and has caused problems on some hosts. If reads hang intermittently, this is the first suspect and dropping the clock is the first remedy.

### 6.2 Registers used

All on register page 0. Verify against the datasheet before relying on these.

| Address | Name | Purpose |
|---|---|---|
| `0x00` | CHIP_ID | Must read `0xA0`. First thing to check. |
| `0x07` | PAGE_ID | Write `0x00` to select page 0 |
| `0x14` to `0x19` | GYR_DATA_X/Y/Z | Gyroscope, 6 bytes, little endian |
| `0x20` to `0x27` | QUA_DATA_W/X/Y/Z | Quaternion, 8 bytes, little endian |
| `0x28` to `0x2D` | LIA_DATA_X/Y/Z | Linear acceleration, 6 bytes |
| `0x35` | CALIB_STAT | Calibration status, see 6.5 |
| `0x39` | SYS_STATUS | System status |
| `0x3A` | SYS_ERR | Error code if SYS_STATUS reports a fault |
| `0x3B` | UNIT_SEL | Unit selection |
| `0x3D` | OPR_MODE | Operating mode |
| `0x3E` | PWR_MODE | Power mode |
| `0x3F` | SYS_TRIGGER | Reset and clock select |
| `0x55` to `0x6A` | Offsets | Calibration offsets, readable and writable |

### 6.3 Initialisation sequence

1. **Wait after power on.** The chip needs roughly 650 ms after reset before it responds. Skipping this is the most common cause of a failed bring up.
2. Read `CHIP_ID` at `0x00`. **Must be `0xA0`.** If not, stop. Nothing downstream will work, and the fault is wiring, address, or pull ups.
3. Write `0x00` to `OPR_MODE` to enter CONFIGMODE. Allow 19 ms.
4. Write `0x00` to `PAGE_ID`.
5. Set `PWR_MODE` to normal, value `0x00`.
6. Configure `UNIT_SEL` for the units you want, and **record the choice in code comments**, because the scale factors in 6.4 depend on it.
7. Write `0x08` to `OPR_MODE` to enter IMUPLUS. Allow 7 ms.
8. Read `CALIB_STAT` and log it.

Mode transition timing matters. CONFIGMODE to an operating mode takes 7 ms, an operating mode back to CONFIGMODE takes 19 ms. Writing registers before the transition completes silently fails.

### 6.4 Scale factors

Values arrive as signed 16 bit little endian. Assemble as `(int16_t)(lsb | (msb << 8))`.

| Quantity | Conversion |
|---|---|
| Quaternion | divide by 16384, which is 2^14 |
| Linear acceleration | divide by 100 for m/s² |
| Gyroscope | divide by 16 for degrees/s, or by 900 for radians/s |
| Euler angles | divide by 16 for degrees |
| Gravity vector | divide by 100 for m/s² |
| Temperature | 1 count per degree C |

Store raw counts in the log. Apply these in Python.

### 6.5 Calibration

`CALIB_STAT` at `0x35` packs four two bit fields, each 0 to 3 where 3 is fully calibrated:

- Bits 7:6 system
- Bits 5:4 gyroscope
- Bits 3:2 accelerometer
- Bits 1:0 magnetometer

In IMUPLUS the magnetometer field is not meaningful. Watch gyroscope and accelerometer.

**Log the calibration byte with every sample.** Without it you cannot tell afterwards whether a run's data is trustworthy.

Gyroscope calibrates by leaving the board still for a few seconds. Accelerometer calibrates by holding the board in several distinct orientations.

Once the basics work, read the offset registers at `0x55` to `0x6A` after a good calibration, store them, and write them back at startup. That avoids recalibrating every power cycle.

### 6.6 Reading efficiently

Read the quaternion, linear acceleration and gyroscope in as few I2C transactions as possible. The register map is contiguous from `0x14` through `0x2D`, so a single burst read of that range captures gyroscope, quaternion and linear acceleration in one transaction. At 100 Hz on a 100 kHz bus, transaction count matters.

---

## 7. Module: GPS

### 7.1 Configuration

- UART1, TX on GP12, RX on GP13
- Baud rate typically 9600, 8 data bits, no parity, 1 stop bit
- Parsing via TinyGPSPlus

Writing an NMEA parser from scratch teaches little after the first sentence type. Use the library, but read its source once to understand what it does.

### 7.2 What to log

Every field, every time:

latitude, longitude, altitude, speed, course over ground, fix quality, satellite count, HDOP, and UTC time.

HDOP and satellite count tell you afterwards how much to trust a fix. Discarding them means discarding the ability to filter bad data later.

### 7.3 Rate mismatch

The IMU runs at 100 Hz. The GPS produces a fix at 1 to 10 Hz. **This mismatch is the reason the Kalman filter exists.** The firmware does not attempt to reconcile it. It records the most recent GPS values in every sample, sets the `new GPS this sample` flag on the sample where a fresh fix arrived, and leaves the rest to offline analysis.

### 7.4 Bring up note

A GPS with no stored almanac can take several minutes to first fix from cold. Testing indoors usually produces nothing at all. Test near a window or outside, and be patient before concluding the module is faulty.

---

## 8. Module: SD logging

This is the highest risk part of the project.

### 8.1 The problem

SD cards are not steadily fast. They are fast on average with unpredictable stalls. Flash cannot be overwritten in place, it must be erased first, and erase happens in blocks far larger than a single write. The card's internal controller performs wear levelling and garbage collection on its own schedule. When it does, the card asserts busy and holds it, sometimes for **tens or hundreds of milliseconds**.

If the sample loop waits on the card, samples are dropped every time this happens.

### 8.2 The solution

A ring buffer between sampling and writing. SdFat provides a `RingBuf` class built for this.

Requirements:

- **Use SdFat, not the stock Arduino SD library.** SdFat is faster and supports exFAT, which a 64 GB SDXC card ships formatted as. The stock library cannot read exFAT at all.
- **Pre-allocate a contiguous file** with `preAllocate()`. Growing a file during logging forces FAT table updates, which is another stall source.
- **Write in 512 byte multiples.** That is the SD sector size. Unaligned writes force read modify write cycles.
- **Sync periodically, not per sample.** Every few seconds. A sync flushes metadata so a power loss costs the last few seconds rather than the whole file.
- **Size the buffer from measurement.** Riding out a 250 ms stall at 4 kB/s needs 1 kB. A 16 kB buffer covers a 4 second stall. The RP2040 has 264 kB of RAM, so be generous.

### 8.3 File naming

Use an incrementing index, `LOG0001.BIN` and so on, scanning at startup for the highest existing number. Do not use timestamps in filenames, because there is no wall clock available until the GPS acquires a fix, which may be minutes after logging begins.

---

## 9. Module: timing

### 9.1 Fixed rate loop

Do not use `delay()`. It sets the gap between iterations rather than the period, so the actual rate varies with whatever work the loop does.

Use a deadline pattern: hold a `next_us` variable, add the period each iteration, and wait until `micros()` reaches it.

### 9.2 The overflow trap

`micros()` returns a 32 bit microsecond counter which wraps at approximately **71 minutes**. An endurance run is four hours, so it will wrap, three times.

**Unsigned subtraction handles the wrap correctly. Direct comparison does not.**

```c
// correct
if ((int32_t)(micros() - next_us) >= 0) { ... }

// wrong, fails at 71 minutes
if (micros() >= next_us) { ... }
```

This is the single most likely bug to survive bench testing and then destroy an endurance run.

### 9.3 Jitter measurement

Record the actual `micros()` at each sample rather than assuming a perfect 10 ms cadence. Comparing consecutive timestamps offline gives a jitter distribution, which is both a health check and evidence for the design review.

---

## 10. Module: robustness

- **Watchdog timer.** Enable it, feed it from the main loop. Without one, a hang means no data for the rest of the run and nobody notices until the card is pulled. Log the reset reason at startup so a watchdog reset is visible in the data.
- **Status LED.** One pattern for logging normally, another for SD failure, another for no IMU. The driver needs to know the system is alive before a run, not after.
- **Startup behaviour with no card.** Decide and document whether the firmware halts or continues. Halting is usually correct, because a run that logs nothing is worse than a run that does not start.
- **Flush on a schedule.** Combined with pre-allocation, this bounds data loss on a power cut.

---

## 11. Bring up procedure

Each step must pass before starting the next. Do not write the whole thing and then debug it.

| Step | Action | Pass criteria |
|---|---|---|
| 1 | Measure the 5 V rail unloaded, then with 100R, 47R and 22R loads | Rail holds within a few percent at 22R, about 227 mA |
| 2 | Measure the LDO case temperature after 10 minutes at load | Stable and within the part's rating, heatsink fitted if needed |
| 3 | Confirm the SD card supply is 3.3 V, and confirm MISO idles at 3.3 V | Never 5 V on any line into the Pico |
| 4 | Blink, flash and serial monitor | Upload works, output visible at 115200 |
| 5 | 100 Hz fixed rate loop with jitter print | 100 lines per second, jitter under a few hundred microseconds |
| 6 | I2C bus scan | Device found at 0x28 or 0x29 |
| 7 | Read BNO055 CHIP_ID | Returns 0xA0 |
| 8 | Enter IMUPLUS, read quaternion | Board level reads approximately 1, 0, 0, 0 |
| 9 | Read linear acceleration, board still | Approximately 0, 0, 0. Proves gravity removal works |
| 10 | Mount SD card, write a test file | File appears and is readable on a computer |
| 11 | Ring buffer logging, single core, 10 minutes | Sample count matches elapsed time at 100 Hz |
| 12 | Move the writer to core 1 | Dropped sample count falls, measure the difference |
| 13 | GPS raw NMEA dump to serial | Sentences appear, outdoors, after a cold start wait |
| 14 | Full record logging with GPS | Parser reads the file, values plausible |
| 15 | One hour bench run | No dropped samples, no resets, file intact |
| 16 | Run past 71 minutes | Confirms the `micros()` wrap is handled |

Step 16 is not optional. It is the only way to catch the overflow bug before it costs a real run.

---

## 12. Offline analysis

### 12.1 Parser

Python, using the `struct` module with a format string matching `sample_t` exactly. Read and check the file header first, and refuse to parse a version the script was not written for.

### 12.2 Filter development order

Do not start at the Kalman filter.

1. **Complementary filter.** A few lines. Teaches the core idea, which is trusting one sensor over short intervals and another over long ones.
2. **One dimensional linear Kalman filter.** Estimate a single position from a noisy measurement. Understand state, covariance, process noise Q, measurement noise R, and what the gain does.
3. **The actual problem.** State is position and velocity in a local flat earth frame. Predict using the BNO's linear acceleration rotated into the world frame by the quaternion. Update whenever a GPS fix arrives.

The hard part is not the mathematics. It is tuning Q and R, and before that, time alignment and unit conversion.

Work in a notebook and plot everything. Plot the raw GPS track, the filtered track, and both together.

### 12.3 Repository layout

```
firmware/
  platformio.ini
  src/
    main.cpp
    bno055.cpp / bno055.h
    gps.cpp / gps.h
    logger.cpp / logger.h
    record.h
docs/
  pinmap.md
  record_format.md
analysis/
  parse_log.py
  kalman.ipynb
```

`record.h` is the contract between the firmware and the Python parser. When it changes, bump `LOG_VERSION` and note the change in `docs/record_format.md`.

---

## 13. Troubleshooting

| Symptom | First things to check |
|---|---|
| BNO055 CHIP_ID reads 0x00 or 0xFF | Wiring, address 0x28 versus 0x29, I2C pull ups, the 650 ms power on wait |
| I2C hangs intermittently | Drop the bus to 100 kHz. The BNO055 clock stretches out of specification |
| Quaternion never changes | Still in CONFIGMODE. Check the OPR_MODE write and the 7 ms delay |
| Linear acceleration is not near zero when still | Accelerometer not calibrated. Check CALIB_STAT bits 3:2 |
| SD mount fails | Card is exFAT and the stock SD library is in use. Use SdFat |
| Dropped samples during logging | Ring buffer too small, or the writer is on core 0 |
| Pico resets under heavy logging | 3.3 V rail sagging under SD write current. Add bulk capacitance close to the card |
| Logging stops around 71 minutes | The `micros()` overflow bug in section 9.2 |
| GPS produces nothing | Indoors, or wrong baud rate. Dump raw bytes before parsing |
| Timestamps drift against wall clock | Expected. `micros()` is relative. Use the GPS UTC pairing in the header |

---

## 14. Handoff checklist

For whoever inherits this project:

- [ ] Read `docs/pinmap.md` and verify it against `Pico.SchDoc` in Altium
- [ ] Confirm the power architecture matches section 2.1, since it was modified after fabrication
- [ ] Measure the LDO case temperature under load before trusting it in the vehicle
- [ ] Confirm `record.h` and `analysis/parse_log.py` agree, and that `LOG_VERSION` matches
- [ ] Run the bring up table in section 11 from step 4, on a known good board
- [ ] Check whether the descoped channels are being restored, and if so read the original design on branch `2027_Wiring_Diagram`
- [ ] Confirm whether the Altium 365 workspace `BAJASENSORS_OFFICIAL` holds design work newer than the GitHub repository

### Known open items

1. GPS is on flying leads. It needs a proper header on the next board revision.
2. The board has no fuse and no reverse polarity protection on the 12 V input beyond what was added externally.
3. The unused half of the TLV2372 has floating inputs. It should be removed or tied off.
4. The battery designator on the silkscreen reads `Baattery`. Fix in the schematic.
5. No firmware has yet logged data on the vehicle. Every performance figure in the design review is calculated, not measured.
