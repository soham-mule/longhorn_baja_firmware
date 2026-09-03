// MPU6050 test for Raspberry Pi Pico (Arduino framework)
// Uses the Adafruit MPU6050 driver (see lib_deps in platformio.ini).
//
// Wiring (I2C0, default Wire pins on the Pico):
//   MPU6050 VCC -> 3V3(OUT)  pin 36
//   MPU6050 GND -> GND       pin 38
//   MPU6050 SDA -> GP4       pin 6
//   MPU6050 SCL -> GP5       pin 7
//   AD0 left floating/low -> address 0x68 (tied to 3V3 -> 0x69)
//
// Most breakout boards have 4.7k pull-ups on SDA/SCL already. If yours
// does not, add them to 3V3.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// I2C pins. Only settable on the earlephilhower core; the mbed core is
// fixed at GP4/GP5.
static const uint8_t PIN_SDA = 4;
static const uint8_t PIN_SCL = 5;

static const uint8_t MPU_ADDR = 0x68;  // 0x69 if AD0 is high

Adafruit_MPU6050 mpu;

// Prints every address that ACKs, so a wrong-address or dead-bus problem is
// obvious from the serial log instead of showing up as a silent init failure.
static void scanBus() {
  Serial.println("I2C scan:");
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  device at 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("  none found - check wiring and pull-ups");
}

static void printField(float value, uint8_t decimals, const char *suffix) {
  Serial.print(value, decimals);
  Serial.print(suffix);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    // Wait for the USB CDC port, but do not block forever when running
    // untethered.
  }

  Serial.println();
  Serial.println("MPU6050 test");

#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
#endif
  Wire.begin();
  Wire.setClock(400000);

  scanBus();

  // begin() checks WHO_AM_I, resets the chip and wakes it from sleep.
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("MPU6050 not found at 0x68. Halting.");
    while (true) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(100);

  Serial.println("Init OK. ax ay az (m/s^2) | gx gy gz (deg/s) | temp (C)");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  if (!mpu.getEvent(&accel, &gyro, &temp)) {
    Serial.println("read failed");
    delay(500);
    return;
  }

  printField(accel.acceleration.x, 3, " ");
  printField(accel.acceleration.y, 3, " ");
  printField(accel.acceleration.z, 3, " | ");

  // The library reports gyro in rad/s; convert for readability.
  printField(gyro.gyro.x * RAD_TO_DEG, 2, " ");
  printField(gyro.gyro.y * RAD_TO_DEG, 2, " ");
  printField(gyro.gyro.z * RAD_TO_DEG, 2, " | ");

  printField(temp.temperature, 1, "\n");

  delay(100);
}
