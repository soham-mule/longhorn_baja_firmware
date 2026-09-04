# Pin map — Baja Sensor ECU

Base routing from `Pico.SchDoc` (LBR-Wiring-Diagram, branch `main`, commit
`0c6c1ae3`). The IMU header now carries an **MPU6050** breakout (GY-521
style) instead of the BNO055 the board was drawn for — the header is plain
I2C, so this is a plug swap, no rework.

| Net | Pico pin | Peripheral | Connects to |
|---|---|---|---|
| `SDA0` | GP0 | I2C0 SDA | MPU6050 SDA |
| `SCL0` | GP1 | I2C0 SCL | MPU6050 SCL |
| `SPISCK0` | GP2 | SPI0 SCK | microSD SCK |
| `MOSI_GP3` | GP3 | SPI0 TX | microSD MOSI / DI |
| `MISO_GP4` | GP4 | SPI0 RX | microSD MISO / DO |
| `SPICS0` | GP5 | SPI0 CSn | microSD CS |
| `GPIO9` | GP9 | GPIO | IMU header pin 1 (INT; unused by firmware) |
| GPS RX | GP12 | UART1 TX | GT-U7 RX |
| GPS TX | GP13 | UART1 RX | GT-U7 TX |
| LED | GP25 | GPIO | onboard status LED |

- MPU6050 I2C address **0x68** (0x69 if AD0 tied high). Bus at 400 kHz.
- ⚠️ The old breadboard IMU wiring (SDA=GP4, SCL=GP5, as in
  `mpu6050_test.cpp`) **cannot be used on the ECU board** — GP4/GP5 are the
  SD card's MISO/CS there. The IMU belongs on the header (GP0/GP1).
- GPS: GT-U7 module (u-blox NEO-6M compatible), NMEA 9600 8N1, on flying
  leads until the next board revision adds a header. Its micro-USB port is
  for standalone u-center testing only — power it from the ECU in service.
- Free pins: GP6–GP8, GP10, GP11, GP14–GP22, GP26–GP28 (analog capable).

Full hardware context, power architecture, and bring-up procedure live in the
firmware specification / handoff document in `src/` (note: it describes the
BNO055 originally fitted; the IMU swap is recorded here and in CLAUDE.md).
