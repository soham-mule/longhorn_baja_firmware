#include "mpu6050.h"

static const uint8_t kRegSmplrtDiv  = 0x19;
static const uint8_t kRegConfig     = 0x1A;  // DLPF
static const uint8_t kRegGyroCfg    = 0x1B;
static const uint8_t kRegAccelCfg   = 0x1C;
static const uint8_t kRegDataStart  = 0x3B;  // ACCEL_XOUT_H
static const uint8_t kRegPwrMgmt1   = 0x6B;
static const uint8_t kRegWhoAmI     = 0x75;

static const uint8_t kWhoAmI        = 0x68;
static const uint8_t kBurstLen      = 14;    // accel 6 + temp 2 + gyro 6

// Full-scale selections. Changing these means changing the values below
// AND nothing else -- the header records them for the parser.
static const uint8_t kAccelCfgFs    = 0x18;  // AFS_SEL=3 -> +/-16 g
static const uint8_t kGyroCfgFs     = 0x10;  // FS_SEL=2  -> +/-1000 dps
static const uint8_t kAccelFsG      = 16;
static const uint16_t kGyroFsDps    = 1000;

static TwoWire* bus = nullptr;
static uint8_t dev_addr = 0x68;

static bool write_reg(uint8_t reg, uint8_t val)
{
    bus->beginTransmission(dev_addr);
    bus->write(reg);
    bus->write(val);
    return bus->endTransmission() == 0;
}

static bool read_regs(uint8_t reg, uint8_t* dst, uint8_t len)
{
    bus->beginTransmission(dev_addr);
    bus->write(reg);
    if (bus->endTransmission(false) != 0) return false;  // repeated start
    if (bus->requestFrom(dev_addr, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) dst[i] = (uint8_t)bus->read();
    return true;
}

// MPU6050 data registers are BIG-endian: high byte first.
static int16_t be16(const uint8_t* p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static bool configure()
{
    // Clock from the gyro X PLL (more stable than the internal RC) and
    // clear the SLEEP bit -- the chip boots asleep.
    if (!write_reg(kRegPwrMgmt1, 0x01)) return false;
    delay(10);
    // DLPF 44 Hz accel / 42 Hz gyro: under the 50 Hz Nyquist of the
    // 100 Hz log rate, and knocks down engine vibration.
    if (!write_reg(kRegConfig, 0x03)) return false;
    // With DLPF on, the internal rate is 1 kHz; divide to 100 Hz so the
    // registers refresh in step with the sample loop.
    if (!write_reg(kRegSmplrtDiv, 9)) return false;
    if (!write_reg(kRegGyroCfg, kGyroCfgFs)) return false;
    if (!write_reg(kRegAccelCfg, kAccelCfgFs)) return false;
    return true;
}

bool mpu6050_init(TwoWire& wire, uint8_t addr)
{
    bus = &wire;
    dev_addr = addr;

    uint8_t id = 0;
    if (!read_regs(kRegWhoAmI, &id, 1) || id != kWhoAmI) return false;

    if (!write_reg(kRegPwrMgmt1, 0x80)) return false;  // device reset
    delay(100);
    return configure();
}

bool mpu6050_reinit()
{
    if (bus == nullptr) return false;
    uint8_t id = 0;
    if (!read_regs(kRegWhoAmI, &id, 1) || id != kWhoAmI) return false;
    return configure();
}

bool mpu6050_read_sample(Mpu6050Sample& out)
{
    uint8_t raw[kBurstLen];
    if (!read_regs(kRegDataStart, raw, kBurstLen)) return false;

    for (int i = 0; i < 3; i++) out.accel[i] = be16(raw + 2 * i);
    out.temp = be16(raw + 6);
    for (int i = 0; i < 3; i++) out.gyro[i] = be16(raw + 8 + 2 * i);
    return true;
}

uint8_t mpu6050_accel_fs_g() { return kAccelFsG; }
uint16_t mpu6050_gyro_fs_dps() { return kGyroFsDps; }
