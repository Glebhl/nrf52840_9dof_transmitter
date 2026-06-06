#include "ICM45686.h"

#include <Arduino.h>

namespace {

// --- ICM45686 User Bank 0 registers -----------------------------------------
constexpr uint8_t REG_ACCEL_DATA_X1 = 0x00;  // start of the 14-byte data block
constexpr uint8_t REG_PWR_MGMT0     = 0x10;
constexpr uint8_t REG_ACCEL_CONFIG0 = 0x1B;
constexpr uint8_t REG_GYRO_CONFIG0  = 0x1C;
constexpr uint8_t REG_FIFO_CONFIG0  = 0x1D;
constexpr uint8_t REG_FIFO_CONFIG3  = 0x21;
constexpr uint8_t REG_WHO_AM_I      = 0x72;
constexpr uint8_t REG_IREG_ADDR_HI  = 0x7C;  // indirect-access burst start

// --- Indirect (IREG) banks, -------------------------------------------------
constexpr uint8_t IPREG_BAR        = 0xA0;
constexpr uint8_t IPREG_BAR_REG_58 = 0x3A;
constexpr uint8_t IPREG_BAR_REG_59 = 0x3B;
constexpr uint8_t IPREG_TOP1       = 0xA2;
constexpr uint8_t SREG_CTRL        = 0x67;

// --- Field values -----------------------------------------------------------
constexpr uint8_t ACCEL_MODE_LN = 0x03;  // low-noise
constexpr uint8_t GYRO_MODE_LN  = 0x03;  // low-noise
constexpr uint8_t ODR_100HZ     = 0x09;

constexpr int16_t kFullScaleCounts = 32768;  // 2^15

// Map a desired accel range (g) to its ACCEL_UI_FS_SEL field value.
// 0=±32g, 1=±16g, 2=±8g, 3=±4g, 4=±2g.
uint8_t accelFsSel(uint16_t fsrG) {
  switch (fsrG) {
    case 32: return 0;
    case 16: return 1;
    case 8:  return 2;
    case 4:  return 3;
    case 2:  return 4;
    default: return 1;  // ±16g
  }
}

// Map a desired gyro range (dps) to its GYRO_UI_FS_SEL field value.
// 0=±4000, 1=±2000, 2=±1000, 3=±500, 4=±250, 5=±125 dps.
uint8_t gyroFsSel(uint16_t fsrDps) {
  switch (fsrDps) {
    case 4000: return 0;
    case 2000: return 1;
    case 1000: return 2;
    case 500:  return 3;
    case 250:  return 4;
    case 125:  return 5;
    default:   return 1;  // ±2000 dps
  }
}

inline int16_t beInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

}  // namespace

bool ICM45686::begin() {
  using S = I2CBus::Status;

  accelScale_ = (float)config_.accelFsrG / kFullScaleCounts;
  gyroScale_  = (float)config_.gyroFsrDps / kFullScaleCounts;

  // A dummy read wakes the I2C interface; the first access after power-up can
  // be flaky, so the WHO_AM_I probe gets one retry.
  uint8_t dummy = 0;
  bus_.readRegister(address_, REG_ACCEL_DATA_X1, &dummy);
  delayMicroseconds(100);

  S s = bus_.readRegister(address_, REG_WHO_AM_I, &lastWhoAmI_);
  if (s != S::Ok) {
    delayMicroseconds(100);
    s = bus_.readRegister(address_, REG_WHO_AM_I, &lastWhoAmI_);
    if (s != S::Ok) return false;
  }

  // Disable internal AP pull resistors.
  if (writeIreg(IPREG_BAR, IPREG_BAR_REG_58, 0xD9 & ~0x48) != S::Ok) return false;
  if (writeIreg(IPREG_BAR, IPREG_BAR_REG_59, 0xB6 & ~0x92) != S::Ok) return false;

  // Big-endian sensor data so beInt16() can parse the block directly.
  if (writeIreg(IPREG_TOP1, SREG_CTRL, 0x02) != S::Ok) return false;

  // Accel + gyro in low-noise mode.
  if (bus_.writeRegister(address_, REG_PWR_MGMT0,
                         (GYRO_MODE_LN << 2) | ACCEL_MODE_LN) != S::Ok) return false;
  delayMicroseconds(300);

  if (bus_.writeRegister(address_, REG_ACCEL_CONFIG0,
                         (accelFsSel(config_.accelFsrG) << 4) | ODR_100HZ) != S::Ok) return false;
  if (bus_.writeRegister(address_, REG_GYRO_CONFIG0,
                         (gyroFsSel(config_.gyroFsrDps) << 4) | ODR_100HZ) != S::Ok) return false;

  if (bus_.writeRegister(address_, REG_FIFO_CONFIG0, 0x87) != S::Ok) return false;  // stop-on-full, 2KB
  if (bus_.writeRegister(address_, REG_FIFO_CONFIG3, 0x0F) != S::Ok) return false;  // stream, hires, A+G

  delay(100);
  return true;
}

I2CBus::Status ICM45686::whoAmI(uint8_t& id) {
  return bus_.readRegister(address_, REG_WHO_AM_I, &id);
}

I2CBus::Status ICM45686::readRaw(ImuRaw& out) {
  uint8_t b[14];
  I2CBus::Status s = bus_.readRegister(address_, REG_ACCEL_DATA_X1, b, sizeof(b));
  if (s != I2CBus::Status::Ok) return s;

  out.accel[0] = beInt16(b[0],  b[1]);
  out.accel[1] = beInt16(b[2],  b[3]);
  out.accel[2] = beInt16(b[4],  b[5]);
  out.gyro[0]  = beInt16(b[6],  b[7]);
  out.gyro[1]  = beInt16(b[8],  b[9]);
  out.gyro[2]  = beInt16(b[10], b[11]);
  out.temp     = beInt16(b[12], b[13]);
  return I2CBus::Status::Ok;
}

I2CBus::Status ICM45686::read(ImuSample& out) {
  ImuRaw raw;
  I2CBus::Status s = readRaw(raw);
  if (s != I2CBus::Status::Ok) return s;

  for (int i = 0; i < 3; ++i) {
    out.accel_g[i]  = raw.accel[i] * accelScale_;
    out.gyro_dps[i] = raw.gyro[i]  * gyroScale_;
  }
  out.temp_c = 25.0f + (float)raw.temp / 128.0f;
  return I2CBus::Status::Ok;
}

// An IREG write is a single burst: [0x7C, addrHigh, addrLow, data].
I2CBus::Status ICM45686::writeIreg(uint8_t bankHigh, uint8_t regLow, uint8_t value) {
  const uint8_t buf[4] = { REG_IREG_ADDR_HI, bankHigh, regLow, value };
  I2CBus::Status s = bus_.write(address_, buf, sizeof(buf));
  delayMicroseconds(10);  // chip needs settle time after an indirect write
  return s;
}
