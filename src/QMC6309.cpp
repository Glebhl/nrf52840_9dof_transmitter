#include "QMC6309.h"

#include <Arduino.h>

namespace {

// --- QMC6309 registers ------------------------------------------------------
constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_OUT_X_L = 0x01;  // start of the 6-byte data block
constexpr uint8_t REG_STATUS  = 0x09;
constexpr uint8_t REG_CTRL1   = 0x0A;
constexpr uint8_t REG_CTRL2   = 0x0B;

// --- STATUS bits ------------------------------------------------------------
constexpr uint8_t STATUS_DRDY = 0x01;  // data ready
constexpr uint8_t STATUS_OVFL = 0x02;  // measurement overflow

// --- Configuration ----------------------------------------------------------
// CTRL2: ODR_100Hz (3 << 4) | RNG_8G (2 << 2) | SET_RESET_ON (0).
constexpr uint8_t CTRL2_VALUE = 0x38;
// CTRL1: LPF_2 (1 << 5) | OSR_8 (0 << 3) | MODE_CONTINUOUS (3).
constexpr uint8_t CTRL1_VALUE = 0x23;

inline int16_t leInt16(uint8_t lo, uint8_t hi) {
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

}  // namespace

bool QMC6309::begin() {
  using S = I2CBus::Status;

  // The first transaction after power-up can be a dummy; retry the ID probe.
  uint8_t dummy = 0;
  bus_.readRegister(address_, REG_CHIP_ID, &dummy);
  delayMicroseconds(100);

  S s = bus_.readRegister(address_, REG_CHIP_ID, &lastChipId_);
  if (s != S::Ok) {
    delayMicroseconds(100);
    s = bus_.readRegister(address_, REG_CHIP_ID, &lastChipId_);
    if (s != S::Ok) return false;
  }
  if (lastChipId_ != kChipIdValue) return false;

  if (bus_.writeRegister(address_, REG_CTRL2, CTRL2_VALUE) != S::Ok) return false;
  if (bus_.writeRegister(address_, REG_CTRL1, CTRL1_VALUE) != S::Ok) return false;

  delay(20);
  return true;
}

I2CBus::Status QMC6309::chipId(uint8_t& id) {
  return bus_.readRegister(address_, REG_CHIP_ID, &id);
}

I2CBus::Status QMC6309::read(MagSample& out) {
  uint8_t status = 0;
  I2CBus::Status s = bus_.readRegister(address_, REG_STATUS, &status);
  if (s != I2CBus::Status::Ok) return s;

  out.dataReady = (status & STATUS_DRDY) != 0;
  out.overflow  = (status & STATUS_OVFL) != 0;
  if (!out.dataReady) return I2CBus::Status::Ok;

  MagRaw raw;
  s = readRaw(raw);
  if (s != I2CBus::Status::Ok) return s;

  for (int i = 0; i < 3; ++i) {
    out.mag_g[i] = raw.mag[i] * kGaussPerLsb;
  }
  return I2CBus::Status::Ok;
}

I2CBus::Status QMC6309::readRaw(MagRaw& out) {
  uint8_t b[6];
  I2CBus::Status s = bus_.readRegister(address_, REG_OUT_X_L, b, sizeof(b));
  if (s != I2CBus::Status::Ok) return s;

  out.mag[0] = leInt16(b[0], b[1]);
  out.mag[1] = leInt16(b[2], b[3]);
  out.mag[2] = leInt16(b[4], b[5]);
  return I2CBus::Status::Ok;
}
