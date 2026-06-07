#pragma once

#include <stdint.h>

#include "I2CBus.h"

// Raw 16-bit magnetometer sample, straight from the registers (no scaling).
struct MagRaw {
  int16_t mag[3];  // X, Y, Z
};

// Scaled magnetometer sample plus the status flags from the same read.
struct MagSample {
  float mag_g[3];     // Gauss
  bool  dataReady;    // a new measurement was available
  bool  overflow;     // a channel saturated
};

// Driver for the QST QMC6309 3-axis magnetometer over I2C.
//
// Shares an I2CBus with the IMU, so it owns no bus state of its own. Data
// registers are little-endian (unlike the ICM45686), which is handled here.
class QMC6309 {
public:
  static constexpr uint8_t kChipIdValue = 0x90;
  static constexpr uint8_t kDefaultAddress = 0x7C;

  QMC6309(I2CBus& bus, uint8_t address = kDefaultAddress)
      : bus_(bus), address_(address) {}

  // Probe and configure the sensor (continuous mode, ±8 G, 100 Hz).
  // Returns true once the chip responds and all writes succeed.
  bool begin();

  // Live read of the chip-ID register (expected kChipIdValue).
  I2CBus::Status chipId(uint8_t& id);

  // The chip ID cached during begin().
  uint8_t lastChipId() const { return lastChipId_; }

  // Read status (data-ready / overflow). On data-ready, also reads and scales
  // the three axes into out.mag_g; otherwise mag_g is left untouched.
  I2CBus::Status read(MagSample& out);

  // Read the three axes unconditionally, without checking data-ready.
  I2CBus::Status readRaw(MagRaw& out);

  float scale() const { return kGaussPerLsb; }

private:
  // Sensitivity for the ±8 G range: 4000 LSB per Gauss.
  static constexpr float kGaussPerLsb = 1.0f / 4000.0f;

  I2CBus& bus_;
  uint8_t address_;
  uint8_t lastChipId_ = 0;
};
