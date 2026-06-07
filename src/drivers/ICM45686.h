#pragma once

#include <stdint.h>

#include "I2CBus.h"

// Raw 16-bit sensor sample, straight from the registers (no scaling).
struct ImuRaw {
  int16_t accel[3];  // X, Y, Z
  int16_t gyro[3];   // X, Y, Z
  int16_t temp;
};

// Scaled, physical-unit sensor sample.
struct ImuSample {
  float accel_g[3];    // g
  float gyro_dps[3];   // deg/s
  float temp_c;        // degrees Celsius
};

// Driver for the InvenSense ICM-45686 6-axis IMU over I2C.
//
// The chip lives on a shared I2CBus, so this class owns no bus state of its
// own — only the device address and the configured scale factors. A future
// QMC6309 magnetometer driver can sit on the same bus alongside it.
class ICM45686 {
public:
  static constexpr uint8_t kWhoAmIValue = 0xE9;
  static constexpr uint8_t kDefaultAddress = 0x69;

  // Full-scale ranges to configure on the sensor. Defaults match the values
  // that were validated on hardware (±16 g, ±2000 dps).
  struct Config {
    uint16_t accelFsrG;
    uint16_t gyroFsrDps;
    Config(uint16_t accelG = 16, uint16_t gyroDps = 2000)
        : accelFsrG(accelG), gyroFsrDps(gyroDps) {}
  };

  ICM45686(I2CBus& bus, uint8_t address = kDefaultAddress, const Config& config = Config())
      : bus_(bus), address_(address), config_(config) {}

  // Probe and configure the sensor (power modes, ranges, FIFO, big-endian
  // data). Returns true once the chip responds and all writes succeed.
  bool begin();

  // Live read of the WHO_AM_I register (expected kWhoAmIValue).
  I2CBus::Status whoAmI(uint8_t& id);

  // The WHO_AM_I value cached during begin().
  uint8_t lastWhoAmI() const { return lastWhoAmI_; }

  // Read all six axes plus temperature.
  I2CBus::Status readRaw(ImuRaw& out);
  I2CBus::Status read(ImuSample& out);

  float accelScale() const { return accelScale_; }
  float gyroScale() const { return gyroScale_; }

private:
  I2CBus::Status writeIreg(uint8_t bankHigh, uint8_t regLow, uint8_t value);

  I2CBus& bus_;
  uint8_t address_;
  Config  config_;
  uint8_t lastWhoAmI_ = 0;
  float   accelScale_ = 0.0f;  // g per LSB,  set in begin()
  float   gyroScale_  = 0.0f;  // dps per LSB, set in begin()
};
