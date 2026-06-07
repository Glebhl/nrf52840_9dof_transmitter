#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Generic I2C master driver for the nRF52840, built directly on TWIM0 with
// EasyDMA. It is bus-level only: every transaction takes the 7-bit device
// address as an argument, so several chips (e.g. ICM45686 + QMC6309) can share
// a single I2CBus instance. Sensor-specific logic lives in the device drivers.
class I2CBus {
public:
  // Result of a transaction. Negative values are errors.
  enum class Status : int8_t {
    Ok         =  0,
    Timeout    = -1,  // peripheral never raised STOPPED/ERROR in time
    AddressNack = -2,  // no device ACKed the address
    DataNack   = -3,  // device NACKed a data byte
    Overrun    = -4,  // RX overrun
    BusError   = -5,  // other TWIM error
    ShortRead  = -6,  // fewer bytes received than requested
  };

  // Initialise TWIM0 on the given Arduino "Dx" pins. Safe to call once in setup.
  void begin(uint8_t sdaArduinoPin, uint8_t sclArduinoPin, uint32_t frequencyHz = 400000UL);

  // Raw write of `len` bytes to `address` (e.g. a register pointer followed by
  // payload). Used for multi-byte command bursts.
  Status write(uint8_t address, const uint8_t* data, size_t len);

  // Write a single 8-bit register.
  Status writeRegister(uint8_t address, uint8_t reg, uint8_t value);

  // Set the register pointer, then read `len` bytes back into `out`.
  Status readRegister(uint8_t address, uint8_t reg, uint8_t* out, size_t len);

  // Read a single 8-bit register.
  Status readRegister(uint8_t address, uint8_t reg, uint8_t* out) {
    return readRegister(address, reg, out, 1);
  }

  // Human-readable name for a status code, handy for logging.
  static const char* statusName(Status s);

private:
  void clearEvents();
  Status waitDone(uint32_t timeoutUs);

  static constexpr uint32_t kTransactionTimeoutUs = 25000;
};
