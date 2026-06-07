#include "I2CBus.h"

#include "nrf.h"
#include "NrfGpio.h"

void I2CBus::begin(uint8_t sdaArduinoPin, uint8_t sclArduinoPin, uint32_t frequencyHz) {
  const uint32_t sda = NrfGpio::fromArduinoPin(sdaArduinoPin);
  const uint32_t scl = NrfGpio::fromArduinoPin(sclArduinoPin);

  NRF_TWIM0->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);

  NRF_TWIM0->PSEL.SDA = sda;
  NRF_TWIM0->PSEL.SCL = scl;
  NrfGpio::configureI2cPin(sda);
  NrfGpio::configureI2cPin(scl);

  // Map the requested speed onto the nearest supported TWIM bitrate.
  uint32_t bitrate;
  if (frequencyHz <= 100000UL) {
    bitrate = TWIM_FREQUENCY_FREQUENCY_K100;
  } else if (frequencyHz <= 250000UL) {
    bitrate = TWIM_FREQUENCY_FREQUENCY_K250;
  } else {
    bitrate = TWIM_FREQUENCY_FREQUENCY_K400;
  }
  NRF_TWIM0->FREQUENCY = bitrate;

  NRF_TWIM0->SHORTS = 0;
  NRF_TWIM0->INTENCLR = 0xFFFFFFFF;
  clearEvents();

  NRF_TWIM0->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

I2CBus::Status I2CBus::write(uint8_t address, const uint8_t* data, size_t len) {
  clearEvents();

  NRF_TWIM0->ADDRESS = address;
  NRF_TWIM0->TXD.PTR = (uint32_t)data;
  NRF_TWIM0->TXD.MAXCNT = len;
  NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STOP_Msk;

  NRF_TWIM0->TASKS_STARTTX = 1;

  Status s = waitDone(kTransactionTimeoutUs);
  NRF_TWIM0->SHORTS = 0;
  return s;
}

I2CBus::Status I2CBus::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  const uint8_t buf[2] = { reg, value };
  return write(address, buf, sizeof(buf));
}

I2CBus::Status I2CBus::readRegister(uint8_t address, uint8_t reg, uint8_t* out, size_t len) {
  clearEvents();

  NRF_TWIM0->ADDRESS = address;

  NRF_TWIM0->TXD.PTR = (uint32_t)&reg;
  NRF_TWIM0->TXD.MAXCNT = 1;

  NRF_TWIM0->RXD.PTR = (uint32_t)out;
  NRF_TWIM0->RXD.MAXCNT = len;

  // Write the register pointer, repeated-start into the read, STOP after.
  NRF_TWIM0->SHORTS =
      TWIM_SHORTS_LASTTX_STARTRX_Msk
    | TWIM_SHORTS_LASTRX_STOP_Msk;

  NRF_TWIM0->TASKS_STARTTX = 1;

  Status s = waitDone(kTransactionTimeoutUs);
  NRF_TWIM0->SHORTS = 0;

  if (s != Status::Ok) return s;
  if (NRF_TWIM0->RXD.AMOUNT != len) return Status::ShortRead;
  return Status::Ok;
}

const char* I2CBus::statusName(Status s) {
  switch (s) {
    case Status::Ok:          return "OK";
    case Status::Timeout:     return "TIMEOUT";
    case Status::AddressNack: return "ADDRESS_NACK";
    case Status::DataNack:    return "DATA_NACK";
    case Status::Overrun:     return "OVERRUN";
    case Status::BusError:    return "BUS_ERROR";
    case Status::ShortRead:   return "SHORT_READ";
    default:                  return "UNKNOWN";
  }
}

void I2CBus::clearEvents() {
  NRF_TWIM0->EVENTS_STOPPED   = 0;
  NRF_TWIM0->EVENTS_ERROR     = 0;
  NRF_TWIM0->EVENTS_TXSTARTED = 0;
  NRF_TWIM0->EVENTS_RXSTARTED = 0;
  NRF_TWIM0->EVENTS_LASTTX    = 0;
  NRF_TWIM0->EVENTS_LASTRX    = 0;
  NRF_TWIM0->ERRORSRC         = 0xFFFFFFFF;
}

// Spin until the transaction stops or errors, forcing a STOP on timeout so the
// bus is always left in a clean state.
I2CBus::Status I2CBus::waitDone(uint32_t timeoutUs) {
  const uint32_t start = micros();

  while (!NRF_TWIM0->EVENTS_STOPPED && !NRF_TWIM0->EVENTS_ERROR) {
    if ((uint32_t)(micros() - start) > timeoutUs) {
      NRF_TWIM0->TASKS_STOP = 1;
      const uint32_t stopStart = micros();
      while (!NRF_TWIM0->EVENTS_STOPPED) {
        if ((uint32_t)(micros() - stopStart) > 1000) break;
      }
      return Status::Timeout;
    }
  }

  if (NRF_TWIM0->EVENTS_ERROR) {
    const uint32_t err = NRF_TWIM0->ERRORSRC;
    NRF_TWIM0->ERRORSRC = err;  // write-1-to-clear
    NRF_TWIM0->TASKS_STOP = 1;

    const uint32_t stopStart = micros();
    while (!NRF_TWIM0->EVENTS_STOPPED) {
      if ((uint32_t)(micros() - stopStart) > 1000) break;
    }

    if (err & TWIM_ERRORSRC_ANACK_Msk)   return Status::AddressNack;
    if (err & TWIM_ERRORSRC_DNACK_Msk)   return Status::DataNack;
    if (err & TWIM_ERRORSRC_OVERRUN_Msk) return Status::Overrun;
    return Status::BusError;
  }

  return Status::Ok;
}
