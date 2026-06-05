#pragma once

#include <Arduino.h>
#include "nrf.h"

// Low-level nRF52840 GPIO helpers shared across the project.
//
// All functions take an *encoded* nRF pin number (P0.x -> x, P1.x -> 32 + x),
// which is what NRF_GPIO_PIN_MAP(port, pin) produces. To go from an Arduino
// "Dx" pin to an encoded pin use NrfGpio::fromArduinoPin().

// Arduino core mapping table: index = Arduino pin, value = encoded nRF pin.
// Declared at global scope to match the core's definition (C++ linkage).
extern const uint32_t g_ADigitalPinMap[];

namespace NrfGpio {

// Convert an Arduino "Dx" pin number to an encoded nRF pin number.
inline uint32_t fromArduinoPin(uint8_t arduinoPin) {
  return ::g_ADigitalPinMap[arduinoPin];
}

// PIN_CNF register for an encoded pin, selecting P0 or P1 automatically.
inline volatile uint32_t* pinConfig(uint32_t pin) {
  if (pin < 32) {
    return &NRF_P0->PIN_CNF[pin];
  }
  return &NRF_P1->PIN_CNF[pin - 32];
}

// Configure a pin as an open-drain I2C line with no internal pull-up
// (external pull-ups expected, SlimeVR style).
inline void configureI2cPin(uint32_t pin) {
  *pinConfig(pin) =
      ((uint32_t)GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)
    | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
    | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
    | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
    | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

// Drive a pin as a push-pull output at the given level. Useful for the
// "stacked" power scheme where VCC/GND of the sensor module come from GPIOs.
inline void configureOutput(uint32_t pin, bool high) {
  *pinConfig(pin) =
      ((uint32_t)GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos)
    | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
    | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
    | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
    | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);

  if (pin < 32) {
    if (high) NRF_P0->OUTSET = (1UL << pin);
    else      NRF_P0->OUTCLR = (1UL << pin);
  } else {
    uint32_t p = pin - 32;
    if (high) NRF_P1->OUTSET = (1UL << p);
    else      NRF_P1->OUTCLR = (1UL << p);
  }
}

}  // namespace NrfGpio
