#include "Power.h"

#include "config.h"
#include "nrf.h"
#include "NrfGpio.h"
#include "Led.h"

namespace Power {

void enterSystemOff(uint8_t wakeArduinoPin) {
  const uint32_t pin = NrfGpio::fromArduinoPin(wakeArduinoPin);

  const uint32_t pull  = GPIO_PIN_CNF_PULL_Pullup;
  const uint32_t sense = GPIO_PIN_CNF_SENSE_Low;

  *NrfGpio::pinConfig(pin) =
      ((uint32_t)GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)
    | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
    | (pull                                 << GPIO_PIN_CNF_PULL_Pos)
    | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)
    | (sense                                << GPIO_PIN_CNF_SENSE_Pos);

  __DSB();
  NRF_POWER->SYSTEMOFF = 1;
  __DSB();

  // We never expect to get here. With a debugger attached, System OFF is emulated
  // as a halt and execution may continue — park the core so it stays asleep.
  while (true) {
    __WFE();
  }
}

void enableStackedPower() {
#if ENABLE_STACKED_POWER
  Serial.println("Stacked power: VCC=P0.17 high, GND=P0.20 low");
  NrfGpio::configureOutput(SENSOR_VCC, true);
  NrfGpio::configureOutput(SENSOR_GND, false);
  delay(100);
#endif
}

void disableStackedPower() {
#if ENABLE_STACKED_POWER
  NrfGpio::configureOutput(SENSOR_VCC, false);
  NrfGpio::configureOutput(SENSOR_GND, false);
#endif
}

void goToSleep() {
  Serial.println("\nLONG PRESS: entering deep sleep (System OFF). Click to wake.");
  Serial.flush();

  Led::blink(3, 60, 60);
  Led::set(false);

  NRF_RADIO->POWER = 0;
  NRF_TWIM0->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);
  Power::disableStackedPower();

  while (digitalRead(BUTTON_PIN) == LOW) delay(10);
  delay(BUTTON_DEBOUNCE_MS);

  Power::enterSystemOff(BUTTON_PIN);
}

}  // namespace Power
