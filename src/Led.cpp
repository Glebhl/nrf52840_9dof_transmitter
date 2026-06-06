#include "Led.h"

#include "config.h"

namespace Led {

void init() {
  pinMode(STATUS_LED_PIN, OUTPUT);
}

void set(bool on) {
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void blink(uint8_t times, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < times; ++i) {
    set(true);  delay(onMs);
    set(false); delay(offMs);
  }
}

}  // namespace Led
