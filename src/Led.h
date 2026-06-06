#pragma once

#include <Arduino.h>

// Status LED helper.
//
// Thin wrapper around the on-board status LED (STATUS_LED_PIN in config.h),
// currently a coarse run/sleep indicator: solid ON while awake, OFF in deep
// sleep. Kept in its own translation unit so blink patterns, non-blocking
// blinking, fades and other effects can grow here without cluttering main.cpp.
//
// The current implementation is plain and blocking; nothing here is fancy yet.
namespace Led {

void init();

void set(bool on);

// Blocking blink: `times` on/off cycles with the given on/off durations (ms).
void blink(uint8_t times, uint16_t onMs, uint16_t offMs);

}  // namespace Led
