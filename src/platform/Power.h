#pragma once

#include <Arduino.h>

// nRF52840 deep-sleep (System OFF) helper.
//
// System OFF is the chip's lowest-power state: the CPU, RAM, and every
// peripheral are powered down, so a bare module sits at roughly 1 µA. The only
// way out is a wakeup event, and on this chip a wakeup is delivered as a *reset*
// — execution resumes from the reset vector and setup() runs again from scratch,
// exactly like a fresh power-on. This is true sleep, not merely idling the CPU
// or switching sensors off.
//
// The wakeup source is a single GPIO armed through its SENSE detector: when the
// pin reaches `wakeLevel` the DETECT signal fires and the chip resets out of OFF.
//
// IMPORTANT: the wake pin must be resting at the *opposite* level when System OFF
// is entered. If it already sits at `wakeLevel` (e.g. the button is still held),
// DETECT is asserted immediately and the chip wakes at once. Callers should wait
// for the button to be released before calling enterSystemOff().
namespace Power {

[[noreturn]] void enterSystemOff(uint8_t wakeArduinoPin);

void enableStackedPower();

void disableStackedPower();

void goToSleep();

}  // namespace Power
