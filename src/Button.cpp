#include "Button.h"

void Button::begin() {
  if (_cfg.usePull) {
    // INPUT_PULLUP for active-LOW, INPUT_PULLDOWN for active-HIGH.
    pinMode(_pin, _cfg.activeLevel == LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  } else {
    pinMode(_pin, INPUT);
  }
  _rawLast      = rawPressed();
  _pressed      = _rawLast;
  _lastChangeAt = millis();
}

void Button::update() {
  const unsigned long now = millis();
  const bool raw = rawPressed();

  // Debounce: only accept a new stable level after it has held for debounceMs.
  if (raw != _rawLast) {
    _rawLast      = raw;
    _lastChangeAt = now;
  } else if (raw != _pressed && (now - _lastChangeAt) >= _cfg.debounceMs) {
    _pressed = raw;
    if (_pressed) {
      // --- Press edge ---
      _pressedAt = now;
      _longFired = false;
    } else {
      // --- Release edge ---
      if (_longFired) {
        // This press was a long-press; emit its release, not a click.
        fire(_onLongPressRelease);
      } else if (_clickPending) {
        // Second release within the gap -> double click.
        _clickPending = false;
        fire(_onDoubleClick);
      } else {
        // First release: arm a pending click unless nobody wants doubles.
        if (_onDoubleClick) {
          _clickPending  = true;
          _lastReleaseAt = now;
        } else {
          fire(_onClick);
        }
      }
    }
  }

  // Long-press: fires once while still held, the moment the threshold passes.
  if (_pressed && !_longFired && (now - _pressedAt) >= _cfg.longPressMs) {
    _longFired    = true;
    _clickPending = false;  // a hold is never also a click
    fire(_onLongPress);
  }

  // A pending single click commits once the double-click window expires.
  if (_clickPending && (now - _lastReleaseAt) >= _cfg.doubleClickGapMs) {
    _clickPending = false;
    fire(_onClick);
  }
}
