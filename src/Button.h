#pragma once

#include <Arduino.h>

// Non-blocking button driver with a small event API.
//
// One physical button, several logical actions: assign a callback per event
// (single click, double click, long-press start, long-press release) and call
// update() every loop. Events are detected from a debounced state machine, so
// nothing here blocks the radio stream.
//
// Designed for an active-LOW button (INPUT_PULLUP, pressed = LOW), which is the
// wiring on this board, but the active level is configurable.
//
// Example:
//   Button btn(BUTTON_PIN);
//   btn.onClick([] { runCalibrationSequence(); });
//   btn.onDoubleClick([] { saveCalibration(); });
//   btn.onLongPress([] { resetCalibration(); });
//   ...
//   void loop() { btn.update(); }
class Button {
public:
  // Callback type. Kept as a plain function pointer so this stays usable from
  // bare-metal/Arduino code without pulling in <functional>.
  using Handler = void (*)();

  struct Config {
    // Minimum stable press time before a press is accepted (ms).
    uint16_t debounceMs = 40;
    // Max gap between release and the next press to count as a double click (ms).
    uint16_t doubleClickGapMs = 300;
    // Hold time that turns a press into a long-press (ms).
    uint16_t longPressMs = 800;
    // Logic level when the button is pressed. LOW for INPUT_PULLUP wiring.
    uint8_t  activeLevel = LOW;
    // Configure the pin with the matching internal pull. When activeLevel is
    // LOW this is INPUT_PULLUP; set false if an external resistor is used.
    bool     usePull = true;
  };

  explicit Button(uint8_t pin) : _pin(pin), _cfg() {}
  Button(uint8_t pin, const Config& cfg) : _pin(pin), _cfg(cfg) {}

  // Configure the GPIO. Call once from setup().
  void begin();

  // Drive the state machine and dispatch any due events. Call every loop().
  void update();

  // --- Event registration. Pass nullptr to clear an action. ----------------
  void onClick(Handler h)          { _onClick = h; }
  void onDoubleClick(Handler h)    { _onDoubleClick = h; }
  void onLongPress(Handler h)      { _onLongPress = h; }       // fires once when hold threshold is reached
  void onLongPressRelease(Handler h) { _onLongPressRelease = h; } // fires when a long press is released

  // Current debounced state, if a caller wants to poll instead of using events.
  bool isPressed() const { return _pressed; }

private:
  bool rawPressed() const { return digitalRead(_pin) == _cfg.activeLevel; }
  void fire(Handler h) const { if (h) h(); }

  uint8_t _pin;
  Config  _cfg;

  // Debounce.
  bool          _rawLast      = false;  // last raw reading
  bool          _pressed      = false;  // debounced state
  unsigned long _lastChangeAt = 0;      // when the raw reading last changed

  // Press/click tracking.
  unsigned long _pressedAt    = 0;      // when the current press began
  bool          _longFired    = false;  // long-press already dispatched this hold
  bool          _clickPending = false;  // one click waiting to see if a second follows
  unsigned long _lastReleaseAt = 0;     // when the pending click was released

  Handler _onClick            = nullptr;
  Handler _onDoubleClick      = nullptr;
  Handler _onLongPress        = nullptr;
  Handler _onLongPressRelease = nullptr;
};
