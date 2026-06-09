#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

#include "config.h"
#include "NrfGpio.h"
#include "I2CBus.h"
#include "ICM45686.h"
#include "QMC6309.h"
#include "Tracker.h"
#include "NrfRadio.h"
#include "RadioPacket.h"
#include "Button.h"
#include "Power.h"
#include "Led.h"
#include "CalibrationStore.h"

static I2CBus    i2c;
static ICM45686  imu(i2c,
                     TRACKER_ICM_ADDR_LSB ? 0x69 : 0x68,
                     ICM45686::Config{ TRACKER_ACCEL_FSR_G, TRACKER_GYRO_FSR_DPS });
static QMC6309   mag(i2c, TRACKER_QMC_ADDR);
static Tracker   tracker(imu, mag);
static NrfRadio  radio;

static Button::Config makeButtonConfig() {
  Button::Config c;
  c.debounceMs       = BUTTON_DEBOUNCE_MS;
  c.doubleClickGapMs = BUTTON_DOUBLE_CLICK_GAP_MS;
  c.longPressMs      = BUTTON_LONG_PRESS_MS;
  return c;
}
static Button    button(BUTTON_PIN, makeButtonConfig());

// Hardware unique id, used as the over-the-air device id (the "MAC").
static uint32_t  g_devId = 0;

// Compile-time flag set describing which fields go on the air.
static const uint8_t kTxFlags =
      (SEND_QUAT  ? RadioLink::kQuat  : 0)
    | (SEND_RPY   ? RadioLink::kRpy   : 0)
    | (SEND_ACCEL ? RadioLink::kAccel : 0)
    | (SEND_GYRO  ? RadioLink::kGyro  : 0)
    | (SEND_TEMP  ? RadioLink::kTemp  : 0)
    | (SEND_MAG   ? RadioLink::kMag   : 0);

static uint16_t  g_seq = 0;

static void waitForSerial() {
  Serial.begin(TRACKER_SERIAL_BAUD);
  const unsigned long start = millis();
  while (!Serial && millis() - start < TRACKER_USB_WAIT_MS) {
    delay(10);
  }
}

// Blocking gyro-bias calibration.
static void runGyroCalibration() {
  Serial.println("\nGYRO CAL: hold still...");
  if (tracker.calibrateGyro()) {
    const Calibration& c = tracker.calibration();
    Serial.print("GYRO CAL ok, bias[dps] = ");
    Serial.print(c.gyroBiasDps[0], 4); Serial.print(", ");
    Serial.print(c.gyroBiasDps[1], 4); Serial.print(", ");
    Serial.println(c.gyroBiasDps[2], 4);
  } else {
    Serial.println("GYRO CAL FAILED (device moved?), bias unchanged");
  }
}

// Blocking hard/soft-iron magnetometer calibration.
static void runMagCalibration() {
  Serial.println("\nMAG CAL: rotate through all orientations...");
  tracker.beginMagCalibration();
  const unsigned long start = millis();
  while (millis() - start < TRACKER_MAG_CAL_MS) {
    tracker.collectMagCalibration();
    delay(10);
  }
  tracker.finishMagCalibration();

  const Calibration& c = tracker.calibration();
  Serial.print("MAG CAL ok, offset[G] = ");
  Serial.print(c.magOffset[0], 5); Serial.print(", ");
  Serial.print(c.magOffset[1], 5); Serial.print(", ");
  Serial.print(c.magOffset[2], 5);
  Serial.print(" | scale = ");
  Serial.print(c.magScale[0], 4); Serial.print(", ");
  Serial.print(c.magScale[1], 4); Serial.print(", ");
  Serial.println(c.magScale[2], 4);
}

// Persist the active calibration to internal flash so it survives reboots.
static void saveCalibration() {
  if (CalibrationStore::save(tracker.calibration())) {
    Serial.println("CAL saved to flash");
  } else {
    Serial.println("CAL save FAILED");
  }
}

// Pull a stored calibration (if any) from flash into the tracker.
static void loadCalibration() {
  Calibration c;  // identity defaults
  if (CalibrationStore::load(c)) {
    tracker.setCalibration(c);
    Serial.print("CAL loaded from flash, gyro bias[dps] = ");
    Serial.print(c.gyroBiasDps[0], 4); Serial.print(", ");
    Serial.print(c.gyroBiasDps[1], 4); Serial.print(", ");
    Serial.println(c.gyroBiasDps[2], 4);
  } else {
    Serial.println("CAL none stored, using defaults");
  }
}

// Drop the stored calibration; the next boot starts from defaults.
static void clearCalibration() {
  Serial.println(CalibrationStore::clear() ? "CAL cleared from flash"
                                           : "CAL nothing to clear");
}

static void runCalibrationSequence() {
  delay(CAL_START_DELAY_MS);
  runGyroCalibration();
  delay(CAL_PHASE_GAP_MS);
  runMagCalibration();
  saveCalibration();
  Serial.println("CAL done, resuming stream");
}

static void handleSerialCommands() {
  while (Serial.available() > 0) {
    switch (Serial.read()) {
      case 'g': case 'G': runGyroCalibration();     break;
      case 'm': case 'M': runMagCalibration();      break;
      case 'c': case 'C': runCalibrationSequence(); break;
      case 's': case 'S': saveCalibration();        break;
      case 'l': case 'L': loadCalibration();        break;
      case 'x': case 'X': clearCalibration();       break;
      default: break;
    }
  }
}

// Append `count` floats from `src` to the packet at `*pos`, advancing it.
static inline void putFloats(uint8_t* pkt, uint8_t& pos, const float* src, uint8_t count) {
  memcpy(&pkt[pos], src, count * sizeof(float));
  pos += count * sizeof(float);
}

// Build and transmit one telemetry packet from the latest tracker state.
static void sendTelemetry() {
  uint8_t pkt[RadioLink::kMaxLen];
  uint8_t pos = 0;

  pkt[pos++] = RadioLink::kMagic;
  pkt[pos++] = kTxFlags;
  memcpy(&pkt[pos], &g_devId, sizeof(g_devId)); pos += sizeof(g_devId);
  memcpy(&pkt[pos], &g_seq,   sizeof(g_seq));   pos += sizeof(g_seq);
  g_seq++;

  const Orientation& o = tracker.orientation();
#if SEND_ACCEL || SEND_GYRO || SEND_TEMP
  const ImuSample&   s = tracker.imu();
#endif
#if SEND_MAG
  const MagSample&   m = tracker.mag();
#endif

#if SEND_QUAT
  putFloats(pkt, pos, o.quaternion, 4);
#endif
#if SEND_RPY
  const float rpy[3] = { o.roll_deg, o.pitch_deg, o.yaw_deg };
  putFloats(pkt, pos, rpy, 3);
#endif
#if SEND_ACCEL
  putFloats(pkt, pos, s.accel_g, 3);
#endif
#if SEND_GYRO
  putFloats(pkt, pos, s.gyro_dps, 3);
#endif
#if SEND_TEMP
  putFloats(pkt, pos, &s.temp_c, 1);
#endif
#if SEND_MAG
  putFloats(pkt, pos, m.mag_g, 3);
#endif

  radio.send(pkt, pos);

#if TRACKER_SERIAL_DEBUG
  Serial.print("TX seq="); Serial.print(g_seq);
  Serial.print(" RPY ");
  Serial.print(o.roll_deg, 2);  Serial.print(", ");
  Serial.print(o.pitch_deg, 2); Serial.print(", ");
  Serial.println(o.yaw_deg, 2);
#endif
}

void setup() {
  Led::init();
  Led::set(true);

  waitForSerial();
  Serial.println();
  Serial.println("BOOT");

  // Button actions.
  button.begin();
  // button.onClick(...);
  button.onDoubleClick(runCalibrationSequence);
  button.onLongPress(Power::goToSleep);

  // Wait for the button to be released.
  if (digitalRead(BUTTON_PIN) == LOW) {
    while (digitalRead(BUTTON_PIN) == LOW) delay(10);
    delay(BUTTON_DEBOUNCE_MS);
    button.begin();
  }

  Power::enableStackedPower();

  i2c.begin(TRACKER_I2C_SDA_PIN, TRACKER_I2C_SCL_PIN, TRACKER_I2C_HZ);
  Serial.println("TWIM0 ready");

  if (!tracker.begin()) {
    Serial.println("Tracker init FAILED (IMU or mag), halting.");
    while (true) delay(1000);
  }

  Serial.print("ICM45686 WHO_AM_I = 0x");
  Serial.println(imu.lastWhoAmI(), HEX);
  if (imu.lastWhoAmI() != ICM45686::kWhoAmIValue) {
    Serial.println("WARNING: unexpected WHO_AM_I (expected 0xE9)");
  }
  Serial.print("QMC6309 CHIP_ID = 0x");
  Serial.println(mag.lastChipId(), HEX);

  // Restore a previously saved calibration (if any) before streaming starts.
  CalibrationStore::begin();
  loadCalibration();

  g_devId = NRF_FICR->DEVICEID[0];
  radio.beginTx();
  Serial.print("RADIO TX ready, devId = 0x");
  Serial.print(g_devId, HEX);
  Serial.print(", flags = 0x");
  Serial.println(kTxFlags, HEX);

  Serial.println("Button D0: click = gyro+mag cal (auto-save), "
                 "double click = save cal, long press = deep sleep");
  Serial.println("Serial: g/m/c cal, s=save l=load x=clear");
  Serial.println("START STREAMING");
}

void loop() {
  handleSerialCommands();
  button.update();

  const I2CBus::Status st = tracker.update();
  if (st != I2CBus::Status::Ok) {
    Serial.print("read error: ");
    Serial.println(I2CBus::statusName(st));
    delay(500);
    return;
  }

  sendTelemetry();

  delay(TELEMETRY_SEND_INTERVAL_MS);
}
