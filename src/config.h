#pragma once

#include "nrf.h"

// Project configuration for the nRF52840 9-DoF tracker.
// Board: SuperMini nRF52840. Sensor module = ICM45686 (+ QMC6309 later),
// both on a single I2C bus.

// --- I2C bus ----------------------------------------------------------------
// Arduino "Dx" pin numbers.
#define TRACKER_I2C_SDA_PIN 24   // P0.24
#define TRACKER_I2C_SCL_PIN 22   // P0.22
#define TRACKER_I2C_HZ      400000UL

// --- ICM45686 ---------------------------------------------------------------
// Address select: false = 0x68, true = 0x69.
#define TRACKER_ICM_ADDR_LSB true
#define TRACKER_ACCEL_FSR_G  16    // ±g    (supported: 2 / 4 / 8 / 16 / 32)
#define TRACKER_GYRO_FSR_DPS 1000  // ±dps  (supported: 125 / 250 / 500 / 1000 / 2000 / 4000)

// --- QMC6309 magnetometer ---------------------------------------------------
#define TRACKER_QMC_ADDR 0x7C

// --- Sensor axis alignment --------------------------------------------------
// Align each chip into the tracker frame. Use these when the IMU and
// magnetometer are mounted with different axis directions.
//
// AXIS_MAP says where output X/Y/Z comes from: 0 = input X, 1 = input Y,
// 2 = input Z. AXIS_SIGN says whether each output axis is inverted:
// 1 = keep direction, -1 = invert direction.
//
// Example: output {X, Y, Z} = {-input Y, input Z, input X}
//   #define TRACKER_ACCEL_AXIS_MAP  {1, 2, 0}
//   #define TRACKER_ACCEL_AXIS_SIGN {-1, 1, 1}
//
#define TRACKER_ACCEL_AXIS_MAP  {0, 1, 2}
#define TRACKER_ACCEL_AXIS_SIGN {1, 1, 1}

#define TRACKER_GYRO_AXIS_MAP  {0, 1, 2}
#define TRACKER_GYRO_AXIS_SIGN {-1, -1, -1}

#define TRACKER_MAG_AXIS_MAP  {0, 1, 2}
#define TRACKER_MAG_AXIS_SIGN {1, 1, 1}

// --- Sensor power -----------------------------------------------------------
// Stacked module powered from GPIOs instead of a dedicated rail.
#define SENSOR_VCC 17  // P0.17 driven high
#define SENSOR_GND 20  // P0.20 driven low
// Set to 1 if the sensor module is powered from GPIOs
// rather than a dedicated 3V3/GND rail.
#define ENABLE_STACKED_POWER 1

// --- Calibration ------------------------------------------------------------
// Gyro bias: samples averaged while still, and the max peak-to-peak spread
// (deg/s) still considered "stationary".
#define TRACKER_GYRO_CAL_SAMPLES        500
#define TRACKER_GYRO_CAL_MAX_SPREAD_DPS 2.0f
// Magnetometer hard/soft-iron: how long to collect samples while rotating.
#define TRACKER_MAG_CAL_MS 15000UL

// --- Orientation estimation (x-io Fusion) -----------------------------------
// FusionConventionNwu is the library default: +X North, +Y West, +Z Up.
// No output coordinate conversion is applied by Tracker.
#define TRACKER_FUSION_CONVENTION FusionConventionNwu
#define TRACKER_FUSION_GAIN 0.5f
#define TRACKER_FUSION_ACCEL_REJECTION_DEG 10.0f
#define TRACKER_FUSION_MAG_REJECTION_DEG   10.0f
#define TRACKER_FUSION_RECOVERY_MS         5000UL

// --- Persistent storage (internal flash via LittleFS) -----------------------
// Calibration is saved here so it survives power cycles. Key is the record name
// on the internal filesystem; bump the version whenever the Calibration struct
// layout changes so stale saves are ignored instead of misread.
#define CALIBRATION_STORAGE_KEY     "cal"
#define CALIBRATION_STORAGE_VERSION 1

// --- Serial -----------------------------------------------------------------
#define TRACKER_SERIAL_BAUD 921600
#define TRACKER_USB_WAIT_MS 3000

// --- Control button ---------------------------------------------------------
// Handled by the Button API (see Button.h). One click runs gyro calibration,
// then (after a short pause) magnetometer calibration. Double click / long
// press are wired in main.cpp and free to extend. Active LOW (INPUT_PULLUP).
#define BUTTON_PIN 6  // P0.06
// Debounce / minimum press time before a click registers (ms).
#define BUTTON_DEBOUNCE_MS 40
// Max gap between two clicks to count as a double click (ms).
#define BUTTON_DOUBLE_CLICK_GAP_MS 300
// Hold time that registers a long press (ms).
#define BUTTON_LONG_PRESS_MS 800
// Pause between the gyro and magnetometer calibration phases (ms).
#define CAL_PHASE_GAP_MS 1500
// Delay before the calibration (ms).
#define CAL_START_DELAY_MS 500

// --- Status LED (temporary indicator) ---------------------------------------
// Red on-board LED used as a coarse run/sleep indicator: solid ON while awake,
// OFF in deep sleep. Driven with plain digitalWrite from main.cpp so the whole
// thing is trivial to rip out later. Adjust the pin / polarity for your board.
#define STATUS_LED_PIN         LED_BUILTIN  // P0.15 on the nice!nano / SuperMini
#define STATUS_LED_ACTIVE_HIGH 1            // most boards: LED lights when pin LOW

// --- Radio telemetry selection ----------------------------------------------
// Which fields this transmitter puts on the air. Each is a float group; the
// receiver decodes whatever is flagged in each packet, so transmitters need not
// agree. Keep packets lean — only enable what the host actually consumes.
#define SEND_QUAT  1  // quaternion w,x,y,z      (4 floats)
#define SEND_RPY   0  // roll, pitch, yaw [deg]  (3 floats)
#define SEND_ACCEL 0  // accel x,y,z [g]         (3 floats)
#define SEND_GYRO  0  // gyro x,y,z [deg/s]      (3 floats)
#define SEND_TEMP  0  // temperature [C]         (1 float)
#define SEND_MAG   0  // mag x,y,z [Gauss]       (3 floats)

// TDMA slot assigned to this tracker. For 5 trackers, flash each unit with a
// unique value 0..4 so their radio packets never share the same time slot.
#define TRACKER_TDMA_SLOT 0

// Nominal telemetry frame period, kept for Fusion recovery timing. Radio sends
// are now paced by receiver beacons rather than delay().
#define TELEMETRY_SEND_INTERVAL_MS 33

// Optional: also mirror the estimated RPY to USB serial as text for debugging.
// Leave 0 in normal operation — the radio is the real output path.
#define TRACKER_SERIAL_DEBUG 0

