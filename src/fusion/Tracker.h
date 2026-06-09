#pragma once

#include <stdint.h>

#include "I2CBus.h"
#include "ICM45686.h"
#include "QMC6309.h"
#include "Calibration.h"
#include "Fusion.h"
#include "config.h"

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct EulerAngles {
  float roll_deg;   // rotation about X
  float pitch_deg;  // rotation about Y
  float yaw_deg;    // rotation about Z
};

// High-level API over the 9-DoF sensor module.
//
// Owns the IMU and magnetometer drivers plus a calibration set. The intended
// use is a single call to update() per loop iteration, after which the latest
// calibrated imu()/mag() samples and estimated quaternion()/euler() pose are
// available:
//
//   Tracker tracker(imu, mag);
//   tracker.begin();
//   ...
//   tracker.update();
//   const Quaternion& q = tracker.quaternion();      // estimated quaternion
//   const EulerAngles& e = tracker.euler();          // estimated angles
//   const ImuSample&   a = tracker.imu();            // calibrated accel/gyro
//
// Calibration routines (gyro bias, hard/soft-iron mag) refine the result and
// can be triggered at runtime.
class Tracker {
public:
  Tracker(ICM45686& imu, QMC6309& mag) : imuDev_(imu), magDev_(mag) {}

  // Initialise both sensors. Returns false if either device fails to come up.
  bool begin();

  // Read both sensors and apply calibration. Returns the IMU read status; the
  // magnetometer is optional and magValid() reports whether it refreshed on
  // this update.
  I2CBus::Status update();

  // --- Estimated outputs ----------------------------------------------------
  const Quaternion& quaternion() const { return quaternion_; }
  const EulerAngles& euler() const { return euler_; }
  FusionAhrsInternalStates fusionInternalStates() const {
    return FusionAhrsGetInternalStates(&fusion_);
  }
  FusionAhrsFlags fusionFlags() const { return FusionAhrsGetFlags(&fusion_); }

  // --- Latest samples (calibrated + scaled + axis-aligned) ------------------
  const ImuSample& imu() const { return imu_; }
  const MagSample& mag() const { return mag_; }
  bool magValid() const { return magValid_; }  // mag() refreshed this update

  // --- Calibration ----------------------------------------------------------
  const Calibration& calibration() const { return cal_; }
  void setCalibration(const Calibration& c) { cal_ = c; }

  // Estimate gyro bias by averaging while the device is held perfectly still.
  // Blocking (~samples * 2 ms). Returns false if the readings moved too much to
  // trust (TRACKER_GYRO_CAL_MAX_SPREAD_DPS), leaving the old bias in place.
  bool calibrateGyro(uint16_t samples = TRACKER_GYRO_CAL_SAMPLES);

  // Hard/soft-iron magnetometer calibration. Call begin, then collect()
  // repeatedly while slowly rotating the device through every orientation
  // (figure-eights), then finish() to compute and store offsets/scales.
  void beginMagCalibration();
  void collectMagCalibration();
  void finishMagCalibration();
  bool magCalibrating() const { return magCalActive_; }

private:
  void applyCalibration(ImuSample& imuSample, MagSample& magSample, bool haveMag) const;
  void alignAxes(ImuSample& imuSample, MagSample& magSample, bool haveMag) const;

  ICM45686&  imuDev_;
  QMC6309&   magDev_;
  FusionAhrs fusion_;
  Calibration cal_;

  Quaternion  quaternion_{1.0f, 0.0f, 0.0f, 0.0f};
  EulerAngles euler_{0.0f, 0.0f, 0.0f};
  ImuSample   imu_{};
  MagSample   mag_{};
  bool        magValid_ = false;

  uint32_t lastUpdateUs_  = 0;
  bool     haveTimestamp_ = false;

  // Min/max accumulators for the in-progress magnetometer calibration.
  bool  magCalActive_ = false;
  float magMin_[3] = {0.0f, 0.0f, 0.0f};
  float magMax_[3] = {0.0f, 0.0f, 0.0f};
};
