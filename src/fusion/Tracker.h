#pragma once

#include <stdint.h>

#include "I2CBus.h"
#include "ICM45686.h"
#include "QMC6309.h"
#include "MahonyAHRS.h"
#include "Calibration.h"
#include "config.h"

// Fused orientation output: the integrated tilt/heading of the device.
// The world frame is Y-up (+X = right, +Y = up, +Z = back / toward viewer),
// matching the HTML test console, so a level tracker reports the identity
// quaternion. roll/pitch/yaw are a raw X-Y-Z Tait-Bryan decomposition of the
// same quaternion (kept as the inverse of the receiver's eulerToQuat); only the
// quaternion is normally transmitted.
struct Orientation {
  float quaternion[4];  // w, x, y, z  (Y-up world frame)
  float roll_deg;       // rotation about X
  float pitch_deg;      // rotation about Y
  float yaw_deg;        // rotation about Z
};

// High-level API over the 9-DoF sensor module.
//
// Owns the IMU and magnetometer drivers, a calibration set, and a Mahony fusion
// filter. The intended use is a single call to update() per loop iteration,
// after which the fused orientation() and the latest calibrated imu()/mag()
// samples are all available:
//
//   Tracker tracker(imu, mag);
//   tracker.begin();
//   ...
//   tracker.update();
//   const Orientation& o = tracker.orientation();   // integrated tilt
//   const ImuSample&   a = tracker.imu();            // calibrated accel/gyro
//
// Calibration routines (gyro bias, hard/soft-iron mag) refine the result and
// can be triggered at runtime.
class Tracker {
public:
  Tracker(ICM45686& imu, QMC6309& mag,
          const MahonyAHRS::Gains& gains =
              MahonyAHRS::Gains(TRACKER_FUSION_KP, TRACKER_FUSION_KI))
      : imuDev_(imu), magDev_(mag), fusion_(gains) {}

  // Initialise both sensors and reset the filter. Returns false if either
  // device fails to come up.
  bool begin();

  // Read both sensors, apply calibration, and advance the fusion filter by the
  // wall-clock time elapsed since the previous update(). Returns the IMU read
  // status; the magnetometer is optional (fusion drops to 6-DoF tilt-only when
  // it has no fresh sample). The first call only seeds the timestamp.
  I2CBus::Status update();

  // --- Fused output ---------------------------------------------------------
  const Orientation& orientation() const { return orientation_; }

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
  MahonyAHRS fusion_;
  Calibration cal_;

  Orientation orientation_{};
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
