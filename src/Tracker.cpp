#include "Tracker.h"

#include <Arduino.h>

namespace {
constexpr float kDegToRad = 0.017453292519943295f;
}  // namespace

bool Tracker::begin() {
  if (!imuDev_.begin()) return false;
  if (!magDev_.begin()) return false;
  fusion_.reset();
  haveTimestamp_ = false;
  magValid_ = false;
  return true;
}

I2CBus::Status Tracker::update() {
  ImuSample imuSample;
  const I2CBus::Status s = imuDev_.read(imuSample);
  if (s != I2CBus::Status::Ok) return s;

  MagSample magSample{};
  bool haveMag = false;
  if (magDev_.read(magSample) == I2CBus::Status::Ok &&
      magSample.dataReady && !magSample.overflow) {
    haveMag = true;
  }

  applyCalibration(imuSample, magSample, haveMag);

  imu_ = imuSample;
  if (haveMag) mag_ = magSample;
  magValid_ = haveMag;

  // Time step from the wall clock; uint32 subtraction handles micros() wrap.
  const uint32_t now = micros();
  float dt = 0.0f;
  if (haveTimestamp_) {
    dt = (now - lastUpdateUs_) * 1e-6f;
  }
  lastUpdateUs_ = now;
  haveTimestamp_ = true;

  // Guard against the seed step (dt == 0) and pathological gaps (> 1 s).
  if (dt > 0.0f && dt < 1.0f) {
    float gyroRad[3];
    gyroRad[1] =  imuSample.gyro_dps[0] * kDegToRad;  // X
    gyroRad[0] =  imuSample.gyro_dps[1] * kDegToRad;  // Y
    gyroRad[2] =  imuSample.gyro_dps[2] * kDegToRad;  // Z (yaw уже верный)

    if (haveMag) {
      // The QMC6309 sits in a different frame than the ICM45686. Its axes are a
      // cyclic permutation of the IMU axes (IMU X = mag Y, IMU Y = mag Z,
      // IMU Z = mag X), so remap into the IMU frame before fusing. The cycle is
      // a proper rotation (no handedness flip). If yaw runs away or locks the
      // wrong way, flip the sign of the offending axis (see note below).
      const float magAligned[3] = {
        magSample.mag_g[1],  // IMU X  <- mag Y
        magSample.mag_g[2],  // IMU Y  <- mag Z
        magSample.mag_g[0],  // IMU Z  <- mag X
      };
      fusion_.update(gyroRad, imuSample.accel_g, magAligned, dt);
    } else {
      fusion_.updateIMU(gyroRad, imuSample.accel_g, dt);
    }
  }

  const float* q = fusion_.quaternion();
  for (int i = 0; i < 4; ++i) orientation_.quaternion[i] = q[i];
  fusion_.eulerDeg(orientation_.roll_deg, orientation_.pitch_deg, orientation_.yaw_deg);

  return I2CBus::Status::Ok;
}

void Tracker::applyCalibration(ImuSample& imuSample, MagSample& magSample,
                               bool haveMag) const {
  for (int i = 0; i < 3; ++i) {
    imuSample.gyro_dps[i] -= cal_.gyroBiasDps[i];
    imuSample.accel_g[i]  -= cal_.accelBiasG[i];
  }
  if (haveMag) {
    for (int i = 0; i < 3; ++i) {
      magSample.mag_g[i] = (magSample.mag_g[i] - cal_.magOffset[i]) * cal_.magScale[i];
    }
  }
}

bool Tracker::calibrateGyro(uint16_t samples) {
  if (samples == 0) return false;

  double sum[3] = {0.0, 0.0, 0.0};
  float  mn[3] = {0.0f, 0.0f, 0.0f};
  float  mx[3] = {0.0f, 0.0f, 0.0f};
  uint16_t got = 0;

  for (uint16_t i = 0; i < samples; ++i) {
    ImuSample s;
    if (imuDev_.read(s) == I2CBus::Status::Ok) {
      for (int a = 0; a < 3; ++a) {
        const float v = s.gyro_dps[a];
        sum[a] += v;
        if (got == 0 || v < mn[a]) mn[a] = v;
        if (got == 0 || v > mx[a]) mx[a] = v;
      }
      ++got;
    }
    delay(2);
  }

  if (got < samples / 2) return false;  // too many failed reads

  for (int a = 0; a < 3; ++a) {
    if (mx[a] - mn[a] > TRACKER_GYRO_CAL_MAX_SPREAD_DPS) {
      return false;  // device was moving — don't trust the average
    }
  }
  for (int a = 0; a < 3; ++a) {
    cal_.gyroBiasDps[a] = (float)(sum[a] / got);
  }

  fusion_.reset();
  haveTimestamp_ = false;
  return true;
}

void Tracker::beginMagCalibration() {
  magCalActive_ = true;
  for (int i = 0; i < 3; ++i) {
    magMin_[i] = 1e30f;
    magMax_[i] = -1e30f;
  }
}

void Tracker::collectMagCalibration() {
  if (!magCalActive_) return;

  MagSample s{};
  if (magDev_.read(s) != I2CBus::Status::Ok) return;
  if (!s.dataReady || s.overflow) return;

  for (int i = 0; i < 3; ++i) {
    if (s.mag_g[i] < magMin_[i]) magMin_[i] = s.mag_g[i];
    if (s.mag_g[i] > magMax_[i]) magMax_[i] = s.mag_g[i];
  }
}

void Tracker::finishMagCalibration() {
  if (!magCalActive_) return;
  magCalActive_ = false;

  float radius[3];
  float avgRadius = 0.0f;
  for (int i = 0; i < 3; ++i) {
    cal_.magOffset[i] = 0.5f * (magMax_[i] + magMin_[i]);  // hard-iron centre
    radius[i] = 0.5f * (magMax_[i] - magMin_[i]);
    avgRadius += radius[i];
  }
  avgRadius /= 3.0f;

  // Soft-iron diagonal: stretch each axis so its swing matches the average.
  for (int i = 0; i < 3; ++i) {
    cal_.magScale[i] = (radius[i] > 1e-6f) ? (avgRadius / radius[i]) : 1.0f;
  }
}
