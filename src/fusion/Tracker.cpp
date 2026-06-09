#include "Tracker.h"

#include <Arduino.h>

namespace {
// A single axis remap: output axis i = input axis src[i], scaled by sign[i].
struct AxisTransform {
  uint8_t src[3];
  int8_t  sign[3];
};

const AxisTransform kAccelXf = {TRACKER_ACCEL_AXIS_MAP, TRACKER_ACCEL_AXIS_SIGN};
const AxisTransform kGyroXf  = {TRACKER_GYRO_AXIS_MAP, TRACKER_GYRO_AXIS_SIGN};
const AxisTransform kMagXf   = {TRACKER_MAG_AXIS_MAP, TRACKER_MAG_AXIS_SIGN};

void applyTransform(float v[3], const AxisTransform& t) {
  const float in[3] = {v[0], v[1], v[2]};
  for (int i = 0; i < 3; ++i) {
    v[i] = in[t.src[i]] * t.sign[i];
  }
}
}  // namespace

bool Tracker::begin() {
  if (!imuDev_.begin()) return false;
  if (!magDev_.begin()) return false;
  quaternion_ = {1.0f, 0.0f, 0.0f, 0.0f};
  euler_ = {0.0f, 0.0f, 0.0f};
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
  alignAxes(imuSample, magSample, haveMag);

  imu_ = imuSample;
  if (haveMag) mag_ = magSample;
  magValid_ = haveMag;

  // Placeholder values for telemetry fields that used to be produced by orientation estimation.
  quaternion_ = {1.0f, 0.0f, 0.0f, 0.0f};
  euler_ = {0.0f, 0.0f, 0.0f};

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

void Tracker::alignAxes(ImuSample& imuSample, MagSample& magSample,
                        bool haveMag) const {
  applyTransform(imuSample.accel_g, kAccelXf);
  applyTransform(imuSample.gyro_dps, kGyroXf);
  if (haveMag) {
    applyTransform(magSample.mag_g, kMagXf);
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
