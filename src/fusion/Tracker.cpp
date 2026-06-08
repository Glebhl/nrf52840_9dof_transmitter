#include "Tracker.h"

#include <Arduino.h>

namespace {
constexpr float kDegToRad = 0.017453292519943295f;

// A single axis remap: output axis i = input axis src[i], scaled by sign[i].
struct AxisTransform {
  uint8_t src[3];
  int8_t  sign[3];
};

// Fold the per-sensor map and the shared body map into one remap, so each
// sample is rotated in a single pass instead of two. The result is identical
// to applying the per-sensor map and then the body map in sequence:
//   v -> body(sensor(v))
AxisTransform compose(const uint8_t sSrc[3], const int8_t sSign[3],
                      const uint8_t bSrc[3], const int8_t bSign[3]) {
  AxisTransform t{};
  for (int i = 0; i < 3; ++i) {
    const uint8_t mid = bSrc[i];
    t.src[i]  = sSrc[mid];
    t.sign[i] = static_cast<int8_t>(sSign[mid] * bSign[i]);
  }
  return t;
}

const uint8_t kAccelSrc[3]  = TRACKER_ACCEL_AXIS_MAP;
const int8_t  kAccelSign[3] = TRACKER_ACCEL_AXIS_SIGN;
const uint8_t kGyroSrc[3]   = TRACKER_GYRO_AXIS_MAP;
const int8_t  kGyroSign[3]  = TRACKER_GYRO_AXIS_SIGN;
const uint8_t kMagSrc[3]    = TRACKER_MAG_AXIS_MAP;
const int8_t  kMagSign[3]   = TRACKER_MAG_AXIS_SIGN;
const uint8_t kBodySrc[3]   = TRACKER_BODY_AXIS_MAP;
const int8_t  kBodySign[3]  = TRACKER_BODY_AXIS_SIGN;

const AxisTransform kAccelXf = compose(kAccelSrc, kAccelSign, kBodySrc, kBodySign);
const AxisTransform kGyroXf  = compose(kGyroSrc, kGyroSign, kBodySrc, kBodySign);
const AxisTransform kMagXf   = compose(kMagSrc, kMagSign, kBodySrc, kBodySign);

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
  alignAxes(imuSample, magSample, haveMag);

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
    const float gyroRad[3] = {
      imuSample.gyro_dps[0] * kDegToRad,
      imuSample.gyro_dps[1] * kDegToRad,
      imuSample.gyro_dps[2] * kDegToRad,
    };

    if (haveMag) {
      fusion_.update(gyroRad, imuSample.accel_g, magSample.mag_g, dt);
    } else {
      fusion_.updateIMU(gyroRad, imuSample.accel_g, dt);
    }
  }

  // The fusion runs in a right-handed Y-up world, but the HTML viewer renders
  // on a left-handed CSS screen (Y points down). That handedness flip reverses
  // rotations about every axis except the reflection axis (X). Conjugating by
  // 180 deg about X — i.e. negating y and z — matches the viewer's convention.
  const float* q = fusion_.quaternion();
  orientation_.quaternion[0] = q[0];
  orientation_.quaternion[1] = q[1];
  orientation_.quaternion[2] = -q[2];
  orientation_.quaternion[3] = -q[3];
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
