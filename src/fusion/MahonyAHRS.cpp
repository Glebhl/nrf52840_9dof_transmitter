#include "MahonyAHRS.h"

#include <math.h>

namespace {
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kEps      = 1e-6f;
}  // namespace

MahonyAHRS::MahonyAHRS(const Gains& gains)
    : twoKp_(2.0f * gains.kp), twoKi_(2.0f * gains.ki) {}

void MahonyAHRS::reset() {
  q_[0] = 1.0f; q_[1] = 0.0f; q_[2] = 0.0f; q_[3] = 0.0f;
  integralFB_[0] = integralFB_[1] = integralFB_[2] = 0.0f;
}

void MahonyAHRS::updateIMU(const float gyro[3], const float accel[3], float dt) {
  float gx = gyro[0], gy = gyro[1], gz = gyro[2];
  float ax = accel[0], ay = accel[1], az = accel[2];

  const float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm > kEps) {
    const float inv = 1.0f / norm;
    ax *= inv; ay *= inv; az *= inv;

    const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];

    // Estimated direction of gravity (half the body-frame gravity vector).
    // Y-up world frame: reference gravity points along +Y, so this is the
    // second row of the body->world rotation matrix (halved).
    const float halfvx = q1 * q2 + q0 * q3;
    const float halfvy = 0.5f - q1 * q1 - q3 * q3;
    const float halfvz = q2 * q3 - q0 * q1;

    // Error = measured gravity x estimated gravity.
    float halfex = ay * halfvz - az * halfvy;
    float halfey = az * halfvx - ax * halfvz;
    float halfez = ax * halfvy - ay * halfvx;

    if (twoKi_ > 0.0f) {
      integralFB_[0] += twoKi_ * halfex * dt;
      integralFB_[1] += twoKi_ * halfey * dt;
      integralFB_[2] += twoKi_ * halfez * dt;
      gx += integralFB_[0];
      gy += integralFB_[1];
      gz += integralFB_[2];
    }
    gx += twoKp_ * halfex;
    gy += twoKp_ * halfey;
    gz += twoKp_ * halfez;
  }

  integrate(gx, gy, gz, dt);
}

void MahonyAHRS::update(const float gyro[3], const float accel[3],
                        const float mag[3], float dt) {
  float mx = mag[0], my = mag[1], mz = mag[2];
  const float mnorm = sqrtf(mx * mx + my * my + mz * mz);
  if (mnorm < kEps) {
    // No usable heading reference; correct tilt only.
    updateIMU(gyro, accel, dt);
    return;
  }

  float gx = gyro[0], gy = gyro[1], gz = gyro[2];
  float ax = accel[0], ay = accel[1], az = accel[2];
  const float anorm = sqrtf(ax * ax + ay * ay + az * az);
  if (anorm < kEps) {
    // No gravity reference; pure integration this step.
    integrate(gx, gy, gz, dt);
    return;
  }

  const float ainv = 1.0f / anorm;
  ax *= ainv; ay *= ainv; az *= ainv;
  const float minv = 1.0f / mnorm;
  mx *= minv; my *= minv; mz *= minv;

  const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];

  // Reference direction of Earth's magnetic field, rotated into the earth
  // frame. Y-up world: vertical is along +Y (hy), the horizontal plane is X-Z.
  // Flatten the measured field to a horizontal magnitude (bx, placed on +X) and
  // a vertical component (by, along +Y).
  const float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) +
                           my * (q1 * q2 - q0 * q3) +
                           mz * (q1 * q3 + q0 * q2));
  const float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) +
                           my * (0.5f - q1 * q1 - q3 * q3) +
                           mz * (q2 * q3 - q0 * q1));
  const float hz = 2.0f * (mx * (q1 * q3 - q0 * q2) +
                           my * (q2 * q3 + q0 * q1) +
                           mz * (0.5f - q1 * q1 - q2 * q2));
  const float bx = sqrtf(hx * hx + hz * hz);  // horizontal magnitude (X-Z plane)
  const float by = hy;                        // vertical component (along +Y)

  // Estimated directions of gravity and magnetic field (half vectors), Y-up.
  const float halfvx = q1 * q2 + q0 * q3;
  const float halfvy = 0.5f - q1 * q1 - q3 * q3;
  const float halfvz = q2 * q3 - q0 * q1;
  const float halfwx = bx * (0.5f - q2 * q2 - q3 * q3) + by * (q1 * q2 + q0 * q3);
  const float halfwy = bx * (q1 * q2 - q0 * q3) + by * (0.5f - q1 * q1 - q3 * q3);
  const float halfwz = bx * (q1 * q3 + q0 * q2) + by * (q2 * q3 - q0 * q1);

  // Error = sum of cross products (measured x estimated) for both references.
  float halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
  float halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
  float halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

  if (twoKi_ > 0.0f) {
    integralFB_[0] += twoKi_ * halfex * dt;
    integralFB_[1] += twoKi_ * halfey * dt;
    integralFB_[2] += twoKi_ * halfez * dt;
    gx += integralFB_[0];
    gy += integralFB_[1];
    gz += integralFB_[2];
  }
  gx += twoKp_ * halfex;
  gy += twoKp_ * halfey;
  gz += twoKp_ * halfez;

  integrate(gx, gy, gz, dt);
}

void MahonyAHRS::integrate(float gx, float gy, float gz, float dt) {
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];
  q_[0] += -q1 * gx - q2 * gy - q3 * gz;
  q_[1] +=  q0 * gx + q2 * gz - q3 * gy;
  q_[2] +=  q0 * gy - q1 * gz + q3 * gx;
  q_[3] +=  q0 * gz + q1 * gy - q2 * gx;

  const float norm = sqrtf(q_[0] * q_[0] + q_[1] * q_[1] +
                           q_[2] * q_[2] + q_[3] * q_[3]);
  if (norm > kEps) {
    const float inv = 1.0f / norm;
    q_[0] *= inv; q_[1] *= inv; q_[2] *= inv; q_[3] *= inv;
  }
}

void MahonyAHRS::eulerDeg(float& roll, float& pitch, float& yaw) const {
  const float q0 = q_[0], q1 = q_[1], q2 = q_[2], q3 = q_[3];

  roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                1.0f - 2.0f * (q1 * q1 + q2 * q2)) * kRadToDeg;

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  pitch = asinf(sinp) * kRadToDeg;

  yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
               1.0f - 2.0f * (q2 * q2 + q3 * q3)) * kRadToDeg;
}
