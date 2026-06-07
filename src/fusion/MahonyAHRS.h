#pragma once

// Mahony complementary filter (AHRS).
//
// Integrates the gyroscope rate into an orientation quaternion and continuously
// corrects the integration drift using two absolute references:
//   - the accelerometer, which sees gravity and pins roll + pitch (tilt);
//   - the magnetometer, which sees magnetic north and pins yaw (heading).
//
// The correction is a PI controller on the cross-product error between the
// estimated and measured reference directions: Kp pulls the estimate toward the
// measurement, Ki slowly cancels the residual gyro bias. This makes the filter
// cheap (no matrix algebra) and well suited to a microcontroller.
//
// Conventions: gyro in rad/s, accel and mag in any consistent unit (they are
// normalised internally). The quaternion is [w, x, y, z].
class MahonyAHRS {
public:
  struct Gains {
    float kp;  // proportional: how hard accel/mag pull the estimate
    float ki;  // integral: gyro-bias rejection (set 0 to disable)
    Gains(float p = 1.0f, float i = 0.05f) : kp(p), ki(i) {}
  };

  explicit MahonyAHRS(const Gains& gains = Gains());

  // Reset to identity orientation and clear the integral term.
  void reset();

  // 9-DoF update (gyro + accel + mag). Corrects all three axes. Falls back to
  // a 6-DoF step if the magnetometer (or accel) vector is degenerate.
  void update(const float gyro[3], const float accel[3], const float mag[3], float dt);

  // 6-DoF update (gyro + accel). Corrects tilt only; yaw is free to drift.
  void updateIMU(const float gyro[3], const float accel[3], float dt);

  // Current orientation as a quaternion [w, x, y, z].
  const float* quaternion() const { return q_; }

  // Current orientation as Tait-Bryan angles in degrees (ZYX / yaw-pitch-roll).
  void eulerDeg(float& roll, float& pitch, float& yaw) const;

private:
  // Integrate the (already bias/error-corrected) rate into the quaternion and
  // renormalise. Rates are in rad/s.
  void integrate(float gx, float gy, float gz, float dt);

  float q_[4]           = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
  float integralFB_[3]  = {0.0f, 0.0f, 0.0f};        // Ki accumulator
  float twoKp_;
  float twoKi_;
};
