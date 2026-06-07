#pragma once

#include <stdint.h>

// Calibration constants for the 9-DoF sensor module.
//
// These are applied to the scaled sensor samples before they reach the fusion
// filter. They survive across runs only if the sketch stores/restores them
// (e.g. to flash); the Tracker just keeps the active set in RAM.
//
//   gyro:  bias removal       g_cal  = g_raw  - gyroBiasDps
//   accel: bias removal       a_cal  = a_raw  - accelBiasG
//   mag:   hard + soft iron    m_cal  = (m_raw - magOffset) * magScale
//
// Defaults are the identity calibration (no correction), which is what you get
// before running any of the Tracker calibration routines.
struct Calibration {
  float gyroBiasDps[3]  = {0.0f, 0.0f, 0.0f};  // deg/s, subtracted from gyro
  float accelBiasG[3]   = {0.0f, 0.0f, 0.0f};  // g, subtracted from accel
  float magOffset[3]    = {0.0f, 0.0f, 0.0f};  // Gauss, hard-iron centre
  float magScale[3]     = {1.0f, 1.0f, 1.0f};  // soft-iron per-axis gain
};
