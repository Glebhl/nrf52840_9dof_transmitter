#pragma once

#include "Calibration.h"

// Persistence for the 9-DoF calibration set.
//
// Thin domain wrapper over Storage (see Storage.h): saves/loads the whole
// Calibration struct as one versioned blob in the internal flash, so gyro bias
// and hard/soft-iron correction survive power cycles instead of being
// recomputed every boot.
//
//   CalibrationStore::begin();
//   Calibration c;                       // identity defaults
//   CalibrationStore::load(c);           // overwritten only if a valid save exists
//   tracker.setCalibration(c);
//   ...
//   CalibrationStore::save(tracker.calibration());   // after a calibration run
//
// Bump CALIBRATION_STORAGE_VERSION (config.h) whenever the Calibration layout
// changes; old saves are then ignored and the caller keeps its defaults.
namespace CalibrationStore {

// Mount the underlying storage. Idempotent; safe to call from setup().
bool begin();

// Write the active calibration to flash. Returns false on a flash error.
bool save(const Calibration& c);

// Load the stored calibration into `c`. Returns false if nothing valid is
// stored (first boot, cleared, or an incompatible version), leaving `c`
// unchanged so the caller's defaults stand.
bool load(Calibration& c);

// Forget the stored calibration (next boot falls back to defaults).
bool clear();

}  // namespace CalibrationStore
