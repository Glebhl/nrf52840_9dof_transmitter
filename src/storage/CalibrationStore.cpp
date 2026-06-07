#include "CalibrationStore.h"

#include "Storage.h"
#include "config.h"

namespace CalibrationStore {

bool begin() {
  return Storage::begin();
}

bool save(const Calibration& c) {
  return Storage::save(CALIBRATION_STORAGE_KEY, CALIBRATION_STORAGE_VERSION,
                       &c, sizeof(c));
}

bool load(Calibration& c) {
  return Storage::load(CALIBRATION_STORAGE_KEY, CALIBRATION_STORAGE_VERSION,
                       &c, sizeof(c));
}

bool clear() {
  return Storage::remove(CALIBRATION_STORAGE_KEY);
}

}  // namespace CalibrationStore
