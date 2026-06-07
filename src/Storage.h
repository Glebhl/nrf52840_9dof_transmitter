#pragma once

#include <stddef.h>
#include <stdint.h>

// Tiny persistence API over the nRF52840 internal flash.
//
// Backed by the Adafruit InternalFileSystem (LittleFS), which reserves a flash
// region that never collides with the application code or the UF2 bootloader —
// so unlike a hand-picked NVMC address this stays safe as the firmware grows.
//
// Stores small named blobs (settings, calibration, …). Each record carries a
// magic + version + length header, so a struct whose layout changed across a
// firmware update is rejected on load instead of being read back as garbage.
// Intended for any POD value you want to survive a power cycle.
//
//   Storage::begin();
//   MyConfig cfg;
//   if (!Storage::load("cfg", 1, &cfg, sizeof(cfg))) cfg = MyConfig{};  // defaults
//   ...
//   cfg.foo = 42;
//   Storage::save("cfg", 1, &cfg, sizeof(cfg));
//
// All calls are blocking (flash erase/program). Keys are short identifiers
// (letters/digits); they map to files on the internal filesystem.
namespace Storage {

// Mount the internal filesystem. Idempotent — safe to call more than once.
// Returns false if the filesystem could not be brought up (rare; the API then
// degrades to "nothing persists" rather than crashing).
bool begin();

// Persist `len` bytes of `data` under `key`, tagged with `version`. Overwrites
// any previous value for the same key. Returns false on a write failure.
bool save(const char* key, uint16_t version, const void* data, size_t len);

// Load a value previously written with save(). Succeeds only if the stored
// record matches `key`, `version` and `len` exactly; otherwise leaves `data`
// untouched and returns false (missing, wrong version, or size mismatch).
bool load(const char* key, uint16_t version, void* data, size_t len);

// True if a record for `key` exists (regardless of version/size).
bool exists(const char* key);

// Delete the record for `key`. Returns false if it did not exist.
bool remove(const char* key);

}  // namespace Storage
