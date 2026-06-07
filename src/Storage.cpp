#include "Storage.h"

#include <string.h>

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

namespace {

// Marks a well-formed Storage record. Bump nothing here on struct changes —
// per-record versioning is the caller's `version` argument.
constexpr uint32_t kMagic = 0x53544F52;  // "STOR"

// Fixed-size header prepended to every payload on flash. Lets load() reject a
// record whose owner, layout version or size no longer matches the request.
struct Header {
  uint32_t magic;
  uint16_t version;
  uint16_t len;
};

bool g_mounted = false;

// LittleFS paths are absolute; turn a short key into "/<key>". Keys are assumed
// short (a handful of chars); longer ones are truncated to fit the buffer.
void makePath(const char* key, char* out, size_t outSize) {
  out[0] = '/';
  size_t i = 0;
  for (; key[i] != '\0' && i + 2 < outSize; ++i) {
    out[i + 1] = key[i];
  }
  out[i + 1] = '\0';
}

}  // namespace

namespace Storage {

bool begin() {
  if (g_mounted) return true;
  g_mounted = InternalFS.begin();
  return g_mounted;
}

bool save(const char* key, uint16_t version, const void* data, size_t len) {
  if (!begin() || data == nullptr || len == 0 || len > UINT16_MAX) return false;

  char path[32];
  makePath(key, path, sizeof(path));

  // Rewrite from scratch: LittleFS opens for write in append mode, so an
  // existing (possibly larger) record must be removed first.
  InternalFS.remove(path);

  File f(InternalFS);
  if (!f.open(path, FILE_O_WRITE)) return false;

  const Header h{kMagic, version, (uint16_t)len};
  bool ok = f.write((const uint8_t*)&h, sizeof(h)) == sizeof(h) &&
            f.write((const uint8_t*)data, len) == (int)len;
  f.close();

  if (!ok) InternalFS.remove(path);  // don't leave a half-written record
  return ok;
}

bool load(const char* key, uint16_t version, void* data, size_t len) {
  if (!begin() || data == nullptr || len == 0) return false;

  char path[32];
  makePath(key, path, sizeof(path));

  File f(InternalFS);
  if (!f.open(path, FILE_O_READ)) return false;

  Header h{};
  bool ok = f.read((uint8_t*)&h, sizeof(h)) == sizeof(h) &&
            h.magic == kMagic && h.version == version && h.len == len &&
            f.read((uint8_t*)data, len) == (int)len;
  f.close();
  return ok;
}

bool exists(const char* key) {
  if (!begin()) return false;
  char path[32];
  makePath(key, path, sizeof(path));
  return InternalFS.exists(path);
}

bool remove(const char* key) {
  if (!begin()) return false;
  char path[32];
  makePath(key, path, sizeof(path));
  return InternalFS.remove(path);
}

}  // namespace Storage
