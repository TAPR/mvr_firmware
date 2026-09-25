//
// nv_store.cpp — see nv_store.h for design notes.
//
#include <Arduino.h>
#include <string.h>
#include <stddef.h>
#include "config.h"
#include "nv_store.h"

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_ESP32)
#include <EEPROM.h>
#else
#error "nv_store.cpp: no persistent-storage backend implemented for this core yet " \
       "(EEPROM.h isn't bundled with the SAMD21 core the way it is for RP2040/ESP32 -- " \
       "a library like FlashStorage would be needed). Either add that backend here, or " \
       "remove the nvStore*() calls from si5351.cpp/menu.cpp if building for SAMD21 " \
       "without persistence."
#endif

// Magic + version + checksum wrapper so a first-ever-boot (blank/erased
// flash), a corrupted write, or a future firmware with a different
// struct layout all get detected and safely fall back to defaults,
// rather than loading garbage as if it were valid configuration.
//
// Version 2 (2026-09): added nmeaPassthroughPersist. Bumping the version
// means a unit that already has a version-1 blob saved will fail the
// version check below and fall back to compile-time defaults for
// EVERYTHING (Si5351 config included) the first time it boots this
// firmware -- same tradeoff as any prior layout change to this struct.
// Saved clock settings, if any, will need to be re-entered via the menu
// once after updating.
struct StoredBlob {
  uint32_t magic;
  uint8_t  version;
  uint8_t  driveMa;
  uint8_t  clkEnabled[3];
  uint32_t clkFreqHz[3];
  uint8_t  nmeaPassthroughPersist;  // 0/1 -- new in version 2
  uint8_t  checksum;
};

static const uint32_t STORE_MAGIC    = 0x3152564DUL;  // arbitrary sentinel value
static const uint8_t  STORE_VERSION  = 2;              // bump if StoredBlob's layout ever changes
static const size_t   EEPROM_RESERVE = 256;             // EEPROM.begin() minimum; StoredBlob is a small fraction of this

static uint8_t computeChecksum(const StoredBlob& b) {
  // Bug fixed 2026-09-03: this used to sum sizeof(StoredBlob) -
  // sizeof(b.checksum) bytes, on the assumption that checksum sits at
  // the very end of the struct's raw memory. It doesn't -- the struct
  // contains uint32_t members, so the compiler pads the *whole struct*
  // out to a 4-byte boundary, leaving a few bytes of padding *after*
  // checksum. That padding was silently absorbed into sizeof(),
  // shifting the summed range to include the checksum field itself.
  // At save time that's harmless (the field is still zero when summed
  // over), but at load time the field holds the real stored value, so
  // summing over it again doubles it into the comparison -- confirmed
  // on real hardware: computed checksums were consistently exactly 2x
  // the stored ones (mod 256). offsetof() gives the field's actual
  // position regardless of any padding, which sizeof() arithmetic
  // can't guarantee. Still correct with nmeaPassthroughPersist added
  // ahead of checksum in version 2 -- offsetof() moves with it
  // automatically, no change needed here.
  const uint8_t* p = (const uint8_t*)&b;
  uint8_t sum = 0;
  for (size_t i = 0; i < offsetof(StoredBlob, checksum); i++) sum = (uint8_t)(sum + p[i]);
  return sum;
}

// Reads and validates the raw blob. Returns false (b left as whatever
// EEPROM.get() put there -- do not trust its contents on a false return)
// if nothing valid is stored, so callers can fall back to an all-zero
// default blob instead of partially-garbage data.
static bool loadRawBlob(StoredBlob& b) {
  EEPROM.get(0, b);
  if (b.magic != STORE_MAGIC || b.version != STORE_VERSION) return false;
  if (computeChecksum(b) != b.checksum) return false;
  return true;
}

// Stamps magic/version/checksum and commits b to flash. Callers build b
// by first loading the existing blob (to preserve whatever they're not
// touching) and then overwriting only their own fields -- see
// nvStoreSave() and nvStoreSavePassthrough() below. This is what lets
// the Si5351 config and the passthrough flag be saved independently
// without one clobbering the other.
static void writeRawBlob(StoredBlob& b) {
  b.magic    = STORE_MAGIC;
  b.version  = STORE_VERSION;
  b.checksum = computeChecksum(b);
  EEPROM.put(0, b);          // only flags dirty if different from what's already stored
  EEPROM.commit();           // only touches flash if dirty
}

void nvStoreInit() {
  EEPROM.begin(EEPROM_RESERVE);
}

bool nvStoreLoad(StoredSi5351Config& cfg) {
  StoredBlob b = {};
  if (!loadRawBlob(b)) return false;

  cfg.driveMa = b.driveMa;
  for (uint8_t c = 0; c < 3; c++) {
    cfg.clkEnabled[c] = (b.clkEnabled[c] != 0);
    cfg.clkFreqHz[c]  = b.clkFreqHz[c];
  }
  return true;
}

void nvStoreSave(const StoredSi5351Config& cfg) {
  StoredBlob b = {};
  loadRawBlob(b);  // best-effort: preserve the passthrough flag if one's
                    // already stored; if this is a first save (or the
                    // stored blob was invalid/older-version), b is left
                    // zero-initialized, which is the correct default
                    // (passthrough off).

  b.driveMa = cfg.driveMa;
  for (uint8_t c = 0; c < 3; c++) {
    b.clkEnabled[c] = cfg.clkEnabled[c] ? 1 : 0;
    b.clkFreqHz[c]  = cfg.clkFreqHz[c];
  }
  writeRawBlob(b);
}

bool nvStoreLoadPassthrough(bool& enabled) {
  StoredBlob b = {};
  if (!loadRawBlob(b)) return false;

  enabled = (b.nmeaPassthroughPersist != 0);
  return true;
}

void nvStoreSavePassthrough(bool enabled) {
  StoredBlob b = {};
  loadRawBlob(b);  // best-effort: preserve the Si5351 config if one's
                    // already stored -- see nvStoreSave() above.

  b.nmeaPassthroughPersist = enabled ? 1 : 0;
  writeRawBlob(b);
}
