#pragma once
//
// nv_store.h — persistent storage for user-configurable settings.
//
// The RP2040 doesn't have true NVRAM (battery-backed or otherwise
// genuinely nonvolatile static RAM) -- what it has is the same QSPI
// flash the program itself lives in. arduino-pico bundles an EEPROM
// emulation library (no extra install needed, unlike Adafruit NeoPixel)
// that reserves a 4KB sector of that flash and presents it with the
// classic Arduino EEPROM.h API -- the same shape AVR and ESP32 boards
// use. This module wraps that.
//
// Flash write endurance is finite -- rated similarly to the flash chip
// itself, not the ~100,000+ cycles of a dedicated EEPROM part -- so this
// is fine for "save when the user changes something via the menu", not
// fine for writing every loop() iteration. nvStoreSave() is only ever
// called from deliberate menu actions (see si5351.cpp), and the
// underlying EEPROM.commit() only actually touches flash when the data
// changed (verified against the bundled library's source), so routine
// use here is not a wear concern.
//
// RP2040-only for now (also compiles for ESP32 using its own, differently
// -verified but API-identical EEPROM.h, in case this ever gets built for
// that variant). Porting to SAMD21 needs a different backing library --
// there's no built-in EEPROM.h on that core -- see the #error in
// nv_store.cpp.
//
#include <Arduino.h>

struct StoredSi5351Config {
  uint8_t  driveMa;
  bool     clkEnabled[3];
  uint32_t clkFreqHz[3];
};

// Call once from setup() (si5351Init() does this internally) before any
// load/save calls.
void nvStoreInit();

// Loads the saved configuration into cfg. Returns false (cfg left
// untouched) if nothing valid has ever been saved -- first boot, or
// flash that's been erased/corrupted -- so the caller can fall back to
// compile-time defaults.
bool nvStoreLoad(StoredSi5351Config& cfg);

// Saves cfg to flash. Cheap to call after every menu change -- see the
// wear note above.
void nvStoreSave(const StoredSi5351Config& cfg);
