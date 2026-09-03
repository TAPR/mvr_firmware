#pragma once
//
// si5351.h — Si5351A clock generator driver for the MVR GPSDO A2 firmware.
//
// Ported from mvr_synth_config.py (AN619 register math, reimplemented
// independently per that script's own header notes). The Linux i2c-dev
// ioctl layer is replaced by the standard Arduino Wire library, which is
// portable across SAMD21/RP2040/ESP32C3 already -- no custom I2C code
// needed here the way gnssSerialBegin() needed a per-core UART path.
//
// CLK0 always uses PLLA; CLK1 and CLK2 share PLLB -- same fixed assignment
// as the original script. Any single-clock change via si5351SetClockFreq()
// recomputes and reprograms ALL THREE outputs from the currently tracked
// state, not just the one that changed. That briefly glitches all three
// outputs on every change -- the original CLI tool had the same behavior,
// it's just now reachable interactively from the menu instead of only at
// boot. Worth knowing if you're watching the RF output on a scope while
// tuning from the serial menu.
//
#include <Arduino.h>

// Bring the synthesizer up: I2C init, wait for device ready, set XTAL load
// cap to 0pF (we drive XA with the VCXO, not a crystal), then apply the
// saved (or compile-time default) configuration. Call once from setup().
// Returns false if the device never became ready or any I2C write during
// configuration failed -- the caller should treat that as fatal (halt)
// rather than continuing as if the synthesizer is actually configured.
bool si5351Init();

// Set a single output's frequency (Hz) and (re-)enable it. clk is 0-2.
// Returns false if clk or freqHz is out of range (2.5 kHz-200 MHz);
// tracked state is left unchanged on a rejected call.
bool si5351SetClockFreq(uint8_t clk, uint32_t freqHz);

// Power down a single output. clk is 0-2.
bool si5351DisableClock(uint8_t clk);

// Set output drive strength (2, 4, 6, or 8 mA) on all three channels --
// the driver doesn't support per-channel drive, matching the original
// script's single --drive argument. Returns false for any other value.
bool si5351SetDrive(uint8_t driveMa);

// Resets all three clocks and drive strength to the compile-time
// SI5351_DEFAULT_* values, applies them, and persists that as the new
// saved configuration (so the reset actually sticks across the next
// boot, rather than being overwritten by whatever was saved before).
void si5351ResetToDefaults();

// Accessors for the menu / status line.
uint32_t si5351GetClockFreq(uint8_t clk);
bool     si5351IsClockEnabled(uint8_t clk);
uint8_t  si5351GetDrive();
