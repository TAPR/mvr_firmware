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
// Fractional-N history: the fractional PLL feedback divider (the P2/P3
// term of the AN619 registers) was originally computed by an exact
// GCD-reduced fraction where possible, falling back to a fixed-
// denominator rounding otherwise. That fallback was replaced by a true
// continued-fraction best-rational-approximation search after bench
// characterization showed the fixed-denominator method could leave a
// static frequency bias up to several parts in 1e9 on certain
// frequencies; the continued-fraction search reduces that to the
// register's fundamental resolution limit on ~98% of achievable
// frequencies (see project notes for the full analysis). A small number
// of isolated, individually-identifiable frequencies still hit that
// hardware floor regardless of algorithm -- see si5351PreviewClockFreq()
// and FREQ_ERROR_WARN_THRESHOLD in config.h.
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
// tracked state is left unchanged on a rejected call. This COMMITS the
// change immediately (writes hardware registers and persists to flash);
// see si5351PreviewClockFreq() below for a way to see what a candidate
// frequency would achieve before committing to it.
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

// ---------------------------------------------------------------------
// Preview API -- see what a candidate frequency would actually achieve
// before committing to it. Read-only: never touches hardware or tracked
// state, safe to call as often as needed (e.g. re-checked on every
// keystroke if a caller wanted to).
// ---------------------------------------------------------------------
struct ClockFreqPreview {
  bool     valid;               // false if clk or freqHz is out of range -- other fields undefined
  uint8_t  clk;
  uint32_t requestedHz;
  double   achievedHz;          // what the actual register values would produce
  double   fractionalError;     // (achievedHz - requestedHz) / requestedHz

  // CLK1 and CLK2 share PLLB, so changing one can shift the OTHER's
  // achieved frequency too, if it's also enabled -- these fields are
  // only meaningful when otherClkAffected is true.
  bool     otherClkAffected;
  uint8_t  otherClk;
  uint32_t otherRequestedHz;
  double   otherAchievedHz;
  double   otherFractionalError;
};

// Computes what si5351SetClockFreq(clk, freqHz) WOULD produce, without
// writing anything to hardware or changing tracked state. Uses the exact
// same computation path applyConfiguration() uses internally, so the
// preview can never diverge from what actually gets programmed if the
// caller goes on to commit it.
ClockFreqPreview si5351PreviewClockFreq(uint8_t clk, uint32_t freqHz);
