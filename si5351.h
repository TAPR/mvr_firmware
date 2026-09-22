#pragma once
//
// si5351.h — Si5351A clock generator driver for the MVR GPSDO B firmware.
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
// register's fundamental resolution limit (worst case ~7.6e-9 across
// the full 500 kHz-30 MHz operating range, affecting roughly 1 in
// 380,000 achievable 1 Hz-resolution frequencies), and to the double-
// precision floor (~1e-14) on everything else -- see
// si5351PreviewClockFreq() and FREQ_ERROR_WARN_THRESHOLD in config.h.
//
// Proven exact-frequency guarantee: because the reference is fixed at
// exactly 10,000,000 Hz (= 2^7 x 5^7), any target frequency that is
// itself an exact multiple of 10 Hz is GUARANTEED to synthesize with
// zero fractional-N error -- not just very small, but exact to the
// full precision of the chip's registers. This follows because the PLL
// feedback ratio's true denominator is always a divisor of 10,000,000,
// and multiplying a multiple-of-10 target by any integer divider keeps
// that factor of 10 intact, which caps the true denominator at
// 1,000,000 -- safely under the chip's 1,048,575 register limit, so
// the exact-fraction branch always fires. Verified exhaustively against
// every multiple of 10 Hz from 500 kHz to 30 MHz (2,950,001 frequencies,
// 100% exact). This is the simplest actionable guidance for users: pick
// frequencies to the nearest 10 Hz for a provable accuracy guarantee.
// See project notes for the full derivation and bench validation.
//
// Denominator-placement testing: a separate hypothesis -- that
// deliberately rescaling an already-exact fraction to a larger
// denominator might push any register-toggle-driven spur further from
// the carrier -- was bench-tested (variant D in the test firmware) and
// found to make no practical difference, in this implementation. Small
// (2-5 dB) differences were observed at a few offsets between the
// rescaled and un-rescaled configurations, but without consistent
// directionality (elevated at some offsets, reduced at others) -- more
// consistent with ordinary measurement variability than a systematic
// effect -- and all more than 100 dB below carrier regardless.
// Production deliberately does NOT rescale denominators as a result --
// the existing exact/minimal-denominator behavior (exactRatio() in
// si5351.cpp) is the validated, correct choice as-is. Note this
// hardware's own noise floor may be masking a real effect a lower-noise
// clock source would reveal; see project notes (Si5351A fractional-N
// report) for the full analysis and that qualifier.
//
// Output-frequency divider restriction (>112.5 MHz): AN619 restricts
// the output Multisynth divider to exactly 4, 6, or 8 above 112.5 MHz
// (900 MHz VCO / 8) -- not a free integer search. The original
// find_pll_params()-style search silently produced invalid divider
// values there (confirmed to affect ~34% of the 112.5-200 MHz range in
// bench analysis); findPllParams() now handles this as an explicit
// special case (restrictedHighFreqDiv() in si5351.cpp), and every
// computed output-stage ratio is checked against the full AN619 rule
// (isValidMultisynthRatio()) since a PLLB-sharing mismatch can in
// principle leave the non-driving clock invalid even below 112.5 MHz.
// See ClockFreqPreview::dividerValid below.
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
  bool     dividerValid;        // false if the resulting Multisynth ratio isn't one the Si5351
                                 // actually supports (AN619 requires exactly 4, 6, or >=8) --
                                 // when false, achievedHz/fractionalError are not meaningful,
                                 // since real hardware behavior for an invalid ratio is undefined

  // CLK1 and CLK2 share PLLB, so changing one can shift the OTHER's
  // achieved frequency too, if it's also enabled -- these fields are
  // only meaningful when otherClkAffected is true.
  bool     otherClkAffected;
  uint8_t  otherClk;
  uint32_t otherRequestedHz;
  double   otherAchievedHz;
  double   otherFractionalError;
  bool     otherDividerValid;   // same caveat as dividerValid, for the other clock
};

// Computes what si5351SetClockFreq(clk, freqHz) WOULD produce, without
// writing anything to hardware or changing tracked state. Uses the exact
// same computation path applyConfiguration() uses internally, so the
// preview can never diverge from what actually gets programmed if the
// caller goes on to commit it.
ClockFreqPreview si5351PreviewClockFreq(uint8_t clk, uint32_t freqHz);
