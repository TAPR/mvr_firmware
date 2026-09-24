//
// si5351.cpp — see si5351.h for the public API and porting notes.
//
// Layout mirrors mvr_synth_config.py where practical:
//   - register/bit constants
//   - low-level I2C helpers (Wire-based, replaces the Linux i2c-dev layer)
//   - AN619 rational-approximation math (find_multisynth_params /
//     find_pll_params / the shared-VCO search for CLK1+CLK2)
//   - register packing
//   - computeClockPlan() -- pure computation, no hardware I/O; shared by
//     applyConfiguration() (which writes it to hardware) and
//     si5351PreviewClockFreq() (which doesn't)
//   - top-level "recompute everything from tracked state" apply function
//   - public API
//
// Not ported: choose_r_divider() from the original script. It was defined
// there but never actually called in configure_si5351() -- dead code for
// low output frequencies below the plain Multisynth range. Left out here
// too; flag if you ever need sub-500ish-kHz outputs and we'll wire in an
// R-divider path.
//
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "config.h"
#include "si5351.h"
#include "nv_store.h"

// ---------------------------------------------------------------------
// Register / bit constants (AN619)
// ---------------------------------------------------------------------
static const uint8_t SI5351_REG_DEVICE_STATUS = 0;
static const uint8_t SI5351_REG_OUTPUT_ENABLE = 3;
static const uint8_t SI5351_REG_CLK_BASE      = 16;
static const uint8_t SI5351_REG_MSNA_BASE     = 26;
static const uint8_t SI5351_REG_MSNB_BASE     = 34;
static const uint8_t SI5351_REG_MS0_BASE      = 42;
static const uint8_t SI5351_REG_PLL_RESET     = 177;
static const uint8_t SI5351_REG_XTAL_LOAD     = 183;
// FBA_INT/FBB_INT (AN619 sec. "Manually Generating an Si5351 Register
// Map", integer-PLL-feedback-mode bits) are bit D6 of these two
// registers -- oddly co-located with the CLK6/CLK7 Control registers,
// which this firmware never otherwise touches (only CLK0-2 are used).
// "In most cases setting this bit will improve jitter when using even
// integer divide values" (AN619) -- set whenever the PLL feedback ratio
// happens to reduce to a pure integer (b=0), which findPllParams() /
// findSharedPllbVco() now actively prefer when available. Read-modify-
// write, not a blind write, specifically so CLK6/CLK7's own bits in
// these registers -- whatever their reset state leaves them as -- are
// never disturbed by a change that has nothing to do with them.
static const uint8_t SI5351_REG_FBA_INT       = 22;
static const uint8_t SI5351_REG_FBB_INT       = 23;
static const uint8_t FBX_INT_BIT              = (1 << 6);

static const uint8_t XTAL_CL_0PF = 0b00;  // driving XA with an external clock, not a crystal

static const uint8_t CLK_PDN      = (1 << 7);
static const uint8_t CLK_INT_MODE = (1 << 6);
static const uint8_t CLK_PLL_SRC  = (1 << 5);
static const uint8_t CLK_SRC_MSX  = 0b11 << 2;
static const uint8_t CLK_IDRV_2MA = 0b00;
static const uint8_t CLK_IDRV_4MA = 0b01;
static const uint8_t CLK_IDRV_6MA = 0b10;
static const uint8_t CLK_IDRV_8MA = 0b11;

static const uint32_t VCO_MIN    = 600000000UL;
static const uint32_t VCO_MAX    = 900000000UL;
static const uint32_t MS_DIV_MIN = 8UL;
static const uint32_t FOUT_MIN   = 2500UL;
static const uint32_t FOUT_MAX   = 200000000UL;
static const uint32_t MAX_DENOM  = 1048575UL;  // 20-bit P3 field

// ---------------------------------------------------------------------
// Tracked state -- the "desired configuration" that applyConfiguration()
// programs in full on every call.
// ---------------------------------------------------------------------
struct ClockState { bool enabled; uint32_t freqHz; };
static ClockState s_clock[3];
static uint8_t    s_driveMa = SI5351_DEFAULT_DRIVE_MA;

// ---------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------
static void si5351WireBegin() {
#if defined(ARDUINO_ARCH_RP2040)
  Wire.setSDA(SI5351_I2C_BUS_SDA);
  Wire.setSCL(SI5351_I2C_BUS_SCL);
  Wire.begin();
#elif defined(ARDUINO_ARCH_ESP32)
  Wire.begin(SI5351_I2C_BUS_SDA, SI5351_I2C_BUS_SCL);
#else
  // SAMD21: Wire is already fixed to the D4/D5 hardware I2C pins in the
  // board variant definition.
  Wire.begin();
#endif
}

static bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(SI5351_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    CMD_SERIAL.print(F("  I2C write failed (reg "));
    CMD_SERIAL.print(reg);
    CMD_SERIAL.print(F(", err "));
    CMD_SERIAL.print(err);
    CMD_SERIAL.println(F(")"));
  }
  return (err == 0);
}

static bool writeRegs(uint8_t regStart, const uint8_t* data, uint8_t len) {
  Wire.beginTransmission(SI5351_I2C_ADDR);
  Wire.write(regStart);
  Wire.write(data, len);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    CMD_SERIAL.print(F("  I2C block write failed (reg "));
    CMD_SERIAL.print(regStart);
    CMD_SERIAL.print(F(", err "));
    CMD_SERIAL.print(err);
    CMD_SERIAL.println(F(")"));
  }
  return (err == 0);
}

static uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(SI5351_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);  // repeated start, keep the bus
  Wire.requestFrom((uint8_t)SI5351_I2C_ADDR, (uint8_t)1);
  if (Wire.available()) return (uint8_t)Wire.read();
  return 0xFF;  // read failure sentinel; device status bit 7 would read as "still init"
}

// Sets or clears one bit in a register via read-modify-write, leaving
// every other bit exactly as it was. Used only for FBA_INT/FBB_INT (see
// their definition above) so CLK6/CLK7's unrelated bits in the same
// registers are never disturbed. Treats a read failure (0xFF sentinel)
// as "don't know the real value" and skips the write rather than
// risking corrupting bits we can't actually see.
static bool setRegBit(uint8_t reg, uint8_t bit, bool set) {
  uint8_t cur = readReg(reg);
  if (cur == 0xFF) {
    CMD_SERIAL.print(F("  WARNING: could not read reg "));
    CMD_SERIAL.print(reg);
    CMD_SERIAL.println(F(" to set FBx_INT -- skipping (I2C read failure)"));
    return false;
  }
  uint8_t next = set ? (uint8_t)(cur | bit) : (uint8_t)(cur & ~bit);
  return writeReg(reg, next);
}

static bool waitForInit(uint32_t timeoutMs = 500) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (!(readReg(SI5351_REG_DEVICE_STATUS) & 0x80)) return true;
    delay(10);
  }
  return false;
}

// ---------------------------------------------------------------------
// AN619 rational-approximation math
// ---------------------------------------------------------------------
static uint32_t u32gcd(uint32_t a, uint32_t b) {
  while (b) { uint32_t t = a % b; a = b; b = t; }
  return a;
}

struct An619Ratio { uint32_t p1, p2, p3; };

// Internal a/b/c form (ratio = a + b/c) used during computation; only
// converted to the register-packed p1/p2/p3 form at the end. Kept
// separate from An619Ratio because achieved-frequency calculations
// (needed by the preview API) are far more natural in a/b/c form.
struct Ratio { uint32_t a, b, c; };

static void packRatio(const Ratio& r, An619Ratio& out) {
  out.p1 = 128UL * r.a + (128UL * r.b) / r.c - 512UL;
  out.p2 = 128UL * r.b - r.c * ((128UL * r.b) / r.c);
  out.p3 = r.c;
}

// Exact GCD-reduced fraction. Returns false if the reduced denominator
// doesn't fit the chip's 20-bit P3 field (i.e. no exact representation
// is possible within this register width).
static bool exactRatio(uint32_t num, uint32_t den, Ratio& out) {
  uint32_t g = u32gcd(num, den);
  uint32_t rn = num / g, rd = den / g;
  if (rd > MAX_DENOM) return false;
  out.a = rn / rd;
  out.b = rn % rd;
  out.c = rd;
  return true;
}

// Best rational approximation via continued-fraction convergents,
// searched up to the chip's maximum denominator. If an exact fraction
// exists within that denominator, this finds the same exact fraction as
// exactRatio() (checked first, as a fast path -- the search would find
// it too, but skipping straight to it avoids 64 needless iterations on
// the overwhelmingly common case).
//
// Replaces this file's original fixed-denominator-rounding fallback
// after bench characterization (project notes, 2026-09) showed that
// method could leave a static frequency bias up to several parts in
// 1e9 on certain frequencies. This continued-fraction search reduces
// that to the register's fundamental resolution limit -- worst case
// ~7.6e-9 fractional frequency error, occurring on roughly 1 in 380,000
// achievable 1 Hz-resolution frequencies across the full 500 kHz-30 MHz
// operating range -- and to the double-precision floor (~1e-14) on
// everything else. See FREQ_ERROR_WARN_THRESHOLD in config.h and
// si5351PreviewClockFreq() for how the remaining hard cases are
// surfaced to the operator rather than silently accepted, and
// si5351.h's header comment for the proven exact-multiple-of-10-Hz
// guarantee that avoids this entirely.
//
// Independent implementation of the standard technique described at
// https://en.wikipedia.org/wiki/Continued_fraction#Best_rational_approximations
static Ratio fractionalRatio(uint32_t num, uint32_t den) {
  Ratio out;
  if (exactRatio(num, den, out)) return out;

  double value = (double)num / (double)den;
  double af = floor(value);
  double f0 = value - af;
  uint32_t a = (uint32_t)af;
  uint32_t b = 0, c = 1;
  double f = f0;
  double delta = f0;
  const double epsilon = 1e-15;
  uint32_t h[2] = { 1, 0 };
  uint32_t k[2] = { 0, 1 };
  for (int i = 0; i < 64; i++) {
    if (f <= epsilon) break;
    double inv = 1.0 / f;
    double anf = floor(inv);
    f = inv - anf;
    uint32_t an = (uint32_t)anf;
    for (uint32_t m = (an + 1) / 2; m <= an; m++) {
      uint32_t hm = m * h[1] + h[0];
      uint32_t km = m * k[1] + k[0];
      if (km > MAX_DENOM) break;
      double d = fabs((double)hm / (double)km - f0);
      if (d < delta) { delta = d; b = hm; c = km; }
    }
    uint32_t hn = an * h[1] + h[0];
    uint32_t kn = an * k[1] + k[0];
    h[0] = h[1]; h[1] = hn;
    k[0] = k[1]; k[1] = kn;
  }
  return Ratio{ a, b, c };
}

struct PllSolution { Ratio ratio; uint32_t fvco; };

// Above VCO_MAX/MS_DIV_MIN (900 MHz / 8 = 112.5 MHz), AN619 restricts the
// output Multisynth divider to exactly 4, 6, or 8 -- not a free integer
// search -- and above that point the output frequency effectively sets
// the VCO:
//   112.5 MHz < fout <= 150 MHz  -> div = 6  (fvco = 6*fout, in [675M,900M])
//   150 MHz   < fout <= FOUT_MAX -> div = 4  (fvco = 4*fout, in [600M,800M]
//                                              for FOUT_MAX=200MHz)
// The general search below already only accepts div>=MS_DIV_MIN(8) for
// fout<=112.5MHz, which is correct there; above that threshold no such
// div exists in [VCO_MIN,VCO_MAX] at all, which is what this special
// case is for. See project notes (Si5351A fractional-N report) for the
// bench finding that motivated this.
static const uint32_t HIGH_FREQ_DIV_THRESHOLD = VCO_MAX / MS_DIV_MIN;  // 112,500,000

static bool restrictedHighFreqDiv(uint32_t fout, uint32_t& divOut) {
  if (fout <= HIGH_FREQ_DIV_THRESHOLD) return false;
  divOut = (fout <= 150000000UL) ? 6UL : 4UL;
  return true;
}

// True iff (a,b,c) is a legal AN619 Multisynth ratio: exactly 4 or 6 (no
// fraction), or >=8 (any fraction, integer or not). Note 5 and 7 are
// NOT valid even as plain integers -- the chip's Multisynth divider
// simply doesn't support them. Checked on every computed output-stage
// ratio (not just the PLL-feedback-driving clock) because a PLLB-sharing
// mismatch can in principle leave the non-driving clock with an invalid
// ratio even when its own target frequency is below 112.5 MHz.
static bool isValidMultisynthRatio(const Ratio& r) {
  if (r.a == 4 || r.a == 6) return (r.b == 0);
  return (r.a >= 8);
}

// Picks the VCO frequency in [VCO_MIN, VCO_MAX] used to synthesize fout,
// preferring (in order):
//   1. The highest-VCO divisor that also makes fvco an exact multiple of
//      fxtal -- eligible for FBA_INT/FBB_INT (integer PLL feedback mode,
//      AN619's jitter-reduction bit) -- if any such divisor exists.
//   2. Otherwise, the highest-VCO divisor available at all.
// Both preferences favor a HIGH VCO, not the lowest one that merely fits.
// This is a deliberate change from an earlier version of this function,
// which took the first (lowest) valid divisor. That was suboptimal for
// phase noise: for a fixed target fout, the feedback divider's
// 20*log10(N) noise multiplication and the output divider's 20*log10(M)
// noise reduction move together and cancel exactly (N/M = fout/fxtal is
// fixed), so the VCO/divisor choice doesn't affect loop-referred noise --
// but it does directly affect how much the VCO's own free-running noise
// gets divided down before reaching the output, which favors the HIGHEST
// achievable VCO. See project notes for the full derivation; not yet
// bench-validated against real hardware (the accuracy proofs and the
// FBA_INT-eligibility math are exact and unaffected by this preference
// either way).
//
// Both preferences are computed directly (O(1)), not by scanning the
// divisor range -- important on RP2040/SAMD21, which have no hardware
// divide, and the range can span thousands of divisors at low output
// frequencies. fout*div increases monotonically with div, so:
//   - the highest achievable VCO is always fout*divHi, no search needed.
//   - fout*div is a multiple of fxtal exactly when div is a multiple of
//     step = fxtal / gcd(fout, fxtal); the largest such divisor <= divHi
//     is (divHi/step)*step, again no search needed.
// Cross-checked against an exhaustive scan over several million cases
// with zero mismatches before this replaced the scanning version.
static PllSolution findPllParams(uint32_t fout, uint32_t fxtal) {
  uint32_t bestFvco;
  uint32_t restrictedDiv;

  if (restrictedHighFreqDiv(fout, restrictedDiv)) {
    bestFvco = fout * restrictedDiv;
  } else {
    uint32_t divLo = (VCO_MIN + fout - 1) / fout;  // ceil
    if (divLo < MS_DIV_MIN) divLo = MS_DIV_MIN;
    uint32_t divHi = VCO_MAX / fout;               // floor

    if (divLo <= divHi) {
      uint32_t highestFvco = fout * divHi;

      uint32_t g = u32gcd(fout, fxtal);
      uint32_t step = fxtal / g;
      uint32_t candidate = (divHi / step) * step;

      bestFvco = (candidate >= divLo) ? (fout * candidate) : highestFvco;
    } else {
      // Shouldn't occur below HIGH_FREQ_DIV_THRESHOLD given how that
      // threshold is derived, but keep a safety net.
      bestFvco = fout * divLo;
      if (bestFvco < VCO_MIN) bestFvco = VCO_MIN;
    }
  }

  PllSolution sol;
  sol.fvco = bestFvco;
  sol.ratio = fractionalRatio(bestFvco, fxtal);
  return sol;
}

// Searches for a single VCO frequency that gives an integer divide for
// BOTH f1 and f2 (CLK1 and CLK2 sharing PLLB), preferring the same way
// findPllParams() does: highest achievable VCO, with a bonus preference
// for one that's also an exact multiple of fxtal (FBA_INT-eligible).
// See findPllParams() above for the phase-noise rationale.
//
// Uses uint64_t throughout -- f1*f2 can reach ~4x10^16, well past
// uint32_t range, before the LCM reduction brings it back down.
//
// Declines to even attempt sharing whenever either target is above
// HIGH_FREQ_DIV_THRESHOLD: above that point the output frequency pins
// the VCO to one of only two possible values (via the restricted-divider
// rule above), so genuine sharing is only possible by rare coincidence,
// and searching for it isn't worth the complexity. The caller's solo
// fallback (findPllParams(), restricted-divider-aware) handles the
// driving clock correctly either way; isValidMultisynthRatio() catches
// the case where the other, non-driving clock ends up incompatible.
//
// Computed directly (O(1)), not by scanning n: any fvcoCand = lcmF*n is
// automatically an exact multiple of both f1 and f2 (that's what lcmF
// means), so the div1/div2 >= MS_DIV_MIN checks reduce to two fixed
// lower bounds on n (n1, n2 below) rather than needing a per-n test --
// every n from max(nLo,n1,n2) up to nHi is valid, so the highest is
// just lcmF*nHi, and the highest multiple-of-fxtal candidate is found
// the same way findPllParams() finds one. Cross-checked against a full
// brute-force scan over 215 million (f1,f2) pairs with zero mismatches
// (found/not-found agreement and value agreement both) before this
// replaced the scanning version.
static bool findSharedPllbVco(uint32_t f1, uint32_t f2, uint32_t fxtal, uint32_t& outFvco) {
  if (f1 > HIGH_FREQ_DIV_THRESHOLD || f2 > HIGH_FREQ_DIV_THRESHOLD) return false;

  uint32_t g = u32gcd(f1, f2);
  uint64_t lcmF = (uint64_t)f1 * (uint64_t)f2 / g;
  if (lcmF == 0 || lcmF > VCO_MAX) return false;

  uint64_t nLo = (VCO_MIN + lcmF - 1) / lcmF;  // ceil
  uint64_t nHi = VCO_MAX / lcmF;               // floor

  uint64_t k1 = lcmF / f1, k2 = lcmF / f2;     // both exact integers -- f1,f2 | lcmF by definition
  uint64_t n1 = (MS_DIV_MIN + k1 - 1) / k1;    // smallest n with div1=k1*n >= MS_DIV_MIN
  uint64_t n2 = (MS_DIV_MIN + k2 - 1) / k2;    // same for div2
  uint64_t feasibleLo = nLo;
  if (n1 > feasibleLo) feasibleLo = n1;
  if (n2 > feasibleLo) feasibleLo = n2;

  if (feasibleLo > nHi) return false;  // no n satisfies every constraint at once

  uint64_t highestFvco = lcmF * nHi;

  uint32_t glcm = u32gcd((uint32_t)lcmF, fxtal);  // lcmF <= VCO_MAX, fits uint32_t safely
  uint32_t step = fxtal / glcm;
  uint64_t candidateN = (nHi / step) * step;

  outFvco = (uint32_t)((candidateN >= feasibleLo) ? (lcmF * candidateN) : highestFvco);
  return true;
}

static void packMultisynthRegs(const An619Ratio& r, uint8_t* out) {
  out[0] = (uint8_t)((r.p3 >> 8) & 0xFF);
  out[1] = (uint8_t)(r.p3 & 0xFF);
  out[2] = (uint8_t)((r.p1 >> 16) & 0x03);
  out[3] = (uint8_t)((r.p1 >> 8) & 0xFF);
  out[4] = (uint8_t)(r.p1 & 0xFF);
  out[5] = (uint8_t)(((r.p3 >> 12) & 0xF0) | ((r.p2 >> 16) & 0x0F));
  out[6] = (uint8_t)((r.p2 >> 8) & 0xFF);
  out[7] = (uint8_t)(r.p2 & 0xFF);
}

static uint8_t driveBits(uint8_t driveMa) {
  switch (driveMa) {
    case 2:  return CLK_IDRV_2MA;
    case 4:  return CLK_IDRV_4MA;
    case 6:  return CLK_IDRV_6MA;
    default: return CLK_IDRV_8MA;  // 8 mA, also the fallback for any bad value
  }
}

// ---------------------------------------------------------------------
// computeClockPlan() -- pure computation, no hardware I/O. This is the
// ONE place that decides which PLL(s) get used, what VCO frequencies
// they land on, and what every clock's ratio and achieved frequency
// will be. applyConfiguration() writes this plan to hardware;
// si5351PreviewClockFreq() just reads it -- so a preview can never show
// something different from what actually gets programmed.
// ---------------------------------------------------------------------
struct ClockComputed {
  bool       enabled;
  uint32_t   requestedFreqHz;
  An619Ratio msRatio;         // output multisynth ratio, register-packed
  bool       msIsInteger;
  bool       dividerValid;     // false if msRatio violates AN619's Multisynth
                                // constraint (a must be exactly 4, 6, or >=8);
                                // see isValidMultisynthRatio() above
  double     achievedHz;
  double     fractionalError;  // 0 if disabled
};

struct ClockPlan {
  bool          usePllA, usePllB;
  uint32_t      fvcoA, fvcoB;
  An619Ratio    pllRatioA, pllRatioB;
  bool          pllAIsInteger, pllBIsInteger;  // true iff that PLL's feedback ratio
                                                // reduced to a pure integer (b=0) --
                                                // FBA_INT/FBB_INT-eligible, see above
  ClockComputed clk[3];
};

static ClockPlan computeClockPlan(const ClockState (&clocks)[3]) {
  ClockPlan plan;
  memset(&plan, 0, sizeof(plan));

  plan.usePllA = clocks[0].enabled;
  plan.usePllB = clocks[1].enabled || clocks[2].enabled;

  Ratio pllA_ab{}, pllB_ab{};

  if (plan.usePllA) {
    PllSolution sol = findPllParams(clocks[0].freqHz, SI5351_REF_FREQ_HZ);
    plan.fvcoA = sol.fvco;
    pllA_ab = sol.ratio;
    packRatio(pllA_ab, plan.pllRatioA);
    plan.pllAIsInteger = (pllA_ab.b == 0);
  }

  if (plan.usePllB) {
    bool gotShared = false;
    if (clocks[1].enabled && clocks[2].enabled) {
      gotShared = findSharedPllbVco(clocks[1].freqHz, clocks[2].freqHz, SI5351_REF_FREQ_HZ, plan.fvcoB);
    }
    if (!gotShared) {
      uint32_t soloFreq = clocks[1].enabled ? clocks[1].freqHz : clocks[2].freqHz;
      PllSolution sol = findPllParams(soloFreq, SI5351_REF_FREQ_HZ);
      plan.fvcoB = sol.fvco;
    }
    pllB_ab = fractionalRatio(plan.fvcoB, SI5351_REF_FREQ_HZ);
    packRatio(pllB_ab, plan.pllRatioB);
    plan.pllBIsInteger = (pllB_ab.b == 0);
  }

  for (uint8_t c = 0; c < 3; c++) {
    plan.clk[c].enabled = clocks[c].enabled;
    plan.clk[c].requestedFreqHz = clocks[c].freqHz;
    if (!clocks[c].enabled) continue;

    bool clkUsesPllB = (c != 0);
    uint32_t fvco = clkUsesPllB ? plan.fvcoB : plan.fvcoA;
    const Ratio& pllAB = clkUsesPllB ? pllB_ab : pllA_ab;

    Ratio msAB = fractionalRatio(fvco, clocks[c].freqHz);
    packRatio(msAB, plan.clk[c].msRatio);
    plan.clk[c].msIsInteger = (msAB.b == 0);
    plan.clk[c].dividerValid = isValidMultisynthRatio(msAB);

    double achievedFvco = (double)SI5351_REF_FREQ_HZ *
                           ((double)pllAB.a + (double)pllAB.b / (double)pllAB.c);
    double achievedHz = achievedFvco / ((double)msAB.a + (double)msAB.b / (double)msAB.c);
    plan.clk[c].achievedHz = achievedHz;
    plan.clk[c].fractionalError =
        (achievedHz - (double)clocks[c].freqHz) / (double)clocks[c].freqHz;
  }

  return plan;
}

static void printFractionalError(double err) {
  CMD_SERIAL.print(F(" ("));
  CMD_SERIAL.print(err, 13);
  CMD_SERIAL.print(F(", "));
  CMD_SERIAL.print(err * 1.0e9, 3);
  CMD_SERIAL.print(F(" ppb)"));
  if (err > FREQ_ERROR_WARN_THRESHOLD || err < -FREQ_ERROR_WARN_THRESHOLD) {
    CMD_SERIAL.print(F("  *** exceeds "));
    CMD_SERIAL.print(FREQ_ERROR_WARN_THRESHOLD, 3);
    CMD_SERIAL.print(F(" warn threshold"));
  }
}

// ---------------------------------------------------------------------
// Top-level configuration -- reprograms all three outputs from s_clock[]
// and s_driveMa. Same "disable everything, recompute everything, bring
// it back up" structure as the original Python configure_si5351(), now
// with the computation itself factored out into computeClockPlan().
// ---------------------------------------------------------------------
static bool applyConfiguration() {
  bool allOk = waitForInit();
  if (!allOk) {
    CMD_SERIAL.println(F("  WARNING: SI5351 SYS_INIT did not clear -- device may not be ready"));
  }

  allOk &= writeReg(SI5351_REG_OUTPUT_ENABLE, 0xFF);
  for (uint8_t c = 0; c < 3; c++) allOk &= writeReg(SI5351_REG_CLK_BASE + c, CLK_PDN);
  allOk &= writeReg(SI5351_REG_XTAL_LOAD, (uint8_t)((XTAL_CL_0PF << 6) | 0x12));

  ClockPlan plan = computeClockPlan(s_clock);

  if (plan.usePllA) {
    uint8_t regs[8];
    packMultisynthRegs(plan.pllRatioA, regs);
    allOk &= writeRegs(SI5351_REG_MSNA_BASE, regs, 8);
    allOk &= setRegBit(SI5351_REG_FBA_INT, FBX_INT_BIT, plan.pllAIsInteger);
    CMD_SERIAL.print(F("  PLLA: VCO = "));
    CMD_SERIAL.print(plan.fvcoA / 1.0e6, 3);
    CMD_SERIAL.print(plan.pllAIsInteger ? F(" MHz (integer feedback, FBA_INT set)") : F(" MHz"));
    CMD_SERIAL.println();
  }

  if (plan.usePllB) {
    uint8_t regs[8];
    packMultisynthRegs(plan.pllRatioB, regs);
    allOk &= writeRegs(SI5351_REG_MSNB_BASE, regs, 8);
    allOk &= setRegBit(SI5351_REG_FBB_INT, FBX_INT_BIT, plan.pllBIsInteger);
    CMD_SERIAL.print(F("  PLLB: VCO = "));
    CMD_SERIAL.print(plan.fvcoB / 1.0e6, 3);
    CMD_SERIAL.print(plan.pllBIsInteger ? F(" MHz (integer feedback, FBB_INT set)") : F(" MHz"));
    CMD_SERIAL.println();
  }

  uint8_t outputEnableMask = 0xFF;
  for (uint8_t c = 0; c < 3; c++) {
    if (!plan.clk[c].enabled) {
      allOk &= writeReg(SI5351_REG_CLK_BASE + c, CLK_PDN);
      CMD_SERIAL.print(F("  CLK")); CMD_SERIAL.print(c);
      CMD_SERIAL.println(F(": powered down"));
      continue;
    }

    uint8_t regs[8];
    packMultisynthRegs(plan.clk[c].msRatio, regs);
    allOk &= writeRegs(SI5351_REG_MS0_BASE + c * 8, regs, 8);

    bool clkUsesPllB = (c != 0);
    uint8_t clkCtrl = (uint8_t)(CLK_SRC_MSX |
                                 (clkUsesPllB ? CLK_PLL_SRC : 0) |
                                 (plan.clk[c].msIsInteger ? CLK_INT_MODE : 0) |
                                 driveBits(s_driveMa));
    allOk &= writeReg(SI5351_REG_CLK_BASE + c, clkCtrl);
    outputEnableMask &= (uint8_t)~(1 << c);

    CMD_SERIAL.print(F("  CLK")); CMD_SERIAL.print(c);
    CMD_SERIAL.print(F(": requested ")); CMD_SERIAL.print(plan.clk[c].requestedFreqHz);
    CMD_SERIAL.print(F(" Hz, actual ")); CMD_SERIAL.print(plan.clk[c].achievedHz, 6);
    CMD_SERIAL.print(F(" Hz, fractional error"));
    printFractionalError(plan.clk[c].fractionalError);
    CMD_SERIAL.print(F(", "));
    CMD_SERIAL.print(clkUsesPllB ? F("PLLB") : F("PLLA"));
    CMD_SERIAL.print(F(", "));
    CMD_SERIAL.println(plan.clk[c].msIsInteger ? F("integer mode") : F("fractional mode"));
    if (!plan.clk[c].dividerValid) {
      CMD_SERIAL.print(F("  *** WARNING: CLK")); CMD_SERIAL.print(c);
      CMD_SERIAL.println(F(" divide ratio is not a value the Si5351 actually supports"));
      CMD_SERIAL.println(F("  *** (AN619 requires exactly 4, 6, or >=8) -- this usually means it's"));
      CMD_SERIAL.println(F("  *** sharing PLLB with another active clock at an incompatible"));
      CMD_SERIAL.println(F("  *** frequency above 112.5 MHz. Actual hardware output is undefined;"));
      CMD_SERIAL.println(F("  *** reconfigure via the menu."));
    }
  }

  allOk &= writeReg(SI5351_REG_PLL_RESET, 0xAC);
  allOk &= writeReg(SI5351_REG_OUTPUT_ENABLE, outputEnableMask);
  return allOk;
}

// ---------------------------------------------------------------------
// Persistence -- saves the current tracked state as the new "last known
// good" configuration. Called from every setter that successfully
// changes something, not from si5351Init() itself (booting from a saved
// or default config isn't itself a change worth re-saving).
// ---------------------------------------------------------------------
static void persistCurrentConfig() {
  StoredSi5351Config cfg;
  cfg.driveMa = s_driveMa;
  for (uint8_t c = 0; c < 3; c++) {
    cfg.clkEnabled[c] = s_clock[c].enabled;
    cfg.clkFreqHz[c]  = s_clock[c].freqHz;
  }
  nvStoreSave(cfg);
}

static void applyDefaults() {
  s_clock[0] = { (bool)SI5351_DEFAULT_CLK0_ENABLED, SI5351_DEFAULT_FREQ_HZ };
  s_clock[1] = { (bool)SI5351_DEFAULT_CLK1_ENABLED, SI5351_DEFAULT_FREQ_HZ };
  s_clock[2] = { (bool)SI5351_DEFAULT_CLK2_ENABLED, SI5351_DEFAULT_FREQ_HZ };
  s_driveMa  = SI5351_DEFAULT_DRIVE_MA;
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------
bool si5351Init() {
  si5351WireBegin();
  nvStoreInit();

  StoredSi5351Config stored;
  if (nvStoreLoad(stored)) {
    CMD_SERIAL.println(F("  Loaded saved configuration from flash."));
    s_driveMa = stored.driveMa;
    for (uint8_t c = 0; c < 3; c++) {
      s_clock[c].enabled = stored.clkEnabled[c];
      s_clock[c].freqHz  = stored.clkFreqHz[c];
    }
  } else {
    CMD_SERIAL.println(F("  No saved configuration found -- using compile-time defaults."));
    applyDefaults();
  }

  CMD_SERIAL.println(F("\nSi5351A Configuration"));
  CMD_SERIAL.print(F("  I2C address:    0x"));
  CMD_SERIAL.println(SI5351_I2C_ADDR, HEX);
  CMD_SERIAL.print(F("  Reference:      "));
  CMD_SERIAL.print(SI5351_REF_FREQ_HZ / 1.0e6, 6);
  CMD_SERIAL.println(F(" MHz on XA (0 pF load cap)"));
  CMD_SERIAL.print(F("  Drive strength: "));
  CMD_SERIAL.print(s_driveMa);
  CMD_SERIAL.println(F(" mA"));

  bool ok = applyConfiguration();
  CMD_SERIAL.println(ok ? F("Done.\n") : F("FAILED -- one or more I2C writes to the Si5351 did not succeed.\n"));
  return ok;
}

bool si5351SetClockFreq(uint8_t clk, uint32_t freqHz) {
  if (clk > 2) return false;
  if (freqHz < FOUT_MIN || freqHz > FOUT_MAX) return false;
  s_clock[clk].enabled = true;
  s_clock[clk].freqHz  = freqHz;
  applyConfiguration();
  persistCurrentConfig();
  return true;
}

bool si5351DisableClock(uint8_t clk) {
  if (clk > 2) return false;
  s_clock[clk].enabled = false;
  applyConfiguration();
  persistCurrentConfig();
  return true;
}

bool si5351SetDrive(uint8_t driveMa) {
  if (driveMa != 2 && driveMa != 4 && driveMa != 6 && driveMa != 8) return false;
  s_driveMa = driveMa;
  applyConfiguration();
  persistCurrentConfig();
  return true;
}

void si5351ResetToDefaults() {
  applyDefaults();
  applyConfiguration();
  persistCurrentConfig();
}

uint32_t si5351GetClockFreq(uint8_t clk) {
  return (clk <= 2) ? s_clock[clk].freqHz : 0;
}

bool si5351IsClockEnabled(uint8_t clk) {
  return (clk <= 2) ? s_clock[clk].enabled : false;
}

uint8_t si5351GetDrive() {
  return s_driveMa;
}

ClockFreqPreview si5351PreviewClockFreq(uint8_t clk, uint32_t freqHz) {
  ClockFreqPreview pv;
  memset(&pv, 0, sizeof(pv));
  pv.clk = clk;
  pv.requestedHz = freqHz;

  if (clk > 2 || freqHz < FOUT_MIN || freqHz > FOUT_MAX) {
    pv.valid = false;
    return pv;
  }

  ClockState whatIf[3];
  memcpy(whatIf, s_clock, sizeof(whatIf));
  whatIf[clk].enabled = true;
  whatIf[clk].freqHz  = freqHz;

  ClockPlan plan = computeClockPlan(whatIf);

  pv.valid           = true;
  pv.achievedHz       = plan.clk[clk].achievedHz;
  pv.fractionalError  = plan.clk[clk].fractionalError;
  pv.dividerValid     = plan.clk[clk].dividerValid;

  if (clk != 0) {
    uint8_t other = (clk == 1) ? 2 : 1;
    if (whatIf[other].enabled) {
      pv.otherClkAffected     = true;
      pv.otherClk             = other;
      pv.otherRequestedHz     = whatIf[other].freqHz;
      pv.otherAchievedHz      = plan.clk[other].achievedHz;
      pv.otherFractionalError = plan.clk[other].fractionalError;
      pv.otherDividerValid    = plan.clk[other].dividerValid;
    }
  }

  return pv;
}
