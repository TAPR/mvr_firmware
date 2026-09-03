//
// si5351.cpp — see si5351.h for the public API and porting notes.
//
// Layout mirrors mvr_synth_config.py where practical:
//   - register/bit constants
//   - low-level I2C helpers (Wire-based, replaces the Linux i2c-dev layer)
//   - AN619 rational-approximation math (find_multisynth_params /
//     find_pll_params / the shared-VCO search for CLK1+CLK2)
//   - register packing
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

// Computes AN619 P1/P2/P3 for the ratio num/den -- used for both the PLL
// feedback multisynth ("a+b/c = fvco/fxtal") and the output multisynth
// ("a+b/c = fvco/fout"). Same rational-approximation math either way, so
// one function covers both (the Python original duplicated this logic
// across three call sites; consolidated here).
static An619Ratio an619Ratio(uint32_t num, uint32_t den) {
  uint32_t a, b, c;
  uint32_t g = u32gcd(num, den);
  uint32_t rn = num / g, rd = den / g;

  if (rd <= 1048575UL) {
    a = rn / rd;
    b = rn % rd;
    c = rd;
  } else {
    a = num / den;
    uint32_t remainder = num - a * den;
    double bf = (double)remainder * 1048575.0 / (double)den;
    b = (uint32_t)llround(bf);
    c = 1048575UL;
    uint32_t g2 = u32gcd(b, c);
    if (g2 > 0) { b /= g2; c /= g2; }
  }

  An619Ratio r;
  r.p1 = 128UL * a + (128UL * b) / c - 512UL;
  r.p2 = 128UL * b - c * ((128UL * b) / c);
  r.p3 = c;
  return r;
}

static An619Ratio findMultisynthParams(uint32_t fout, uint32_t fvco) {
  return an619Ratio(fvco, fout);
}

struct PllSolution { An619Ratio ratio; uint32_t fvco; };

// Picks the lowest VCO frequency in [VCO_MIN, VCO_MAX] giving an integer
// output divide, falling back to VCO_MIN scaled up if none exists.
static PllSolution findPllParams(uint32_t fout, uint32_t fxtal) {
  uint32_t bestFvco = 0;
  uint32_t divLo = (VCO_MIN + fout - 1) / fout;  // ceil
  uint32_t divHi = VCO_MAX / fout;               // floor

  for (uint32_t div = divLo; div <= divHi; div++) {
    if (div >= MS_DIV_MIN) {
      uint32_t fvco = fout * div;
      if (fvco >= VCO_MIN && fvco <= VCO_MAX) { bestFvco = fvco; break; }
    }
  }
  if (bestFvco == 0) {
    bestFvco = fout * divLo;
    if (bestFvco < VCO_MIN) bestFvco = VCO_MIN;
  }

  PllSolution sol;
  sol.fvco = bestFvco;
  sol.ratio = an619Ratio(bestFvco, fxtal);
  return sol;
}

// Searches for a single VCO frequency that gives an integer divide for
// BOTH f1 and f2 (CLK1 and CLK2 sharing PLLB). Uses uint64_t throughout --
// f1*f2 can reach ~4x10^16, well past uint32_t range, before the LCM
// reduction brings it back down.
static bool findSharedPllbVco(uint32_t f1, uint32_t f2, uint32_t& outFvco) {
  uint32_t g = u32gcd(f1, f2);
  uint64_t lcmF = (uint64_t)f1 * (uint64_t)f2 / g;

  if (lcmF > 0 && lcmF <= VCO_MAX) {
    uint64_t nLo = (VCO_MIN + lcmF - 1) / lcmF;  // ceil
    uint64_t nHi = VCO_MAX / lcmF;               // floor
    for (uint64_t n = nLo; n <= nHi; n++) {
      uint64_t fvcoCand = lcmF * n;
      uint32_t div1 = (uint32_t)(fvcoCand / f1);
      uint32_t div2 = (uint32_t)(fvcoCand / f2);
      if (fvcoCand == (uint64_t)div1 * f1 && fvcoCand == (uint64_t)div2 * f2 &&
          div1 >= MS_DIV_MIN && div2 >= MS_DIV_MIN) {
        outFvco = (uint32_t)fvcoCand;
        return true;
      }
    }
  }
  return false;
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
// Top-level configuration -- reprograms all three outputs from s_clock[]
// and s_driveMa. Same "disable everything, recompute everything, bring
// it back up" structure as the original Python configure_si5351().
// ---------------------------------------------------------------------
static bool applyConfiguration() {
  bool allOk = waitForInit();
  if (!allOk) {
    CMD_SERIAL.println(F("  WARNING: SI5351 SYS_INIT did not clear -- device may not be ready"));
  }

  allOk &= writeReg(SI5351_REG_OUTPUT_ENABLE, 0xFF);
  for (uint8_t c = 0; c < 3; c++) allOk &= writeReg(SI5351_REG_CLK_BASE + c, CLK_PDN);
  allOk &= writeReg(SI5351_REG_XTAL_LOAD, (uint8_t)((XTAL_CL_0PF << 6) | 0x12));

  bool usePllA = s_clock[0].enabled;
  bool usePllB = s_clock[1].enabled || s_clock[2].enabled;
  uint32_t fvcoA = 0, fvcoB = 0;

  if (usePllA) {
    PllSolution sol = findPllParams(s_clock[0].freqHz, SI5351_REF_FREQ_HZ);
    fvcoA = sol.fvco;
    uint8_t regs[8];
    packMultisynthRegs(sol.ratio, regs);
    allOk &= writeRegs(SI5351_REG_MSNA_BASE, regs, 8);
    CMD_SERIAL.print(F("  PLLA: VCO = "));
    CMD_SERIAL.print(fvcoA / 1.0e6, 3);
    CMD_SERIAL.println(F(" MHz"));
  }

  if (usePllB) {
    bool gotShared = false;
    if (s_clock[1].enabled && s_clock[2].enabled) {
      gotShared = findSharedPllbVco(s_clock[1].freqHz, s_clock[2].freqHz, fvcoB);
      if (!gotShared) {
        CMD_SERIAL.println(F("  WARNING: no shared VCO for CLK1+CLK2 -- using CLK1's VCO, CLK2 may not land exactly"));
      }
    }
    if (!gotShared) {
      uint32_t soloFreq = s_clock[1].enabled ? s_clock[1].freqHz : s_clock[2].freqHz;
      PllSolution sol = findPllParams(soloFreq, SI5351_REF_FREQ_HZ);
      fvcoB = sol.fvco;
    }
    An619Ratio ratioB = an619Ratio(fvcoB, SI5351_REF_FREQ_HZ);
    uint8_t regs[8];
    packMultisynthRegs(ratioB, regs);
    allOk &= writeRegs(SI5351_REG_MSNB_BASE, regs, 8);
    CMD_SERIAL.print(F("  PLLB: VCO = "));
    CMD_SERIAL.print(fvcoB / 1.0e6, 3);
    CMD_SERIAL.println(F(" MHz"));
  }

  uint8_t outputEnableMask = 0xFF;
  for (uint8_t c = 0; c < 3; c++) {
    if (!s_clock[c].enabled) {
      allOk &= writeReg(SI5351_REG_CLK_BASE + c, CLK_PDN);
      CMD_SERIAL.print(F("  CLK")); CMD_SERIAL.print(c);
      CMD_SERIAL.println(F(": powered down"));
      continue;
    }

    bool clkUsesPllB = (c != 0);   // CLK0->PLLA, CLK1/CLK2->PLLB, fixed assignment
    uint32_t fvco = clkUsesPllB ? fvcoB : fvcoA;
    An619Ratio msRatio = findMultisynthParams(s_clock[c].freqHz, fvco);
    uint8_t regs[8];
    packMultisynthRegs(msRatio, regs);
    allOk &= writeRegs(SI5351_REG_MS0_BASE + c * 8, regs, 8);

    uint32_t div = fvco / s_clock[c].freqHz;
    bool isInteger = (fvco == div * s_clock[c].freqHz) && (div % 2 == 0);
    uint8_t clkCtrl = (uint8_t)(CLK_SRC_MSX |
                                 (clkUsesPllB ? CLK_PLL_SRC : 0) |
                                 (isInteger ? CLK_INT_MODE : 0) |
                                 driveBits(s_driveMa));
    allOk &= writeReg(SI5351_REG_CLK_BASE + c, clkCtrl);
    outputEnableMask &= (uint8_t)~(1 << c);

    // Recover actual output frequency from the registers we just wrote,
    // same inverse-AN619 check the original script printed.
    double actual = (double)fvco * 128.0 * msRatio.p3 /
                     ((double)(msRatio.p1 + 512) * msRatio.p3 + msRatio.p2);
    double errorHz = actual - (double)s_clock[c].freqHz;
    CMD_SERIAL.print(F("  CLK")); CMD_SERIAL.print(c);
    CMD_SERIAL.print(F(": requested ")); CMD_SERIAL.print(s_clock[c].freqHz);
    CMD_SERIAL.print(F(" Hz, actual ")); CMD_SERIAL.print(actual, 3);
    CMD_SERIAL.print(F(" Hz (error ")); CMD_SERIAL.print(errorHz, 3);
    CMD_SERIAL.print(F(" Hz), "));
    CMD_SERIAL.print(clkUsesPllB ? F("PLLB") : F("PLLA"));
    CMD_SERIAL.print(F(", "));
    CMD_SERIAL.println(isInteger ? F("integer mode") : F("fractional mode"));
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
