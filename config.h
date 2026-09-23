#pragma once
//
// config.h — compile-time configuration for the MVR GPSDO B firmware.
//
// This is the single place to edit board pins, GNSS behavior, Si5351
// defaults, and UX timing.  Everything here is a #define so the values are
// baked in at compile time -- there is no runtime UBX reconfiguration of
// the GNSS module in this sketch (only the Si5351 and the NMEA passthrough
// on/off state get runtime menu control; see menu.h).
//

// ---------------------------------------------------------------------
// Firmware version
// ---------------------------------------------------------------------
// Format: YYYYMMDD.N -- date of the last meaningful change, and a
// sequence number starting at 1 for that date (bump N again for a
// further revision released the same day; reset to 1 on the next
// date with a change). Bumped by hand -- there's no automatic tracking.
// Printed alongside the compiler's __DATE__/__TIME__ (see
// statusPrintVersion() in status.cpp), which always reflects exactly
// when a given board's firmware was actually built, version bump or
// not -- useful for telling apart boards flashed at different points
// during development even between version bumps.
//
// 20260917.1: replaced the Si5351 PLL feedback divider's fixed-
// denominator rounding fallback with a continued-fraction best-
// rational-approximation search (si5351.cpp, fractionalRatio()), and
// added a preview-and-confirm step to the menu's frequency-entry flow
// (menu.cpp) -- see FREQ_ERROR_WARN_THRESHOLD below and si5351.h's
// si5351PreviewClockFreq(). Bench-validated; see project notes for the
// full analysis and test results.
//
// 20260922.1: bench test program complete (10 runs, TimePod/53100A vs
// ULN and HP 5071A cesium references). Findings folded in: (1) the
// continued-fraction algorithm's worst-case bound corrected to ~7.6e-9
// (an earlier analysis pass had a units error inflating this to
// 4.5e-7 -- caught and fixed before this bench validation); (2) proven
// exact-multiple-of-10-Hz guarantee added to si5351.h and surfaced in
// the menu as user guidance; (3) tested a denominator-maximization idea
// for spur mitigation and found no practical benefit, in this
// implementation -- small (2-5 dB) differences observed at a few
// offsets showed no consistent direction and stayed >100 dB below
// carrier, so production intentionally does not rescale denominators.
// Full report available in project notes.
//
// 20260922.2: (1) fixed a real bug found while widening the report's
// verified-frequency range past 30 MHz: above 112.5 MHz (900 MHz VCO /
// 8), AN619 restricts the Si5351's output Multisynth divider to exactly
// 4, 6, or 8, not a free integer search -- the previous find_pll_params()
// logic didn't know this and could silently select an invalid divider
// (confirmed to affect ~34% of the 112.5-200 MHz range, reachable via
// the existing FOUT_MAX=200MHz menu range). findPllParams() now handles
// this correctly as an explicit case, every computed output-stage ratio
// is validated against the full AN619 rule (isValidMultisynthRatio()),
// and the menu hard-rejects (not just warns on) any frequency that would
// produce an invalid divider -- most likely a PLLB-sharing mismatch
// between CLK1/CLK2 when one needs >112.5 MHz. See si5351.h and
// ClockFreqPreview::dividerValid. (2) renamed all "A2" hardware-revision
// references to "B", reflecting the production board revision.
//
// 20260922.3: added a runtime-toggleable raw NMEA passthrough (menu
// option), replacing the old compile-time NMEA_PASSTHROUGH define. While
// enabled and the menu is closed, CMD_SERIAL carries a clean stream of
// exactly what the GNSS module sends on UART -- no periodic status line,
// no lock/GPS transition events -- so it can be piped to an NMEA
// consumer. Pressing any key pauses the stream and opens the menu, same
// wake gesture as always; passthrough resumes automatically on menu exit
// or timeout unless turned off from the menu. Session-only by design --
// does not persist across a reboot. See gnss.h/.cpp (gnssSetPassthrough(),
// gnssIsPassthroughEnabled()), status.cpp (printEvents() suppression),
// and menu.cpp (new toggle option).
#define FIRMWARE_VERSION "20260922.3"

// ---------------------------------------------------------------------
// Board / pin assignments
// ---------------------------------------------------------------------
// XIAO D0-D10 numbering, consistent across the SAMD21, ESP32C3, and
// RP2040 module variants of the MVR B board (confirmed identical
// physical pin mapping for UART and I2C across all three).
//
//   D6 (TX) -> GPS02-UBX pin 12 (RXD)
//   D7 (RX) <- GPS02-UBX pin 11 (TXD)
//   D4/D5   -> Si5351A I2C (SDA/SCL), via Wire
//   D1      -> raw 4046 PC2 tap (via 470R series stub), loop-lock monitor
//
#define GNSS_SERIAL        Serial1     // GPS02-UBX UART
#define GNSS_BAUD          38400       // GPS02-UBX factory default
#define PLL_LOCK_PIN       D1          // hardware lock-detector output (RC filter + Schmitt-trigger
                                        // inverter on the raw 4046 PCP signal, with feedback hysteresis)
                                        // -- an already-clean, debounced binary level, NOT the raw
                                        // phase-detector signal. See status.cpp.

#define CMD_SERIAL         Serial      // USB CDC: status output + command menu
#define CMD_SERIAL_BAUD    115200      // cosmetic on native-USB boards, but conventional

// ---------------------------------------------------------------------
// GNSS timepulse configuration (compile-time only; no runtime changes)
// ---------------------------------------------------------------------
// Ported from mvr_gnss_config.py.  Note: freqPeriod (the *unlocked*
// timepulse frequency field) must be a valid nonzero value or the M10
// ignores it; the actual suppression while unlocked comes entirely from
// driving pulseLenRatio (duty) to 0%, confirmed working on M10 hardware.
// The value below doesn't matter beyond "nonzero" -- 1 Hz is what the
// original script used and it's been bench-validated, so leave it alone
// unless you have a specific reason to change it.
#define GNSS_TP_FREQ_UNLOCKED_HZ   1UL
#define GNSS_TP_FREQ_LOCKED_HZ     25000UL   // matches DIV_400 comparison frequency
#define GNSS_TP_DUTY_UNLOCKED_PCT  0.0f      // 0% = output suppressed while unlocked
#define GNSS_TP_DUTY_LOCKED_PCT    50.0f

// C/N0 input filter -- fast unlock declaration on antenna removal/loss
#define GNSS_CNO_THRESHOLD_DBHZ    20        // dBHz
#define GNSS_CNO_MIN_SATS          4         // SVs required above threshold

// The NiceRF GPS02-UBX module doesn't implement battery-backed BBR, so a
// save-to-flash attempt here just costs time (a full write/verify cycle)
// and clutters the console with a result that will always be the same.
// Left at 0 by default; flip to 1 if a module that does support BBR/flash
// ever gets used here. Code path is unchanged either way -- see Step 4 in
// configureGnssOnce().
#define GNSS_SAVE_TO_FLASH         0

#define GNSS_CONFIG_MAX_RETRIES    5
#define GNSS_CONFIG_RETRY_DELAY_MS 5000UL
#define GNSS_ACK_TIMEOUT_MS        2000UL
#define GNSS_BOOT_SETTLE_MS        5000UL   // time to let the module's own boot-up (power rail settle,
                                             // internal init, boot-time NMEA chatter) finish before we
                                             // start sending it commands -- bumped up from 300ms after
                                             // real hardware showed non-responses at the shorter delay

// Raw NMEA passthrough (streaming everything the GNSS module sends on
// UART out to CMD_SERIAL) is now a runtime menu toggle rather than a
// compile-time define -- see gnssSetPassthrough()/gnssIsPassthroughEnabled()
// in gnss.h and the menu option in menu.cpp. Off by default at every boot;
// there is no #define here to flip anymore. Internal fix/sat-count parsing
// (gnssGetStatus()) keeps running regardless of whether passthrough is on.

// ---------------------------------------------------------------------
// Si5351A synthesizer configuration
// ---------------------------------------------------------------------
#define SI5351_I2C_ADDR         0x60
#define SI5351_I2C_BUS_SDA      D4
#define SI5351_I2C_BUS_SCL      D5
#define SI5351_REF_FREQ_HZ      10000000UL   // TCXO/VCXO driving XA
#define SI5351_DEFAULT_FREQ_HZ  27000000UL   // CLK0's default frequency, and the placeholder value
                                              // stored for CLK1/CLK2 while off by default -- inert
                                              // until enabled via the menu, which always sets a
                                              // fresh frequency at the same time
#define SI5351_DEFAULT_CLK0_ENABLED 1        // primary output, on by default
#define SI5351_DEFAULT_CLK1_ENABLED 0        // off by default -- reduces radiated emissions from an
#define SI5351_DEFAULT_CLK2_ENABLED 0        // output most users won't be using
#define SI5351_DEFAULT_DRIVE_MA 8            // boot default drive strength

// Fractional-N accuracy warning threshold, as a fractional frequency
// error (dimensionless, e.g. 1e-11 = 10 ppt). Derived from an exhaustive
// bench-validated analysis of the continued-fraction algorithm's actual
// achievable error across every 1 Hz-resolution frequency from 500 kHz
// to 30 MHz (see project notes): 99.97% of all such frequencies land
// below 1e-11 with the continued-fraction algorithm, so a value at this
// threshold flags only the rare, specific frequencies that hit the
// Si5351's fundamental 20-bit register resolution limit -- these are
// isolated single-Hz points, not bands, and shifting the target by even
// 1 Hz normally clears the warning entirely. The menu shows this
// preview-and-confirm before committing any new CLK frequency; it does
// not block the choice, since the operator may have a specific reason
// to accept it anyway. NOTE: any frequency that is an exact multiple of
// 10 Hz is PROVEN to always synthesize exactly (see si5351.h) -- the
// menu suggests this as a remedy whenever the warning fires.
#define FREQ_ERROR_WARN_THRESHOLD  1.0e-11

// ---------------------------------------------------------------------
// Loop-lock monitor (PLL_LOCK signal on PLL_LOCK_PIN)
// ---------------------------------------------------------------------
// PLL_LOCK is the output of an analog lock-detector circuit (RC filter +
// Schmitt-trigger inverter with feedback hysteresis on the raw 4046 PCP
// signal), confirmed HIGH when locked against the hardware PLL_LOCK LED.
//
// Bench data (2026-09) showed sustained transition rates of ~150-260/sec
// on PLL_LOCK even during confirmed, steady-state lock -- real electrical
// activity (almost certainly small ongoing residual corrections, normal
// for any real loop), too brief for an LED plus the eye to perceive but
// well within what fast digital polling catches. A simple "wait for N ms
// of total silence" debounce doesn't tolerate that; a leaky integrator
// (smoothing the same way the analog RC stage does) does. See status.cpp
// for the full design history, including the two earlier (wrong) designs
// this replaced.
//
// The three values below have since been validated against real hardware
// in both directions -- acquisition and loss of lock, via antenna
// disconnect/reconnect -- across two independent boards, with clean,
// monotonic convergence and no flapping near either threshold. Not
// bench-calibrated with instruments, just confirmed behaviorally; revisit
// if a build ever shows different flutter characteristics.
#define LOCK_FILTER_TIME_CONSTANT_MS 500UL   // leaky-integrator time constant for the "% time spent
                                              // HIGH recently" estimate
#define LOCK_HIGH_FRACTION_PCT       95.0f   // declare LOCKED once the filtered estimate reaches this
#define LOCK_LOW_FRACTION_PCT        80.0f   // declare unlocked once it drops back below this --
                                              // gap between the two is deliberate hysteresis

// Uncomment to include raw level / high-fraction / transition-count
// detail on the periodic status line's Loop field (see printStatusLine()
// in status.cpp) -- this is what made the three redesigns above possible
// to diagnose; worth turning back on if the loop or hardware ever changes
// enough to need retuning the thresholds again.
// #define LOCK_STATUS_VERBOSE     1

// ---------------------------------------------------------------------
// UX timing
// ---------------------------------------------------------------------
#define STATUS_INTERVAL_MS      30000UL  // periodic status line, when not in the menu
#define MENU_IDLE_TIMEOUT_MS    30000UL  // fall back to run mode after this much menu inactivity
#define STATUS_STALE_MS         5000UL   // GNSS/lock data older than this is treated as "no data"

// ---------------------------------------------------------------------
// Status LED (onboard addressable RGB, where the board has one -- e.g.
// XIAO RP2040's NeoPixel. No-op on boards without one; see status.cpp)
// ---------------------------------------------------------------------
#define STATUS_LED_BRIGHTNESS   40       // 0-255; it's a bright LED, keep this modest
