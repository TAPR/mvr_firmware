//
// status.cpp — see status.h for the public API and why this merges what
// used to be three files.
//
#include <Arduino.h>
#include <stdio.h>
#include "config.h"
#include "status.h"
#include "gnss.h"
#include "si5351.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <Adafruit_NeoPixel.h>
#endif

// -----------------------------------------------------------------------
// PLL lock detection (formerly lock_monitor.h/.cpp)
// -----------------------------------------------------------------------
// Design history, briefly, because it took three tries to get right:
//   1. Originally assumed PLL_LOCK_PIN carried the raw, unfiltered 4046
//      PCP output and tried to determine lock via duty-cycle math in
//      firmware. Wrong -- PCP and PC2 are different pins, and this pin
//      isn't raw PCP at all.
//   2. Corrected to: this pin is PLL_LOCK, the output of an analog
//      lock-detector circuit (RC filter + Schmitt-trigger inverter with
//      feedback hysteresis) that already cleans up PCP's raw behavior.
//      Assumed a simple "wait for N ms of silence" debounce would be
//      enough.
//   3. Bench data showed that was also incomplete: even during
//      confirmed, steady-state lock, PLL_LOCK sustains roughly
//      150-260 transitions/sec -- real residual correction activity,
//      invisible to an LED and the eye, but well within what fast
//      digital polling catches. Switched to a leaky integrator
//      (exponential moving average of "% time spent HIGH"), the same
//      smoothing intent as the analog RC stage, with two thresholds
//      (LOCK_HIGH_FRACTION_PCT / LOCK_LOW_FRACTION_PCT, config.h) for
//      hysteresis.
//
// Confirmed against real hardware in both directions -- acquisition and
// loss of lock -- against the PLL_LOCK LED and an antenna
// disconnect/reconnect test, across two independent boards.
//
struct LockStatus {
  bool     valid;
  bool     rawLevel;        // instantaneous PLL_LOCK level (HIGH = locked)
  float    highFractionPct; // leaky-integrator estimate of % of recent time spent HIGH
  bool     locked;          // highFractionPct crossed LOCK_HIGH_FRACTION_PCT and hasn't yet
                             // dropped below LOCK_LOW_FRACTION_PCT
  uint32_t transitionCount; // cumulative raw level transitions since boot -- diagnostic
  uint32_t lastUpdateMs;
};

static float      s_highFraction    = 0.0f;
static bool        s_lockPrimed      = false;
static uint32_t    s_lastPollMs      = 0;
static bool        s_lastRawLevel    = false;
static uint32_t    s_transitionCount = 0;
static LockStatus  s_lock = { false, false, 0.0f, false, 0, 0 };

static void lockPoll() {
  bool     level = digitalRead(PLL_LOCK_PIN);
  uint32_t now   = millis();

  if (level != s_lastRawLevel) {
    s_lastRawLevel = level;
    s_transitionCount++;
  }

  if (!s_lockPrimed) {
    s_highFraction = level ? 1.0f : 0.0f;
    s_lastPollMs   = now;
    s_lockPrimed   = true;
  } else {
    uint32_t dt = now - s_lastPollMs;
    s_lastPollMs = now;
    if (dt > 0) {
      float alpha = (float)dt / ((float)dt + (float)LOCK_FILTER_TIME_CONSTANT_MS);
      s_highFraction += alpha * ((level ? 1.0f : 0.0f) - s_highFraction);
    }
  }

  s_lock.rawLevel        = level;
  s_lock.highFractionPct = s_highFraction * 100.0f;
  s_lock.transitionCount = s_transitionCount;
  s_lock.valid           = true;
  s_lock.lastUpdateMs    = now;

  if (s_lock.highFractionPct >= LOCK_HIGH_FRACTION_PCT) {
    s_lock.locked = true;
  } else if (s_lock.highFractionPct <= LOCK_LOW_FRACTION_PCT) {
    s_lock.locked = false;
  }
  // else: inside the hysteresis gap between the two thresholds -- keep
  // whatever the previous decision was.
}

// -----------------------------------------------------------------------
// Onboard RGB status LED (formerly status_led.h/.cpp)
// -----------------------------------------------------------------------
enum class LedColor : uint8_t { OFF, RED, AMBER, GREEN, BLUE };

#if defined(ARDUINO_ARCH_RP2040)
static const uint8_t NEOPIXEL_PIN     = 12;  // GPIO12, per Seeed's XIAO RP2040 schematic
static const uint8_t NEOPIXEL_PWR_PIN = 11;  // GPIO11, power gate for the LED
static Adafruit_NeoPixel s_pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

static void ledInit() {
  pinMode(NEOPIXEL_PWR_PIN, OUTPUT);
  digitalWrite(NEOPIXEL_PWR_PIN, HIGH);  // power the LED before talking to it
  s_pixel.begin();
  s_pixel.setBrightness(STATUS_LED_BRIGHTNESS);
  s_pixel.show();  // off until the first real ledSet() call
}

static void ledSet(LedColor color) {
  uint32_t rgb;
  switch (color) {
    case LedColor::RED:   rgb = s_pixel.Color(255, 0, 0);   break;
    case LedColor::AMBER: rgb = s_pixel.Color(255, 90, 0);  break;
    case LedColor::GREEN: rgb = s_pixel.Color(0, 255, 0);   break;
    case LedColor::BLUE:  rgb = s_pixel.Color(0, 0, 255);   break;
    case LedColor::OFF:
    default:              rgb = s_pixel.Color(0, 0, 0);     break;
  }
  s_pixel.setPixelColor(0, rgb);
  s_pixel.show();
}
#else  // no onboard addressable RGB LED on this board (SAMD21, ESP32C3)
static void ledInit() { /* no-op */ }
static void ledSet(LedColor) { /* no-op */ }
#endif

// -----------------------------------------------------------------------
// Status/event reporting (formerly status_report.h/.cpp)
// -----------------------------------------------------------------------
static uint32_t s_lastStatusMs = 0;
static bool     s_eventsPrimed = false;
static bool     s_lastGnssOk   = false;
static bool     s_lastLockOk   = false;

// Lock history, for statusPrintLockHistory() (menu command). Deliberately
// not persisted to flash -- tied to the current boot session, same as
// uptime itself. Note: based on millis(), which wraps at ~49.7 days; a
// unit run continuously past that point will show a misleadingly small
// uptime rather than the true elapsed time. Not addressed here since it
// wasn't asked for and would need real design thought (an RTC or a
// wraparound-counting scheme) rather than a quick patch.
static uint32_t s_gnssUnlockCount = 0;
static uint32_t s_pllUnlockCount  = 0;
static bool     s_hasUnlocked     = false;
static uint32_t s_lastUnlockMs    = 0;
static char     s_lastUnlockType  = ' ';  // 'G' or 'P'

static bool gnssIsOk(const GnssStatus& gnss, uint32_t now) {
  return gnss.valid && gnss.fixQuality > 0 && (now - gnss.lastUpdateMs) < STATUS_STALE_MS;
}

static bool lockIsOk(const LockStatus& lock, uint32_t now) {
  return lock.valid && lock.locked && (now - lock.lastUpdateMs) < STATUS_STALE_MS;
}

static LedColor computeStatusColor(bool gnssOk, bool lockOk) {
  if (gnssOk && lockOk) return LedColor::GREEN;
  if (gnssOk || lockOk) return LedColor::AMBER;
  return LedColor::RED;
}

// Prints immediate, one-line events on GPS-fix and PLL-lock gained/lost
// transitions -- e.g. so "time to first lock" is visible without
// waiting for the next periodic status line. The very first call primes
// the tracked state without printing, so boot doesn't announce a
// spurious "lost" event for a fix/lock that was never there to begin
// with. Also updates the lock-history counters used by
// statusPrintLockHistory().
static void printEvents(bool gnssOk, bool lockOk, uint32_t now) {
  uint32_t nowSec = now / 1000UL;

  if (!s_eventsPrimed) {
    s_lastGnssOk   = gnssOk;
    s_lastLockOk   = lockOk;
    s_eventsPrimed = true;
    return;
  }

  if (gnssOk != s_lastGnssOk) {
    CMD_SERIAL.print(F("[")); CMD_SERIAL.print(nowSec); CMD_SERIAL.print(F("] GPS: "));
    CMD_SERIAL.println(gnssOk ? F("LOCKED") : F("LOST"));
    if (!gnssOk) {
      s_gnssUnlockCount++;
      s_lastUnlockMs   = now;
      s_lastUnlockType = 'G';
      s_hasUnlocked    = true;
    }
    s_lastGnssOk = gnssOk;
  }
  if (lockOk != s_lastLockOk) {
    CMD_SERIAL.print(F("[")); CMD_SERIAL.print(nowSec); CMD_SERIAL.print(F("] Loop: "));
    CMD_SERIAL.println(lockOk ? F("LOCKED") : F("LOST"));
    if (!lockOk) {
      s_pllUnlockCount++;
      s_lastUnlockMs   = now;
      s_lastUnlockType = 'P';
      s_hasUnlocked    = true;
    }
    s_lastLockOk = lockOk;
  }
}

static void printClockField(uint8_t clk) {
  CMD_SERIAL.print(F(" CLK")); CMD_SERIAL.print(clk); CMD_SERIAL.print(F(" "));
  if (si5351IsClockEnabled(clk)) {
    CMD_SERIAL.print(si5351GetClockFreq(clk));
    CMD_SERIAL.print(F("Hz"));
  } else {
    CMD_SERIAL.print(F("off"));
  }
}

static void printStatusLine(const GnssStatus& gnss, const LockStatus& lock, uint32_t now) {
  CMD_SERIAL.print(F("["));
  CMD_SERIAL.print(now / 1000UL);
  CMD_SERIAL.print(F("] GPS: "));
  if (gnssIsOk(gnss, now)) {
    CMD_SERIAL.print(gnssFixQualityName(gnss.fixQuality));
    CMD_SERIAL.print(F(", "));
    CMD_SERIAL.print(gnss.numSV);
    CMD_SERIAL.print(F(" sats"));
  } else {
    CMD_SERIAL.print(F("no data"));
  }

  CMD_SERIAL.print(F(" | Loop: "));
  if (lock.valid && (now - lock.lastUpdateMs) < STATUS_STALE_MS) {
    CMD_SERIAL.print(lock.locked ? F("LOCKED") : F("unlocked"));
#ifdef LOCK_STATUS_VERBOSE
    CMD_SERIAL.print(F(" (raw "));
    CMD_SERIAL.print(lock.rawLevel ? F("HIGH") : F("LOW"));
    CMD_SERIAL.print(F(", "));
    CMD_SERIAL.print(lock.highFractionPct, 1);
    CMD_SERIAL.print(F("% high, "));
    CMD_SERIAL.print(lock.transitionCount);
    CMD_SERIAL.print(F(" transitions)"));
#endif
  } else {
    CMD_SERIAL.print(F("no data"));
  }

  CMD_SERIAL.print(F(" |"));
  printClockField(0);
  printClockField(1);
  printClockField(2);
  CMD_SERIAL.println();
}

static void formatHms(uint32_t totalSeconds, char* buf, size_t bufSize) {
  uint32_t hh = totalSeconds / 3600UL;
  uint32_t mm = (totalSeconds % 3600UL) / 60UL;
  uint32_t ss = totalSeconds % 60UL;
  snprintf(buf, bufSize, "%02lu:%02lu:%02lu", (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
}

static void printHaltMessage(const __FlashStringHelper* message) {
  CMD_SERIAL.println();
  CMD_SERIAL.println(F("*** HALTED ***"));
  CMD_SERIAL.println(message);
  CMD_SERIAL.println(F("Power-cycle after fixing the underlying problem."));
}

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------
void statusInit() {
  pinMode(PLL_LOCK_PIN, INPUT);
  s_lastRawLevel = digitalRead(PLL_LOCK_PIN);
  s_highFraction = s_lastRawLevel ? 1.0f : 0.0f;
  s_lastPollMs   = millis();
  s_lockPrimed   = true;

  ledInit();
  ledSet(LedColor::BLUE);  // booting
}

void statusPrintVersion() {
  CMD_SERIAL.print(F("=== MVR GPSDO B Firmware v"));
  CMD_SERIAL.print(F(FIRMWARE_VERSION));
  CMD_SERIAL.print(F(" (built "));
  CMD_SERIAL.print(F(__DATE__));
  CMD_SERIAL.print(F(" "));
  CMD_SERIAL.print(F(__TIME__));
  CMD_SERIAL.println(F(") ==="));
}

void statusHaltWithError(const __FlashStringHelper* message) {
  printHaltMessage(message);

  uint32_t lastPrintMs = millis();
  bool     blinkOn     = false;

  while (true) {
    if ((millis() - lastPrintMs) >= 5000UL) {
      printHaltMessage(message);
      lastPrintMs = millis();
    }
    blinkOn = !blinkOn;
    ledSet(blinkOn ? LedColor::RED : LedColor::OFF);
    delay(400);
  }
}

void statusPoll(bool menuActive) {
  lockPoll();

  GnssStatus gnss  = gnssGetStatus();
  uint32_t   now   = millis();
  bool       gnssOk = gnssIsOk(gnss, now);
  bool       lockOk = lockIsOk(s_lock, now);

  ledSet(computeStatusColor(gnssOk, lockOk));
  printEvents(gnssOk, lockOk, now);

  if (menuActive) {
    s_lastStatusMs = now;  // don't fire a status line the instant the menu exits
    return;
  }

#ifndef NMEA_PASSTHROUGH
  if ((now - s_lastStatusMs) >= STATUS_INTERVAL_MS) {
    printStatusLine(gnss, s_lock, now);
    s_lastStatusMs = now;
  }
#endif
}

void statusPrintLockHistory() {
  uint32_t now       = millis();
  uint32_t uptimeSec = now / 1000UL;

  char uptimeBuf[16];
  formatHms(uptimeSec, uptimeBuf, sizeof(uptimeBuf));

  CMD_SERIAL.println(F("=== Lock History ==="));
  CMD_SERIAL.print(F("  Uptime:      ")); CMD_SERIAL.println(uptimeBuf);
  CMD_SERIAL.print(F("  GPS unlocks: ")); CMD_SERIAL.println(s_gnssUnlockCount);
  CMD_SERIAL.print(F("  PLL unlocks: ")); CMD_SERIAL.println(s_pllUnlockCount);

  CMD_SERIAL.print(F("  Last unlock: "));
  if (!s_hasUnlocked) {
    CMD_SERIAL.println(F("none since boot"));
  } else {
    char agoBuf[16];
    formatHms((now - s_lastUnlockMs) / 1000UL, agoBuf, sizeof(agoBuf));
    CMD_SERIAL.print(agoBuf);
    CMD_SERIAL.print(F(" ago ("));
    CMD_SERIAL.print(s_lastUnlockType == 'G' ? F("GPS") : F("PLL"));
    CMD_SERIAL.println(F(")"));
  }
  CMD_SERIAL.println(F("  (all figures reset every ~49.7 days when the uptime timer wraps)"));
}
