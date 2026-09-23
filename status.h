#pragma once
//
// status.h — device status: PLL lock detection, the onboard RGB status
// LED, and console status/event reporting.
//
// Combines what used to be three separate files (lock_monitor.*,
// status_led.*, status_report.*) into one. They were split apart
// originally because lock_monitor carried real complexity (an interrupt
// handler and duty-cycle math); once that was replaced by a much
// simpler leaky-integrator approach, all three turned out to be small,
// tightly coupled, and serving one job -- interpret hardware state,
// drive the LED, tell the operator -- so keeping them apart no longer
// bought anything. LockStatus and LedColor are private to status.cpp
// now; nothing outside this module ever touched them directly.
//
#include <Arduino.h>

// Configures the PLL_LOCK input and the onboard LED (a no-op on boards
// without one), and shows the boot color. Call once from setup(),
// before the slower GNSS/Si5351 setup steps.
void statusInit();

// Prints one line identifying the firmware version and build date/time
// (from FIRMWARE_VERSION in config.h and the compiler's __DATE__/__TIME__).
// Used both in the boot banner and at the top of the menu, so it's
// visible from the console at a glance either way.
void statusPrintVersion();

// Call every loop() iteration. Updates PLL lock detection and the LED
// color every time; always updates the lock-history counters (see
// statusPrintLockHistory()) on transitions. Printing to CMD_SERIAL is
// gated so it never mixes with gnss.cpp's raw NMEA passthrough: while
// passthrough is enabled and the menu is closed, both the immediate GPS/
// PLL lock-gained/lock-lost event lines and the periodic formatted status
// line (normally every STATUS_INTERVAL_MS) are suppressed, leaving
// CMD_SERIAL carrying a clean NMEA stream. They resume automatically as
// soon as passthrough is turned off or the menu opens.
void statusPoll(bool menuActive);

// Prints a boot-time fatal error and never returns: blinks the status
// LED red and re-prints the message every 5 seconds indefinitely. Call
// this when a subsystem the firmware depends on (GNSS, Si5351) fails to
// configure during setup() -- continuing on as if things are fine would
// leave the unit silently non-functional with no indication why.
void statusHaltWithError(const __FlashStringHelper* message);

// Prints uptime and a summary of GPS/PLL lock-loss history since boot:
// counts of each, and time since the most recent loss of either.
// Callable on demand -- see menu.cpp for where it's wired up.
void statusPrintLockHistory();
