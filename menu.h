#pragma once
//
// menu.h — interactive serial menu for runtime Si5351 control, plus the
// NMEA passthrough on/off toggle (see gnss.h).
//
// Normal operation prints periodic status, or a raw NMEA stream if
// passthrough is on (see main sketch); any keypress on CMD_SERIAL wakes
// this menu either way. Selections apply immediately and return to the
// menu, which times out back to normal run mode (whichever of the two
// that currently is) after MENU_IDLE_TIMEOUT_MS of inactivity.
//
#include <Arduino.h>

// True while the menu is active. The main sketch checks this to decide
// whether to route input to menuPoll() or to menuCheckWake(), and to
// suppress the periodic status line while the menu has the terminal.
bool menuIsActive();

// Call every loop() iteration while the menu is NOT active. Checks for
// a "wake up" keypress on CMD_SERIAL and, if seen, drains it and shows
// the menu.
void menuCheckWake();

// Call every loop() iteration while the menu IS active. Non-blocking:
// processes at most one line of input per call and enforces
// MENU_IDLE_TIMEOUT_MS.
void menuPoll();
