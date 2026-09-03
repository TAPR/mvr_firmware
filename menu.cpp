//
// menu.cpp — see menu.h for the public API and design notes.
//
#include <Arduino.h>
#include <stdlib.h>
#include "config.h"
#include "menu.h"
#include "si5351.h"
#include "status.h"

enum class MenuState : uint8_t { IDLE, ROOT, AWAIT_FREQ, AWAIT_DRIVE, AWAIT_CONTINUE };

static MenuState s_state          = MenuState::IDLE;
static uint8_t   s_targetClk      = 0;
static uint32_t  s_lastActivityMs = 0;
static char      s_lineBuf[32];
static uint8_t   s_lineLen        = 0;

static void printRoot() {
  CMD_SERIAL.println();
  statusPrintVersion();
  CMD_SERIAL.println(F("--- Si5351 Menu ---"));
  for (uint8_t c = 0; c < 3; c++) {
    CMD_SERIAL.print(F("  "));
    CMD_SERIAL.print(c + 1);
    CMD_SERIAL.print(F(") Set CLK")); CMD_SERIAL.print(c);
    CMD_SERIAL.print(F(" frequency  (currently "));
    if (si5351IsClockEnabled(c)) {
      CMD_SERIAL.print(si5351GetClockFreq(c));
      CMD_SERIAL.println(F(" Hz)"));
    } else {
      CMD_SERIAL.println(F("off)"));
    }
  }
  CMD_SERIAL.print(F("  4) Set drive strength, all channels (currently "));
  CMD_SERIAL.print(si5351GetDrive());
  CMD_SERIAL.println(F(" mA)"));
  CMD_SERIAL.println(F("  5) Show this menu again"));
  CMD_SERIAL.println(F("  6) Show lock history"));
  CMD_SERIAL.println(F("  7) Restore compile-time defaults (overwrites saved config)"));
  CMD_SERIAL.println(F("  8) Exit menu"));
  CMD_SERIAL.print(F("> "));
}

static void enterRoot() {
  s_state = MenuState::ROOT;
  printRoot();
  s_lastActivityMs = millis();
}

static void exitMenu() {
  s_state = MenuState::IDLE;
  CMD_SERIAL.println(F("Exiting menu, back to normal status output.\n"));
}

bool menuIsActive() {
  return s_state != MenuState::IDLE;
}

void menuCheckWake() {
  if (s_state != MenuState::IDLE) return;
  if (!CMD_SERIAL.available()) return;
  // Any keypress wakes the menu. Drain whatever's waiting -- usually
  // just the keystroke(s) the user typed to get our attention -- so it
  // doesn't get misread as a menu selection once the prompt is up.
  while (CMD_SERIAL.available()) CMD_SERIAL.read();
  enterRoot();
}

static void handleRootSelection(const char* line) {
  int choice = atoi(line);
  switch (choice) {
    case 1: case 2: case 3:
      s_targetClk = (uint8_t)(choice - 1);
      CMD_SERIAL.print(F("Enter new frequency for CLK"));
      CMD_SERIAL.print(s_targetClk);
      CMD_SERIAL.print(F(" in Hz (2500-200000000), or 0 to power it down: "));
      s_state = MenuState::AWAIT_FREQ;
      break;
    case 4:
      CMD_SERIAL.print(F("Enter drive strength in mA (2, 4, 6, or 8): "));
      s_state = MenuState::AWAIT_DRIVE;
      break;
    case 5:
      printRoot();
      break;
    case 6:
      statusPrintLockHistory();
      CMD_SERIAL.println(F("(press Enter to return to the menu)"));
      s_state = MenuState::AWAIT_CONTINUE;
      break;
    case 7:
      si5351ResetToDefaults();
      CMD_SERIAL.println(F("Restored compile-time defaults and saved."));
      printRoot();
      break;
    case 8:
      exitMenu();
      break;
    default:
      CMD_SERIAL.println(F("Not a valid option."));
      printRoot();
      break;
  }
}

static void handleFreqEntry(const char* line) {
  long freq = atol(line);
  if (freq == 0) {
    si5351DisableClock(s_targetClk);
    CMD_SERIAL.print(F("CLK")); CMD_SERIAL.print(s_targetClk);
    CMD_SERIAL.println(F(" powered down."));
  } else if (freq < 0 || !si5351SetClockFreq(s_targetClk, (uint32_t)freq)) {
    CMD_SERIAL.println(F("Rejected -- out of range (2500-200000000 Hz)."));
  } else {
    CMD_SERIAL.print(F("CLK")); CMD_SERIAL.print(s_targetClk);
    CMD_SERIAL.print(F(" set to ")); CMD_SERIAL.print(freq);
    CMD_SERIAL.println(F(" Hz."));
  }
  enterRoot();
}

static void handleDriveEntry(const char* line) {
  int drive = atoi(line);
  if (!si5351SetDrive((uint8_t)drive)) {
    CMD_SERIAL.println(F("Rejected -- must be 2, 4, 6, or 8 mA."));
  } else {
    CMD_SERIAL.print(F("Drive strength set to ")); CMD_SERIAL.print(drive);
    CMD_SERIAL.println(F(" mA on all channels."));
  }
  enterRoot();
}

void menuPoll() {
  if (s_state == MenuState::IDLE) return;

  if ((millis() - s_lastActivityMs) > MENU_IDLE_TIMEOUT_MS) {
    CMD_SERIAL.println(F("\nMenu timed out, back to normal status output.\n"));
    s_state = MenuState::IDLE;
    return;
  }

  while (CMD_SERIAL.available()) {
    char c = (char)CMD_SERIAL.read();
    s_lastActivityMs = millis();
    if (c == '\r' || c == '\n') {
      CMD_SERIAL.println();  // move to a fresh line on the terminal, whichever line ending it sent
      s_lineBuf[s_lineLen] = '\0';
      if (s_state == MenuState::AWAIT_CONTINUE) {
        if (s_lineLen > 0) {
          // Typed a real selection instead of a bare Enter -- don't
          // discard it, act on it as if it were entered at the root
          // menu rather than forcing a second, separate submission.
          s_state = MenuState::ROOT;
          handleRootSelection(s_lineBuf);
        } else {
          enterRoot();  // bare Enter -- just redraw the menu
        }
      } else if (s_lineLen > 0) {
        switch (s_state) {
          case MenuState::ROOT:        handleRootSelection(s_lineBuf); break;
          case MenuState::AWAIT_FREQ:  handleFreqEntry(s_lineBuf);     break;
          case MenuState::AWAIT_DRIVE: handleDriveEntry(s_lineBuf);    break;
          default: break;
        }
      }
      s_lineLen = 0;
      if (s_state == MenuState::IDLE) return;  // exited mid-drain, stop here
    } else if (s_lineLen < sizeof(s_lineBuf) - 1) {
      CMD_SERIAL.write(c);  // echo what was typed -- no local echo can be assumed
      s_lineBuf[s_lineLen++] = c;
    } else {
      s_lineLen = 0;  // overflow guard: drop the line, resync on next '\r'/'\n'
    }
  }
}
