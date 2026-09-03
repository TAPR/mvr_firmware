//
// mvr_firmware.ino — MVR GPSDO A2 firmware, top-level setup()/loop().
//
// Combines GNSS module configuration (gnss.*), Si5351 synthesizer control
// (si5351.*), a runtime serial menu for the Si5351 (menu.*), and device
// status -- PLL lock detection, the onboard RGB LED, and console
// status/event reporting -- all in status.*.
//
// Kept deliberately thin: the Arduino IDE's automatic function-prototype
// generation for .ino files mis-parses functions using custom enum/struct
// types, so anything beyond simple setup()/loop() glue lives in a .cpp
// file instead -- see status.h for the specifics.
//
// Requires the Adafruit NeoPixel library (Sketch > Include Library >
// Manage Libraries > search "Adafruit NeoPixel" > Install) for the RGB
// status LED on RP2040 boards. status.cpp compiles the LED code to a
// no-op stub on boards without an onboard NeoPixel, but the library
// still needs to be present for the #include to resolve when building
// for RP2040.
//
#include <Arduino.h>
#include "config.h"
#include "gnss.h"
#include "si5351.h"
#include "status.h"
#include "menu.h"

void setup() {
  CMD_SERIAL.begin(CMD_SERIAL_BAUD);
  uint32_t waitStart = millis();
  while (!CMD_SERIAL && (millis() - waitStart) < 3000) {
    // Give native USB CDC a moment to enumerate; don't hang forever if
    // nobody's watching.
  }

  CMD_SERIAL.println();
  statusPrintVersion();

  statusInit();  // PLL_LOCK pin + boot LED color, before the slower steps below

  if (!gnssConfigureWithRetry()) {
    statusHaltWithError(F("GNSS configuration failed after all retries -- check module "
                           "power and the D6/D7 UART wiring."));
    // never returns
  }

  if (!si5351Init()) {
    statusHaltWithError(F("Si5351 configuration failed -- check I2C wiring (D4/D5) and "
                           "that the device is powered."));
    // never returns
  }

  CMD_SERIAL.println(F("Setup complete. Type any character for the Si5351 menu."));
}

void loop() {
  gnssPollNmea();

  bool menuActive = menuIsActive();
  if (menuActive) {
    menuPoll();
    menuActive = menuIsActive();  // may have just exited
  } else {
    menuCheckWake();
    menuActive = menuIsActive();  // may have just woken
  }

  statusPoll(menuActive);
}
