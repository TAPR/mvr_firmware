#pragma once
//
// gnss.h — GPS02-UBX (u-blox M10) configuration and status monitoring for
// the MVR GPSDO B firmware.
//
// Ported from mvr_gnss_config.py / poll_tp5.py / reset_tp5.py (the RPi Hat
// software).  Configuration values (timepulse frequency, C/N0 filter
// thresholds, save-to-flash) are compile-time constants in config.h -- this
// sketch does not expose runtime GNSS reconfiguration, only the Si5351
// synthesizer gets that (see menu.h).
//
#include <Arduino.h>

struct GnssStatus {
  bool     valid;          // true once at least one GGA sentence has parsed OK
  uint8_t  fixQuality;     // NMEA GGA fix-quality field: 0=none,1=GPS,2=DGPS,4=RTK fixed,5=RTK float,6=dead reckoning
  uint8_t  numSV;          // satellites used in the fix, from GGA
  uint32_t lastUpdateMs;   // millis() of the last successfully parsed GGA sentence
};

// Opens GNSS_SERIAL at GNSS_BAUD on the correct pins for the active core.
// Called internally by gnssConfigureWithRetry(); exposed separately in case
// the sketch ever needs to reopen the port without repeating configuration.
void gnssSerialBegin();

// Runs the full GNSS module configuration sequence (timepulse, C/N0 input
// filter, NMEA message enables, optional save-to-flash) once, retrying the
// whole sequence up to GNSS_CONFIG_MAX_RETRIES times on any step failure --
// same retry contract as the old mvr_gnss_configure.sh wrapper.  Call once
// from setup().  Returns true if some attempt fully succeeded.
bool gnssConfigureWithRetry();

// Call every loop() iteration.  Non-blocking: drains whatever bytes are
// waiting on GNSS_SERIAL, accumulates NMEA lines, validates checksums, and
// updates the status struct whenever a GGA sentence parses cleanly.  Also
// echoes raw NMEA lines to CMD_SERIAL when NMEA_PASSTHROUGH is defined.
void gnssPollNmea();

// Latest parsed fix status.  Check .valid and .lastUpdateMs before trusting
// it -- a stale timestamp means the module has stopped talking (bad wiring,
// power loss, etc).
GnssStatus gnssGetStatus();

// Short human-readable label for a GGA fix-quality value, for status lines.
const char* gnssFixQualityName(uint8_t fixQuality);
