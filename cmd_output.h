#pragma once
//
// cmd_output.h — CMD_SERIAL's underlying object: wraps the physical USB
// CDC serial port so status/menu/boot text can, on request, be sent as
// a proprietary NMEA 0183 sentence ($PMVR,...) instead of plain text.
//
// Why: when NMEA passthrough is streaming the GNSS module's real
// sentences out CMD_SERIAL for a host (gpsd, an ntpd/chrony NMEA/PPS
// refclock, etc.) to consume, anything else the firmware would
// normally print -- the boot banner, the periodic status line,
// lock/GPS lock-transition events, the Si5351/GNSS setup-sequence
// messages -- would otherwise land on the same stream as plain text
// and could confuse a strict parser. Wrapping it as an unrecognized
// but well-formed $P (proprietary) sentence is the standard NMEA 0183
// mechanism for exactly this: a compliant parser that doesn't
// recognize "PMVR" is expected to skip the line, not choke on it.
//
// This is a drop-in Stream -- config.h's CMD_SERIAL macro will point
// straight at the single global instance below -- so none of the many
// existing CMD_SERIAL.print()/println() call sites elsewhere in the
// firmware need to change. Two things DO need to call this module
// directly, though (wired up in later files, not here):
//   - Whoever decides passthrough/menu state (gnss.cpp, menu.cpp) calls
//     setWrapEnabled() when that state changes. In particular, the menu
//     itself is always plain/unwrapped text -- interactive keystroke
//     echo depends on wrap mode being off (see the .cpp for why).
//   - The one place that echoes a real, already-valid GNSS NMEA
//     sentence verbatim (gnss.cpp's handleNmeaLine()) must call
//     writeRawLine() instead of the normal print path -- that text is
//     the actual passthrough payload and must never be wrapped.
//
#include <Arduino.h>

class CmdOutput : public Stream {
public:
  // Forwarders for the physical USB CDC port's own extras, which a
  // plain Stream doesn't have but the main sketch calls directly on
  // CMD_SERIAL: begin() to bring the port up, and the boolean
  // conversion (used as `while (!CMD_SERIAL) { ... }` to wait for a
  // terminal to actually connect on native USB boards). Both simply
  // forward to the real Serial object -- CmdOutput itself has no
  // concept of either.
  void begin(unsigned long baud) { Serial.begin(baud); }
  operator bool() const { return (bool)Serial; }

  // Print interface (Stream extends Print).
  size_t write(uint8_t c) override;

  // Stream interface -- keystroke input always passes straight through
  // to the physical port, regardless of wrap mode.
  int available() override;
  int read() override;
  int peek() override;

  // Turns $PMVR wrapping on/off for subsequently flushed lines. Resets
  // any partially-buffered line left over from the previous mode
  // rather than risk flushing it, wrapped or not, under the wrong one.
  void setWrapEnabled(bool enabled);
  bool isWrapEnabled() const { return wrapEnabled_; }

  // Writes line + CRLF straight to the physical port, bypassing both
  // the line buffer and $PMVR wrapping entirely. For real, already
  // checksum-valid GNSS NMEA sentences only -- see handleNmeaLine() in
  // gnss.cpp. line must be NUL-terminated with no embedded CR or LF.
  void writeRawLine(const char* line);

private:
  void flushLine();
  void emitWrapped(const char* text, uint8_t len);

  static const uint8_t kLineBufSize = 200;  // comfortably covers the longest
                                             // current status/menu line plus
                                             // $PMVR/checksum overhead; see
                                             // the truncation note in the .cpp
  char    lineBuf_[kLineBufSize];
  uint8_t lineLen_ = 0;
  bool    wrapEnabled_ = false;
};

// Single global instance -- config.h's CMD_SERIAL macro resolves to this.
extern CmdOutput cmdOutput;
