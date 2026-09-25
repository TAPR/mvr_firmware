//
// cmd_output.cpp — see cmd_output.h for design notes.
//
#include <Arduino.h>
#include <stdio.h>
#include "cmd_output.h"

CmdOutput cmdOutput;

size_t CmdOutput::write(uint8_t c) {
  if (!wrapEnabled_) {
    // Not wrapping -- e.g. the menu is open, or passthrough/wrap mode is
    // off entirely. Write straight through, unbuffered, so interactive
    // use (character-by-character echo, the "> " prompt with no
    // trailing newline, backspace erase sequences in menu.cpp) behaves
    // exactly as it always has. This path must stay byte-for-byte
    // immediate -- buffering it would silently break live keystroke
    // echo, since a full line (with its trailing '\n') may not arrive
    // for a long time, if ever, during interactive use.
    return Serial.write(c);
  }

  // Wrapping -- buffer until a full line is available, since a $PMVR
  // sentence's checksum has to cover the whole line at once and can't
  // be computed incrementally as bytes trickle in.
  if (c == '\n') {
    flushLine();
  } else if (lineLen_ < sizeof(lineBuf_) - 1) {
    lineBuf_[lineLen_++] = (char)c;
  }
  // else: buffer full -- silently truncate the rest of this line; it
  // still flushes (wrapped, with whatever fit) at the next '\n'. A
  // truncated status line is far preferable to a buffer overrun or a
  // silently dropped one.
  return 1;
}

int CmdOutput::available() { return Serial.available(); }
int CmdOutput::read()      { return Serial.read(); }
int CmdOutput::peek()      { return Serial.peek(); }

void CmdOutput::setWrapEnabled(bool enabled) {
  wrapEnabled_ = enabled;
  lineLen_     = 0;  // defensive: discard any partial line buffered under
                      // the previous mode rather than risk flushing it,
                      // wrapped or not, under the wrong one. Under normal
                      // single-threaded loop() operation this should
                      // never actually hold anything at a mode-change
                      // boundary (mode only changes between top-level
                      // print sequences, never mid-sequence), but the
                      // cost of clearing defensively is free.
}

void CmdOutput::writeRawLine(const char* line) {
  // Writes straight to the physical port, completely bypassing both the
  // line buffer and $PMVR wrapping -- for verbatim NMEA sentences
  // already received from the GNSS module and checksum-validated by
  // handleNmeaLine(). These ARE the payload passthrough mode exists to
  // deliver; they must never be altered or wrapped, in any mode.
  Serial.print(line);
  Serial.print(F("\r\n"));
}

void CmdOutput::flushLine() {
  uint8_t len = lineLen_;
  if (len > 0 && lineBuf_[len - 1] == '\r') len--;  // strip println()'s CR half of "\r\n"

  lineLen_ = 0;  // reset the buffer regardless, before emitting

  if (len == 0) return;  // drop blank lines (spacing printlns) instead of
                          // wrapping an empty payload

  emitWrapped(lineBuf_, len);
}

void CmdOutput::emitWrapped(const char* text, uint8_t len) {
  // $PMVR,MSG,<text>*CS -- standard NMEA 0183 proprietary-sentence form:
  // "$P" plus a 3-character mnemonic ("MVR"), then whatever fields the
  // originator wants. A parser that doesn't recognize "PMVR" is
  // expected to skip the whole line once it sees a well-formed
  // sentence (leading '$', trailing '*' + 2 hex digits) of a type it
  // doesn't recognize -- that's the entire point of using this form
  // rather than emitting the text plain.
  //
  // Checksum is the standard NMEA 0183 XOR of every byte between '$'
  // and '*' -- the same algorithm handleNmeaLine() in gnss.cpp already
  // uses to validate real GNSS sentences on receive, applied here on
  // transmit instead.
  static const char kHeader[] = "PMVR,MSG,";
  uint8_t csum = 0;
  for (const char* p = kHeader; *p; p++) csum ^= (uint8_t)*p;

  Serial.print('$');
  Serial.print(kHeader);

  for (uint8_t i = 0; i < len; i++) {
    char c = text[i];
    // Sanitize: a literal '*' in the payload would let a strict parser
    // read past our intended checksum and misframe the sentence. None
    // of the firmware's current status/menu text contains one, but
    // this guards against it regardless. ',' is left alone -- NMEA
    // fields are comma-delimited, and an unrecognized sentence type is
    // skipped whole by a compliant parser, not parsed field-by-field.
    if (c == '*') c = '\'';
    Serial.write((uint8_t)c);
    csum ^= (uint8_t)c;
  }

  char csbuf[6];
  snprintf(csbuf, sizeof(csbuf), "*%02X\r\n", csum);
  Serial.print(csbuf);
}
