//
// gnss.cpp — see gnss.h for the public API and porting notes.
//
// Layout mirrors the original Python fairly closely so it's easy to diff
// against mvr_gnss_config.py / poll_tp5.py / reset_tp5.py:
//   - low-level UBX byte packing + checksum
//   - frame send / ACK-NAK read (blocking, with timeout -- same as the
//     original read_ack()/send_command())
//   - message builders (CFG-TP5, CFG-VALSET, CFG-MSG, CFG-CFG save)
//   - top-level configure sequence + retry wrapper
//   - background NMEA line accumulator + GGA parser for status
//
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "gnss.h"

// ---------------------------------------------------------------------
// UBX protocol constants
// ---------------------------------------------------------------------
static const uint8_t UBX_SYNC1 = 0xB5;
static const uint8_t UBX_SYNC2 = 0x62;

static const uint8_t UBX_CLASS_CFG     = 0x06;
static const uint8_t UBX_ID_CFG_TP5    = 0x31;
static const uint8_t UBX_ID_CFG_MSG    = 0x01;
static const uint8_t UBX_ID_CFG_CFG    = 0x09;
static const uint8_t UBX_ID_CFG_VALSET = 0x8A;

static const uint8_t UBX_CLASS_ACK  = 0x05;
static const uint8_t UBX_ID_ACK_ACK = 0x01;
static const uint8_t UBX_ID_ACK_NAK = 0x00;

static const uint32_t VALSET_KEY_INFIL_NCNOTHRS = 0x201100aaUL;  // U1: min SVs above C/N0 threshold
static const uint32_t VALSET_KEY_INFIL_CNOTHRS  = 0x201100abUL;  // U1: C/N0 threshold in dBHz

static const uint8_t VALSET_LAYER_RAM = 0x01;

static const uint8_t CFG_DEV_BBR   = 0x02;
static const uint8_t CFG_DEV_FLASH = 0x04;
static const uint32_t CFG_SAVE_ALL = 0x1F1FUL;

struct NmeaMsgDef { uint8_t cls; uint8_t id; const char* desc; };
static const NmeaMsgDef kNmeaMessages[] = {
  { 0xF0, 0x00, "GGA  (Fix data)" },
  { 0xF0, 0x01, "GLL  (Lat/Lon)" },
  { 0xF0, 0x02, "GSA  (DOP/active sats)" },
  { 0xF0, 0x03, "GSV  (Sats in view)" },
  { 0xF0, 0x04, "RMC  (Recommended minimum)" },
  { 0xF0, 0x05, "VTG  (Course/speed)" },
};
static const uint8_t kNmeaMessageCount = sizeof(kNmeaMessages) / sizeof(kNmeaMessages[0]);

// ---------------------------------------------------------------------
// Low-level byte packing (little-endian, matches struct.pack('<...') in
// the Python original)
// ---------------------------------------------------------------------
static inline void ubxAppendU8(uint8_t* buf, size_t& idx, uint8_t v) {
  buf[idx++] = v;
}
static inline void ubxAppendU16(uint8_t* buf, size_t& idx, uint16_t v) {
  buf[idx++] = (uint8_t)(v & 0xFF);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void ubxAppendU32(uint8_t* buf, size_t& idx, uint32_t v) {
  for (int i = 0; i < 4; i++) buf[idx++] = (uint8_t)((v >> (8 * i)) & 0xFF);
}
static inline void ubxAppendI16(uint8_t* buf, size_t& idx, int16_t v) {
  ubxAppendU16(buf, idx, (uint16_t)v);
}
static inline void ubxAppendI32(uint8_t* buf, size_t& idx, int32_t v) {
  ubxAppendU32(buf, idx, (uint32_t)v);
}

static inline void ubxChecksumAppend(uint8_t byte, uint8_t& ckA, uint8_t& ckB) {
  ckA = (uint8_t)(ckA + byte);
  ckB = (uint8_t)(ckB + ckA);
}

static void ubxSendFrame(uint8_t msgClass, uint8_t msgId,
                          const uint8_t* payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  uint8_t header[4] = {
    msgClass, msgId,
    (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF)
  };
  for (int i = 0; i < 4; i++) ubxChecksumAppend(header[i], ckA, ckB);
  for (uint16_t i = 0; i < len; i++) ubxChecksumAppend(payload[i], ckA, ckB);

  GNSS_SERIAL.write(UBX_SYNC1);
  GNSS_SERIAL.write(UBX_SYNC2);
  GNSS_SERIAL.write(header, 4);
  if (len) GNSS_SERIAL.write(payload, len);
  GNSS_SERIAL.write(ckA);
  GNSS_SERIAL.write(ckB);
}

// ---------------------------------------------------------------------
// Message builders
// ---------------------------------------------------------------------

// CFG-TP5, TP1 (tp_index=0). Flags 0x6F: active | lockGnssFreq |
// lockedOtherSet | isFreq | alignToTow | polarity (isLength stays clear --
// pulseLenRatio fields are duty fractions, not microseconds). Confirmed
// working combination from the RPi-era bring-up.
static uint16_t buildCfgTp5Payload(uint8_t* buf) {
  size_t i = 0;
  auto dutyToU32 = [](float pct) -> uint32_t {
    if (pct <= 0.0f) return 0UL;
    if (pct >= 100.0f) return 0xFFFFFFFFUL;
    return (uint32_t)((pct / 100.0f) * 4294967295.0);
  };
  const uint32_t flags = 0x6Ful;

  ubxAppendU8 (buf, i, 0);                              // tpIdx (TP1)
  ubxAppendU8 (buf, i, 1);                               // version
  ubxAppendU16(buf, i, 0);                               // reserved0
  ubxAppendI16(buf, i, 0);                               // antCableDelay [ns]
  ubxAppendI16(buf, i, 0);                               // rfGroupDelay [ns]
  ubxAppendU32(buf, i, GNSS_TP_FREQ_UNLOCKED_HZ);         // freqPeriod
  ubxAppendU32(buf, i, GNSS_TP_FREQ_LOCKED_HZ);           // freqPeriodLock
  ubxAppendU32(buf, i, dutyToU32(GNSS_TP_DUTY_UNLOCKED_PCT)); // pulseLenRatio
  ubxAppendU32(buf, i, dutyToU32(GNSS_TP_DUTY_LOCKED_PCT));   // pulseLenRatioLock
  ubxAppendI32(buf, i, 0);                               // userConfigDelay [ns]
  ubxAppendU32(buf, i, flags);                            // flags
  return (uint16_t)i;
}

static uint16_t buildCfgMsgPayload(uint8_t* buf, uint8_t nmeaClass, uint8_t nmeaId,
                                    uint8_t rateUart1) {
  size_t i = 0;
  ubxAppendU8(buf, i, nmeaClass);
  ubxAppendU8(buf, i, nmeaId);
  ubxAppendU8(buf, i, 0);          // I2C
  ubxAppendU8(buf, i, rateUart1);  // UART1
  ubxAppendU8(buf, i, 0);          // UART2
  ubxAppendU8(buf, i, 0);          // USB
  ubxAppendU8(buf, i, 0);          // SPI
  ubxAppendU8(buf, i, 0);          // reserved
  return (uint16_t)i;
}

static uint16_t buildCfgCfgSavePayload(uint8_t* buf) {
  size_t i = 0;
  ubxAppendU32(buf, i, 0x00000000UL);            // clearMask
  ubxAppendU32(buf, i, CFG_SAVE_ALL);            // saveMask
  ubxAppendU32(buf, i, 0x00000000UL);            // loadMask
  ubxAppendU8 (buf, i, CFG_DEV_BBR | CFG_DEV_FLASH); // deviceMask
  return (uint16_t)i;
}

struct ValsetU1Item { uint32_t keyId; uint8_t value; };

static uint16_t buildValsetU1Payload(uint8_t* buf, const ValsetU1Item* items,
                                      uint8_t count, uint8_t layerMask) {
  size_t i = 0;
  ubxAppendU8 (buf, i, 0);          // version
  ubxAppendU8 (buf, i, layerMask);  // layers
  ubxAppendU16(buf, i, 0);          // reserved
  for (uint8_t k = 0; k < count; k++) {
    ubxAppendU32(buf, i, items[k].keyId);
    ubxAppendU8 (buf, i, items[k].value);
  }
  return (uint16_t)i;
}

// ---------------------------------------------------------------------
// Frame reading / ACK-NAK handling
// ---------------------------------------------------------------------
struct UbxFrame {
  uint8_t  msgClass;
  uint8_t  msgId;
  uint16_t len;
  uint8_t  payload[40];   // largest payload we build/expect is CFG-TP5 at 32 bytes
};

enum class AckResult : uint8_t { ACK, NAK, TIMEOUT };

// Reads one complete, checksum-valid UBX frame within timeoutMs. Stray
// bytes (NMEA text mixed in on the same wire, corrupted frames) are
// silently discarded and scanning resumes -- same behavior as the
// original Python read_ack()/read_ubx().
static bool ubxReadFrame(UbxFrame* frame, uint32_t timeoutMs) {
  uint32_t start = millis();
  enum State { WAIT_SYNC1, WAIT_SYNC2, WAIT_CLASS, WAIT_ID,
               WAIT_LEN1, WAIT_LEN2, WAIT_PAYLOAD, WAIT_CKA, WAIT_CKB };
  State state = WAIT_SYNC1;
  uint8_t ckA = 0, ckB = 0, gotCkA = 0;
  uint16_t payloadIdx = 0;

  while ((millis() - start) < timeoutMs) {
    if (!GNSS_SERIAL.available()) { yield(); continue; }
    uint8_t b = (uint8_t)GNSS_SERIAL.read();

    switch (state) {
      case WAIT_SYNC1:
        state = (b == UBX_SYNC1) ? WAIT_SYNC2 : WAIT_SYNC1;
        break;
      case WAIT_SYNC2:
        state = (b == UBX_SYNC2) ? WAIT_CLASS : WAIT_SYNC1;
        break;
      case WAIT_CLASS:
        ckA = 0; ckB = 0;
        frame->msgClass = b;
        ubxChecksumAppend(b, ckA, ckB);
        state = WAIT_ID;
        break;
      case WAIT_ID:
        frame->msgId = b;
        ubxChecksumAppend(b, ckA, ckB);
        state = WAIT_LEN1;
        break;
      case WAIT_LEN1:
        frame->len = b;
        ubxChecksumAppend(b, ckA, ckB);
        state = WAIT_LEN2;
        break;
      case WAIT_LEN2:
        frame->len = (uint16_t)(frame->len | ((uint16_t)b << 8));
        ubxChecksumAppend(b, ckA, ckB);
        payloadIdx = 0;
        if (frame->len == 0) state = WAIT_CKA;
        else if (frame->len > sizeof(frame->payload)) state = WAIT_SYNC1;  // too big for us, resync
        else state = WAIT_PAYLOAD;
        break;
      case WAIT_PAYLOAD:
        frame->payload[payloadIdx++] = b;
        ubxChecksumAppend(b, ckA, ckB);
        if (payloadIdx >= frame->len) state = WAIT_CKA;
        break;
      case WAIT_CKA:
        gotCkA = b;
        state = WAIT_CKB;
        break;
      case WAIT_CKB:
        if (gotCkA == ckA && b == ckB) return true;
        state = WAIT_SYNC1;  // bad checksum, resync
        break;
    }
  }
  return false;
}

static AckResult ubxReadAck(uint8_t expectedClass, uint8_t expectedId, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    uint32_t elapsed = millis() - start;
    uint32_t remaining = (elapsed < timeoutMs) ? (timeoutMs - elapsed) : 0;
    UbxFrame frame;
    if (!ubxReadFrame(&frame, remaining)) return AckResult::TIMEOUT;
    if (frame.msgClass == UBX_CLASS_ACK && frame.len == 2 &&
        frame.payload[0] == expectedClass && frame.payload[1] == expectedId) {
      return (frame.msgId == UBX_ID_ACK_ACK) ? AckResult::ACK : AckResult::NAK;
    }
    // Not the frame we're waiting for (unrelated UBX or stray NMEA) -- keep looping.
  }
  return AckResult::TIMEOUT;
}

static bool ubxSendCommand(uint8_t msgClass, uint8_t msgId,
                            const uint8_t* payload, uint16_t len,
                            const char* description) {
  while (GNSS_SERIAL.available()) GNSS_SERIAL.read();  // don't let stale bytes look like our ACK
  ubxSendFrame(msgClass, msgId, payload, len);
  AckResult r = ubxReadAck(msgClass, msgId, GNSS_ACK_TIMEOUT_MS);

  const char* status = (r == AckResult::ACK) ? "OK  " : "FAIL";
  const char* reason  = (r == AckResult::ACK) ? "ACK"
                       : (r == AckResult::NAK) ? "NAK" : "TIMEOUT";
  CMD_SERIAL.print("  [");
  CMD_SERIAL.print(status);
  CMD_SERIAL.print("] ");
  CMD_SERIAL.print(description);
  CMD_SERIAL.print("  (");
  CMD_SERIAL.print(reason);
  CMD_SERIAL.println(")");
  return (r == AckResult::ACK);
}

// ---------------------------------------------------------------------
// Top-level configuration sequence
// ---------------------------------------------------------------------
static bool configureGnssOnce() {
  bool allOk = true;
  uint8_t payload[40];
  uint16_t len;

  CMD_SERIAL.println(F("Step 1: Configure timepulse (UBX-CFG-TP5)"));
  len = buildCfgTp5Payload(payload);
  allOk &= ubxSendCommand(UBX_CLASS_CFG, UBX_ID_CFG_TP5, payload, len,
                           "TP1: unlocked (0% duty) -> locked timepulse");

  CMD_SERIAL.println(F("Step 2: C/N0 input filter for fast unlock detection (UBX-CFG-VALSET)"));
  ValsetU1Item items[2] = {
    { VALSET_KEY_INFIL_CNOTHRS,  GNSS_CNO_THRESHOLD_DBHZ },
    { VALSET_KEY_INFIL_NCNOTHRS, GNSS_CNO_MIN_SATS },
  };
  len = buildValsetU1Payload(payload, items, 2, VALSET_LAYER_RAM);
  allOk &= ubxSendCommand(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, len,
                           "C/N0 threshold for fast unlock detection");

  CMD_SERIAL.println(F("Step 3: Enable NMEA navigation messages on UART1 (UBX-CFG-MSG)"));
  for (uint8_t k = 0; k < kNmeaMessageCount; k++) {
    len = buildCfgMsgPayload(payload, kNmeaMessages[k].cls, kNmeaMessages[k].id, 1);
    allOk &= ubxSendCommand(UBX_CLASS_CFG, UBX_ID_CFG_MSG, payload, len,
                             kNmeaMessages[k].desc);
  }

#if GNSS_SAVE_TO_FLASH
  CMD_SERIAL.println(F("Step 4: Save configuration to BBR/flash (UBX-CFG-CFG)"));
  len = buildCfgCfgSavePayload(payload);
  bool saveOk = ubxSendCommand(UBX_CLASS_CFG, UBX_ID_CFG_CFG, payload, len,
                                "Save all settings to BBR/flash");
  if (!saveOk) {
    CMD_SERIAL.println(F("         (NAK on save usually means V_BCKP battery is not installed --"));
    CMD_SERIAL.println(F("          configuration is still active in RAM)"));
  }
  allOk &= saveOk;
#else
  CMD_SERIAL.println(F("Step 4: Skipped (GNSS_SAVE_TO_FLASH is 0)"));
#endif

  return allOk;
}

void gnssSerialBegin() {
#if defined(ARDUINO_ARCH_ESP32)
  GNSS_SERIAL.begin(GNSS_BAUD, SERIAL_8N1, /*rxPin=*/D7, /*txPin=*/D6);
#elif defined(ARDUINO_ARCH_RP2040)
  GNSS_SERIAL.setRX(D7);
  GNSS_SERIAL.setTX(D6);
  GNSS_SERIAL.begin(GNSS_BAUD);
#else
  // SAMD21 and other cores where Serial1 is already fixed to the D6/D7
  // hardware UART in the board variant definition.
  GNSS_SERIAL.begin(GNSS_BAUD);
#endif
}

bool gnssConfigureWithRetry() {
  gnssSerialBegin();
  delay(GNSS_BOOT_SETTLE_MS);  // let the module's power supply and boot-up finish before commanding it
  while (GNSS_SERIAL.available()) GNSS_SERIAL.read();

  CMD_SERIAL.println(F("\nGPS02-UBX Configuration"));
  CMD_SERIAL.print(F("  TP when locked : "));
  CMD_SERIAL.print(GNSS_TP_FREQ_LOCKED_HZ);
  CMD_SERIAL.println(F(" Hz, 50% duty"));
  CMD_SERIAL.println(F("  TP when unlocked: 0% duty (output suppressed)"));

  for (uint8_t attempt = 1; attempt <= GNSS_CONFIG_MAX_RETRIES; attempt++) {
    CMD_SERIAL.print(F("Attempt "));
    CMD_SERIAL.print(attempt);
    CMD_SERIAL.print(F(" of "));
    CMD_SERIAL.println(GNSS_CONFIG_MAX_RETRIES);

    if (configureGnssOnce()) {
      CMD_SERIAL.println(F("GNSS configuration succeeded.\n"));
      return true;
    }
    if (attempt < GNSS_CONFIG_MAX_RETRIES) {
      CMD_SERIAL.print(F("Attempt failed, retrying in "));
      CMD_SERIAL.print(GNSS_CONFIG_RETRY_DELAY_MS / 1000);
      CMD_SERIAL.println(F("s..."));
      delay(GNSS_CONFIG_RETRY_DELAY_MS);
    }
  }
  CMD_SERIAL.println(F("GNSS configuration FAILED after all retries.\n"));
  return false;
}

// ---------------------------------------------------------------------
// Background NMEA line accumulation + GGA parsing for status
// ---------------------------------------------------------------------
static GnssStatus s_status = { false, 0, 0, 0 };
static char       s_lineBuf[96];
static uint8_t    s_lineLen = 0;
static bool       s_passthroughEnabled = false;  // runtime-only, session-only -- see gnss.h

// line is mutable and NUL-terminated, no trailing CR/LF.  Mutated in place
// by strtok() -- any use of the raw text (checksum check, passthrough
// echo) happens before tokenizing.
//
// menuActive suppresses the echo -- not just the periodic status line --
// so a passthrough session doesn't scroll NMEA text through the menu
// prompt while the operator is trying to read/type it. Internal fix/sat-
// count parsing below is unaffected either way.
static void handleNmeaLine(char* line, bool menuActive) {
  const char* star = strchr(line, '*');
  bool checksumOk = false;
  if (line[0] == '$' && star != nullptr) {
    uint8_t calc = 0;
    for (const char* p = line + 1; p < star; p++) calc ^= (uint8_t)*p;
    uint8_t given = (uint8_t)strtoul(star + 1, nullptr, 16);
    checksumOk = (calc == given);
  }

  if (s_passthroughEnabled && !menuActive) {
    CMD_SERIAL.println(line);  // forward raw text regardless of checksum result
  }

  if (!checksumOk) return;
  // Standard 2-character talker ID ($GP.../$GN...), so the sentence type
  // starts at offset 3.  u-blox NMEA output always follows this.
  if (strlen(line) < 6 || strncmp(line + 3, "GGA", 3) != 0) return;

  char* field[15] = { nullptr };
  uint8_t idx = 0;
  char* tok = strtok(line, ",");
  while (tok != nullptr && idx < 15) {
    field[idx++] = tok;
    tok = strtok(nullptr, ",");
  }
  if (idx < 8) return;  // malformed / truncated sentence

  s_status.fixQuality   = (uint8_t)atoi(field[6]);
  s_status.numSV        = (uint8_t)atoi(field[7]);
  s_status.valid         = true;
  s_status.lastUpdateMs  = millis();
}

void gnssPollNmea(bool menuActive) {
  while (GNSS_SERIAL.available()) {
    char c = (char)GNSS_SERIAL.read();
    if (c == '\r') continue;
    if (c == '\n') {
      s_lineBuf[s_lineLen] = '\0';
      if (s_lineLen > 0) handleNmeaLine(s_lineBuf, menuActive);
      s_lineLen = 0;
    } else if (s_lineLen < sizeof(s_lineBuf) - 1) {
      s_lineBuf[s_lineLen++] = c;
    } else {
      s_lineLen = 0;  // overflow guard: drop the line, resync on the next '\n'
    }
  }
}

void gnssSetPassthrough(bool enable) {
  s_passthroughEnabled = enable;
}

bool gnssIsPassthroughEnabled() {
  return s_passthroughEnabled;
}

GnssStatus gnssGetStatus() {
  return s_status;
}

const char* gnssFixQualityName(uint8_t fixQuality) {
  switch (fixQuality) {
    case 0:  return "no fix";
    case 1:  return "GPS fix";
    case 2:  return "DGPS fix";
    case 4:  return "RTK fixed";
    case 5:  return "RTK float";
    case 6:  return "dead reckoning";
    default: return "unknown";
  }
}
