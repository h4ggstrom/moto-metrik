// moto-metrik: GPS + IMU logger.
// Valid NMEA sentences from the GPS and raw IMU samples ($PIMU) are written to
// the SD card (one file per boot). All analysis happens offline with
// tools/analyze.py.

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <sys/time.h>
#include <time.h>

#include "config.h"
#include "imu.h"

static HardwareSerial &gpsSerial = Serial2;
static SPIClass sdSpi(VSPI);
static TinyGPSPlus gps;

static File logFile;
static char logPath[40];
static bool sdOk = false;
static bool clockSet = false;
static uint32_t lastFlush = 0;
static uint32_t bytesLogged = 0;
static bool imuOk = false;

// ---------------------------------------------------------------------------
// u-blox configuration (UBX binary protocol)
// ---------------------------------------------------------------------------

static void ubxSend(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  auto ck = [&](uint8_t b) { ckA += b; ckB += ckA; };
  uint8_t hdr[] = {cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);
  for (uint8_t b : hdr) { gpsSerial.write(b); ck(b); }
  for (uint16_t i = 0; i < len; i++) { gpsSerial.write(payload[i]); ck(payload[i]); }
  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
  gpsSerial.flush();
  delay(50);
}

// Wait for UBX-ACK-ACK / ACK-NAK for a CFG message. 1 = ACK, 0 = NAK, -1 = timeout.
static int ubxWaitAck(uint8_t cls, uint8_t id, uint32_t timeoutMs = 500) {
  const uint8_t hdr[] = {0xB5, 0x62, 0x05};
  uint8_t buf[8];
  uint8_t n = 0;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (gpsSerial.available()) {
      uint8_t b = gpsSerial.read();
      if (n < 3 && b != hdr[n]) { n = (b == 0xB5); if (n) buf[0] = b; continue; }
      buf[n++] = b;
      if (n == 8) {  // B5 62 05 ackId 02 00 cls id
        if (buf[6] == cls && buf[7] == id) return buf[3] == 0x01 ? 1 : 0;
        n = 0;
      }
    }
    delay(2);
  }
  return -1;
}

static const char *ackStr(int r) { return r == 1 ? "ACK" : r == 0 ? "NAK" : "no reply"; }

// Output rate of an NMEA message on this port, in navigation epochs (0 = off).
static void ubxSetNmeaRate(uint8_t msgId, uint8_t rate) {
  const uint8_t p[] = {0xF0, msgId, rate};
  ubxSend(0x06, 0x01, p, sizeof(p));  // CFG-MSG
}

// Poll a UBX message (empty payload) and copy the answer's payload.
// Returns the payload length, or -1 on timeout.
static int ubxPoll(uint8_t cls, uint8_t id, uint8_t *out, uint16_t maxLen, uint32_t timeoutMs = 1000) {
  while (gpsSerial.available()) gpsSerial.read();
  ubxSend(cls, id, nullptr, 0);
  const uint8_t hdr[] = {0xB5, 0x62, cls, id};
  uint8_t n = 0;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (!gpsSerial.available()) { delay(1); continue; }
    uint8_t b = gpsSerial.read();
    if (n < 4) {
      n = (b == hdr[n]) ? n + 1 : (b == 0xB5);
      continue;
    }
    // header matched: length, payload, checksum
    while (gpsSerial.available() < 1 && millis() - start < timeoutMs) delay(1);
    uint16_t len = b | (gpsSerial.read() << 8);
    for (uint16_t i = 0; i < len + 2; i++) {
      while (!gpsSerial.available()) {
        if (millis() - start >= timeoutMs) return -1;
        delay(1);
      }
      uint8_t c = gpsSerial.read();
      if (i < len && i < maxLen) out[i] = c;
    }
    return len < maxLen ? len : maxLen;
  }
  return -1;
}

static void ubxSetRate(uint16_t ms) {
  const uint8_t p[] = {(uint8_t)(ms & 0xFF), (uint8_t)(ms >> 8), 1, 0, 1, 0};
  ubxSend(0x06, 0x08, p, sizeof(p));  // CFG-RATE
}

// SBAS (EGNOS) on/off. With corrections applied, the NEO-M8N drops from 10 Hz
// to ~5 Hz (every other epoch skipped, measured on 2026-10-15): keep it off.
static void ubxSetSbas(bool enabled) {
  uint8_t p[8] = {0};
  p[0] = enabled ? 0x01 : 0x00;  // mode
  p[1] = 0x07;                   // usage: range + diffCorr + integrity (when enabled)
  p[2] = 3;                      // maxSBAS
  ubxSend(0x06, 0x16, p, sizeof(p));  // CFG-SBAS
}

// Constellations: GPS + one of Galileo / GLONASS. The NEO-M8N holds 10 Hz with
// two concurrent GNSS, not three. QZSS follows GPS (u-blox recommendation).
// The receiver restarts its GNSS engine after this message.
static void ubxSetGnss(bool galileo) {
  // gnssId, resTrkCh, maxTrkCh, reserved, flags (enable bit0, sigCfgMask = L1/E1)
  const uint8_t blocks[][8] = {
      {0, 8, 16, 0, 0x01, 0x00, 0x01, 0x01},                      // GPS
      {1, 1, 3, 0, 0x00, 0x00, 0x01, 0x01},                       // SBAS (off)
      {2, 4, 8, 0, (uint8_t)(galileo ? 1 : 0), 0x00, 0x01, 0x01},  // Galileo
      {3, 8, 16, 0, 0x00, 0x00, 0x01, 0x01},                      // BeiDou (off)
      {5, 0, 3, 0, 0x01, 0x00, 0x01, 0x01},                       // QZSS
      {6, 8, 14, 0, (uint8_t)(galileo ? 0 : 1), 0x00, 0x01, 0x01},  // GLONASS
  };
  uint8_t p[4 + sizeof(blocks)];
  p[0] = 0;     // msgVer
  p[1] = 0;     // numTrkChHw (read-only)
  p[2] = 0xFF;  // numTrkChUse: all available
  p[3] = sizeof(blocks) / 8;
  memcpy(p + 4, blocks, sizeof(blocks));
  ubxSend(0x06, 0x3E, p, sizeof(p));  // CFG-GNSS
}

// Navigation filter dynamics: the default "portable" model over-smooths
// speed/course at track dynamics.
static void ubxSetDynamicModel(uint8_t model) {
  uint8_t p[36] = {0};
  p[0] = 0x01;     // mask: apply dynModel only
  p[2] = model;
  ubxSend(0x06, 0x24, p, sizeof(p));  // CFG-NAV5
}

static void ubxSetBaud(uint32_t baud) {
  uint8_t p[20] = {0};
  p[0] = 1;                                     // UART1
  p[4] = 0xD0; p[5] = 0x08;                     // 8N1
  for (int i = 0; i < 4; i++) p[8 + i] = (baud >> (8 * i)) & 0xFF;
  p[12] = 0x07;                                 // in: UBX+NMEA+RTCM
  p[14] = 0x03;                                 // out: UBX+NMEA
  ubxSend(0x06, 0x00, p, sizeof(p));            // CFG-PRT
}

static void updateLed();

// Listen for NMEA traffic at the current baud rate.
static bool gpsTalking(uint32_t windowMs) {
  uint32_t start = millis();
  char prev = 0;
  while (millis() - start < windowMs) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (prev == '$' && c == 'G') return true;
      prev = c;
    }
    updateLed();  // keep signalling while probing blocks the main loop
    delay(5);
  }
  return false;
}

static uint32_t gpsProbeBaud() {
  const uint32_t bauds[] = {GPS_TARGET_BAUD, 9600, 38400, 57600};
  for (uint32_t b : bauds) {
    gpsSerial.updateBaudRate(b);
    while (gpsSerial.available()) gpsSerial.read();
    if (gpsTalking(1500)) return b;
  }
  return 0;
}

// Every write to the log goes through here. A short write means the card
// stopped answering (loose wire, card pulled): flag it, sdRecover() takes over.
static uint32_t sdFailures = 0;
static void logWrite(const uint8_t *buf, size_t len) {
  if (!sdOk) return;
  if (logFile.write(buf, len) != len) {
    sdOk = false;
    sdFailures++;
    Serial.println("[sd] write failed - card lost");
    return;
  }
  bytesLogged += len;
}

// Append "*CS\r\n" to a "$..." sentence in buf (len chars) and write it to the log.
static void logSentence(char *buf, int len, size_t cap) {
  uint8_t cs = 0;
  for (int i = 1; i < len; i++) cs ^= (uint8_t)buf[i];
  len += snprintf(buf + len, cap - len, "*%02X\r\n", cs);
  logWrite((const uint8_t *)buf, len);
}

// Session metadata ($PMMK lines) is kept so it can be replayed at the top of
// a new file after an SD recovery: the IMU scale factors must be in every file.
static char infoLines[6][200];
static int nInfo = 0;

// Write a "$PMMK,<text>*CS" sentence to the log (and the console): session
// metadata that NMEA parsers skip.
static void logInfo(const char *text) {
  char line[200];
  int n = snprintf(line, sizeof(line), "$PMMK,%s", text);
  uint8_t cs = 0;
  for (int i = 1; i < n; i++) cs ^= (uint8_t)line[i];
  snprintf(line + n, sizeof(line) - n, "*%02X\r\n", cs);
  Serial.printf("[info] %s", line);
  logWrite((const uint8_t *)line, strlen(line));
  if (nInfo < 6) strlcpy(infoLines[nInfo++], line, sizeof(infoLines[0]));
}

static void gpsReportVersion() {
  uint8_t buf[160];
  int len = ubxPoll(0x0A, 0x04, buf, sizeof(buf));  // MON-VER
  if (len >= 40) {
    char text[190];
    int n = snprintf(text, sizeof(text), "MONVER,%.30s,%.10s", (char *)buf, (char *)buf + 30);
    for (int off = 40; off + 30 <= len && n < (int)sizeof(text) - 32; off += 30)
      n += snprintf(text + n, sizeof(text) - n, ",%.30s", (char *)buf + off);
    for (char *c = text; *c; c++)
      if (*c == '*' || *c == '$') *c = '_';
    logInfo(text);
  } else {
    Serial.println("[gps] MON-VER: no reply");
  }

  len = ubxPoll(0x06, 0x3E, buf, sizeof(buf));  // CFG-GNSS
  if (len >= 4) {
    static const char *names[] = {"GPS", "SBAS", "GAL", "BDS", "IMES", "QZSS", "GLO"};
    char text[160];
    int n = snprintf(text, sizeof(text), "GNSS,sbas_corr=%s,ch=%u/%u", GPS_SBAS ? "on" : "off", buf[1], buf[2]);
    for (int b = 0; b < buf[3] && 4 + 8 * b + 8 <= len; b++) {
      const uint8_t *blk = buf + 4 + 8 * b;
      const char *name = blk[0] < 7 ? names[blk[0]] : "?";
      n += snprintf(text + n, sizeof(text) - n, ",%s:%s", name, (blk[4] & 0x01) ? "on" : "off");
    }
    logInfo(text);
  } else {
    Serial.println("[gps] CFG-GNSS: no reply");
  }
}

// Find the GPS baud rate and push our settings. Also used to recover a GPS
// that lost power mid-session: it then reboots on factory settings (9600 baud, 1 Hz).
static bool gpsConfigure() {
  uint32_t baud = gpsProbeBaud();
  if (!baud) {
    Serial.println("[gps] no NMEA on any baud rate - check wiring (GPS TX -> GPIO16)");
    gpsSerial.updateBaudRate(GPS_TARGET_BAUD);
    return false;
  }
  Serial.printf("[gps] found NMEA at %u baud\n", baud);

  // RMC (position/speed/course/date) and GGA (fix/sats/hdop/alt) every epoch;
  // GSA (satellites used) and GSV (satellites in view) at a low rate to see
  // the receiver load; GLL and VTG off.
  ubxSetNmeaRate(0x01, 0);  // GLL
  ubxSetNmeaRate(0x02, GPS_SAT_INFO_EPOCHS);  // GSA
  ubxSetNmeaRate(0x03, GPS_SAT_INFO_EPOCHS);  // GSV
  ubxSetNmeaRate(0x05, 0);  // VTG

  if (baud != GPS_TARGET_BAUD) {
    ubxSetBaud(GPS_TARGET_BAUD);
    gpsSerial.updateBaudRate(GPS_TARGET_BAUD);
    if (gpsTalking(1500)) {
      baud = GPS_TARGET_BAUD;
      Serial.printf("[gps] switched to %u baud\n", baud);
    } else {
      Serial.println("[gps] baud switch not accepted (not u-blox?), staying");
      gpsSerial.updateBaudRate(baud);
    }
  }

  while (gpsSerial.available()) gpsSerial.read();
  ubxSetRate(GPS_RATE_MS);
  Serial.printf("[gps] rate %u ms: %s\n", GPS_RATE_MS, ackStr(ubxWaitAck(0x06, 0x08)));
  ubxSetDynamicModel(GPS_DYN_MODEL);
  Serial.printf("[gps] dynamic model %u: %s\n", GPS_DYN_MODEL, ackStr(ubxWaitAck(0x06, 0x24)));
  ubxSetSbas(GPS_SBAS);
  Serial.printf("[gps] SBAS %s: %s\n", GPS_SBAS ? "on" : "off", ackStr(ubxWaitAck(0x06, 0x16)));
  ubxSetGnss(GPS_GALILEO);
  Serial.printf("[gps] GNSS GPS+%s: %s\n", GPS_GALILEO ? "Galileo" : "GLONASS", ackStr(ubxWaitAck(0x06, 0x3E)));
  gpsReportVersion();
  return true;
}

static void gpsInit() {
  gpsSerial.setRxBufferSize(4096);  // absorbs SD write latency spikes
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsConfigure();
}

// ---------------------------------------------------------------------------
// SD logging
// ---------------------------------------------------------------------------

static bool sdMount(int attempts) {
  // After a soft reset (flash, brownout) the card keeps power and can be stuck
  // mid-transaction: retry, it usually answers on the second attempt.
  for (int attempt = 1; attempt <= attempts; attempt++) {
    if (SD.begin(SD_CS_PIN, sdSpi, SD_SPI_FREQ)) return true;
    Serial.printf("[sd] mount attempt %d failed\n", attempt);
    SD.end();
    delay(200);
  }
  return false;
}

// Open a new log file: named after the current local time once the clock is
// set, /log_NNNN.nmea before that (renamed at the first fix).
static bool sdOpenNew() {
  if (clockSet) {
    time_t now = time(nullptr);
    struct tm local;
    localtime_r(&now, &local);
    strftime(logPath, sizeof(logPath), "/%Y-%m-%d_%H-%M-%S.nmea", &local);
  } else {
    for (int i = 1; i < 10000; i++) {
      snprintf(logPath, sizeof(logPath), "/log_%04d.nmea", i);
      if (!SD.exists(logPath)) break;
    }
  }
  logFile = SD.open(logPath, FILE_WRITE);
  if (!logFile) {
    Serial.printf("[sd] cannot open %s\n", logPath);
    return false;
  }
  sdOk = true;
  Serial.printf("[sd] logging to %s\n", logPath);
  return true;
}

static void sdInit() {
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!sdMount(5)) {
    Serial.println("[sd] mount failed");
    return;
  }
  Serial.printf("[sd] card %llu MB\n", SD.cardSize() / (1024 * 1024));
  sdOpenNew();
}

// Card lost (or never mounted): retry every SD_RECOVER_MS. On success, logging
// resumes in a new file that starts with the session metadata.
static void sdRecover() {
  static uint32_t lastAttempt = 0;
  if (sdOk || millis() - lastAttempt < SD_RECOVER_MS) return;
  lastAttempt = millis();
  logFile.close();
  SD.end();
  if (!sdMount(1)) return;
  if (!sdOpenNew()) return;
  char t[60];
  logSentence(t, snprintf(t, sizeof(t), "$PMMK,RESUMED,failures=%u", sdFailures), sizeof(t));
  for (int i = 0; i < nInfo; i++) logWrite((const uint8_t *)infoLines[i], strlen(infoLines[i]));
  Serial.println("[sd] recovered");
}

// BOOT button (GPIO0, usable once booted): a short press closes the current
// log and starts a new one, to split sessions without unplugging.
static uint32_t ledAckUntil = 0;  // LED acknowledgement blink window

static void newSession() {
  if (sdOk) {
    char t[40];
    logSentence(t, snprintf(t, sizeof(t), "$PMMK,SESSION_END"), sizeof(t));
    logFile.close();
  }
  sdOk = false;
  if (!sdOpenNew()) return;  // sdRecover() retries if the card is gone
  char t[40];
  logSentence(t, snprintf(t, sizeof(t), "$PMMK,SESSION_START,button"), sizeof(t));
  for (int i = 0; i < nInfo; i++) logWrite((const uint8_t *)infoLines[i], strlen(infoLines[i]));
  ledAckUntil = millis() + 1500;
  Serial.println("[session] new log started by button");
}

static void pollButton() {
  static bool wasDown = false;
  static uint32_t downSince = 0, lastPress = 0;
  bool down = digitalRead(SESSION_BUTTON_PIN) == LOW;
  if (down && !wasDown) downSince = millis();
  // Act on release after >= 50 ms (debounce), at most once every 3 s.
  if (!down && wasDown && millis() - downSince >= 50 && millis() - lastPress > 3000) {
    lastPress = millis();
    newSession();
  }
  wasDown = down;
}

// ---------------------------------------------------------------------------
// NMEA line filter: only complete sentences with a valid checksum reach the
// SD card. A loose wire floods the UART with noise, which must not be logged.
// ---------------------------------------------------------------------------

static char nmeaLine[128];
static size_t nmeaLen = 0;
static uint32_t lastValidSentence = 0;  // millis() of the last valid sentence
static uint32_t rmcCount = 0;           // one RMC per navigation epoch
static uint32_t nmeaStartMs = 0;        // millis() when the current sentence's '$' was read

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static bool nmeaChecksumOk(const char *line, size_t len) {
  // $<body>*HH
  if (len < 4 || line[0] != '$') return false;
  uint8_t sum = 0;
  size_t i = 1;
  for (; i < len && line[i] != '*'; i++) sum ^= (uint8_t)line[i];
  if (i + 3 != len) return false;
  int hi = hexVal(line[i + 1]), lo = hexVal(line[i + 2]);
  return hi >= 0 && lo >= 0 && sum == (uint8_t)(hi << 4 | lo);
}

static void nmeaFeed(char c) {
  if (c == '$') {  // start of sentence: drop any unfinished one
    nmeaStartMs = millis();
    nmeaLen = 0;
    nmeaLine[nmeaLen++] = c;
    return;
  }
  if (nmeaLen == 0) return;  // outside a sentence (noise, UBX binary)
  if (c == '\r') return;
  if (c == '\n') {
    if (nmeaChecksumOk(nmeaLine, nmeaLen)) {
      lastValidSentence = millis();
      if (nmeaLen > 6 && memcmp(nmeaLine + 3, "RMC", 3) == 0) {
        rmcCount++;
        // Reception time of this epoch, to align IMU samples with GPS time.
        char t[40];
        logSentence(t, snprintf(t, sizeof(t), "$PTRX,%u", nmeaStartMs), sizeof(t));
      }
      nmeaLine[nmeaLen] = '\r';  // room left: nmeaLen < sizeof(nmeaLine) - 1
      nmeaLine[nmeaLen + 1] = '\n';
      logWrite((const uint8_t *)nmeaLine, nmeaLen + 2);
    }
    nmeaLen = 0;
    return;
  }
  if (c < 0x20 || c > 0x7E || nmeaLen >= sizeof(nmeaLine) - 2) {
    nmeaLen = 0;  // not text: noise
    return;
  }
  nmeaLine[nmeaLen++] = c;
}

static bool gpsSilent() { return millis() - lastValidSentence > GPS_SILENT_MS; }

// GPS silent for a while (wire came loose, GPS rebooted on factory settings):
// probe and reconfigure it, retrying every GPS_RECOVER_MS.
static void gpsWatchdog() {
  static uint32_t lastAttempt = 0;
  static bool reported = false;
  if (!gpsSilent()) {
    if (reported) Serial.println("[gps] back");
    reported = false;
    return;
  }
  if (!reported) {
    Serial.printf("[gps] silent for %u ms\n", millis() - lastValidSentence);
    reported = true;
  }
  if (millis() - lastAttempt < GPS_RECOVER_MS) return;
  if (sdOk) logFile.flush();  // probing blocks for a few seconds
  Serial.println("[gps] reconfiguring");
  if (gpsConfigure()) lastValidSentence = millis();
  lastAttempt = millis();
}

// ---------------------------------------------------------------------------
// Clock: set the system time from the first GPS fix so FAT timestamps are
// right, and rename the log after the boot time (local time), e.g.
// /2026-10-14_13-51-20.nmea. FAT forbids ':' in names.
// ---------------------------------------------------------------------------

static void syncClockAndRename() {
  if (clockSet) return;
  // Date/time are also sent before a fix and can be wrong then: require a fix.
  if (!gps.location.isValid() || gps.location.age() > 500 || !gps.date.isValid() ||
      gps.date.year() < 2024 || gps.time.age() > 500)
    return;

  struct tm utc = {};
  utc.tm_year = gps.date.year() - 1900;
  utc.tm_mon = gps.date.month() - 1;
  utc.tm_mday = gps.date.day();
  utc.tm_hour = gps.time.hour();
  utc.tm_min = gps.time.minute();
  utc.tm_sec = gps.time.second();
  setenv("TZ", "UTC0", 1);  // mktime() works in local time: convert as UTC
  tzset();
  time_t epoch = mktime(&utc);
  setenv("TZ", LOG_TZ, 1);
  tzset();

  // Fix time + time elapsed since that sentence was parsed.
  int64_t us = (int64_t)gps.time.centisecond() * 10000 + (int64_t)gps.time.age() * 1000;
  struct timeval tv = {(time_t)(epoch + us / 1000000), (suseconds_t)(us % 1000000)};
  settimeofday(&tv, nullptr);
  clockSet = true;

  time_t boot = epoch - (time_t)(millis() / 1000);
  struct tm local;
  localtime_r(&boot, &local);
  char newPath[40];
  strftime(newPath, sizeof(newPath), "/%Y-%m-%d_%H-%M-%S.nmea", &local);
  Serial.printf("[clock] set from GPS: %04d-%02d-%02d %02d:%02d:%02d UTC\n", gps.date.year(), gps.date.month(),
                gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());

  if (!sdOk) return;
  logFile.close();
  if (!SD.exists(newPath) && SD.rename(logPath, newPath)) {
    Serial.printf("[sd] renamed %s -> %s\n", logPath, newPath);
    strlcpy(logPath, newPath, sizeof(logPath));
  } else {
    Serial.printf("[sd] rename to %s failed, keeping %s\n", newPath, logPath);
  }
  logFile = SD.open(logPath, FILE_APPEND);
  if (!logFile) {
    Serial.printf("[sd] cannot reopen %s\n", logPath);
    sdOk = false;
  }
}

// ---------------------------------------------------------------------------
// Status LED: fast blink = SD error, double blink = no data from GPS,
// slow blink = waiting for fix, on = logging with fix
// ---------------------------------------------------------------------------

static void updateLed() {
  uint32_t now = millis();
  uint32_t phase = now % 1000;
  bool on;
  if (now < ledAckUntil) on = (now / 60) % 2;  // new session acknowledged: flicker
  else if (!sdOk) on = (now / 100) % 2;
  else if (gpsSilent()) on = phase < 100 || (phase >= 200 && phase < 300);
  else if (!gps.location.isValid() || gps.location.age() > 2000) on = (now / 500) % 2;
  else on = true;
  digitalWrite(LED_PIN, on);
}

static void printStatus() {
  static uint32_t last = 0;
  static uint32_t lastRmc = 0;
  if (millis() - last < 1000) return;
  last = millis();
  float hz = rmcCount - lastRmc;  // RMC per second = navigation rate
  lastRmc = rmcCount;
  static uint32_t lastImu = 0;
  uint32_t imuHz = imuSamples() - lastImu;
  lastImu = imuSamples();
  Serial.printf("[status] fix=%d hz=%.0f sats=%u hdop=%.1f speed=%.1fkm/h imu=%s/%uHz err=%u drop=%u logged=%uB\n",
                gps.location.isValid() && gps.location.age() < 2000, hz, gps.satellites.value(),
                gps.hdop.hdop(), gps.speed.kmph(), imuOk ? "ok" : "off", imuHz, imuErrors(), imuDropped(),
                bytesLogged);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SESSION_BUTTON_PIN, INPUT_PULLUP);
  delay(200);
  Serial.println("\n=== moto-metrik ===");
  setenv("TZ", LOG_TZ, 1);
  tzset();
  sdInit();
  gpsInit();
#if IMU_ENABLED
  imuOk = imuBegin();
  char info[120];
  snprintf(info, sizeof(info), "IMU,%s,rate_hz=%d,accel_lsb_per_g=%d,gyro_lsb_per_dps=%.1f", imuOk ? "ICM20948" : "none",
           IMU_RATE_HZ, IMU_ACCEL_LSB_PER_G, IMU_GYRO_LSB_PER_DPS);
  logInfo(info);
#endif
  lastValidSentence = millis();  // grace period before the watchdog kicks in
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
    nmeaFeed(c);
  }

  ImuSample m;
  while (imuPop(m)) {
    char t[80];
    int n = snprintf(t, sizeof(t), "$PIMU,%u,%d,%d,%d,%d,%d,%d,%d", m.ms, m.ax, m.ay, m.az, m.gx, m.gy, m.gz, m.temp);
    logSentence(t, n, sizeof(t));
  }

  if (sdOk && millis() - lastFlush >= LOG_FLUSH_MS) {
    logFile.flush();
    lastFlush = millis();
  }
  sdRecover();
  pollButton();

  gpsWatchdog();
  syncClockAndRename();
  updateLed();
  printStatus();
}
