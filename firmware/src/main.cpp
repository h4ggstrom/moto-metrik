// moto-metrik: GPS logger.
// Raw NMEA from the GPS is written verbatim to the SD card (one file per boot).
// All analysis (laps, traces) happens offline with tools/analyze.py.

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <sys/time.h>
#include <time.h>

#include "config.h"

static HardwareSerial &gpsSerial = Serial2;
static SPIClass sdSpi(VSPI);
static TinyGPSPlus gps;

static File logFile;
static char logPath[40];
static bool sdOk = false;
static bool clockSet = false;
static uint32_t lastFlush = 0;
static uint32_t bytesLogged = 0;

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

static void ubxDisableNmea(uint8_t msgId) {
  const uint8_t p[] = {0xF0, msgId, 0};
  ubxSend(0x06, 0x01, p, sizeof(p));  // CFG-MSG
}

static void ubxSetRate(uint16_t ms) {
  const uint8_t p[] = {(uint8_t)(ms & 0xFF), (uint8_t)(ms >> 8), 1, 0, 1, 0};
  ubxSend(0x06, 0x08, p, sizeof(p));  // CFG-RATE
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

  // Keep only RMC (position/speed/course/date) and GGA (fix/sats/hdop/alt).
  ubxDisableNmea(0x01);  // GLL
  ubxDisableNmea(0x02);  // GSA
  ubxDisableNmea(0x03);  // GSV
  ubxDisableNmea(0x05);  // VTG

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

static void sdInit() {
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  // After a soft reset (flash, brownout) the card keeps power and can be stuck
  // mid-transaction: retry, it usually answers on the second attempt.
  bool mounted = false;
  for (int attempt = 1; attempt <= 5 && !mounted; attempt++) {
    mounted = SD.begin(SD_CS_PIN, sdSpi, SD_SPI_FREQ);
    if (!mounted) {
      Serial.printf("[sd] mount attempt %d failed\n", attempt);
      SD.end();
      delay(200);
    }
  }
  if (!mounted) {
    Serial.println("[sd] mount failed");
    return;
  }
  Serial.printf("[sd] card %llu MB\n", SD.cardSize() / (1024 * 1024));

  // Temporary name until the GPS gives the date (see syncClockAndRename).
  for (int i = 1; i < 10000; i++) {
    snprintf(logPath, sizeof(logPath), "/log_%04d.nmea", i);
    if (!SD.exists(logPath)) break;
  }
  logFile = SD.open(logPath, FILE_WRITE);
  if (!logFile) {
    Serial.printf("[sd] cannot open %s\n", logPath);
    return;
  }
  sdOk = true;
  Serial.printf("[sd] logging to %s\n", logPath);
}

// ---------------------------------------------------------------------------
// NMEA line filter: only complete sentences with a valid checksum reach the
// SD card. A loose wire floods the UART with noise, which must not be logged.
// ---------------------------------------------------------------------------

static char nmeaLine[128];
static size_t nmeaLen = 0;
static uint32_t lastValidSentence = 0;  // millis() of the last valid sentence

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
    nmeaLen = 0;
    nmeaLine[nmeaLen++] = c;
    return;
  }
  if (nmeaLen == 0) return;  // outside a sentence (noise, UBX binary)
  if (c == '\r') return;
  if (c == '\n') {
    if (nmeaChecksumOk(nmeaLine, nmeaLen)) {
      lastValidSentence = millis();
      if (sdOk) {
        logFile.write((const uint8_t *)nmeaLine, nmeaLen);
        logFile.write((const uint8_t *)"\r\n", 2);
        bytesLogged += nmeaLen + 2;
      }
    }
    nmeaLen = 0;
    return;
  }
  if (c < 0x20 || c > 0x7E || nmeaLen >= sizeof(nmeaLine)) {
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
  if (!sdOk) on = (now / 100) % 2;
  else if (gpsSilent()) on = phase < 100 || (phase >= 200 && phase < 300);
  else if (!gps.location.isValid() || gps.location.age() > 2000) on = (now / 500) % 2;
  else on = true;
  digitalWrite(LED_PIN, on);
}

static void printStatus() {
  static uint32_t last = 0;
  static uint32_t lastSentences = 0;
  if (millis() - last < 1000) return;
  last = millis();
  // RMC + GGA per epoch -> sentences/2 = update rate
  uint32_t sentences = gps.passedChecksum();
  float hz = (sentences - lastSentences) / 2.0f;
  lastSentences = sentences;
  Serial.printf("[status] fix=%d hz=%.0f sats=%u hdop=%.1f lat=%.6f lon=%.6f speed=%.1fkm/h logged=%uB chk_fail=%u\n",
                gps.location.isValid() && gps.location.age() < 2000, hz, gps.satellites.value(),
                gps.hdop.hdop(), gps.location.lat(), gps.location.lng(), gps.speed.kmph(),
                bytesLogged, gps.failedChecksum());
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(200);
  Serial.println("\n=== moto-metrik ===");
  setenv("TZ", LOG_TZ, 1);
  tzset();
  sdInit();
  gpsInit();
  lastValidSentence = millis();  // grace period before the watchdog kicks in
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
    nmeaFeed(c);
  }

  if (sdOk && millis() - lastFlush >= LOG_FLUSH_MS) {
    logFile.flush();
    lastFlush = millis();
  }

  gpsWatchdog();
  syncClockAndRename();
  updateLed();
  printStatus();
}
