// moto-metrik: GPS logger.
// Raw NMEA from the GPS is written verbatim to the SD card (one file per boot).
// All analysis (laps, traces) happens offline with tools/analyze.py.

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>

#include "config.h"

static HardwareSerial &gpsSerial = Serial2;
static SPIClass sdSpi(VSPI);
static TinyGPSPlus gps;

static File logFile;
static bool sdOk = false;
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

static void ubxDisableNmea(uint8_t msgId) {
  const uint8_t p[] = {0xF0, msgId, 0};
  ubxSend(0x06, 0x01, p, sizeof(p));  // CFG-MSG
}

static void ubxSetRate(uint16_t ms) {
  const uint8_t p[] = {(uint8_t)(ms & 0xFF), (uint8_t)(ms >> 8), 1, 0, 1, 0};
  ubxSend(0x06, 0x08, p, sizeof(p));  // CFG-RATE
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

static void gpsInit() {
  gpsSerial.setRxBufferSize(4096);  // absorbs SD write latency spikes
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  uint32_t baud = gpsProbeBaud();
  if (!baud) {
    Serial.println("[gps] no NMEA on any baud rate - check wiring (GPS TX -> GPIO16)");
    gpsSerial.updateBaudRate(9600);
    return;
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

  ubxSetRate(GPS_RATE_MS);
}

// ---------------------------------------------------------------------------
// SD logging
// ---------------------------------------------------------------------------

static void sdInit() {
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, sdSpi, SD_SPI_FREQ)) {
    Serial.println("[sd] mount failed");
    return;
  }
  Serial.printf("[sd] card %llu MB\n", SD.cardSize() / (1024 * 1024));

  char path[24];
  for (int i = 1; i < 10000; i++) {
    snprintf(path, sizeof(path), "/log_%04d.nmea", i);
    if (!SD.exists(path)) break;
  }
  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) {
    Serial.printf("[sd] cannot open %s\n", path);
    return;
  }
  sdOk = true;
  Serial.printf("[sd] logging to %s\n", path);
}

// ---------------------------------------------------------------------------
// Status LED: fast blink = SD error, slow blink = waiting for fix, on = logging with fix
// ---------------------------------------------------------------------------

static void updateLed() {
  uint32_t now = millis();
  bool on;
  if (!sdOk) on = (now / 100) % 2;
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
  sdInit();
  gpsInit();
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
    if (sdOk) {
      logFile.write((uint8_t)c);
      bytesLogged++;
    }
  }

  if (sdOk && millis() - lastFlush >= LOG_FLUSH_MS) {
    logFile.flush();
    lastFlush = millis();
  }

  updateLed();
  printStatus();
}
