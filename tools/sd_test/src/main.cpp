// SD card wiring diagnostic: raw SPI, no filesystem.
//   SD VCC -> VIN, GND -> GND, MISO -> GPIO19, MOSI -> GPIO23, SCK -> GPIO18, CS -> GPIO5
// Step 1 checks power + CS + MISO with pin levels only; step 2 sends CMD0
// (go idle) and CMD8, which exercises SCK and MOSI.

#include <Arduino.h>
#include <SPI.h>

static const int MISO_PIN = 19, MOSI_PIN = 23, SCK_PIN = 18, CS_PIN = 5;

static void levelTest() {
  Serial.println("[1] pin levels (MISO with internal pull-down):");
  pinMode(CS_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT_PULLDOWN);
  digitalWrite(CS_PIN, HIGH);
  delay(5);
  int hi = digitalRead(MISO_PIN);
  digitalWrite(CS_PIN, LOW);
  delay(5);
  int lo = digitalRead(MISO_PIN);
  digitalWrite(CS_PIN, HIGH);
  Serial.printf("    CS high -> MISO=%d (expect 0: card output off)\n", hi);
  Serial.printf("    CS low  -> MISO=%d (expect 1: card drives its idle-high output)\n", lo);
  if (hi == 0 && lo == 1) Serial.println("    OK: module powered, CS and MISO wired");
  else if (hi == 0 && lo == 0) Serial.println("    FAIL: nothing on MISO -> check VCC/GND, CS (GPIO5), MISO (GPIO19), card inserted");
  else Serial.println("    ODD: MISO high with CS high -> MISO stuck high or another device on the line");
}

static uint8_t command(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t *extra = nullptr, int nExtra = 0) {
  SPI.transfer(0xFF);
  SPI.transfer(0x40 | cmd);
  for (int s = 24; s >= 0; s -= 8) SPI.transfer((arg >> s) & 0xFF);
  SPI.transfer(crc);
  uint8_t r = 0xFF;
  for (int i = 0; i < 10 && r == 0xFF; i++) r = SPI.transfer(0xFF);
  for (int i = 0; i < nExtra; i++) extra[i] = SPI.transfer(0xFF);
  return r;
}

static void cmdTest() {
  Serial.println("[2] CMD0 / CMD8 at 250 kHz:");
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, HIGH);
  for (int i = 0; i < 10; i++) SPI.transfer(0xFF);  // >= 74 clocks with CS high
  digitalWrite(CS_PIN, LOW);
  uint8_t r0 = 0xFF;
  for (int tries = 0; tries < 5 && r0 != 0x01; tries++) r0 = command(0, 0, 0x95);
  uint8_t ext[4] = {0};
  uint8_t r8 = command(8, 0x1AA, 0x87, ext, 4);
  digitalWrite(CS_PIN, HIGH);
  SPI.transfer(0xFF);
  SPI.endTransaction();
  Serial.printf("    CMD0 -> 0x%02X (expect 0x01 = idle)\n", r0);
  Serial.printf("    CMD8 -> 0x%02X %02X %02X %02X %02X (expect 0x01 .. 01 AA)\n", r8, ext[0], ext[1], ext[2], ext[3]);
  if (r0 == 0x01) Serial.println("    OK: card answers, SCK and MOSI wired");
  else if (r0 == 0xFF) Serial.println("    FAIL: no answer -> SCK (GPIO18) or MOSI (GPIO23), or card/power");
  else if (r0 == 0x00) Serial.println("    FAIL: MISO stuck low -> MISO (GPIO19) or GND");
  else Serial.println("    ODD answer: bad contact or noise on SCK/MOSI");
  SPI.end();
}

void setup() {
  Serial.begin(115200);
  delay(300);
}

// Many SD modules have 10k pull-ups on every line: an ESP32 pin read with its
// weak internal pull-down then reads 1 if the wire reaches the module, 0 if not.
static void continuityTest() {
  Serial.println("[0] continuity (pin as input with pull-down; 1 = wire reaches a pull-up on the module):");
  const struct { const char *name; int pin; } pins[] = {
      {"MISO GPIO19", MISO_PIN}, {"MOSI GPIO23", MOSI_PIN}, {"SCK  GPIO18", SCK_PIN}, {"CS   GPIO5 ", CS_PIN}};
  for (auto &p : pins) {
    pinMode(p.pin, INPUT_PULLDOWN);
    delay(5);
    Serial.printf("    %s -> %d\n", p.name, digitalRead(p.pin));
  }
}

void loop() {
  Serial.println("\n=== SD wiring diagnostic ===");
  continuityTest();
  levelTest();
  cmdTest();
  delay(3000);
}
