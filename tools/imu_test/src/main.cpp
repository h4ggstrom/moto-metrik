// ICM-20948 diagnostic over I2C, raw registers (no library), so every step
// of the bring-up is visible.
//   VCC -> 3V3, GND -> GND, SCLK -> GPIO22 (SCL), SDI -> GPIO21 (SDA),
//   NCS -> GPIO4 (driven high: I2C mode), SDO -> GPIO26 (driven low: address
//   0x68; both addresses are scanned anyway), FSYNC -> GPIO25 (driven low)

#include <Arduino.h>
#include <Wire.h>

static const int SDA_PIN = 21, SCL_PIN = 22, NCS_PIN = 4, FSYNC_PIN = 25, SDO_PIN = 26;
static const uint8_t MAG_ADDR = 0x0C;  // AK09916, inside the ICM-20948

// Bank 0 registers
static const uint8_t WHO_AM_I = 0x00, USER_CTRL = 0x03, PWR_MGMT_1 = 0x06, PWR_MGMT_2 = 0x07,
                     INT_PIN_CFG = 0x0F, ACCEL_XOUT_H = 0x2D, REG_BANK_SEL = 0x7F;

static uint8_t imuAddr = 0;

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static int readReg(uint8_t addr, uint8_t reg) {
  uint8_t v;
  return readRegs(addr, reg, &v, 1) ? v : -1;
}

static void scanBus() {
  Serial.println("[1] I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    device at 0x%02X\n", a);
      found++;
      if (a == 0x68 || a == 0x69) imuAddr = a;
    }
  }
  if (!found) Serial.println("    nothing answers: check VCC/GND/SCL/SDA and NCS on GPIO4");
}

static bool initImu() {
  Serial.printf("[2] WHO_AM_I @0x%02X: ", imuAddr);
  writeReg(imuAddr, REG_BANK_SEL, 0x00);
  int who = readReg(imuAddr, WHO_AM_I);
  Serial.printf("0x%02X %s\n", who, who == 0xEA ? "OK (ICM-20948)" : "UNEXPECTED (expected 0xEA)");
  if (who != 0xEA) return false;

  writeReg(imuAddr, PWR_MGMT_1, 0x80);  // device reset
  delay(100);
  writeReg(imuAddr, PWR_MGMT_1, 0x01);  // wake, auto clock
  delay(50);
  writeReg(imuAddr, PWR_MGMT_2, 0x00);  // accel + gyro on
  delay(50);
  Serial.printf("    PWR_MGMT_1 after wake: 0x%02X (expect 0x01)\n", readReg(imuAddr, PWR_MGMT_1));

  // Magnetometer: expose the AK09916 on our bus (I2C master off, bypass on)
  writeReg(imuAddr, USER_CTRL, 0x00);
  writeReg(imuAddr, INT_PIN_CFG, 0x02);
  delay(10);
  int wia = readReg(MAG_ADDR, 0x01);
  Serial.printf("[5] magnetometer AK09916 WIA2: 0x%02X %s\n", wia,
                wia == 0x09 ? "OK" : "UNEXPECTED (expected 0x09) - mag dead or not exposed");
  if (wia == 0x09) writeReg(MAG_ADDR, 0x31, 0x08);  // continuous 100 Hz
  return true;
}

struct Sample {
  float ax, ay, az, gx, gy, gz, temp, mx, my, mz;
  bool magOk;
};

static bool readSample(Sample &s) {
  uint8_t b[14];
  if (!readRegs(imuAddr, ACCEL_XOUT_H, b, 14)) return false;
  auto i16 = [&](int i) { return (int16_t)(b[i] << 8 | b[i + 1]); };
  s.ax = i16(0) / 16384.0f;  // +-2 g default
  s.ay = i16(2) / 16384.0f;
  s.az = i16(4) / 16384.0f;
  s.gx = i16(6) / 131.0f;  // +-250 dps default
  s.gy = i16(8) / 131.0f;
  s.gz = i16(10) / 131.0f;
  s.temp = (i16(12) - 21) / 333.87f + 21.0f;

  uint8_t m[8];
  s.magOk = readRegs(MAG_ADDR, 0x10, m, 1) && (m[0] & 0x01) && readRegs(MAG_ADDR, 0x11, m, 8);
  if (s.magOk) {
    auto l16 = [&](int i) { return (int16_t)(m[i + 1] << 8 | m[i]); };
    s.mx = l16(0) * 0.15f;  // uT
    s.my = l16(2) * 0.15f;
    s.mz = l16(4) * 0.15f;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ICM-20948 diagnostic ===");
  pinMode(NCS_PIN, OUTPUT);
  digitalWrite(NCS_PIN, HIGH);  // NCS high = I2C interface
  pinMode(FSYNC_PIN, OUTPUT);
  digitalWrite(FSYNC_PIN, LOW);  // unused FSYNC must not float
  pinMode(SDO_PIN, OUTPUT);
  digitalWrite(SDO_PIN, LOW);  // AD0 low = address 0x68
  delay(10);
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  scanBus();
  if (!imuAddr) {
    Serial.println("    no ICM-20948 at 0x68/0x69");
    return;
  }
  if (!initImu()) return;
  Serial.println("[3/4] live data (keep the board still and flat): accel g | gyro dps | temp C | mag uT");
}

void loop() {
  if (!imuAddr) {
    delay(3000);
    scanBus();
    return;
  }
  // Average 50 samples over ~0.5 s to show mean and noise.
  const int N = 50;
  double sum[6] = {0}, sq[6] = {0};
  Sample s{};
  int ok = 0;
  for (int i = 0; i < N; i++) {
    if (readSample(s)) {
      float v[6] = {s.ax, s.ay, s.az, s.gx, s.gy, s.gz};
      for (int k = 0; k < 6; k++) { sum[k] += v[k]; sq[k] += v[k] * v[k]; }
      ok++;
    }
    delay(10);
  }
  if (!ok) {
    Serial.println("read failed");
    return;
  }
  double m[6], sd[6];
  for (int k = 0; k < 6; k++) {
    m[k] = sum[k] / ok;
    sd[k] = sqrt(fmax(0.0, sq[k] / ok - m[k] * m[k]));
  }
  double norm = sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
  Serial.printf("acc %+.3f %+.3f %+.3f |%.3f| sd %.3f %.3f %.3f  gyro %+6.2f %+6.2f %+6.2f sd %.2f %.2f %.2f  T %.1f",
                m[0], m[1], m[2], norm, sd[0], sd[1], sd[2], m[3], m[4], m[5], sd[3], sd[4], sd[5], s.temp);
  if (s.magOk) Serial.printf("  mag %+6.1f %+6.1f %+6.1f |%.1f|", s.mx, s.my, s.mz,
                             sqrt(s.mx * s.mx + s.my * s.my + s.mz * s.mz));
  Serial.println();
}
