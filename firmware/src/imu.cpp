#include "imu.h"

#include <Wire.h>

#include "config.h"

// ICM-20948 registers
static const uint8_t REG_BANK_SEL = 0x7F;
// bank 0
static const uint8_t WHO_AM_I = 0x00, PWR_MGMT_1 = 0x06, PWR_MGMT_2 = 0x07, ACCEL_XOUT_H = 0x2D;
// bank 2
static const uint8_t GYRO_SMPLRT_DIV = 0x00, GYRO_CONFIG_1 = 0x01, ACCEL_SMPLRT_DIV_2 = 0x11,
                     ACCEL_CONFIG = 0x14;

static uint8_t addr = 0;
static QueueHandle_t queue = nullptr;
static volatile uint32_t nSamples = 0, nErrors = 0, nDropped = 0;

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool bank(uint8_t b) { return writeReg(REG_BANK_SEL, b << 4); }

static bool configure() {
  bool ok = bank(0);
  ok &= writeReg(PWR_MGMT_1, 0x80);  // reset
  delay(100);
  ok &= writeReg(PWR_MGMT_1, 0x01);  // wake, auto clock
  delay(20);
  ok &= writeReg(PWR_MGMT_2, 0x00);  // accel + gyro on
  ok &= bank(2);
  // Internal ODR ~220 Hz, DLPF ~50 Hz: anti-aliasing for 100 Hz reads.
  ok &= writeReg(GYRO_SMPLRT_DIV, 4);                                      // 1.1 kHz / 5
  ok &= writeReg(GYRO_CONFIG_1, (3 << 3) | (IMU_GYRO_FS_SEL << 1) | 0x01);  // DLPF 51 Hz
  ok &= writeReg(ACCEL_SMPLRT_DIV_2, 4);                                   // 1.125 kHz / 5
  ok &= writeReg(ACCEL_CONFIG, (3 << 3) | (IMU_ACCEL_FS_SEL << 1) | 0x01);  // DLPF 50 Hz
  ok &= bank(0);
  return ok;
}

static void task(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / IMU_RATE_HZ);
  TickType_t wake = xTaskGetTickCount();
  uint8_t b[14];
  for (;;) {
    vTaskDelayUntil(&wake, period);
    uint32_t ms = millis();
    if (!readRegs(ACCEL_XOUT_H, b, sizeof(b))) {
      nErrors++;
      continue;
    }
    auto i16 = [&](int i) { return (int16_t)(b[i] << 8 | b[i + 1]); };
    ImuSample s{ms, i16(0), i16(2), i16(4), i16(6), i16(8), i16(10), i16(12)};
    if (xQueueSend(queue, &s, 0) != pdTRUE) nDropped++;  // logger stalled: drop, never block
    else nSamples++;
  }
}

bool imuBegin() {
  pinMode(IMU_NCS_PIN, OUTPUT);
  digitalWrite(IMU_NCS_PIN, HIGH);  // NCS high = I2C interface
  pinMode(IMU_SDO_PIN, OUTPUT);
  digitalWrite(IMU_SDO_PIN, LOW);  // AD0 low = 0x68
  pinMode(IMU_FSYNC_PIN, OUTPUT);
  digitalWrite(IMU_FSYNC_PIN, LOW);
  delay(10);
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 400000);
  Wire.setTimeOut(20);

  for (uint8_t a : {0x68, 0x69}) {
    addr = a;
    uint8_t who = 0;
    if (bank(0) && readRegs(WHO_AM_I, &who, 1) && who == 0xEA) break;
    addr = 0;
  }
  if (!addr) return false;
  if (!configure()) return false;

  queue = xQueueCreate(IMU_QUEUE_LEN, sizeof(ImuSample));
  if (!queue) return false;
  xTaskCreatePinnedToCore(task, "imu", 4096, nullptr, 5, nullptr, 0);
  return true;
}

bool imuPop(ImuSample &s) { return queue && xQueueReceive(queue, &s, 0) == pdTRUE; }

uint32_t imuSamples() { return nSamples; }
uint32_t imuErrors() { return nErrors; }
uint32_t imuDropped() { return nDropped; }
