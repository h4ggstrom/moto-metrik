#pragma once

#include <Arduino.h>

// ICM-20948 sampled at IMU_RATE_HZ by a FreeRTOS task on core 0. Samples are
// queued with their millis() timestamp; the main loop drains them to the log.
// If the IMU is missing or fails, the task simply produces nothing: GPS logging
// never depends on it.

struct ImuSample {
  uint32_t ms;
  int16_t ax, ay, az;  // raw, IMU_ACCEL_LSB_PER_G
  int16_t gx, gy, gz;  // raw, IMU_GYRO_LSB_PER_DPS
  int16_t temp;        // raw: degC = (temp - 21) / 333.87 + 21
};

// Probe and configure the IMU, start the sampling task. Returns false if the
// IMU did not answer (nothing is started then).
bool imuBegin();

// Pop one queued sample; false when the queue is empty.
bool imuPop(ImuSample &s);

// Counters for the status line.
uint32_t imuSamples();
uint32_t imuErrors();
uint32_t imuDropped();
