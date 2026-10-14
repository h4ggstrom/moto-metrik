#pragma once

// ---------------------------------------------------------------------------
// GPS (UART2)
// RX0/TX0 (GPIO3/GPIO1) are wired to the USB-serial chip: a GPS there fights
// with the USB link (upload failures, garbage on the monitor). Use UART2.
//   GPS TX -> ESP32 GPIO16 (RX2)
//   GPS RX -> ESP32 GPIO17 (TX2)
// ---------------------------------------------------------------------------
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Target settings pushed to u-blox modules (ignored by other chipsets).
#define GPS_TARGET_BAUD 115200
#define GPS_RATE_MS 100  // 10 Hz (NEO-6M caps at 5 Hz -> it will reject and keep 1 Hz; use 200)

// ---------------------------------------------------------------------------
// SD card (SPI) - native VSPI pins; GPIO21/22 stay free for the IMU (I2C)
// ---------------------------------------------------------------------------
#define SD_SCK_PIN 18
#define SD_MISO_PIN 19
#define SD_MOSI_PIN 23
#define SD_CS_PIN 5
#define SD_SPI_FREQ 4000000

// Flush the log file this often so a power cut loses at most this much data.
#define LOG_FLUSH_MS 1000

// ---------------------------------------------------------------------------
// Status LED (onboard, GPIO2 on most devkits)
// ---------------------------------------------------------------------------
#define LED_PIN 2
