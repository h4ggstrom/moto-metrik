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
#define GPS_DYN_MODEL 4  // u-blox CFG-NAV5 dynModel: 4 = automotive
#define GPS_RATE_MS 100  // 10 Hz: NEO-M8N max with concurrent GNSS
#define GPS_SBAS 0  // EGNOS corrections: off, they halve the NEO-M8N rate at 10 Hz
#define GPS_GALILEO 0  // 1 = GPS+Galileo, 0 = GPS+GLONASS. FW 3.01 never used a Galileo SV (Carole, 2026-10-16)
#define GPS_SAT_INFO_EPOCHS 10  // GSA/GSV once every N epochs (1 Hz at 10 Hz)
#define GPS_SILENT_MS 2000   // no valid sentence for this long -> GPS considered lost
#define GPS_RECOVER_MS 5000  // retry interval for re-probing/reconfiguring a lost GPS

// ---------------------------------------------------------------------------
// SD card (SPI) - native VSPI pins; GPIO21/22 stay free for the IMU (I2C)
// ---------------------------------------------------------------------------
#define SD_SCK_PIN 18
#define SD_MISO_PIN 19
#define SD_MOSI_PIN 23
#define SD_CS_PIN 5
#define SD_SPI_FREQ 4000000

// Local time zone for log file names and FAT timestamps (POSIX TZ: Paris).
#define LOG_TZ "CET-1CEST,M3.5.0,M10.5.0/3"

// Retry interval for remounting a lost SD card.
#define SD_RECOVER_MS 3000

// Flush the log file this often so a power cut loses at most this much data.
#define LOG_FLUSH_MS 1000

// ---------------------------------------------------------------------------
// IMU (ICM-20948 over I2C). Set IMU_ENABLED to 0 to log GPS only.
// ---------------------------------------------------------------------------
#define IMU_ENABLED 1
#define IMU_SDA_PIN 21
#define IMU_SCL_PIN 22
#define IMU_NCS_PIN 4     // driven high: I2C mode
#define IMU_SDO_PIN 26    // driven low: address 0x68
#define IMU_FSYNC_PIN 25  // driven low: unused
#define IMU_RATE_HZ 100
#define IMU_QUEUE_LEN 512  // 5 s of samples: covers any SD write stall
#define IMU_ACCEL_FS_SEL 2  // 0..3 = +-2/4/8/16 g
#define IMU_GYRO_FS_SEL 2   // 0..3 = +-250/500/1000/2000 dps
#define IMU_ACCEL_LSB_PER_G (16384 >> IMU_ACCEL_FS_SEL)
#define IMU_GYRO_LSB_PER_DPS (131.0f / (1 << IMU_GYRO_FS_SEL))

// ---------------------------------------------------------------------------
// Status LED (onboard, GPIO2 on most devkits)
// ---------------------------------------------------------------------------
#define LED_PIN 2

// Session button: the devkit's BOOT button. Short press = start a new log file.
#define SESSION_BUTTON_PIN 0
