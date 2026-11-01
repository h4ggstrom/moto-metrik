#pragma once

#include <Arduino.h>

// CAN bus sniffer (ESP32 TWAI + SN65HVD230 transceiver), strictly listen-only:
// the controller never transmits, not even ACK or error frames, so it cannot
// disturb the bike's bus. The bit rate is found by trying the usual ones until
// valid frames arrive (the bike may be switched on long after the box). Frames
// are queued with their millis() timestamp; the main loop drains them to the
// log. Each ID is kept at most every CAN_ID_MIN_MS to bound the log size.

struct CanFrame {
  uint32_t ms;
  uint32_t id;
  bool ext;  // 29-bit identifier
  uint8_t dlc;
  uint8_t data[8];
};

// Start the sniffer task. Returns false if CAN is disabled in config.h.
bool canBegin();

// Pop one queued frame; false when the queue is empty.
bool canPop(CanFrame &f);

// Locked bit rate in bit/s, 0 while still searching.
uint32_t canBitrate();

// Counters for the status line.
uint32_t canFrames();   // received (before the per-ID rate limit)
uint32_t canDropped();  // lost because the queue was full
