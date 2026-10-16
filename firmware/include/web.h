#pragma once

#include <Arduino.h>

// Paddock Wi-Fi: access point + captive DNS + HTTP server with the session
// browser page (tools/web, bundled by gen_web.py) and a small JSON API.
// Only runs while the logger is idle: serving files blocks the SD card.

struct WebStatus {
  bool fix;
  int sats;
  int hz;
  bool imu;
  bool sd;
  uint32_t freeMb;
  bool logging;
};

struct WebHooks {
  WebStatus (*status)();
  void (*startSession)();  // manual start from the page
};

void webStart(const WebHooks &hooks);
void webStop();
void webLoop();
bool webActive();
