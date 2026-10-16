#include "web.h"

#include <DNSServer.h>
#include <SD.h>
#include <WebServer.h>
#include <WiFi.h>

#include "config.h"
#include "web_page.h"

static WebServer server(80);
static DNSServer dns;
static WebHooks hooks{};
static bool active = false;
static bool routesSet = false;

static void sendPage() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", (const char *)WEB_PAGE_GZ, WEB_PAGE_GZ_LEN);
}

static void sendStatus() {
  WebStatus s = hooks.status();
  char json[200];
  snprintf(json, sizeof(json),
           "{\"fix\":%s,\"sats\":%d,\"hz\":%d,\"imu\":%s,\"sd\":%s,\"free_mb\":%u,\"logging\":%s}",
           s.fix ? "true" : "false", s.sats, s.hz, s.imu ? "true" : "false", s.sd ? "true" : "false", s.freeMb,
           s.logging ? "true" : "false");
  server.send(200, "application/json", json);
}

static void sendFiles() {
  File root = SD.open("/");
  if (!root) {
    server.send(503, "text/plain", "SD not mounted");
    return;
  }
  String json = "[";
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    String name = f.name();
    if (!f.isDirectory() && name.endsWith(".nmea")) {
      if (json.length() > 1) json += ",";
      json += "{\"name\":\"" + name + "\",\"size\":" + String((uint32_t)f.size()) + "}";
    }
    f.close();
  }
  root.close();
  server.send(200, "application/json", json + "]");
}

static void sendFile() {
  String name = server.arg("name");
  // Plain file names in the root only: no path traversal.
  if (name.isEmpty() || name.indexOf('/') >= 0 || name.indexOf("..") >= 0 || !name.endsWith(".nmea")) {
    server.send(400, "text/plain", "bad name");
    return;
  }
  File f = SD.open("/" + name);
  if (!f) {
    server.send(404, "text/plain", "not found");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  server.streamFile(f, "text/plain");
  f.close();
}

static void startSession() {
  server.send(200, "text/plain", "ok");
  delay(200);  // let the answer leave before the Wi-Fi goes down
  hooks.startSession();
}

void webStart(const WebHooks &h) {
  if (active) return;
  hooks = h;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WEB_SSID, WEB_PASSWORD);
  dns.start(53, "*", WiFi.softAPIP());  // captive portal: every name resolves to us
  if (!routesSet) {
    server.on("/", sendPage);
    server.on("/api/status", sendStatus);
    server.on("/api/files", sendFiles);
    server.on("/api/file", sendFile);
    server.on("/api/session/new", HTTP_POST, startSession);
    server.onNotFound(sendPage);  // captive-portal probes land on the page
    routesSet = true;
  }
  server.begin();
  active = true;
  Serial.printf("[web] Wi-Fi \"%s\" up, http://%s\n", WEB_SSID, WiFi.softAPIP().toString().c_str());
}

void webStop() {
  if (!active) return;
  server.stop();
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  active = false;
  Serial.println("[web] Wi-Fi off");
}

void webLoop() {
  if (!active) return;
  dns.processNextRequest();
  server.handleClient();
}

bool webActive() { return active; }
