#include "can.h"

#include <driver/gpio.h>
#include <driver/twai.h>

#include "config.h"

static QueueHandle_t queue = nullptr;
static volatile uint32_t bitrate = 0;
static volatile uint32_t nFrames = 0, nDropped = 0;

// Last logged time per ID, for the per-ID rate limit. IDs beyond the table
// are logged unthrottled (a bike bus carries a few dozen IDs).
static uint32_t seenId[128];
static uint32_t seenMs[128];
static int nSeen = 0;

static bool rateLimited(uint32_t key, uint32_t ms) {
  for (int i = 0; i < nSeen; i++) {
    if (seenId[i] != key) continue;
    if (ms - seenMs[i] < CAN_ID_MIN_MS) return true;
    seenMs[i] = ms;
    return false;
  }
  if (nSeen < 128) {
    seenId[nSeen] = key;
    seenMs[nSeen++] = ms;
  }
  return false;
}

static bool install(uint32_t rate) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN,
                                                        TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = 64;
  twai_timing_config_t t;
  switch (rate) {
    case 1000000: t = TWAI_TIMING_CONFIG_1MBITS(); break;
    case 500000: t = TWAI_TIMING_CONFIG_500KBITS(); break;
    case 250000: t = TWAI_TIMING_CONFIG_250KBITS(); break;
    default: t = TWAI_TIMING_CONFIG_125KBITS(); break;
  }
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  // No transceiver fitted: hold RX recessive so the controller sees an idle bus.
  gpio_pullup_en((gpio_num_t)CAN_RX_PIN);
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  return true;
}

static void uninstall() {
  twai_stop();
  twai_driver_uninstall();
}

static void canTask(void *) {
  static const uint32_t rates[] = {500000, 1000000, 250000, 125000};
  int r = 0;
  // Search: listen 1 s at each rate; a wrong rate gives bus errors, no frames.
  for (;;) {
    if (!install(rates[r])) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    twai_message_t msg;
    int good = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 1000 && good < 5)
      if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) good++;
    if (good >= 5) break;
    uninstall();
    r = (r + 1) % (sizeof(rates) / sizeof(rates[0]));
  }
  bitrate = rates[r];
  Serial.printf("[can] locked at %u bit/s (listen-only)\n", bitrate);

  // Keep the rate for good: a silent bus just means the bike is off.
  for (;;) {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(1000)) != ESP_OK) continue;
    if (msg.rtr) continue;
    nFrames++;
    CanFrame f;
    f.ms = millis();
    f.id = msg.identifier;
    f.ext = msg.extd;
    f.dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
    memcpy(f.data, msg.data, f.dlc);
    if (rateLimited(f.id | (f.ext ? 0x80000000u : 0), f.ms)) continue;
    if (xQueueSend(queue, &f, 0) != pdTRUE) nDropped++;
  }
}

bool canBegin() {
#if CAN_ENABLED
  queue = xQueueCreate(CAN_QUEUE_LEN, sizeof(CanFrame));
  if (!queue) return false;
  xTaskCreatePinnedToCore(canTask, "can", 4096, nullptr, 4, nullptr, 0);
  return true;
#else
  return false;
#endif
}

bool canPop(CanFrame &f) { return queue && xQueueReceive(queue, &f, 0) == pdTRUE; }
uint32_t canBitrate() { return bitrate; }
uint32_t canFrames() { return nFrames; }
uint32_t canDropped() { return nDropped; }
