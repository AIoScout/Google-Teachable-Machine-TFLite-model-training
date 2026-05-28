#include <Arduino.h>

#include "model_settings.h"

static constexpr int kDebugBaud = 115200;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 12;
static constexpr int kUartTxPin = 13;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;

#define UartFromP4 Serial1
#define DBG Serial0

struct Packet {
  uint8_t msg_type;
  uint16_t frame_id;
  uint8_t label_id;
  uint8_t confidence;
  uint8_t flags;
};

static uint32_t s_bytes_rx = 0;
static uint32_t s_packets_ok = 0;

static bool read_packet(Packet *out) {
  static uint8_t buf[8];
  static uint8_t idx = 0;
  static uint8_t state = 0;

  while (UartFromP4.available() > 0) {
    const uint8_t b = (uint8_t)UartFromP4.read();
    s_bytes_rx++;

    if (state == 0) {
      if (b == kSync0) {
        buf[0] = b;
        idx = 1;
        state = 1;
      }
      continue;
    }

    if (state == 1) {
      if (b == kSync1) {
        buf[1] = b;
        idx = 2;
        state = 2;
      } else if (b == kSync0) {
        buf[0] = b;
        idx = 1;
        state = 1;
      } else {
        state = 0;
      }
      continue;
    }

    buf[idx++] = b;
    if (idx < sizeof(buf)) {
      continue;
    }

    state = 0;
    idx = 0;

    if (buf[2] != kMsgTypeInference) {
      return false;
    }

    out->msg_type = buf[2];
    out->frame_id = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    out->label_id = buf[5];
    out->confidence = buf[6];
    out->flags = buf[7];
    return true;
  }

  return false;
}

void setup() {
  DBG.begin(kDebugBaud);
  const uint32_t t0 = millis();
  while (!DBG && (millis() - t0) < 3000) {
    delay(10);
  }
  delay(100);
  DBG.println("S3 UART receiver start");

  UartFromP4.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
}

void loop() {
  static uint32_t last_beat_ms = 0;
  const uint32_t now = millis();
  if (now - last_beat_ms > 1000) {
    last_beat_ms = now;
    DBG.print("S3 alive  bytes=");
    DBG.print(s_bytes_rx);
    DBG.print(" packets=");
    DBG.println(s_packets_ok);
  }

  Packet p;
  if (read_packet(&p)) {
    s_packets_ok++;
    const char *label = "Unknown";
    if (p.label_id < kCategoryCount) {
      label = kCategoryLabels[p.label_id];
    }

    DBG.print("frame=");
    DBG.print(p.frame_id);
    DBG.print(" label=");
    DBG.print(p.label_id);
    DBG.print("(");
    DBG.print(label);
    DBG.print(")");
    DBG.print(" conf=");
    DBG.print(p.confidence);
    DBG.print(" flags=0x");
    DBG.println(p.flags, HEX);
  }
  delay(1);
}
