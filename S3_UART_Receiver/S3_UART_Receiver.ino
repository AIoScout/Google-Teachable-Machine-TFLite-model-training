#include <Arduino.h>

#include "model_settings.h"

static constexpr int kDebugBaud = 921600;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 36;
static constexpr int kUartTxPin = 37;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;

static HardwareSerial UartFromP4(1);

struct Packet {
  uint8_t msg_type;
  uint16_t frame_id;
  uint8_t label_id;
  uint8_t confidence;
  uint8_t flags;
};

static bool read_packet(Packet *out) {
  static uint8_t buf[8];
  static uint8_t idx = 0;
  static uint8_t state = 0;

  while (UartFromP4.available() > 0) {
    const uint8_t b = (uint8_t)UartFromP4.read();

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
  Serial.begin(kDebugBaud);
  delay(100);
  Serial.println("S3 UART receiver start");

  UartFromP4.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
}

void loop() {
  Packet p;
  if (read_packet(&p)) {
    const char *label = "Unknown";
    if (p.label_id < kCategoryCount) {
      label = kCategoryLabels[p.label_id];
    }

    Serial.print("frame=");
    Serial.print(p.frame_id);
    Serial.print(" label=");
    Serial.print(p.label_id);
    Serial.print("(");
    Serial.print(label);
    Serial.print(")");
    Serial.print(" conf=");
    Serial.print(p.confidence);
    Serial.print(" flags=0x");
    Serial.println(p.flags, HEX);
  }
  delay(1);
}
