#include <Arduino.h>

#include "model_settings.h"

static constexpr int kDebugBaud = 115200;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 44;
static constexpr int kUartTxPin = 43;

static constexpr int kP4DetectAdcPin = 8;
static constexpr int kP4DetectThreshold = 2000;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;

#define UartFromP4 Serial0
#define DBG Serial

struct Packet {
  uint8_t msg_type;
  uint16_t frame_id;
  uint8_t label_id;
  uint8_t confidence;
  uint8_t flags;
};

static uint32_t s_bytes_rx = 0;
static uint32_t s_packets_ok = 0;
static bool s_p4_connected = false;
static bool s_uart_started = false;

static constexpr int kP4DetectSamples = 16;

static int read_adc_avg() {
  int32_t sum = 0;
  for (int i = 0; i < kP4DetectSamples; i++) {
    sum += analogRead(kP4DetectAdcPin);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return (int)(sum / kP4DetectSamples);
}

static void wait_for_p4_connected_blocking() {
  pinMode(kP4DetectAdcPin, INPUT);

  uint32_t last_log_ms = 0;
  while (!s_p4_connected) {
    const int v = read_adc_avg();
      // DBG.print("adc=");
      // DBG.println(v);
    if (v >= kP4DetectThreshold) {
      s_p4_connected = true;
      DBG.print("P4 detected adc=");
      DBG.println(v);
      break;
    }
    const uint32_t now = millis();
    if (now - last_log_ms > 500) {
      last_log_ms = now;
      DBG.print("Waiting for P4 adc=");
      DBG.println(v);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

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

static void detect_task(void *arg) {
  (void)arg;
  wait_for_p4_connected_blocking();
  vTaskDelete(nullptr);
}

static void uart_task(void *arg) {
  (void)arg;
  while (!s_p4_connected) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (!s_uart_started) {
    s_uart_started = true;
    UartFromP4.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
    DBG.printf("UART started rx=%d tx=%d baud=%d\n", kUartRxPin, kUartTxPin, kUartBaud);
  }

  uint32_t last_avail_ms = 0;
  for (;;) {
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
    } else {
      const uint32_t now = millis();
      if (now - last_avail_ms >= 1000) {
        last_avail_ms = now;
        DBG.printf("UART rx_avail=%d bytes=%lu packets=%lu\n", UartFromP4.available(),
                   (unsigned long)s_bytes_rx, (unsigned long)s_packets_ok);
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

static void heartbeat_task(void *arg) {
  (void)arg;
  for (;;) {
    DBG.printf("S3 alive connected=%d bytes=%lu packets=%lu\n", (int)s_p4_connected,
               (unsigned long)s_bytes_rx, (unsigned long)s_packets_ok);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  DBG.begin(kDebugBaud);
  vTaskDelay(pdMS_TO_TICKS(200));
  DBG.println("S3 UART receiver start");

  // xTaskCreate(heartbeat_task, "hb", 4096, nullptr, 1, nullptr);
  xTaskCreate(detect_task, "detect", 4096, nullptr, 2, nullptr);
  
  xTaskCreate(uart_task, "uart", 8192, nullptr, 2, nullptr);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
