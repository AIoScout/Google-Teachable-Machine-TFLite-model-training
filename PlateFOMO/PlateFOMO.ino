/*
  PlateFOMO — Multi-character license plate recognition on ESP32-P4.

  Uses FOMO (Faster Objects, More Objects) via TFLite Micro to detect
  and classify multiple characters on a license plate in a single inference.

  Pipeline:
    1. Camera capture → B-G difference preprocessing → blob-based plate ROI
    2. 96×96 grayscale int8 → FOMO model (MobileNetV2 backbone, grid output)
    3. Post-process: dequant → local maxima → NMS → sort by x (left→right)
    4. Send character sequence over UART to ESP32-S3

  Copyright 2024 The TensorFlow Authors. All Rights Reserved.
  Licensed under the Apache License, Version 2.0.
*/

#include "main_functions.h"
#include "image_provider.h"
#include "model_settings.h"
#include "fomo_postprocess.h"
#include "tm_model_data.h"
#include "model_resolver.h"

#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Enable ESP-NN acceleration for ESP32-P4 if available.
#if __has_include("esp_nn.h") && !defined(ESP_NN)
#define ESP_NN 1
#endif
#if __has_include("esp_nn_conv2d.h")
#include "esp_nn_conv2d.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <SD_MMC.h>
#include <FFat.h>
#include <driver/gpio.h>
#include <errno.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>

// ── Globals ─────────────────────────────────────────────────────────────────
namespace {

tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;

// Transmit gate (controlled by S3 via UART control packets).
static volatile bool s_transmit_enabled = true;

// RX diagnostics.
static volatile uint32_t s_rx_bytes = 0;
static volatile uint32_t s_rx_ack_stop = 0;
static volatile uint32_t s_rx_resume = 0;

static HardwareSerial UartToS3(1);

// ── Constants ───────────────────────────────────────────────────────────────
static constexpr int kDebugBaud = 921600;
static constexpr int kUartBaud  = 921600;
static constexpr int kUartRxPin = 10;
static constexpr int kUartTxPin = 11;

static constexpr int kHandshakePwmPin  = 9;
static constexpr int kHandshakePwmDuty = 255;

// Sync bytes and message types.
static constexpr uint8_t kSync0             = 0xAA;
static constexpr uint8_t kSync1             = 0x55;
static constexpr uint8_t kMsgTypeInference  = 0x01;  // P4 → S3
static constexpr uint8_t kMsgTypeControl    = 0x02;  // S3 → P4

// S3 → P4 control commands.
static constexpr uint8_t kCtrlAckStop          = 0x01;
static constexpr uint8_t kCtrlResumeJunction   = 0x02;

// UART packet: multi-character detection.
// Layout: [AA 55 01 frame_id(LE16) sequence_len num_det
//          class_0 conf_0 ... class_N conf_N checksum]
// `sequence_len` is the total payload byte count from `frame_id_lo` through
// the last detection byte (excludes sync + msg_type + checksum).
// Max payload with kMaxDetections=16: 2+2+2×16 = 36 bytes → 39-byte packet.
static constexpr uint8_t kMaxUartPayload = 2 + 2 + 2 * kMaxDetections;

// FreeRTOS stack sizes.
static constexpr uint32_t kUartTxTaskStackBytes = 4 * 1024;
static constexpr uint32_t kUartRxTaskStackBytes = 4 * 1024;
static constexpr uint32_t kInferenceTaskStackBytes = 32 * 1024;
static constexpr uint32_t kSdTaskStackBytes = 8 * 1024;

// SD card logging.
static constexpr bool kEnableSdLogger = false;
static constexpr int kImageBytes = kNumCols * kNumRows;

// Tensor arena.
constexpr int kTensorArenaSize = 384 * 1024;  // 384 KB — FOMO is heavier
static uint8_t* tensor_arena = nullptr;

// UART queue.
struct UartPacket {
  uint16_t frame_id;
  uint8_t  num_detections;
  uint8_t  det_class[kMaxDetections];
  uint8_t  det_conf[kMaxDetections];
};
static QueueHandle_t s_uart_queue = nullptr;

// ── Helpers ─────────────────────────────────────────────────────────────────
static uint8_t calc_checksum(const uint8_t* data, size_t len) {
  uint8_t csum = 0;
  for (size_t i = 0; i < len; i++) csum ^= data[i];
  return csum;
}

static void uart_enable()  { s_transmit_enabled = true;  Serial.println("UART TX: ENABLED");  }
static void uart_disable() { s_transmit_enabled = false; Serial.println("UART TX: DISABLED"); }

// ── FreeRTOS tasks ──────────────────────────────────────────────────────────

static void uart_tx_task(void* arg) {
  (void)arg;
  for (;;) {
    UartPacket pkt;
    if (xQueueReceive(s_uart_queue, &pkt, portMAX_DELAY) != pdTRUE) continue;
    if (!s_transmit_enabled) continue;

    // Build multi-character UART packet.
    // Format: sync0 sync1 msg_type frame_id_lo frame_id_hi
    //         payload_len_lo payload_len_hi
    //         (class_id conf)*  checksum
    //
    // payload_len = 2 (for num_det byte + reserved) + 2 * num_detections
    uint8_t num = pkt.num_detections;
    uint16_t payload_len = 2 + 2 * (uint16_t)num;   // num_det + reserved + per-det pairs
    uint8_t total_len = 2 + 1 + 2 + 2 + payload_len + 1;  // sync×2 + msgtype + frame_id(LE16) + payload_len(LE16) + payload + csum

    uint8_t buf[64];  // big enough for kMaxDetections=16
    uint8_t* w = buf;
    *w++ = kSync0;
    *w++ = kSync1;
    *w++ = kMsgTypeInference;
    *w++ = (uint8_t)(pkt.frame_id & 0xFF);
    *w++ = (uint8_t)((pkt.frame_id >> 8) & 0xFF);
    *w++ = (uint8_t)(payload_len & 0xFF);
    *w++ = (uint8_t)((payload_len >> 8) & 0xFF);
    *w++ = num;             // num_detections
    *w++ = 0;               // reserved / flags

    for (int i = 0; i < num; i++) {
      *w++ = pkt.det_class[i];
      *w++ = pkt.det_conf[i];
    }

    *w++ = calc_checksum(buf, w - buf);  // checksum over everything up to checksum byte
    UartToS3.write(buf, (size_t)(w - buf));
  }
}

static void uart_rx_task(void* arg) {
  (void)arg;
  static uint8_t rx_buf[5];
  static uint8_t rx_idx  = 0;
  static uint8_t rx_state = 0;  // 0=wait sync0, 1=wait sync1, 2=collect rest

  uint32_t last_log_ms = 0;

  for (;;) {
    while (UartToS3.available() > 0) {
      uint8_t b = (uint8_t)UartToS3.read();
      s_rx_bytes++;

      if (rx_state == 0) {
        if (b == kSync0) { rx_buf[0] = b; rx_idx = 1; rx_state = 1; }
        continue;
      }
      if (rx_state == 1) {
        if (b == kSync1) { rx_buf[1] = b; rx_idx = 2; rx_state = 2; }
        else if (b == kSync0) { rx_buf[0] = b; rx_idx = 1; rx_state = 1; }
        else { rx_state = 0; }
        continue;
      }

      // rx_state == 2
      rx_buf[rx_idx++] = b;
      if (rx_idx < 5) continue;

      rx_state = 0;
      rx_idx  = 0;

      if (rx_buf[2] != kMsgTypeControl) {
        Serial.printf("UART RX: unknown msg_type 0x%02X\n", rx_buf[2]);
        continue;
      }

      uint8_t expected = calc_checksum(rx_buf, 4);
      if (rx_buf[4] != expected) {
        Serial.printf("UART RX: bad csum got=0x%02X exp=0x%02X\n",
                      rx_buf[4], expected);
        continue;
      }

      uint8_t cmd = rx_buf[3];
      if (cmd == kCtrlAckStop) {
        s_rx_ack_stop++;
        Serial.printf("UART RX: ACK_STOP #%lu — disabling TX\n",
                      (unsigned long)s_rx_ack_stop);
        uart_disable();
      } else if (cmd == kCtrlResumeJunction) {
        s_rx_resume++;
        Serial.printf("UART RX: RESUME #%lu — enabling TX\n",
                      (unsigned long)s_rx_resume);
        uart_enable();
      } else {
        Serial.printf("UART RX: unknown cmd 0x%02X\n", cmd);
      }
    }

    uint32_t now = millis();
    if (now - last_log_ms >= 5000) {
      last_log_ms = now;
      Serial.printf("UART RX: bytes=%lu ack=%lu resume=%lu tx=%d\n",
                    (unsigned long)s_rx_bytes, (unsigned long)s_rx_ack_stop,
                    (unsigned long)s_rx_resume, (int)s_transmit_enabled);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ── Inference task ──────────────────────────────────────────────────────────
static void inference_task(void* arg) {
  (void)arg;
  uint16_t frame_id = 0;

  uint64_t total_capture_us = 0;
  uint64_t total_invoke_us  = 0;
  uint64_t total_loop_us    = 0;
  uint32_t timed_frames     = 0;

  for (;;) {
    if (!input || !interpreter) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    uint64_t t_loop_start = esp_timer_get_time();

    // ── Capture + preprocess ─────────────────────────────────────────────
    // For plate detection, always use CROP_MODE_PLATE to search the
    // upper-centre region of the frame for the license plate.
    SetCropMode(CROP_MODE_PLATE);

    uint64_t t_cap_start = esp_timer_get_time();
    if (kTfLiteOk != GetImage(error_reporter, OUT_WIDTH, OUT_HEIGHT,
                               kNumChannels, input->data.int8)) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    uint64_t t_capture_us = esp_timer_get_time() - t_cap_start;
    frame_id++;

    // ── Inference ────────────────────────────────────────────────────────
    uint64_t t_inv_start = esp_timer_get_time();
    if (kTfLiteOk != interpreter->Invoke()) {
      Serial.println("Invoke failed");
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    uint64_t t_invoke_us = esp_timer_get_time() - t_inv_start;

    // Accumulate timing.
    total_capture_us += t_capture_us;
    total_invoke_us  += t_invoke_us;
    total_loop_us    += esp_timer_get_time() - t_loop_start;
    timed_frames++;

    // ── FOMO post-processing ─────────────────────────────────────────────
    TfLiteTensor* output = interpreter->output(0);

    FomoResult result;
    if (output->type == kTfLiteInt8) {
      result = FomoPostProcess(output->data.int8,
                               output->params.scale,
                               output->params.zero_point);
    } else if (output->type == kTfLiteFloat32) {
      result = FomoPostProcessFloat(output->data.f);
    } else {
      Serial.printf("Unsupported output type: %d\n", output->type);
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // ── Queue UART packet ────────────────────────────────────────────────
    if (s_uart_queue) {
      UartPacket pkt;
      pkt.frame_id      = frame_id;
      pkt.num_detections = (uint8_t)result.num_detections;
      for (int i = 0; i < result.num_detections && i < kMaxDetections; i++) {
        pkt.det_class[i] = result.detections[i].class_id;
        pkt.det_conf[i]  = result.detections[i].confidence;
      }
      xQueueOverwrite(s_uart_queue, &pkt);
    }

    // ── Debug output ─────────────────────────────────────────────────────
    if (frame_id % 10 == 0 && timed_frames > 0) {
      uint32_t avg_cap  = (uint32_t)(total_capture_us / timed_frames);
      uint32_t avg_inv  = (uint32_t)(total_invoke_us  / timed_frames);
      uint32_t avg_loop = (uint32_t)(total_loop_us    / timed_frames);

      Serial.print("[PlateFOMO] frame=");
      Serial.print(frame_id);
      Serial.print(" det=");
      Serial.print(result.num_detections);
      Serial.print(" → ");

      // Print detected plate characters
      for (int i = 0; i < result.num_detections; i++) {
        if (result.detections[i].class_id < kFomoNumClasses) {
          Serial.print(kFomoClassLabels[result.detections[i].class_id]);
        } else {
          Serial.print("?");
        }
        Serial.print("(");
        Serial.print(result.detections[i].confidence);
        Serial.print(") ");
      }

      Serial.print("| cap=");
      Serial.print(avg_cap / 1000);
      Serial.print("ms inv=");
      Serial.print(avg_inv / 1000);
      Serial.print("ms loop=");
      Serial.print(avg_loop / 1000);
      Serial.print("ms fps≈");
      Serial.print(avg_loop > 0 ? 1000000UL / avg_loop : 0);
      Serial.print(" tx=");
      Serial.print(s_transmit_enabled ? "ON" : "OFF");
      Serial.println();

      total_capture_us = 0;
      total_invoke_us  = 0;
      total_loop_us    = 0;
      timed_frames     = 0;
    }

    taskYIELD();
  }
}

}  // anonymous namespace

// ── Arduino setup ───────────────────────────────────────────────────────────
void setup() {
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  Serial.begin(kDebugBaud);
  delay(100);
  Serial.println("=== PlateFOMO — Multi-char License Plate Recognition ===");

  // Handshake pin (signal to S3 that we're ready).
  pinMode(kHandshakePwmPin, OUTPUT);
  analogWrite(kHandshakePwmPin, kHandshakePwmDuty);

  // UART to S3.
  pinMode(kUartRxPin, INPUT_PULLDOWN);  // prevent GPIO10/11 crosstalk
  UartToS3.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
  s_uart_queue = xQueueCreate(1, sizeof(UartPacket));

  // ── Load model ────────────────────────────────────────────────────────
  model = tflite::GetModel(g_platefomo_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model schema version mismatch: got %d, expected %d. "
                         "Have you replaced the placeholder tm_model_data.cpp?",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Use the manual op resolver (or replace with Edge Impulse generated one).
  auto& op_resolver = FomoOpResolver();

  // ── Allocate tensor arena ─────────────────────────────────────────────
  Serial.printf("SRAM free heap: %d, largest block: %d (requesting %d)\n",
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                kTensorArenaSize);

  tensor_arena = (uint8_t*)heap_caps_aligned_alloc(
      16, kTensorArenaSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const char* arena_loc = "SRAM";

  if (tensor_arena == nullptr) {
    tensor_arena = (uint8_t*)heap_caps_aligned_alloc(
        16, kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    arena_loc = "PSRAM (SRAM full — reduce model size for speed)";
  }

  if (tensor_arena == nullptr) {
    Serial.println("FATAL: Could not allocate tensor arena!");
    return;
  }
  Serial.printf("Tensor arena: %d bytes in %s\n", kTensorArenaSize, arena_loc);

  // ── Create interpreter ────────────────────────────────────────────────
  static tflite::MicroInterpreter static_interpreter(
      model, op_resolver, tensor_arena, kTensorArenaSize, nullptr, nullptr, false);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    return;
  }

  input = interpreter->input(0);

  // ── Diagnostics ───────────────────────────────────────────────────────
  Serial.printf("Arena used: %d / %d (%d%%)\n",
                interpreter->arena_used_bytes(), kTensorArenaSize,
                (int)(interpreter->arena_used_bytes() * 100LL / kTensorArenaSize));

  Serial.printf("Input: %dx%dx%d type=%s\n",
                input->dims->data[1], input->dims->data[2], input->dims->data[3],
                input->type == kTfLiteInt8 ? "int8" : "float32");

  {
    TfLiteTensor* out = interpreter->output(0);
    Serial.printf("Output: %dx%dx%d type=%s q=(%.4f,%d)\n",
                  out->dims->data[1], out->dims->data[2], out->dims->data[3],
                  out->type == kTfLiteInt8 ? "int8" : "float32",
                  out->params.scale, out->params.zero_point);
  }

  // Check that the FOMO output shape matches our settings.
  {
    TfLiteTensor* out = interpreter->output(0);
    if (out->dims->data[1] != kFomoGridHeight ||
        out->dims->data[2] != kFomoGridWidth  ||
        out->dims->data[3] != kFomoOutputChannels) {
      Serial.printf("WARNING: Model output shape [%d,%d,%d] does not match "
                    "model_settings.h [%d,%d,%d]. Update the constants!\n",
                    out->dims->data[1], out->dims->data[2], out->dims->data[3],
                    kFomoGridHeight, kFomoGridWidth, kFomoOutputChannels);
    }
  }

#if defined(ESP_NN)
  Serial.println("ESP-NN: ENABLED");
#else
  Serial.println("ESP-NN: NOT FOUND (reference kernels — SLOW)");
#endif

  Serial.printf("FOMO grid: %dx%d, %d classes + bg\n",
                kFomoGridWidth, kFomoGridHeight, kFomoNumClasses);
  Serial.println("Ready.");

  // ── Launch FreeRTOS tasks ─────────────────────────────────────────────
#if defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1)
  xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", kUartTxTaskStackBytes,
                          nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", kUartRxTaskStackBytes,
                          nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(inference_task, "fomo", kInferenceTaskStackBytes,
                          nullptr, 3, nullptr, 1);
#else
  xTaskCreate(uart_tx_task, "uart_tx", kUartTxTaskStackBytes,
              nullptr, 2, nullptr);
  xTaskCreate(uart_rx_task, "uart_rx", kUartRxTaskStackBytes,
              nullptr, 2, nullptr);
  xTaskCreate(inference_task, "fomo", kInferenceTaskStackBytes,
              nullptr, 3, nullptr);
#endif
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
