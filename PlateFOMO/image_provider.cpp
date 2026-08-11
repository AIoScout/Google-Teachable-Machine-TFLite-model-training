/*
  PlateFOMO — Image provider (B-G difference pipeline + camera interface).

  Adapted from TFLite/image_provider.cpp.  The preprocessing pipeline is
  identical to AItraining/image_preprocess.py — any changes MUST be mirrored
  there.
*/

#include "image_provider.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#if __has_include(<sys/ioctl.h>)
#include <sys/ioctl.h>
#endif
#if __has_include(<sys/mman.h>)
#include <sys/mman.h>
#endif
#if __has_include("esp_log.h")
#include "esp_log.h"
#endif
#if __has_include("esp_err.h")
#include "esp_err.h"
#endif
#if __has_include("nvs_flash.h")
#include "nvs_flash.h"
#endif
#if __has_include("esp_timer.h")
#include "esp_timer.h"
#endif
#if __has_include("driver/ledc.h")
#include "driver/ledc.h"
#endif
#if __has_include("driver/gpio.h")
#include "driver/gpio.h"
#endif
#if __has_include("imx219.h")
#include "imx219.h"
#endif
#if __has_include("esp_cam_sensor_xclk.h")
#include "esp_cam_sensor_xclk.h"
#define PLATEFOMO_HAS_XCLK_ROUTER 1
#else
#define PLATEFOMO_HAS_XCLK_ROUTER 0
#endif
#if __has_include("esp_heap_caps.h")
#include "esp_heap_caps.h"
#endif

#if defined(ARDUINO_ARCH_ESP32P4)
#include <ESP32_P4_IMX219.h>
#define PLATEFOMO_HAS_ARDUINO_IMX219_LIB 1
#elif __has_include("ESP32_P4_IMX219.h")
#include "ESP32_P4_IMX219.h"
#define PLATEFOMO_HAS_ARDUINO_IMX219_LIB 1
#elif __has_include(<ESP32_P4_IMX219.h>)
#include <ESP32_P4_IMX219.h>
#define PLATEFOMO_HAS_ARDUINO_IMX219_LIB 1
#else
#define PLATEFOMO_HAS_ARDUINO_IMX219_LIB 0
#endif

#if !PLATEFOMO_HAS_ARDUINO_IMX219_LIB && __has_include("esp_video_init.h")
#ifdef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#undef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#endif
#define CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE 1
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_video_device.h"
#endif

// ── Crop mode global ────────────────────────────────────────────────────────
volatile int s_crop_mode = CROP_MODE_PLATE;
static int s_last_lut_mode = -1;

void SetCropMode(int mode) {
  if (mode == CROP_MODE_JUNCTION || mode == CROP_MODE_PLATE) {
    s_crop_mode = mode;
#if PLATEFOMO_HAS_ARDUINO_IMX219_LIB
    g_imx219_crop_mode = mode;
#endif
  }
}

#if !PLATEFOMO_HAS_ARDUINO_IMX219_LIB && __has_include("esp_video_init.h")
#define PLATEFOMO_HAS_ESP_VIDEO 1
#else
#define PLATEFOMO_HAS_ESP_VIDEO 0
#endif

// ── Demosaic LUTs ───────────────────────────────────────────────────────────
static int*  s_x_lut = NULL;
static int*  s_y_lut = NULL;
static uint32_t s_raw_bytesperline = 0;
static const char* kTag = "platefomo_cam";

#if __has_include("esp_log.h")
#define CAM_LOGI(...) ESP_LOGI(kTag, __VA_ARGS__)
#define CAM_LOGW(...) ESP_LOGW(kTag, __VA_ARGS__)
#else
#define CAM_LOGI(...) do { printf(__VA_ARGS__); printf("\n"); } while (0)
#define CAM_LOGW(...) do { printf(__VA_ARGS__); printf("\n"); } while (0)
#endif

// ── LUT initialisation ──────────────────────────────────────────────────────
static void init_demosaic_luts(int width, int height, int crop_mode) {
  if (s_x_lut == NULL) {
    s_x_lut = (int*)malloc(OUT_WIDTH * sizeof(int));
  }
  if (s_y_lut == NULL) {
    s_y_lut = (int*)malloc(OUT_HEIGHT * sizeof(int));
  }

  int crop_w, crop_h, x_offset, y_offset;

  if (crop_mode == CROP_MODE_JUNCTION) {
    // Junction: lower 40%-75% of frame (matches Python _junction_bbox).
    const float top_frac    = 0.40f;
    const float bottom_frac = 0.75f;
    crop_w    = 1232;
    crop_h    = (int)(height * (bottom_frac - top_frac));
    x_offset  = (width - crop_w) / 2;
    y_offset  = (int)(height * top_frac);
  } else {
    // Plate mode: search upper-centre 30-70% of frame.
    // License plates are typically in the upper centre when the camera
    // is mounted at bumper / grille height pointing level.
    const float left_frac   = 0.25f;
    const float right_frac  = 0.75f;
    const float top_frac    = 0.20f;
    const float bottom_frac = 0.60f;

    int sx = (int)(width  * left_frac);
    int ex = (int)(width  * right_frac);
    int sy = (int)(height * top_frac);
    int ey = (int)(height * bottom_frac);

    int sw = ex - sx;
    int sh = ey - sy;
    crop_w = (sw < sh) ? sw : sh;
    crop_h = crop_w;
    x_offset = sx + (sw - crop_w) / 2;
    y_offset = sy + (sh - crop_h) / 2;

    if (x_offset < 0) x_offset = 0;
    if (y_offset < 0) y_offset = 0;
    if (x_offset + crop_w > width)  x_offset = width  - crop_w;
    if (y_offset + crop_h > height) y_offset = height - crop_h;
  }

  float x_step = (float)crop_w / OUT_WIDTH;
  float y_step = (float)crop_h / OUT_HEIGHT;

  for (int y = 0; y < OUT_HEIGHT; y++) {
    s_y_lut[y] = (y_offset + (int)(y * y_step + 0.5f)) & ~1;
  }
  for (int x = 0; x < OUT_WIDTH; x++) {
    s_x_lut[x] = (x_offset + (int)(x * x_step + 0.5f)) & ~1;
  }

  s_last_lut_mode = crop_mode;
}

// ── GetImage — public API ───────────────────────────────────────────────────

TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter,
                      int image_width, int image_height, int channels,
                      int8_t* image_data) {
  if (image_width != OUT_WIDTH || image_height != OUT_HEIGHT || channels != 1) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "GetImage expects %dx%dx1, got %dx%dx%d",
                         OUT_WIDTH, OUT_HEIGHT,
                         image_width, image_height, channels);
    return kTfLiteError;
  }

#if PLATEFOMO_HAS_ARDUINO_IMX219_LIB
  // ── Arduino IMX219 library backend ───────────────────────────────────
  static bool backend_ok = false;
  if (!backend_ok) {
    backend_ok = esp32_p4_imx219_begin();
    if (!backend_ok) {
      TF_LITE_REPORT_ERROR(error_reporter, "esp32_p4_imx219_begin failed");
      return kTfLiteError;
    }
  }

  bool updated = false;
  for (int tries = 0; tries < 100; tries++) {
    if (esp32_p4_imx219_update()) { updated = true; break; }
    usleep(2000);
  }
  if (!updated) {
    TF_LITE_REPORT_ERROR(error_reporter, "esp32_p4_imx219_update timeout");
    return kTfLiteError;
  }

  const uint8_t* rgb_raw = esp32_p4_imx219_rgb();

  // ── Auto White Balance (Gray World) ──────────────────────────────────
  static int16_t bg_raw[OUT_WIDTH * OUT_HEIGHT];
  static uint8_t bg_u8[OUT_WIDTH * OUT_HEIGHT];

  int64_t sum_r = 0, sum_g = 0, sum_b = 0;
  int awb_n = 0;
  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    int rr = rgb_raw[i * 3 + 0];
    int gg = rgb_raw[i * 3 + 1];
    int bb = rgb_raw[i * 3 + 2];
    if (rr < 10 && gg < 10 && bb < 10) continue;
    sum_r += rr; sum_g += gg; sum_b += bb;
    awb_n++;
  }

  uint16_t wb_r = 200, wb_b = 200;  // default ×2.0
  if (awb_n > 100) {
    int avg_r = (int)(sum_r / awb_n);
    int avg_g = (int)(sum_g / awb_n);
    int avg_b = (int)(sum_b / awb_n);
    if (avg_r > 0 && avg_g > 0 && avg_b > 0) {
      int gr = (avg_g * 100) / avg_r;
      int gb = (avg_g * 100) / avg_b;
      if (gr <  50) gr =  50;  if (gr > 400) gr = 400;
      if (gb <  50) gb =  50;  if (gb > 400) gb = 400;
      wb_r = (uint16_t)gr;
      wb_b = (uint16_t)gb;
    }
  }

  // ── B-G extraction ───────────────────────────────────────────────────
  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    int r_raw = rgb_raw[i * 3 + 0];
    int g_raw = rgb_raw[i * 3 + 1];
    int b_raw = rgb_raw[i * 3 + 2];

    int r = (r_raw * (int)wb_r) / 100;
    int b = (b_raw * (int)wb_b) / 100;
    int g = g_raw;
    if (r > 255) r = 255;
    if (b > 255) b = 255;

    if (r < 10 && g < 10 && b < 10) {
      bg_raw[i] = 0;  bg_u8[i] = 128;  continue;
    }
    int diff = b - g;
    bg_raw[i] = (int16_t)diff;
    bg_u8[i]  = (uint8_t)((diff + 255) / 2);
  }

  // ── 5×5 box blur → contrast stretch → binary mask ────────────────────
  static int16_t bg_blur[OUT_WIDTH * OUT_HEIGHT];
  int16_t bg_min = 32767, bg_max = -32768;
  for (int y = 2; y < OUT_HEIGHT - 2; y++) {
    for (int x = 2; x < OUT_WIDTH - 2; x++) {
      int sum = 0;
      for (int dy = -2; dy <= 2; dy++)
        for (int dx = -2; dx <= 2; dx++)
          sum += bg_raw[(y + dy) * OUT_WIDTH + (x + dx)];
      int16_t v = (int16_t)(sum / 25);
      int idx = y * OUT_WIDTH + x;
      bg_blur[idx] = v;
      if (v < bg_min) bg_min = v;
      if (v > bg_max) bg_max = v;
    }
  }

  int bg_span = (int)bg_max - (int)bg_min;
  static uint8_t mask[OUT_WIDTH * OUT_HEIGHT];
  bool any_signal = (bg_span >= 20);

  if (any_signal) {
    for (int y = 2; y < OUT_HEIGHT - 2; y++) {
      for (int x = 2; x < OUT_WIDTH - 2; x++) {
        int idx = y * OUT_WIDTH + x;
        int stretched = ((int)bg_blur[idx] - (int)bg_min) * 255 / bg_span;
        mask[idx] = (stretched > 80) ? 255 : 0;
      }
    }
  }

  // ── Morphology + blob detection (auto-ROI) ───────────────────────────
  int crop_x1 = 0, crop_y1 = 0, crop_w = OUT_WIDTH, crop_h = OUT_HEIGHT;
  bool found_roi = false;

  if (any_signal) {
    // 3×3 erode
    static uint8_t tmp[OUT_WIDTH * OUT_HEIGHT];
    for (int y = 1; y < OUT_HEIGHT - 1; y++) {
      for (int x = 1; x < OUT_WIDTH - 1; x++) {
        int idx = y * OUT_WIDTH + x;
        tmp[idx] = (mask[idx] && mask[idx - 1] && mask[idx + 1]
                 && mask[idx - OUT_WIDTH] && mask[idx + OUT_WIDTH]) ? 255 : 0;
      }
    }
    // 3×3 dilate ×2
    for (int pass = 0; pass < 2; pass++) {
      for (int y = 1; y < OUT_HEIGHT - 1; y++) {
        for (int x = 1; x < OUT_WIDTH - 1; x++) {
          int idx = y * OUT_WIDTH + x;
          mask[idx] = tmp[idx];
          tmp[idx] = (mask[idx] || mask[idx - 1] || mask[idx + 1]
                   || mask[idx - OUT_WIDTH] || mask[idx + OUT_WIDTH]) ? 255 : 0;
        }
      }
      for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
        uint8_t s = mask[i]; mask[i] = tmp[i]; tmp[i] = s;
      }
    }

    // Largest connected component
    #define MAX_RUNS 512
    struct { int x1, x2, y, label; } runs[MAX_RUNS];
    int num_runs = 0;

    for (int y = 1; y < OUT_HEIGHT - 1 && num_runs < MAX_RUNS; y++) {
      int x = 1;
      while (x < OUT_WIDTH - 1) {
        while (x < OUT_WIDTH - 1 && !mask[y * OUT_WIDTH + x]) x++;
        if (x >= OUT_WIDTH - 1) break;
        int x1 = x;
        while (x < OUT_WIDTH - 1 && mask[y * OUT_WIDTH + x]) x++;
        runs[num_runs].x1 = x1;
        runs[num_runs].x2 = x - 1;
        runs[num_runs].y  = y;
        runs[num_runs].label = num_runs;
        num_runs++;
      }
    }

    if (num_runs > 1) {
      int parent[MAX_RUNS];
      for (int i = 0; i < num_runs; i++) parent[i] = i;
      auto find  = [&](int a) { while (parent[a] != a) a = parent[a]; return a; };
      auto unite = [&](int a, int b) { parent[find(a)] = find(b); };

      for (int i = 0; i < num_runs; i++) {
        for (int j = i + 1; j < num_runs; j++) {
          if (runs[j].y > runs[i].y + 1) break;
          if (runs[j].y == runs[i].y + 1
              && runs[j].x1 <= runs[i].x2 && runs[j].x2 >= runs[i].x1)
            unite(i, j);
        }
      }

      int comp_size[MAX_RUNS] = {0};
      for (int i = 0; i < num_runs; i++)
        comp_size[find(i)] += runs[i].x2 - runs[i].x1 + 1;

      int best_root = 0, best_size = 0;
      for (int i = 0; i < num_runs; i++) {
        int root = find(i);
        if (comp_size[root] > best_size) {
          best_size = comp_size[root];
          best_root = root;
        }
      }

      if (best_size >= 16) {
        int min_x = OUT_WIDTH, min_y = OUT_HEIGHT, max_x = 0, max_y = 0;
        for (int i = 0; i < num_runs; i++) {
          if (find(i) == best_root) {
            if (runs[i].x1 < min_x) min_x = runs[i].x1;
            if (runs[i].x2 > max_x) max_x = runs[i].x2;
            if (runs[i].y  < min_y) min_y = runs[i].y;
            if (runs[i].y  > max_y) max_y = runs[i].y;
          }
        }
        int bw = max_x - min_x + 1, bh = max_y - min_y + 1;
        int side = (bw > bh) ? bw : bh;
        int pad = side / 5; if (pad < 1) pad = 1;
        side += pad * 2;
        int ccx = (min_x + max_x) / 2, ccy = (min_y + max_y) / 2;
        int half = side / 2;
        crop_x1 = ccx - half; if (crop_x1 < 0) crop_x1 = 0;
        crop_y1 = ccy - half; if (crop_y1 < 0) crop_y1 = 0;
        crop_w  = side;       if (crop_x1 + crop_w > OUT_WIDTH)  crop_w = OUT_WIDTH  - crop_x1;
        crop_h  = side;       if (crop_y1 + crop_h > OUT_HEIGHT) crop_h = OUT_HEIGHT - crop_y1;
        if (crop_w >= 8 && crop_h >= 8) found_roi = true;
      }
    }
  }

  // ── Final crop → BT.601 luminance → int8 ─────────────────────────────
  if (!found_roi) {
    // Full frame → BT.601
    for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
      int idx3 = i * 3;
      uint8_t r = rgb_raw[idx3 + 0];
      uint8_t g = rgb_raw[idx3 + 1];
      uint8_t b = rgb_raw[idx3 + 2];
      uint8_t lum = (uint8_t)(((uint16_t)r * 30 + (uint16_t)g * 59 + (uint16_t)b * 11) / 100);
      image_data[i] = (int8_t)((int)lum - 128);
    }
  } else {
    // Nearest-neighbour resize from cropped region
    for (int y = 0; y < OUT_HEIGHT; y++) {
      int src_y = crop_y1 + y * crop_h / OUT_HEIGHT;
      if (src_y >= OUT_HEIGHT) src_y = OUT_HEIGHT - 1;
      for (int x = 0; x < OUT_WIDTH; x++) {
        int src_x = crop_x1 + x * crop_w / OUT_WIDTH;
        if (src_x >= OUT_WIDTH) src_x = OUT_WIDTH - 1;
        int idx3 = (src_y * OUT_WIDTH + src_x) * 3;
        uint8_t r = rgb_raw[idx3 + 0];
        uint8_t g = rgb_raw[idx3 + 1];
        uint8_t b = rgb_raw[idx3 + 2];
        uint8_t lum = (uint8_t)(((uint16_t)r * 30 + (uint16_t)g * 59 + (uint16_t)b * 11) / 100);
        image_data[y * OUT_WIDTH + x] = (int8_t)((int)lum - 128);
      }
    }
  }

  // ── Contrast stretch  (span ≥ 24 → expand to full [0,255]) ───────────
  {
    uint8_t min_val = 255, max_val = 0;
    for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
      uint8_t v = (uint8_t)((int)image_data[i] + 128);
      if (v < min_val) min_val = v;
      if (v > max_val) max_val = v;
    }
    int span = (int)max_val - (int)min_val;
    if (span >= 24) {
      for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
        uint8_t v = (uint8_t)((int)image_data[i] + 128);
        int stretched = ((int)v - (int)min_val) * 255 / span;
        image_data[i] = (int8_t)(stretched - 128);
      }
    }
  }

  return kTfLiteOk;

#elif PLATEFOMO_HAS_ESP_VIDEO
  // ── V4L2 / esp_video backend (debug / fallback) ──────────────────────
  // (Simplified — see TFLite/image_provider.cpp for full V4L2 implementation.)
  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    image_data[i] = 0;
  }
  TF_LITE_REPORT_ERROR(error_reporter, "esp_video backend not yet implemented in PlateFOMO");
  return kTfLiteOk;
#else
  // ── No camera backend — return zeros ──────────────────────────────────
  for (int i = 0; i < OUT_WIDTH * OUT_HEIGHT; i++) {
    image_data[i] = 0;
  }
  return kTfLiteOk;
#endif
}

void ImageProviderDeinit() {
  if (s_x_lut) { free(s_x_lut); s_x_lut = NULL; }
  if (s_y_lut) { free(s_y_lut); s_y_lut = NULL; }
}
