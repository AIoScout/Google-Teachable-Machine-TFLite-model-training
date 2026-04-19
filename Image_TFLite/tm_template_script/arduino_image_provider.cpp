/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
  ==============================================================================*/

#include "image_provider.h"
#include "esp_camera.h"
#include "tensorflow/lite/micro/micro_log.h"

// Camera pin definitions for ESP32-S3 (ESP-S3-EYE)
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 15
#define CAMERA_PIN_SIOD 4
#define CAMERA_PIN_SIOC 5
#define CAMERA_PIN_D7 16
#define CAMERA_PIN_D6 17
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D4 12
#define CAMERA_PIN_D3 10
#define CAMERA_PIN_D2 8
#define CAMERA_PIN_D1 9
#define CAMERA_PIN_D0 11
#define CAMERA_PIN_VSYNC 6
#define CAMERA_PIN_HREF 7
#define CAMERA_PIN_PCLK 13

#define XCLK_FREQ_HZ 20000000

TfLiteStatus InitCamera() {
  static bool g_is_camera_initialized = false;
  if (g_is_camera_initialized) return kTfLiteOk;

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAMERA_PIN_D0;
  config.pin_d1 = CAMERA_PIN_D1;
  config.pin_d2 = CAMERA_PIN_D2;
  config.pin_d3 = CAMERA_PIN_D3;
  config.pin_d4 = CAMERA_PIN_D4;
  config.pin_d5 = CAMERA_PIN_D5;
  config.pin_d6 = CAMERA_PIN_D6;
  config.pin_d7 = CAMERA_PIN_D7;
  config.pin_xclk = CAMERA_PIN_XCLK;
  config.pin_pclk = CAMERA_PIN_PCLK;
  config.pin_vsync = CAMERA_PIN_VSYNC;
  config.pin_href = CAMERA_PIN_HREF;
  config.pin_sccb_sda = CAMERA_PIN_SIOD;
  config.pin_sccb_scl = CAMERA_PIN_SIOC;
  config.pin_pwdn = CAMERA_PIN_PWDN;
  config.pin_reset = CAMERA_PIN_RESET;
  config.xclk_freq_hz = XCLK_FREQ_HZ;
  config.frame_size = FRAMESIZE_96X96; 
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    MicroPrintf("Camera init failed with error 0x%x", err);
    return kTfLiteError;
  }

  // Adjust sensor settings
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    // Teachable Machine 训练时通常使用镜像或正常视角
    // 如果识别不准，可以尝试切换以下设置
    s->set_vflip(s, 0);   // 垂直翻转
    s->set_hmirror(s, 0); // 水平镜像
    
    if (s->id.PID == OV3660_PID) {
      s->set_brightness(s, 1);
      s->set_saturation(s, -2);
    }
  }

  g_is_camera_initialized = true;
  return kTfLiteOk;
}

TfLiteStatus GetImage(int image_width, int image_height, int channels, int8_t* image_data) {
  if (InitCamera() != kTfLiteOk) {
    return kTfLiteError;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    MicroPrintf("Camera capture failed");
    return kTfLiteError;
  }

  if (fb->width != image_width || fb->height != image_height) {
    MicroPrintf("Camera capture size mismatch: got %dx%d, expected %dx%d", 
                fb->width, fb->height, image_width, image_height);
    esp_camera_fb_return(fb);
    return kTfLiteError;
  }

  // Teachable Machine 的 int8 模型通常期望输入范围在 [-128, 127]
  // 对应原始像素 [0, 255]
  for (int i = 0; i < image_width * image_height * channels; ++i) {
    image_data[i] = (int8_t)(fb->buf[i] - 128);
  }

  esp_camera_fb_return(fb);
  return kTfLiteOk;
}
