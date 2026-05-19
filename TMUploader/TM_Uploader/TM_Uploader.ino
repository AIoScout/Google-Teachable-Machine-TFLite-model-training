/* Copyright 2021 Google LLC All Rights Reserved.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
  ==============================================================================
  */

/*
  ESP32-S3 - Camera Capture for Teachable Machine

  This sketch captures 96x96 grayscale images from the ESP32-S3 camera 
  and sends them via Serial to a Processing connector.
*/

#include <Arduino.h>
#include "esp_camera.h"
#include "ImageProvider.h"

// Note: Camera pin definitions are usually handled in board_config.h or directly
// For ESP32-S3 N16R8 (Sense/Eye), the pins are typically:
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

void setup() {
  // Use 115200 baud rate as requested
  Serial.begin(115200);
  while (!Serial)
    ;

  // Configure camera settings
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  // Directly capture at 96x96 resolution
  config.frame_size = FRAMESIZE_96X96;
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // Get grayscale directly from hardware
  config.grab_mode = CAMERA_GRAB_LATEST;      // Always get the freshest frame
  config.fb_location = CAMERA_FB_IN_PSRAM;    // Use PSRAM for frame buffer
  config.jpeg_quality = 12;
  config.fb_count = 2;  // Double buffering for smoother capture

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.println("Camera Init Failed");
    return;
  }

  // Adjust sensor settings if needed
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    // s->set_vflip(s, 1);        // Flip vertically
    s->set_brightness(s, 1);   // Increase brightness slightly
    s->set_saturation(s, -2);  // Lower saturation
  }
  // s->set_vflip(s, 1);          // General vertical flip

  Serial.println("ESP32-S3 Camera Ready!");

  // Clear any garbage data in the serial buffer
  delay(500);
  while (Serial.available()) Serial.read();
}

// Image dimensions (must match Teachable Machine input)
const int kNumCols = 96;
const int kNumRows = 96;
const int kNumChannels = 1;  // Grayscale has 1 channel
const int bytesPerFrame = kNumCols * kNumRows;

uint8_t data[kNumCols * kNumRows * kNumChannels];
const uint8_t syncHeader[] = { 0xAA, 0x55, 0xAA };  // 3-byte header for frame synchronization

void loop() {
  // 1. Get image from camera
  if (GetImage(kNumCols, kNumRows, kNumChannels, data)) {
    // 2. Send synchronization header
    Serial.write(syncHeader, 3);
    // 3. Send raw image data
    Serial.write(data, bytesPerFrame);
  }
}
