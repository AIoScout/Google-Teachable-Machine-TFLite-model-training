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

#include "ImageProvider.h"
#include "esp_camera.h"

/**
 * GetImage: Captures a frame from the camera and copies it to a buffer.
 * 
 * @param image_width  Expected width of the image (96)
 * @param image_height Expected height of the image (96)
 * @param channels     Number of color channels (1 for grayscale)
 * @param image_data   Pointer to the buffer where image data will be stored
 * @return true if successful, false otherwise
 */
bool GetImage(int image_width, int image_height, int channels, uint8_t* image_data) {
  // Fetch a frame buffer from the camera sensor
  
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    // Return false if capture failed
    return false;
  }

  // Check if the captured frame matches our requirements
  if (fb->width != image_width || fb->height != image_height) {
    // Return the buffer to the pool before failing
    esp_camera_fb_return(fb);
    return false;
  }

  // Directly copy the raw grayscale data to our target buffer
  // Since we use PIXFORMAT_GRAYSCALE, fb->len is exactly width * height
  memcpy(image_data, fb->buf, fb->len);

  // Return the frame buffer so it can be reused for the next capture
  esp_camera_fb_return(fb);
  return true;
}

/**
 * GetDummyImage: Generates a solid gray image for testing purposes.
 */
bool GetDummyImage(int image_width, int image_height, int channels, uint8_t* image_data) {
  for (int i = 0; i < image_width * image_height * channels; ++i) {
    image_data[i] = 128; // Fill with medium gray
  }
  return true;
}
