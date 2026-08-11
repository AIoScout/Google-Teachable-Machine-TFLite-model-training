/*
  PlateFOMO — Image provider interface.
  Same B-G difference + blob-detection pipeline as TFLite/,
  adapted for license-plate ROI detection.
*/

#ifndef PLATEFOMO_IMAGE_PROVIDER_H_
#define PLATEFOMO_IMAGE_PROVIDER_H_

#include <stdint.h>

#include <tensorflow/lite/c/common.h>
#if __has_include("tensorflow/lite/micro/micro_error_reporter.h")
#include <tensorflow/lite/micro/micro_error_reporter.h>
#elif __has_include("tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h")
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#endif

// Crop modes for the image capture pipeline.
#define CROP_MODE_PLATE     0   // License plate ROI (search window crop)
#define CROP_MODE_JUNCTION  1   // Lower 40–75 % for road / junction

// Camera sensor dimensions (IMX219 native).
#define IMG_WIDTH  1536
#define IMG_HEIGHT 1232

// Model input dimensions.
#define OUT_WIDTH  96
#define OUT_HEIGHT 96

// Override IMG_SIZE for the ESP32_P4_IMX219 library.
#ifdef IMG_SIZE
#undef IMG_SIZE
#endif
#define IMG_SIZE 96

// ── Public API ───────────────────────────────────────────────────────────────

// Fetch a preprocessed 96×96 grayscale int8 image from the camera.
// The B-G difference pipeline + blob-based auto-crop runs internally.
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter,
                      int image_width, int image_height, int channels,
                      int8_t* image_data);

// Crop mode global — set before calling GetImage().
extern volatile int s_crop_mode;
void SetCropMode(int mode);

// Release LUTs and camera resources.
void ImageProviderDeinit();

#endif  // PLATEFOMO_IMAGE_PROVIDER_H_
