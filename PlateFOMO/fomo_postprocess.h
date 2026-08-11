/*
  PlateFOMO — FOMO post-processing: centroid detection, NMS, and sorting.
*/

#ifndef PLATEFOMO_POSTPROCESS_H_
#define PLATEFOMO_POSTPROCESS_H_

#include <stdint.h>
#include "model_settings.h"

// ── Detection result ─────────────────────────────────────────────────────────
struct FomoDetection {
  uint8_t class_id;      // 0 .. kFomoNumClasses-1
  uint8_t confidence;    // 0-255 (dequantised from int8)
  uint8_t grid_x;        // grid cell column (0 .. kFomoGridWidth-1)
  uint8_t grid_y;        // grid cell row    (0 .. kFomoGridHeight-1)
  uint8_t pixel_x;       // mapped to input-image pixel x (0 .. OUT_WIDTH-1)
  uint8_t pixel_y;       // mapped to input-image pixel y (0 .. OUT_HEIGHT-1)
};

// ── Post-processing results ──────────────────────────────────────────────────
struct FomoResult {
  FomoDetection detections[kMaxDetections];
  int num_detections;    // number of valid entries in detections[]
};

// ── API ──────────────────────────────────────────────────────────────────────

// Process a FOMO int8 output tensor and return a sorted list of detections.
//
// `output_int8` points to the model's int8 output tensor data, laid out as:
//   [kFomoGridHeight][kFomoGridWidth][kFomoOutputChannels]
// where channel 0 is the background score and channels 1..N are class scores.
//
// `output_scale` and `output_zero_point` come from the TfLiteTensor quant params.
// For int8 models:  output_scale is typically 1/255 ≈ 0.00392 and
// zero_point is -128 (mapping int8 [-128,127] → float [-1,1]).
//
// The function:
//   1. Dequantises int8 → uint8 confidence [0, 255] for each cell/class.
//   2. Finds local maxima (centroids) above the confidence threshold.
//   3. Applies per-class NMS to suppress duplicates.
//   4. Sorts detections left-to-right by pixel_x.
FomoResult FomoPostProcess(const int8_t* output_int8,
                           float output_scale,
                           int32_t output_zero_point);

// Alternative: process from float32 output (if using a float model for
// debugging on desktop before quantising to int8).
FomoResult FomoPostProcessFloat(const float* output_float);

#endif  // PLATEFOMO_POSTPROCESS_H_
