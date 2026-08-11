/*
  PlateFOMO — Multi-character license plate recognition using FOMO
  (Faster Objects, More Objects) on ESP32-P4 with TFLite Micro.

  This file defines all model and pipeline constants.  Modify these to match
  your trained FOMO model and class list.
*/

#ifndef PLATEFOMO_MODEL_SETTINGS_H_
#define PLATEFOMO_MODEL_SETTINGS_H_

#include <stdint.h>
#include "image_provider.h"

// ── Input image ──────────────────────────────────────────────────────────────
constexpr int kNumCols    = OUT_WIDTH;       // 96
constexpr int kNumRows    = OUT_HEIGHT;      // 96
constexpr int kNumChannels = 1;              // grayscale int8
constexpr int kMaxImageSize = kNumCols * kNumRows * kNumChannels;

// ── FOMO output grid ─────────────────────────────────────────────────────────
// These values MUST match the trained model's output layer shape:
//   [1, kFomoGridHeight, kFomoGridWidth, 1 + kFomoNumClasses]
//
// Typical values for a MobileNetV2 alpha=0.35 backbone with 96×96 input:
//   - 8× reduction → 12×12 grid
//   - 4× reduction → 24×24 grid
constexpr int kFomoGridWidth  = 12;
constexpr int kFomoGridHeight = 12;
constexpr int kFomoGridCells  = kFomoGridWidth * kFomoGridHeight;

// ── Character classes ────────────────────────────────────────────────────────
// Number of character classes (excludes background — FOMO has an implicit
// background / "no object" channel at index 0).
//   Chinese plates: ~34 provinces + 10 digits + 26 letters ≈ 70
//   EU plates:      10 digits + 26 letters = 36
// Adjust to your use case.
constexpr int kFomoNumClasses = 36;

// FOMO output channels: [background, class_0, class_1, ..., class_N-1]
constexpr int kFomoOutputChannels = 1 + kFomoNumClasses;

// Total output tensor size in bytes (int8).
constexpr int kFomoOutputSize = kFomoGridWidth * kFomoGridHeight * kFomoOutputChannels;

// ── Detection thresholds ─────────────────────────────────────────────────────
// Confidence threshold (after dequant to [0,255]).  A cell whose max class
// score is below this value is considered "no object".
constexpr uint8_t kFomoConfidenceThreshold = 128;  // midpoint of int8 → uint8

// Non-maximum suppression: two detections of the SAME class within this
// pixel radius (in grid-cell units) are merged (keep the higher confidence).
constexpr int kFomoNmsRadius = 1;

// Minimum horizontal separation (in grid-cell units) between adjacent
// characters.  Characters closer than this are merged as duplicates.
constexpr int kFomoMinCharSpacing = 1;

// ── Maximum characters per frame ─────────────────────────────────────────────
constexpr int kMaxDetections = 16;

// ── Class labels ─────────────────────────────────────────────────────────────
extern const char* kFomoClassLabels[kFomoNumClasses];

#endif  // PLATEFOMO_MODEL_SETTINGS_H_
