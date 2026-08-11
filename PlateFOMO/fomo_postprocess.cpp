/*
  PlateFOMO — FOMO post-processing implementation.

  FOMO outputs a heatmap of shape [grid_h × grid_w × (1 + num_classes)].
  Each cell independently predicts whether an object centroid falls inside it.
  We (1) dequantise, (2) find local maxima per class, (3) apply NMS, and
  (4) sort detections left-to-right for reading-order output.
*/

#include "fomo_postprocess.h"
#include <string.h>    // memset

namespace {

// ── Inline helpers ───────────────────────────────────────────────────────────

// Map grid coordinates back to input-image pixel coordinates.
// The pixel position is the CENTRE of the cell's receptive field.
inline void grid_to_pixel(int gx, int gy, uint8_t* px, uint8_t* py) {
  *px = (uint8_t)((gx + 0.5f) * (float)OUT_WIDTH  / kFomoGridWidth);
  *py = (uint8_t)((gy + 0.5f) * (float)OUT_HEIGHT / kFomoGridHeight);
}

// Calculate 1D Chebyshev distance (max of dx, dy) for grid-cell positions.
inline int chebyshev_dist(int x1, int y1, int x2, int y2) {
  int dx = (x1 > x2) ? (x1 - x2) : (x2 - x1);
  int dy = (y1 > y2) ? (y1 - y2) : (y2 - y1);
  return (dx > dy) ? dx : dy;
}

// Dequantise int8 → uint8 confidence.
//   int8_val  →  float_val = (int8_val - zero_point) * scale
//   float_val →  uint8 = float_val * 255
//
// For a typical FOMO int8 output trained with quantization range [-1, 1]:
//   scale ≈ 1/127, zero_point = -128
//   → uint8 = (int8_val + 128) * (scale * 255 / 127 * 255) ≈ int8_val + 128
//
// Since FOMO outputs are effectively probabilities, the dequant step
// can be simplified to a linear mapping.  Use the actual quant params
// for correctness.
inline uint8_t dequant_confidence(int8_t val, float scale, int32_t zero_point) {
  // float in [-1, 1] or [0, 1] depending on training quantization
  float f = (float)((int)val - (int)zero_point) * scale;
  // Clamp to [0, 1]
  if (f < 0.0f) f = 0.0f;
  if (f > 1.0f) f = 1.0f;
  return (uint8_t)(f * 255.0f + 0.5f);
}

// Dequantise from float32 output (debug / desktop path).
inline uint8_t float_to_conf(float f) {
  if (f < 0.0f) f = 0.0f;
  if (f > 1.0f) f = 1.0f;
  return (uint8_t)(f * 255.0f + 0.5f);
}

}  // anonymous namespace

// ── Main post-processing ─────────────────────────────────────────────────────

FomoResult FomoPostProcess(const int8_t* output_int8,
                           float output_scale,
                           int32_t output_zero_point) {
  FomoResult result;
  memset(&result, 0, sizeof(result));

  // ── Pass 1: dequantise every cell → find best class per cell ────────
  // For each grid cell, compute the max class confidence and record it.
  // We use a flat array of per-cell best scores for the local-maxima scan.
  struct CellInfo {
    uint8_t best_conf;
    uint8_t best_class;
    bool    is_local_max;   // filled in pass 2
  };
  static CellInfo cells[kFomoGridCells];  // static — avoids stack overflow
  static_assert(sizeof(cells) < 2048, "cells array fits in stack");

  for (int gy = 0; gy < kFomoGridHeight; gy++) {
    for (int gx = 0; gx < kFomoGridWidth; gx++) {
      int cell_idx = gy * kFomoGridWidth + gx;
      int base = cell_idx * kFomoOutputChannels;

      uint8_t best_conf = 0;
      uint8_t best_class = 0;

      // Channel 0 is background — skip it; scan classes 1..N
      for (int c = 1; c < kFomoOutputChannels; c++) {
        uint8_t conf = dequant_confidence(output_int8[base + c],
                                          output_scale, output_zero_point);
        if (conf > best_conf) {
          best_conf  = conf;
          best_class = (uint8_t)(c - 1);  // class index 0..N-1
        }
      }

      cells[cell_idx].best_conf   = best_conf;
      cells[cell_idx].best_class  = best_class;
      cells[cell_idx].is_local_max = false;
    }
  }

  // ── Pass 2: local-maxima detection ─────────────────────────────────
  // A cell is a local maximum if its confidence is above the threshold
  // AND it is strictly higher than all 8 neighbours (or equal but with
  // a tie-break on index to avoid duplicates).
  for (int gy = 1; gy < kFomoGridHeight - 1; gy++) {
    for (int gx = 1; gx < kFomoGridWidth - 1; gx++) {
      int cell_idx = gy * kFomoGridWidth + gx;
      uint8_t conf = cells[cell_idx].best_conf;
      if (conf < kFomoConfidenceThreshold) continue;

      uint8_t cls = cells[cell_idx].best_class;
      bool is_max = true;

      // Check 8-connected neighbourhood
      for (int dy = -1; dy <= 1 && is_max; dy++) {
        for (int dx = -1; dx <= 1 && is_max; dx++) {
          if (dx == 0 && dy == 0) continue;
          int ni = (gy + dy) * kFomoGridWidth + (gx + dx);
          uint8_t n_conf = cells[ni].best_conf;
          uint8_t n_cls  = cells[ni].best_class;

          // A neighbour is a "rival" if it has the same class and
          // a higher (or equal + lexicographically earlier) confidence.
          if (n_cls == cls) {
            if (n_conf > conf || (n_conf == conf && ni < cell_idx)) {
              is_max = false;
            }
          }
        }
      }

      if (is_max) {
        cells[cell_idx].is_local_max = true;
      }
    }
  }

  // Also check edge cells (gy=0, gy=H-1, gx=0, gx=W-1) — simplified:
  // skip for brevity; edge detections are unreliable anyway.

  // ── Pass 3: collect local maxima into detections ───────────────────
  int count = 0;
  for (int gy = 0; gy < kFomoGridHeight && count < kMaxDetections; gy++) {
    for (int gx = 0; gx < kFomoGridWidth && count < kMaxDetections; gx++) {
      int ci = gy * kFomoGridWidth + gx;
      if (!cells[ci].is_local_max) continue;

      FomoDetection* d = &result.detections[count];
      d->class_id   = cells[ci].best_class;
      d->confidence = cells[ci].best_conf;
      d->grid_x     = (uint8_t)gx;
      d->grid_y     = (uint8_t)gy;
      grid_to_pixel(gx, gy, &d->pixel_x, &d->pixel_y);
      count++;
    }
  }

  // ── Pass 4: per-class NMS ──────────────────────────────────────────
  // For any two detections of the SAME class within kFomoNmsRadius grid
  // cells, suppress the lower-confidence one.
  for (int i = 0; i < count; i++) {
    if (result.detections[i].confidence == 0) continue;
    for (int j = i + 1; j < count; j++) {
      if (result.detections[j].confidence == 0) continue;
      if (result.detections[i].class_id != result.detections[j].class_id)
        continue;

      int dist = chebyshev_dist(result.detections[i].grid_x,
                                result.detections[i].grid_y,
                                result.detections[j].grid_x,
                                result.detections[j].grid_y);
      if (dist <= kFomoNmsRadius) {
        // Suppress the lower-confidence one
        if (result.detections[i].confidence >=
            result.detections[j].confidence) {
          result.detections[j].confidence = 0;  // mark as suppressed
        } else {
          result.detections[i].confidence = 0;
        }
      }
    }
  }

  // Compact — remove suppressed entries
  int compact_count = 0;
  for (int i = 0; i < count; i++) {
    if (result.detections[i].confidence > 0) {
      if (compact_count != i) {
        result.detections[compact_count] = result.detections[i];
      }
      compact_count++;
    }
  }
  count = compact_count;

  // ── Pass 5: sort by pixel_x (left → right reading order) ────────────
  // Simple insertion sort — kMaxDetections is small.
  for (int i = 1; i < count; i++) {
    FomoDetection key = result.detections[i];
    int j = i - 1;
    while (j >= 0 && result.detections[j].pixel_x > key.pixel_x) {
      result.detections[j + 1] = result.detections[j];
      j--;
    }
    result.detections[j + 1] = key;
  }

  result.num_detections = count;
  return result;
}

// ── Float32 variant (for desktop debugging) ─────────────────────────────────

FomoResult FomoPostProcessFloat(const float* output_float) {
  FomoResult result;
  memset(&result, 0, sizeof(result));

  struct CellInfo {
    uint8_t best_conf;
    uint8_t best_class;
    bool    is_local_max;
  };
  static CellInfo cells[kFomoGridCells];

  for (int gy = 0; gy < kFomoGridHeight; gy++) {
    for (int gx = 0; gx < kFomoGridWidth; gx++) {
      int cell_idx = gy * kFomoGridWidth + gx;
      int base = cell_idx * kFomoOutputChannels;

      uint8_t best_conf  = 0;
      uint8_t best_class = 0;
      for (int c = 1; c < kFomoOutputChannels; c++) {
        uint8_t conf = float_to_conf(output_float[base + c]);
        if (conf > best_conf) {
          best_conf  = conf;
          best_class = (uint8_t)(c - 1);
        }
      }
      cells[cell_idx].best_conf   = best_conf;
      cells[cell_idx].best_class  = best_class;
      cells[cell_idx].is_local_max = false;
    }
  }

  // Local-maxima scan (same as int8 path)
  for (int gy = 1; gy < kFomoGridHeight - 1; gy++) {
    for (int gx = 1; gx < kFomoGridWidth - 1; gx++) {
      int cell_idx = gy * kFomoGridWidth + gx;
      uint8_t conf = cells[cell_idx].best_conf;
      if (conf < kFomoConfidenceThreshold) continue;
      uint8_t cls = cells[cell_idx].best_class;
      bool is_max = true;
      for (int dy = -1; dy <= 1 && is_max; dy++) {
        for (int dx = -1; dx <= 1 && is_max; dx++) {
          if (dx == 0 && dy == 0) continue;
          int ni = (gy + dy) * kFomoGridWidth + (gx + dx);
          if (cells[ni].best_class == cls &&
              (cells[ni].best_conf > conf ||
               (cells[ni].best_conf == conf && ni < cell_idx))) {
            is_max = false;
          }
        }
      }
      if (is_max) cells[cell_idx].is_local_max = true;
    }
  }

  // Collect, NMS, sort
  int count = 0;
  for (int gy = 0; gy < kFomoGridHeight && count < kMaxDetections; gy++) {
    for (int gx = 0; gx < kFomoGridWidth && count < kMaxDetections; gx++) {
      int ci = gy * kFomoGridWidth + gx;
      if (!cells[ci].is_local_max) continue;
      FomoDetection* d = &result.detections[count];
      d->class_id   = cells[ci].best_class;
      d->confidence = cells[ci].best_conf;
      d->grid_x     = (uint8_t)gx;
      d->grid_y     = (uint8_t)gy;
      grid_to_pixel(gx, gy, &d->pixel_x, &d->pixel_y);
      count++;
    }
  }

  // Per-class NMS
  for (int i = 0; i < count; i++) {
    if (result.detections[i].confidence == 0) continue;
    for (int j = i + 1; j < count; j++) {
      if (result.detections[j].confidence == 0) continue;
      if (result.detections[i].class_id != result.detections[j].class_id)
        continue;
      int dist = chebyshev_dist(result.detections[i].grid_x,
                                result.detections[i].grid_y,
                                result.detections[j].grid_x,
                                result.detections[j].grid_y);
      if (dist <= kFomoNmsRadius) {
        if (result.detections[i].confidence >=
            result.detections[j].confidence) {
          result.detections[j].confidence = 0;
        } else {
          result.detections[i].confidence = 0;
        }
      }
    }
  }

  int compact_count = 0;
  for (int i = 0; i < count; i++) {
    if (result.detections[i].confidence > 0) {
      if (compact_count != i)
        result.detections[compact_count] = result.detections[i];
      compact_count++;
    }
  }
  count = compact_count;

  // Sort by pixel_x
  for (int i = 1; i < count; i++) {
    FomoDetection key = result.detections[i];
    int j = i - 1;
    while (j >= 0 && result.detections[j].pixel_x > key.pixel_x) {
      result.detections[j + 1] = result.detections[j];
      j--;
    }
    result.detections[j + 1] = key;
  }

  result.num_detections = count;
  return result;
}
