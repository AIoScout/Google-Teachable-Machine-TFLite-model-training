# PlateFOMO — Multi-Character License Plate Recognition on ESP32-P4

FOMO (Faster Objects, More Objects) based multi-character detection using
TensorFlow Lite Micro on the ESP32-P4 + IMX219 camera pipeline.

## Architecture

```
 ┌──────────────┐     ┌─────────────────┐     ┌──────────────┐
 │ IMX219 Camera │ ──▶ │ B-G Preprocess  │ ──▶ │ FOMO Model   │
 │  1536×1232    │     │ + Blob Auto-Crop│     │ 96×96×1 →    │
 │  RAW10 BGGR   │     │ → 96×96 gray    │     │ 12×12×37     │
 └──────────────┘     └─────────────────┘     └──────┬───────┘
                                                     │
                    ┌─────────────────────────────────┘
                    ▼
  ┌──────────────────────────────────────┐
  │  Post-Process:                       │
  │  1. Dequant int8 → uint8 confidence  │
  │  2. Local maxima (centroid) per cell │
  │  3. Per-class NMS (remove duplicates)│
  │  4. Sort by x (left → right)         │
  └──────────────────┬───────────────────┘
                     ▼
  ┌──────────────────────────────────────┐
  │  UART → ESP32-S3                     │
  │  [AA 55 01 fid payload csum]         │
  │  payload: num_chars + class_i conf_i │
  └──────────────────────────────────────┘
```

## How FOMO Works

FOMO divides the input image into a grid (12×12 by default). Each grid cell
independently predicts whether the **centroid** of an object falls within it,
and what class that object belongs to. There is no bounding-box regression —
this makes FOMO dramatically lighter than SSD or YOLO and suitable for MCUs.

| Property | Value |
|---|---|
| Input | 96×96×1 grayscale int8 |
| Backbone | MobileNetV2 (alpha 0.25–0.35) |
| Grid output | kFomoGridWidth × kFomoGridHeight (default 12×12) |
| Output channels | 1 + kFomoNumClasses (background + per-class heatmap) |
| Detection type | Centroid-based, fully convolutional |
| RAM (tensor arena) | ~384 KB (adjustable) |

## File Structure

| File | Purpose |
|---|---|
| `PlateFOMO.ino` | Main Arduino sketch: tasks, UART, inference loop |
| `image_provider.h` | Camera + preprocessing interface |
| `image_provider.cpp` | B-G difference pipeline, blob auto-crop, camera init |
| `model_settings.h` | Grid size, class count, thresholds |
| `model_settings.cpp` | Class label strings |
| `fomo_postprocess.h` | FOMO output parsing API |
| `fomo_postprocess.cpp` | Dequant → local-maxima → NMS → sort |
| `model_resolver.h` | TFLite Micro op resolver for FOMO |
| `tm_model_data.h` | Model byte array declaration |
| `tm_model_data.cpp` | **Replace with your trained model** (placeholder) |
| `main_functions.h` / `arduino_main.cpp` | Arduino entry points |

## Training a FOMO Model

### Option A: Edge Impulse (Recommended)

1. Go to [studio.edgeimpulse.com](https://studio.edgeimpulse.com)
2. Create a new project → "Object Detection" → select "FOMO"
3. Upload training images with centroid annotations per character
4. Configure MobileNetV2 alpha (0.25 for speed, 0.35 for accuracy)
5. Set image size to 96×96, grayscale
6. Train → export as "C++ library" (Arduino)
7. Copy the generated `.cpp` model data into `tm_model_data.cpp`

### Option B: Manual Training

```python
# 1. Prepare dataset: labeled character centroids on license plates
# 2. Train with TensorFlow using Edge Impulse's FOMO training script:
#    https://github.com/edgeimpulse/example-fomo-tensorflow

import tensorflow as tf

# FOMO loss = per-pixel cross-entropy weighted by object density
# Output must be a grid of centroids (Gaussian-blurred), not bboxes.
```

### Key training considerations

- **Centroid annotations**: Label only the centre pixel of each character,
  not bounding boxes. FOMO learns from centroid heatmaps.
- **Object separation**: Characters must be at least 1 grid cell apart
  (with 12×12 grid on 96×96 input ≈ 8 pixels/cell).
- **Class balance**: Ensure all characters appear roughly equally.
- **Quantization**: Use int8 post-training quantization for MCU deployment.

## UART Protocol

### P4 → S3: Multi-character detection (variable length)

```
Byte   0:     0xAA  (sync0)
Byte   1:     0x55  (sync1)
Byte   2:     0x01  (msg_type = inference)
Byte   3-4:   frame_id  (uint16 LE)
Byte   5-6:   payload_len (uint16 LE) = 2 + 2 * num_detections
Byte   7:     num_detections
Byte   8:     reserved (0x00)
Byte   9-10:  class_id_0, confidence_0
Byte   11-12: class_id_1, confidence_1
  ... up to kMaxDetections=16
Byte   N:     checksum (XOR of bytes 0..N-1)
```

### S3 → P4: Control messages (5 bytes, same as TFLite/)

```
Byte 0:     0xAA  (sync0)
Byte 1:     0x55  (sync1)
Byte 2:     0x02  (msg_type = control)
Byte 3:     command (0x01=ACK_STOP, 0x02=RESUME_JUNCTION)
Byte 4:     checksum (XOR of bytes 0..3)
```

## Configuration (`model_settings.h`)

Adjust these for your use case:

| Constant | Default | Description |
|---|---|---|
| `kFomoGridWidth` | 12 | Grid columns (model-dependent) |
| `kFomoGridHeight` | 12 | Grid rows (model-dependent) |
| `kFomoNumClasses` | 36 | 0–9 + A–Z (change for Chinese plates) |
| `kFomoConfidenceThreshold` | 128 | uint8 confidence threshold |
| `kFomoNmsRadius` | 1 | NMS radius in grid cells |
| `kMaxDetections` | 16 | Max characters per frame |

## Memory Budget (ESP32-P4)

| Component | Size |
|---|---|
| Tensor arena | ~384 KB (adjust based on model) |
| Model weights | Stored in Flash (const) |
| RGB buffer (96×96×3) | 27 KB (PSRAM) |
| B-G working buffers | ~54 KB (BSS) |
| FreeRTOS stacks | ~48 KB |
| **Total SRAM** | ~500 KB (fits P4's ~780 KB internal SRAM) |

## Build & Upload

```bash
# Arduino IDE:
#   1. Open PlateFOMO/PlateFOMO.ino
#   2. Board: ESP32P4 Dev Module
#   3. Install dependencies: TFLite Micro, ESP32_P4_IMX219, ESP-NN
#   4. Replace tm_model_data.cpp with your trained model
#   5. Upload

# Monitor serial output:
#   screen /dev/cu.usbmodem* 921600
```

## Differences from TFLite/

| Aspect | TFLite/ | PlateFOMO/ |
|---|---|---|
| Output format | Flat [kCategoryCount] vector | Grid [H×W×(1+C)] heatmap |
| Inference | Single argmax → one label | Centroid peaks → multiple labels |
| Model architecture | MobileNetV2 + FC + Softmax | MobileNetV2 truncated → grid |
| Op resolver | Conv2D, DWConv2D, FC, Softmax | Conv2D, DWConv2D, AvgPool, Add, Pad |
| Post-processing | `pick_label_and_confidence()` | `FomoPostProcess()` (5-pass pipeline) |
| Serial output | 9-byte fixed packet | Variable-length (up to ~40 bytes) |
| State machine | Junction ↔ Sign | Always plate mode |
| Tensor arena | 375 KB | 384 KB (FOMO needs more) |

## Next Steps

1. **Train a FOMO model** (Edge Impulse recommended — easiest centroid
   annotation workflow)
2. **Replace `tm_model_data.cpp`** with your trained int8 model bytes
3. **Adjust `model_settings.h`** to match your model's grid size and classes
4. **Test on desktop first** — the `FomoPostProcessFloat()` variant works
   with float32 TFLite models for debugging
5. **Tune thresholds** — `kFomoConfidenceThreshold` and `kFomoNmsRadius`
   are the main knobs for precision/recall trade-off

## Constraints

- **Characters must be ~8 px apart** (12×12 grid on 96×96 input).
  If your characters are closer, increase the grid resolution.
- **Model size grows with grid resolution**. A 24×24 grid means 576 cells ×
  37 channels = 21,312 output values — the backbone depth must also increase.
- **B-G preprocessing assumes blue/white plates**. For other plate colours,
  modify the colour channel extraction in `image_provider.cpp`.
- **Same pipeline rule**: Any change to `image_provider.cpp` MUST be mirrored
  in `AItraining/image_preprocess.py` for training data consistency.
