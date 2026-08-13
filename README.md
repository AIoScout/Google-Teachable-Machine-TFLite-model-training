# ESP32-P4 IMX219 — All-Black Sign Detection (Middle ROI)

Branch: `esp32p4-middle-detect`

Simplified pipeline for **all-black signs on white paper**. Uses raw G-channel brightness thresholding and center-focused shadow-search auto-crop. No colour processing needed — black is black in any lighting.

## Core Pipeline

```
G-channel → Dark/Lum mask → Shadow-search ROI → Crop → Resize → Contrast stretch
```

1. **G-channel**: Raw green channel as luminance (best SNR from IMX219 sensor)
2. **Dark/Lum mask**: `is_sign = (G > dark_thresh) & (G < lum_thresh)` — defaults 0/100
3. **Shadow-search**: `_focus_bbox()` finds dark objects with edges in center 70% of frame
4. **Resize**: Configurable (48–192, default 96×96)
5. **Contrast stretch**: Linear to full [0,255]

## Preview Toggles (independent states, they stack)

| Input | ROI | Orig | Preview shows |
|---|---|---|---|
| OFF | — | — | No preview (source closed) |
| ON | OFF | OFF | Raw camera image |
| ON | ON | OFF | Cropped region only (no filter) |
| ON | OFF | ON | Processed filter on full frame (dark/lum threshold) |
| ON | ON | ON | Cropped AND filtered |

## Preprocessing Modal (gear icon per class)

| Control | Default | Range | Purpose |
|---|---|---|---|
| Preprocess Mode | Auto | Auto / Manual ROI | Shadow search or manual crop |
| Image Size | 96×96 | 48/96/128/192 | Output resolution (syncs with training) |
| ROI Overlay | Show | Show / Hide | Green crop box on original sample |
| Dark Thr | 0 | 0–100 | Pixels darker than this → white |
| Lum Thr | 100 | 50–255 | Pixels brighter than this → white |

Keyboard: F1 Auto, F2 Manual ROI, S Save, D Delete, Esc Close, ←→↑↓ navigate samples, Shift+arrows nudge ROI.

## Backend Preview Variants

The `/preview/predict` endpoint returns four image variants:

| Key | Content |
|---|---|
| `image_b64` | Raw camera frame |
| `crop_image_b64` | Crop region only, no threshold |
| `full_image_b64` | Thresholded full frame, no crop |
| `processed_image_b64` | Thresholded + cropped (model input view) |

## Search Window

15–85% both axes (center 70%). Prior center (0.50, 0.50). Fallback center crop. Min 18 focus pixels.

## MCU (TFLite)

- Shadow-search blob detection in center window
- Fixed ×2.0 WB via `esp32_p4_imx219_rgb_wb(200, 200)`
- Export generates `model_resolver.h` from the actual ops in the .tflite

## Data Collection Sketch

- `IMX219_RGB_Serial.ino` — dynamic Gray World AWB with IIR smoothing
- Raw sensor capture via `esp32_p4_imx219_rgb()`

## Export Files

`*.tflite`, `*_model_data.h/.cpp`, `model.h/.cpp`, `model_settings.h/.cpp`, `model_resolver.h`, `labels.txt`

## Notes

- Streamlit 1.57: `st.components.v1.html()` deprecated — SPA served via `st.iframe()` + HTTP `/shell` route
- Model output: softmax stripped by int8 conversion — preview auto-detects and applies it
- Uploads handle grayscale, RGB, RGBA, palette formats
