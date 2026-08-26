# ESP32-P4 IMX219 — All-Black Sign Detection (Middle ROI)

Branch: `esp32p4-middle-detect`

Simplified pipeline for **all-black signs on white paper**. Uses raw G-channel brightness thresholding and center-focused shadow-search auto-crop. No colour processing needed — black is black in any lighting.

## Core Pipeline

```
G-channel → Dark/Lum mask → Shadow-search ROI → Crop → Resize → Contrast stretch
```

1. **G-channel**: Raw green channel as luminance (best SNR from IMX219 sensor)
2. **Dark/Lum mask**: `is_sign = (G > dark_thresh) & (G < lum_thresh)` — defaults 0/100 (preview + sign_pct OOD stats only)
3. **Model input** (training cache = live predict = device firmware): **center 60 % crop** → BT.601 luminance of the raw (no-WB) crop → bilinear 96×96 → contrast stretch (span ≥ 24) → int8 gray−128
4. **Shadow-search** (`_focus_bbox()`): dark-object + edge search — preview aid only, never touches the model-input pixels
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

- Center 60 % crop (`BG_FALLBACK_CENTER_FRAC=0.60`, blob search compiled out) — identical geometry to the host training cache
- No WB — model input is BT.601 luminance of the raw sensor RGB (matches the grayscale serial stream the app trains on)
- OOD gating aligned with the host preview gates: sign_pct 0.3–70 %, max_prob ≥ 0.60, entropy ≤ 0.70
- Export generates `model_resolver.h` from the actual ops in the .tflite

## Data Collection Sketches

- `IMX219_RGB_Serial.ino` — WB-corrected RGB (R×2 / B×2) — used by purple-sign projects
- `IMX219_Grayscale_Serial` (MODE_GRAY) — BT.601 of raw sensor RGB, **no WB** — this is what the small/upper road-sign projects train on

## Export Files

`*.tflite`, `*_model_data.h/.cpp`, `model.h/.cpp`, `model_settings.h/.cpp`, `model_resolver.h`, `labels.txt`

## Notes

- Streamlit 1.57: `st.components.v1.html()` deprecated — SPA served via `st.iframe()` + HTTP `/shell` route
- Model output: softmax stripped by int8 conversion — preview auto-detects and applies it
- Uploads handle grayscale, RGB, RGBA, palette formats
