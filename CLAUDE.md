# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-P4 + IMX219 camera pipeline for blue/purple sign detection. Uses B-G colour-difference preprocessing, morphological blob detection for auto-crop, and TFLite int8 inference. Targets ESP32-P4 (camera + inference), ESP32-S3 (UART receiver), and desktop (AItraining app).

**Two P4 sketches** share the same camera library:
- **TFLite.ino** — on-device inference: full pipeline → TFLite → sends labels over UART
- **IMX219_RGB_Serial** (library example) — data collection: sends WB-corrected 96×96 RGB to AItraining

## Repository Structure

Three directories are **git submodules** — changes inside them must be committed within their own repos:
- `AItraining/` → `koilkl/Aitraining` — Python/Streamlit desktop training app
- `ESP32-P4-IMX219-PoC/` → `koilkl/ESP32-P4-IMX219-PoC` — ESP-IDF camera firmware
- `TFLite/` → `koilkl/TFLite` — Arduino sketch with camera + TFLite Micro inference

Non-submodule directories:
- `S3_UART_Receiver/` — Arduino sketch for ESP32-S3: receives labels, sends ACK_STOP / RESUME_JUNCTION
- `arduino/` — custom Arduino board core for ESP32-P4 (zipped, install via `CORE_REBUILD.md`)

## Image Processing Pipeline

The ACTIVE pipeline (device `GetImage` BG mode, host live predict, and the
training cache — verified 2026-08-26, all three produce the same transform):

```
1. Camera RGB → Library: demosaic + downsize to 96×96×3 (raw, NO WB)
2. Crop: center 60 % square (side = floor(96*0.60) = 57, box [20,77))
       — Python _center_bbox(frac=0.60) / device BG_FALLBACK_CENTER_FRAC
       (the G-channel dark/lum mask and _focus_bbox search are preview aids
        only; they never touch the model-input pixels)
3. BT.601 luminance of the crop: (r*30 + g*59 + b*11) / 100  (no WB)
4. Bilinear resize to 96×96  (device float version ≈ PIL BILINEAR, ±1 LSB)
5. Contrast stretch: span ≥ 24 → expand to full [0,255]  (round half up)
6. int8 conversion: gray - 128 → TFLite input (scale 1/255, zp -128)
```

Training data = `IMX219_Grayscale_Serial` stream (library `esp32_p4_imx219_gray()`
= BT.601 of RAW demosaiced RGB, **no white balance**). `use_preprocessed_dataset`
is hardcoded true, so the training cache rebuild (`fast_mode=True`) is the
canonical transform; the model never sees `_focus_bbox` crops.

**Dead / legacy paths (do NOT "fix" device code to match these):**
- `_find_bg_roi` (B-G blob auto-crop, image_preprocess.py) — no callers.
- Device B-G blob search — compiled out (`BG_ENABLE_BLOB_SEARCH 0` default).
- `preprocess_array` / `focus_and_enhance_array` — dead + latent NameError.
- Host `_focus_bbox` live crop — replaced by `fast_mode=True` in
  `prepare_inference_inputs` (it crops ~13 px right of the sign and flipped
  24/40 LEFT samples to RIGHT on the training set).

**WB ×2.0 (R/B) exists ONLY in the RGB data-collection path** (`CameraSendRgbToSerialWb(200,200)`,
`IMX219_RGB_Serial` example — used by purple-sign projects). The grayscale
pipeline applies no WB anywhere; adding WB to the model input of a
grayscale-trained model is a distribution mismatch.

## ESP32_P4_IMX219 Library API

```
bool esp32_p4_imx219_begin()
bool esp32_p4_imx219_update()                  // capture frame

const uint8_t *esp32_p4_imx219_rgb()           // raw RGB
const uint8_t *esp32_p4_imx219_rgb_wb(r, b)    // WB-corrected (r/100, b/100)
const uint8_t *esp32_p4_imx219_gray()           // BT.601 grayscale

void esp32_p4_imx219_send_rgb_to_serial()       // raw RGB over Serial
void esp32_p4_imx219_send_rgb_to_serial_wb(r,b) // WB-corrected over Serial
```

WB gains: `200` = ×2.0. Must be identical across TFLite.ino, example sketch, and AItraining.

## Serial Protocols

**P4 → S3 (inference, 9 bytes):** `0xAA 0x55 0x01` + frame_id(LE16) + label_id + confidence + flags + checksum(XOR 0-7)
**S3 → P4 (control, 5 bytes):** `0xAA 0x55 0x02` + command + checksum(XOR 0-3)
  - `0x01` ACK_STOP / `0x02` RESUME_JUNCTION
**P4 → AItraining (data collection):** `0xAA 0x55 0xAA` + 96×96×3 RGB (27,648 bytes)

## P4 FreeRTOS Tasks (TFLite.ino)

| Task | Core | Priority | Stack | Role |
|---|---|---|---|---|
| `tflm` | 1 | 3 | 32 KB | Capture → pipeline → inference → queue packet |
| `uart_tx` | 0 | 2 | 4 KB | Send packets; drop when `s_transmit_enabled==false` |
| `uart_rx` | 0 | 2 | 4 KB | Read S3 control packets; toggle `s_transmit_enabled` |
| `sd` | 0 | 1 | 8 KB | Optional: write PGM frames to SD |

## AItraining Preprocess Modes

Two modes remain:
- **`auto_by_label`** (default): G-channel dark/lum mask (stats/preview only)
  → **center 60 % crop** (fast_mode=True, matches device + training cache)
- **`manual_roi`**: user-defined ROI → crop → resize

No B-G anywhere in the live path. The dark/lum mask only drives previews and
the sign_pct OOD gate. Per-class `bg_dark_thresh`/`bg_lum_thresh` from
class_preprocess are used by the training path (`preprocess_for_label`), not
by live predict (which uses 0/100 defaults); device mask defaults (0/100) and
OOD gates (sign_pct [0.3,70] %, max_prob ≥ 0.60, entropy ≤ 0.70) match live
predict.

## Build & Run Commands

```bash
# AItraining
cd AItraining && pip install -r requirements.txt && python desktop_launcher.py

# P4 TFLite inference
# Arduino IDE: open TFLite/TFLite.ino, board=ESP32P4 Dev Module, upload

# P4 data collection
# Arduino IDE: Examples → ESP32_P4_IMX219 → IMX219_RGB_Serial, upload

# S3 receiver
# Arduino IDE: open S3_UART_Receiver/S3_UART_Receiver.ino, board=ESP32S3 Dev Module
# Debug serial: 115200 baud (USB CDC)
```

## Critical Constraints

- **Python 3.13 + PyInstaller**: crashes. Use Python ≤ 3.12.
- **New P4 board**: FFat unavailable; use `StorageBackend::SdMmc`.
- **GPIO10/GPIO11 crosstalk**: `pinMode(kUartRxPin, INPUT_PULLDOWN)` before `UartToS3.begin()`.
- **Pipeline consistency**: Any change to `image_provider.cpp` MUST be mirrored in `image_preprocess.py` and vice versa. The canonical transform is: center-60 % crop → BT.601 (30/59/11) of raw no-WB RGB → bilinear → contrast stretch (span ≥ 24) → int8 gray−128.
- **WB gains**: `200` (×2.0) applies ONLY to the RGB data-collection path (`CameraSendRgbToSerialWb` / `IMX219_RGB_Serial`) used by purple-sign projects. The grayscale pipeline (device model input, training cache) must stay at 100/100 passthrough.
