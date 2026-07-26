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

B-G extraction + auto-crop runs identically on P4 (C++) and in AItraining (Python):

```
1. Camera RGB → Library: demosaic + downsize to 96×96×3
2. WB correction:  R×2.0, B×2.0  (compensates for sensor green tint)
3. B-G extraction: diff = B - G   (signed int16)
4. Non-black mask: if R<10 & G<10 & B<10 → push diff to 0  (ignore shadows)
5. 5×5 Box Blur (C++) / GaussianBlur (Python)
6. Contrast stretch: (diff - min) / (max - min) × 255    skip if span < 20
7. Binary mask: stretched > 80 → white (sign), else black
8. Morphology: erode 1× + dilate 2×  (remove noise, reconnect fragments)
9. Blob detection: largest connected component → bbox
10. Crop: square bbox + 20% padding → nearest-neighbour resize to 96×96
11. B-G grayscale: (diff + 255) / 2 → uint8
12. Contrast stretch: span ≥ 24 → expand to full [0,255]
13. int8 conversion: gray - 128 → TFLite input
```

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

Only two modes remain:
- **`auto_by_label`** (default): B-G → auto blob detection → crop → resize
- **`manual_roi`**: B-G → user-defined ROI → crop → resize

B-G is always applied. The mode only controls ROI selection.

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
- **Pipeline consistency**: Any change to `image_provider.cpp` MUST be mirrored in `image_preprocess.py` and vice versa. WB gains, thresholds, and blur kernel size must match.
- **WB gains**: `200` (×2.0) in three places — `image_provider.cpp`, `IMX219_RGB_Serial.ino`, `image_preprocess.py`.
