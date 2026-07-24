# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-P4 + IMX219 camera pipeline for streaming 96×96 grayscale frames to Google Teachable Machine, training int8-quantized TFLite models, and running on-device inference. Targets ESP32-P4 (camera + inference), ESP32-S3 (UART receiver/display), and desktop (training host).

## Repository Structure

Three directories are **git submodules** — changes inside them must be committed within their own repos:
- `AItraining/` → `koilkl/Aitraining` — Python/Streamlit desktop training app
- `ESP32-P4-IMX219-PoC/` → `koilkl/ESP32-P4-IMX219-PoC` — ESP-IDF camera firmware
- `TFLite/` → `koilkl/TFLite` — Arduino sketch with camera + TFLite Micro inference

Non-submodule directories:
- `TFLite-for-Roadsign/` — standalone CLI training pipeline for road sign classification (pixi-based)For
- `S3_UART_Receiver/` — Arduino sketch for ESP32-S3 to decode inference packets from P4 over UART
- `TMConnector/` — Processing sketch bridging serial → WebSocket for Google Teachable Machine
- `SDReader/` / `FFatReader/` — storage test/utility sketches for the ESP32-P4
- `arduino/` — custom Arduino board core for ESP32-P4 (zipped, with install guide)

## Architecture & Data Flow

```
IMX219 (MIPI CSI-2) → ESP32-P4 → UART (0xAA 0x55 0xAA + 96×96 bytes, 921600 baud)
                                    ├── ESP32-S3 (UART receiver, prints labels)
                                    │       └── S3 → P4 control: 0xAA 0x55 0x02 + cmd + csum (ACK_STOP / RESUME_JUNCTION)
                                    └── TMConnector (Processing) → WebSocket :8889 → Teachable Machine
                                             │
                                    AItraining (Python/Streamlit) ← serial or WebSocket
                                         │
                                    int8 TFLite model exported as C arrays (model.h/cpp)
                                         │
                                    Deployed back to TFLite/ Arduino sketch on ESP32-P4
```

The serial sync protocol uses a 3-byte header (`0xAA 0x55 0xAA`) followed by `side × side` raw grayscale bytes. The TFLite sketch also sends 9-byte inference result packets over UART to the S3 receiver: `0xAA 0x55 0x01` + frame_id (LE uint16) + label_id (uint8) + confidence (uint8) + flags (uint8) + checksum (XOR).

The S3 can send 5-byte control packets back to P4: `0xAA 0x55 0x02` + command (uint8) + checksum (XOR). Commands: `0x01` = ACK_STOP (confirmed sign, P4 stops TX), `0x02` = RESUME_JUNCTION (tasks done, P4 resumes TX + switches to junction crop mode).

## Build & Run Commands

### AItraining (Python desktop app)

```bash
cd AItraining
pip install -r requirements.txt
python desktop_launcher.py          # starts Streamlit server + pywebview window
pip install -r requirements-dev.txt  # for packaging: pyinstaller, dmgbuild
```

PyInstaller packaging is currently broken on Python 3.13 — TensorFlow submodule collection crashes the child process. See `debug-pyinstaller-tensorflow-crash.md` for details and work-in-progress hypotheses.

### TFLite-for-Roadsign (CLI training)

```bash
cd TFLite-for-Roadsign
pixi run train                      # defined in pixi.toml → python src/trainer.py
```

Supports flags: `--preview-image`, `--preview-all`, `--no-focus`, `--no-c-array`.

### ESP-IDF firmware (ESP32-P4-IMX219-PoC)

```bash
cd ESP32-P4-IMX219-PoC
idf.py build flash monitor          # requires ESP-IDF v5.5.4 + ESP32-P4 toolchain
```

### Arduino sketches (TFLite, S3_UART_Receiver, SDReader, FFatReader)

Requires a **custom Arduino board core** for ESP32-P4 — install from `arduino/esp32_mannual.zip` following `arduino/CORE_REBUILD.md`. Board: `ESP32P4 Dev Module` under `ESP32_mannual`. See `setting_p4.png` and `setting_s3.png` for required Tools menu configuration. Baud rate: **921600** for P4 examples, **115200** for S3 debug serial.

### Processing bridge (TMConnector)

Open `TMConnector/TM_Connector/TM_Connector.pde` in Processing IDE. Requires libraries: `Websockets` and `ControlP5` (install via Tools → Add Tool → Libraries).

## Key Patterns

### Bidirectional P4 ↔ S3 Communication

The P4 and S3 share a bidirectional UART link at 921600 baud. P4 uses `HardwareSerial(1)` (RX=10, TX=11), S3 uses `Serial0` (RX=44, TX=43).

**P4 → S3 (inference results, 9 bytes):**
`0xAA 0x55 0x01` + frame_id(LE uint16) + label_id(uint8) + confidence(uint8) + flags(uint8) + checksum(XOR Byte0..7)

**S3 → P4 (control commands, 5 bytes):**
`0xAA 0x55 0x02` + command(uint8) + checksum(XOR Byte0..3)

Commands:
- `0x01` (`kCtrlAckStop`): S3 confirmed a sign → P4 calls `uart_control_disable()` to stop TX
- `0x02` (`kCtrlResumeJunction`): S3 tasks done → P4 calls `uart_control_enable()` + sets `s_detection_state=0` + `s_no_sign_frames=0` (switches to junction crop mode)

**P4 FreeRTOS UART tasks:**

| Task | Core | Priority | Stack | Role |
|---|---|---|---|---|
| `uart_tx` | 0 | 2 | 4 KB | Sends inference packets; drops when `s_transmit_enabled == false` |
| `uart_rx` | 0 | 2 | 4 KB | Reads control packets from S3; sets `s_transmit_enabled` + `s_detection_state` |

**TX gating:** `uart_control_enable()` / `uart_control_disable()` set `volatile bool s_transmit_enabled`. When disabled, `uart_tx_task` drains the queue (no backpressure) but doesn't write to UART. Inference, crop mode, and SD logging continue unaffected.

**S3 sign confirmation:** Tracks consecutive high-confidence frames in `uart_task`:
- `kSignConfirmFrames` (5): consecutive frames of same sign class needed
- `kSignConfirmConfidence` (180): minimum confidence (~70%)
- On confirm: sends ACK_STOP → waits `kTaskDurationMs` (3000ms) → sends RESUME_JUNCTION
- Replace the `vTaskDelay(kTaskDurationMs)` with real task logic (display, actuator, etc.)

### Model Training & Export Pipeline

The training pipeline (`AItraining/trainer.py`, `TFLite-for-Roadsign/src/trainer.py`) follows the same pattern:
1. Build a 3-conv-layer CNN with augmentation
2. Train with representative dataset calibration for int8 quantization
3. Convert Keras model → quantized TFLite flatbuffer
4. Export as C byte arrays: `model.h` (extern declaration), `model.cpp` (data), `model_settings.h` (dimensions, label count, labels array), `model_resolver.h` (TFLite op registration)

These generated files are the ones consumed by the `TFLite/` Arduino sketch on-device.

### Auto-Generated Files (do not hand-edit)

In `TFLite/`: `tm_model_data.h`, `tm_model_data.cpp`, `model_settings.h`, `model_resolver.h` are all produced by the training export step. Changes to model architecture or labels should be made in the training code, not in these headers.

### Storage Backends (ESP32-P4)

The TFLite sketch supports two storage backends for saving captured frames:
- **SD_MMC** — MicroSD/TF card slot; can be unstable on some P4 boards (errors `0x107`/`0x108`)
- **FFat** — flash FAT partition (~9 MB, ~1000 frames); **unavailable on the current new P4 board**

Use `StorageBackend::SdMmc` for the new board. FFat export via `FFatReader/export_ffat.py` over serial, or via USB MSC mode.

## Debugging Conventions

Debug sessions are documented as structured markdown files at the repo root (`debug-*.md`), each following a hypothesis-driven format: symptom, hypotheses table (likelihood × effort), reproduction steps, and verification conclusions. Session logs live in `.dbg/` as NDJSON files with corresponding `.env` files mapping session names to log paths. When debugging a known issue, check whether a `debug-*.md` already covers it, and follow the same structured format for new investigations.

## Recent Fixes & Design Notes

### `kClassTypes` export bug (fixed in `record_controller.py:1191-1204` + `image_preprocess.py:409-414`)

Two-part fix:

1. **`_export_run`**: Now merges live class_labels from dataset metadata (`_class_labels_load`) with training-time labels: `{**live_class_labels, **train_class_labels}`. Also passes the merged dict to `class_clip_mode()` in the fallback.

2. **`class_clip_mode()`**: Added `if class_labels: return CLIP_MODE_SIGN` guard. When the dict is non-empty (user IS using labels), unknown classes now default to sign (0) instead of falling through to keyword heuristics. Empty/None dict still falls through for legacy projects.

Before this fix, classes left at the default "Sign" value in the UI were absent from `STATE.class_labels` → absent from training cfg → absent from metadata → keyword heuristic misclassified names containing road-direction tokens (`END`, `LEFT`, `RIGHT`, `STRAIGHT`, `TURN`, etc.) as road (type 1).

### Progressive lag / memory leak (fixed in `app.py:6843`)

`_tm_sample_previews(classes)` was called **without `limit_per_class`** on every Streamlit rerun, loading ALL images from ALL classes as base64 data URIs into the HTML response. As the dataset grows, this became linearly slower.

**Fix**: Changed to `limit_per_class=12`, matching `_processed_previews_payload`. New captures are added client-side via JS, so the server only ships initial thumbnails.

### Road-type class confusion (straight-line vs cross)

The class label system (`sign` vs `road`) feeds both road subtypes through **identical junction-mode preprocessing** (`_junction_bbox`: 40–75% height horizontal strip, aspect-preserving pad). The model must distinguish them purely on pixel content in the same narrow crop zone — difficult at 96×96 with only 25–26 samples each. To improve, use per-class preprocessing overrides (`class_preprocess`) to give different road types different crop modes, and collect more data (50–80+ per class).

## Known Constraints & Pitfalls

- **Python 3.13 + PyInstaller**: `collect_all('tensorflow')` crashes during packaging (mutex lock error in native runtime). Python ≤ 3.12 is recommended for packaging.
- **New ESP32-P4 board**: FFat partition is not available; use SD_MMC only (`StorageBackend::SdMmc`).
- **Safari**: Not supported for Teachable Machine WebSocket Network input — use Chrome.
- **160×160 frame size**: Serial bandwidth at 921600 baud may not reliably deliver 25,600-byte payloads within the read timeout; currently only 96×96 is proven stable.
- **Class label default**: `getClassLabel()` in the JS frontend defaults to `'sign'` but never populates `STATE.class_labels` — unset classes are entirely absent from the dict. The export fix above handles this, but any new code paths that read `class_labels` must account for missing entries.
