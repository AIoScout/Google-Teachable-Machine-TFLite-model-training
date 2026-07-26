# ESP32-P4 IMX219 — Blue/Purple Sign Detection

ESP32-P4 + IMX219 (MIPI CSI-2) pipeline for detecting blue/purple geometric signs using **B-G colour-difference preprocessing** and TFLite int8 inference. Two ESP32 boards: P4 runs camera + inference, S3 receives results and controls the P4 via bidirectional UART.

## Project Structure

- **TFLite**: Arduino sketch — on-device inference: B-G pipeline → TFLite → UART labels to S3
- **S3_UART_Receiver**: Arduino sketch — receives inference packets, confirms signs, sends ACK_STOP/RESUME_JUNCTION back to P4
- **AItraining**: Python/Streamlit desktop app for data collection (from P4 serial or webcam), B-G preprocessing, model training, and export
- **ESP32-P4-IMX219-PoC**: ESP-IDF project with raw camera streaming (legacy)
- **TMConnector**: Processing sketch bridging serial → WebSocket for Google Teachable Machine (legacy)
- **arduino**: Custom ESP32-P4 Arduino board core (zipped, install guide in `CORE_REBUILD.md`)
- **SDReader** / **FFatReader**: Storage test/utility sketches

## Setup Instructions for Remote AI training 

### 1. Hardware Preparation

- ESP32-P4 development board
- IMX219 camera connected via MIPI CSI-2
- Arduino users: follow [CORE_REBUILD.md](/arduino/CORE_REBUILD.md) to install the ESP32-P4 board core (paths included).

### 2.a Build & Flash ESP32-P4 Firmware(If using ESP-IDF)

1. Build and flash `ESP32-P4-IMX219-PoC` with ESP-IDF.
2. Ensure the firmware outputs frames as `0xAA 0x55 0xAA + 96*96 bytes` (grayscale) over the selected serial port.

### 2.b Build & Flash ESP32-P4 Firmware(If using Arduino)
1. Install Espressif's `esp32` Boards Manager package (version **3.3.0**) in Arduino IDE.
2. Copy this repository's custom core to:
   - macOS default: `/Users/<you>/Documents/Arduino/hardware/esp32_mannual/esp32p4/`
   - Windows default: `C:\Users\<you>\Documents\Arduino\hardware\esp32_mannual\esp32p4\`
3. Restart Arduino IDE, then select:
   - Tools → Board → `ESP32_mannual` → `ESP32P4 Dev Module`
4. Configure the Arduino IDE Tools menu for ESP32-P4 exactly like the screenshot: [setting_p4.png](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/setting_p4.png).

![setting_p4](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/setting_p4.png)

5. Open the built-in example:
   - File → Examples → `ESP32_P4_IMX219` → `IMX219_Grayscale_Serial`
6. Upload to the ESP32-P4 and make sure it outputs frames as `0xAA 0x55 0xAA + 96*96 bytes` over Serial.

### 2.c Build & Flash ESP32-S3 Receiver (If using Arduino)

1. Open [S3_UART_Receiver/S3_UART_Receiver.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/S3_UART_Receiver/S3_UART_Receiver.ino).
2. Select board: `ESP32S3 Dev Module`.
3. Configure the Arduino IDE Tools menu for ESP32-S3 exactly like the screenshot: [setting_s3.png](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/setting_s3.png).

![setting_s3](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/setting_s3.png)

4. Upload to the ESP32-S3 and open Serial Monitor at **115200** baud (USB CDC) to check logs.

### 3. Run Processing Script (TMConnector)

1. Download and install [Processing IDE](https://processing.org/).
2. Install required libraries:
   - In Processing, go to `Tools` -> `Add Tool...` -> `Libraries`.
   - Search and install `Websockets` and `ControlP5`.
3. Open `TMConnector/TM_Connector/TM_Connector.pde`.
4. Click the "Run" button.
5. In the pop-up window, select the serial port corresponding to your ESP32-P4 from the dropdown list.
6. You should now see a real-time 96x96 grayscale preview on the left.

### 4. Connect to Teachable Machine

1. Visit Teachable Machine **Network** mode:
   - <https://teachablemachine.withgoogle.com/train/image?network=true>
2. Choose "device" as the input source.
3. Host: `localhost`  Port: `8889`  then click Connect.
4. Ensure the connection indicator (small dot on the right) in the Processing window turns green.

Notes:

- Safari is sometimes not supported for this workflow (WebSocket Network input in Teachable Machine is unreliable/blocked). Use Chrome is a better choice.

## About the core instructions

- The current [CORE_REBUILD.md](/arduino/CORE_REBUILD.md) is a simplified “how to install and use the provided core” guide (mainly about correct paths).
- Originally, this repository also used a “rebuild the Arduino core with esp32-arduino-lib-builder” workflow to bring `esp_video`/CSI into the core. If you need to rebuild from source, use Espressif's official lib-builder documentation: <https://docs.espressif.com/projects/arduino-esp32/en/latest/lib_builder.html>

## Frame Saving (SD / FFat)

Saving frames to the on-board MicroSD/TF slot via SD_MMC can be unstable on some ESP32-P4 boards (common errors: `0x107` timeout, `0x108` invalid response). This repository provides two storage options:

- **SD_MMC (MicroSD/TF)**: removable, easy to copy files on a computer, but may be unstable depending on the board hardware.
- **FFat (flash FAT partition)**: files are stored in the device flash (`ffat` partition). On the current new ESP32-P4 board, FFat is not available.

### TFLite storage backend

In [TFLite.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/TFLite/TFLite.ino) you can select the storage backend:

- `StorageBackend::Auto`: try SD_MMC (`/sdcard`) first, then fall back to FFat (`/ffat`)
- `StorageBackend::SdMmc`: SD_MMC only
- `StorageBackend::FFat`: FFat only

New ESP32-P4 board note:

- The current new board cannot use FFat. Use `StorageBackend::SdMmc`.

### Tools

- SD_MMC test: [SDReader/SDReader.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/SDReader/SDReader.ino)
- FFat export/clean tool: [FFatReader/FFatReader.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/FFatReader/FFatReader.ino)
  - Serial export receiver script: [FFatReader/export_ffat.py](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/FFatReader/export_ffat.py)

### Exporting FFat files

- **Serial export**: use the commands in FFatReader (`bundle_runs`, `get`, `clean_*`) and receive data with `export_ffat.py`.
- **USB MSC export**: set `kInterfaceMode = UsbMsc` in FFatReader and configure Arduino IDE exactly like [FFatReader/MSC_setting.png](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/FFatReader/MSC_setting.png). The board will appear as a USB drive on your computer.

### FFat capacity (rule of thumb)

With the default partition scheme `app3M_fat9.9M_flash16MB`, FFat is about **9MB**.

- One 96×96 grayscale frame stored as raw bytes is `96 * 96 = 9216 bytes(Not include the PGM file header)`
- Approx maximum number of frames:
  - `~ 9 * 1024 * 1024 / 9216 ≈ 1000 frames`

When FFat is full, the TFLite sketch will stop saving new frames and keep running inference.

## Key Features

- **B-G pipeline + auto-crop**: B-G extraction → blur → contrast stretch → blob detection → auto-crop. Identical on P4 (C++) and AItraining (Python). See `CLAUDE.md` for full 13-step pipeline.
- **Auto crop**: Finds the sign automatically via morphological blob detection. Falls back to full-frame when no sign is detected.
- **WB correction**: Software white-balance (R×2.0, B×2.0) compensates for sensor green tint. Applied in library (`rgb_wb`), TFLite.ino, and AItraining.
- **Bidirectional P4 ↔ S3**: S3 confirms signs and sends `ACK_STOP` / `RESUME_JUNCTION`. See `S3_UART_Receiver/README.md`.
- **RGB data collection**: `IMX219_RGB_Serial` example sends WB-corrected 96×96×3 RGB. AItraining "device" source receives and saves raw images.
- **Baud rates**: 921600 (P4↔S3, P4 debug), 115200 (S3 debug).
- **Sync header**: `0xAA 0x55 0xAA` on all serial streams.
