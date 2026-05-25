# ESP32-P4 IMX219 Teachable Machine Camera Uploader

This project uses **ESP32-P4 + IMX219 (MIPI CSI-2)** to stream **96x96 grayscale** frames to Google Teachable Machine (Image Project) for model training.

## Project Structure

- **ESP32-P4-IMX219-PoC**: ESP-IDF project that captures IMX219 frames and can output 96x96 grayscale over Serial.
- **TMConnector**: Processing script. It receives Serial data, displays a preview, and forwards images to the web interface via WebSockets.
- **AItraining**: The local training platform
- TFLite: The template of using TensorFlow Lite in Arduino.
- arduino: The customized board core for CSI camera and TensorFlow Lite in the esp32p4

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
4. Open the built-in example:
   - File → Examples → `ESP32_P4_IMX219` → `IMX219_Grayscale_Serial`
5. Upload to the ESP32-P4 and make sure it outputs frames as `0xAA 0x55 0xAA + 96*96 bytes` over Serial.

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

## Key Features

- **Native Resolution**: Uses 96x96 resolution directly from the hardware, which is the standard input size for Teachable Machine. No extra cropping required.
- **Baud Rate**: Use **921600** for the Arduino examples in this repo.
- **Synchronization**: Uses a built-in `0xAA 0x55 0xAA` sync header to prevent image shifting or tearing.
