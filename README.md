# ESP32-P4 IMX219 Teachable Machine Camera Uploader

This project uses **ESP32-P4 + IMX219 (MIPI CSI-2)** to stream **96x96 grayscale** frames to Google Teachable Machine (Image Project) for model training.

## Project Structure

- **ESP32-P4-IMX219-PoC**: ESP-IDF project that captures IMX219 frames and can output 96x96 grayscale over Serial.
- **TMConnector**: Processing script. It receives Serial data, displays a preview, and forwards images to the web interface via WebSockets.

## Setup Instructions

### 1. Hardware Preparation
- ESP32-P4 development board
- IMX219 camera connected via MIPI CSI-2

### 2. Build & Flash ESP32-P4 Firmware
1. Build and flash `ESP32-P4-IMX219-PoC` with ESP-IDF.
2. Ensure the firmware outputs frames as `0xAA 0x55 0xAA + 96*96 bytes` (grayscale) over the selected serial port.

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
   - https://teachablemachine.withgoogle.com/train/image?network=true
2. Choose "Network" as the input source.
3. Host: `localhost`  Port: `8889`  then click Connect.
4. Ensure the connection indicator (small dot on the right) in the Processing window turns green.

Notes:
- Safari is not supported for this workflow (WebSocket Network input in Teachable Machine is unreliable/blocked). Use Chrome.

## Key Features
- **Native Resolution**: Uses 96x96 resolution directly from the hardware, which is the standard input size for Teachable Machine. No extra cropping required.
- **Baud Rate**: Fixed at **115200** for stable communication.
- **Synchronization**: Uses a built-in `0xAA 0x55 0xAA` sync header to prevent image shifting or tearing.
