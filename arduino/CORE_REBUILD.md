# ESP32-P4 Core Setup (Arduino IDE)

This document only covers two things:

1. Install Espressif's `esp32` Boards Manager package (version 3.3.0)
2. Install this repository's custom core `esp32_mannual:esp32p4` into the correct Arduino hardware path

## 1) Install the `esp32` Boards Manager package (v3.3.0)

In Arduino IDE:

- Tools → Board → Boards Manager
- Search `esp32`
- Select `esp32` by Espressif Systems
- Choose version `3.3.0` and install

If you cannot find `esp32` in Boards Manager, add this Boards Manager URL in Arduino IDE Preferences first:

- `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`

## 2) Install this core into Arduino's hardware folder

First, confirm your Sketchbook location (shown in Arduino IDE Preferences). We use `<Sketchbook>` as the placeholder below.

Place the `esp32p4` folder into:

- `<Sketchbook>/hardware/esp32_mannual/esp32p4/`

Common default paths:

- macOS: `/Users/<you>/Documents/Arduino/hardware/esp32_mannual/esp32p4/`
- Windows: `C:\Users\<you>\Documents\Arduino\hardware\esp32_mannual\esp32p4\`

Restart Arduino IDE after copying the files.

## 3) Select the board in Arduino IDE

- Tools → Board → `ESP32_mannual` → `ESP32P4 Dev Module`

## 4) Avoid duplicate library conflicts (recommended)

If an old library with the same name exists, Arduino IDE may pick the wrong one. Remove/rename:

- `<Sketchbook>/libraries/ESP32_P4_IMX219`
