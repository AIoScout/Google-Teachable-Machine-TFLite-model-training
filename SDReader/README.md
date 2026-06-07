# SDReader (ESP32-P4)

This sketch is a minimal SD card test for ESP32-P4.

## What it does

- Mounts the SD card using `SD_MMC` (SDMMC mode)
- Creates a new folder named `run_0001`, `run_0002`, ... (by scanning the SD root)
- Writes a small test image to `/<run_xxxx>/test.pgm`

If you want a stable flash-based storage test (no MicroSD required), use [FFatReader](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/FFatReader/FFatReader.ino).

## How to use

1. Insert an SD card.
2. Upload `SDReader/SDReader.ino` to ESP32-P4.
3. Open Serial Monitor at **921600** baud.
4. Look for logs like:
   - `SD mounted ...`
   - `Run dir: /sdcard/run_0001`
   - `Write /sdcard/run_0001/test.pgm OK`
5. Remove the SD card and check the `run_000x` folder on your computer.

## Configurable parameters

All parameters are at the top of [SDReader.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/SDReader/SDReader.ino).

- `kDebugBaud`: Serial Monitor baud rate
- `kUseSdMmcCustomPins`: set `true` if your board requires custom SD_MMC pins
- `kSdClkPin`, `kSdCmdPin`, `kSdD0Pin`, `kSdD1Pin`, `kSdD2Pin`, `kSdD3Pin`: custom SD_MMC pins

## Notes / common errors

- `sdmmc_card_init failed (0x107)`: SD card did not respond in time (power / pull-ups / contact / timing).
- `send_if_cond returned 0x108`: invalid SD command response (often caused by unstable signals).
- The on-board TF/MicroSD slot usually needs proper pull-ups on CMD and DAT lines. If internal pull-ups are not enough, add external 10k pull-ups to 3.3V.
- exFAT is not supported by default; format the card as FAT32.
