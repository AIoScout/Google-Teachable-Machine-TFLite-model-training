# FFatReader (ESP32-P4)

This sketch is a minimal **FFat** test for ESP32-P4.

FFat uses an internal FAT partition in the device flash (your partition table contains a partition named `ffat`), so it does not depend on the on-board MicroSD slot.

## What it does

- Mounts FFat at `/ffat`
- If no filesystem is detected, it can format the `ffat` partition automatically (configurable)
- Supports exporting files from `/ffat` to your computer (Serial bundle or USB MSC)
- Supports cleaning specific folders or all `run_XXXX` folders

## How to use

1. Upload `FFatReader/FFatReader.ino` to ESP32-P4.
2. Open Serial Monitor at **921600** baud.
3. Look for `Mounted FFat: total=... used=...`
4. Type `help` to see commands

## Serial export tool

After boot, the sketch accepts simple commands on the Serial Monitor:

- `help`
- `gen_template` (create a new `run_XXXX` folder and write a `test.pgm`)
- `ls` or `ls /ffat/run_0001`
- `get_last` (dump the latest `test.pgm` written by this sketch)
- `get /ffat/run_0001/test.pgm`
- `bundle_runs` (dump all files under `/ffat/run_*/` into one stream)
- `bundle /ffat` (dump all files under a directory into one stream)
- `clean_runs` (delete all `/ffat/run_*/` folders)
- `clean /ffat/run_0001` (delete a specific folder/file recursively)
- `clean_all` (delete everything under `/ffat`)

The `get*` commands print:

- `BEGIN <path> <size>`
- base64 payload (multiple lines)
- `END`

### Python receiver

Use [export\_ffat.py](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/FFatReader/export_ffat.py) to fetch the file and save it on your computer:

```bash
python3 FFatReader/export_ffat.py --port /dev/tty.usbmodemXXXX --baud 921600 --out out.pgm --cmd get_last
```

To export all `run_XXXX` folders:

```bash
python3 FFatReader/export_ffat.py --port /dev/tty.usbmodemXXXX --baud 921600 --out-dir out_runs --cmd bundle_runs
```

## Configurable parameters

All parameters are at the top of [FFatReader.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/FFatReader/FFatReader.ino).

- `kDebugBaud`: Serial Monitor baud rate
- `kFormatOnFail`: format the FFat partition if mount fails (recommended for first run)
- `kInterfaceMode`:
  - `Serial`: use the Serial export/clean commands
  - `UsbMsc`: expose `/ffat` to your computer as a USB Mass Storage drive

## Exporting images to your computer

FFat stores files in the on-board flash, so you cannot remove a card to read it.

Two common ways to export:

1. USB MSC (Mass Storage) mode: expose the FFat partition to the host computer as a USB drive.
2. Serial dump: stream the file bytes through UART/USB and reconstruct the file on the computer.

### USB MSC requirements

- Set `kInterfaceMode = UsbMsc` in `FFatReader.ino` and upload
- Arduino IDE Tools:
  - USB Mode: `USB-OTG (TinyUSB)`
  - CDC On Boot: `Enabled` (optional, only for logs)
- Use the exact Arduino IDE settings shown in [MSC\_setting.png](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/FFatReader/MSC_setting.png).
- Make sure plug the type-c cable in the OTG port with your computer. 

In MSC mode, the sketch does not mount `/ffat` in the firmware. Use your computer file manager to copy/delete files.

## FFat capacity limit (important)

FFat is not infinite. With the partition scheme used by this repository (`app3M_fat9M_16MB`), FFat is roughly **9MB** usable.

- One 96×96 grayscale frame is `96 * 96 = 9216 bytes`
- Approx max frames:
  - `~ 9 * 1024 * 1024 / 9216 ≈ 1000 frames`

The real number is slightly lower due to filesystem overhead and PGM headers.

When FFat is full:

- `TFLite` will stop saving new frames (inference continues).
- You can delete old runs using FFatReader (`clean_runs`, `clean_all`, or `clean <path>`).
