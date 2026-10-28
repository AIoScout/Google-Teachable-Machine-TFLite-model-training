# Attribution: ESP32-P4 board core

The `esp32_mannual:esp32p4` core is **derived from Espressif's
arduino-esp32** (Boards Manager package `esp32` v3.3.x):

- Upstream: <https://github.com/espressif/arduino-esp32>
- Copyright: Espressif Systems (Shanghai) Co., Ltd. and contributors
- License: **GNU LGPL-2.1** (see the core's `LICENSE.md`); bundled
  ESP-IDF components are Apache-2.0.

## Modifications by AIoScout (<https://aioscout.io>)

Modified files (diffed against the installed Boards-Manager 3.3.1 package):

- `boards.txt` — board definitions / menu options for the custom core
- `platform.txt` — toolchain paths for `esp32_mannual` (riscv32 gcc/gdb,
  bundled `esp32-arduino-libs`)
- `cores/esp32/main.cpp` — enable FPU on riscv (`rv_utils_enable_fpu()`)
- `cores/esp32/esp32-hal-uart.c` — LP UART clock-source adjustment
- `cores/esp32/esp32-hal-tinyusb.c`, `cores/esp32/esp32-hal-hosted.c`,
  `cores/esp32/chip-debug-report.cpp` — small local adjustments
- `libraries/SD_MMC/src/SD_MMC.cpp` — SD power-cycling sequence
  (external power, 500 ms on/off delay, internal pullup flag)
- `variants/esp32p4/pins_arduino.h` — SDMMC power pin level/channel

Added files / folders:

- `cores/esp32/esp32-idf-compat.c`
- `cores/esp32/esp32p4_video_wrap.cpp`
- `libraries/ESP32_P4_IMX219/` — IMX219 camera driver library
- `libraries/ESP32_P4_OV5647/` — OV5647 camera driver library
- Partition schemes: `tools/partitions/*.csv` (huge_app 4/8/16MB,
  huge_app_ffat 8/16MB, `v_huge_app.csv`, `partition.csv`) and copies
  under `variants/esp32p4/`
- (`docs/`, `package/`, `tests/`, `idf_component_examples/`, extra
  `tools/` scripts, `README.md`, `LICENSE.md`, `CODE_OF_CONDUCT.md` are
  carried over from the upstream GitHub repository copy.)

## LGPL-2.1 obligations we comply with

1. **Same license for modified files**: files derived from LGPL-2.1
   code keep their LGPL-2.1 license and original copyright headers —
   do not delete those headers. New copyright lines may be added above
   them (e.g. `Copyright 2026 AIoScout`). Our OWN new files (the two
   camera libraries, `esp32p4_video_wrap.cpp`, partition CSVs) may keep
   any license we choose.
2. **Source availability**: the complete source of this core ships
   WITH the core (it is installed as source into the Arduino hardware
   folder), which satisfies the LGPL source requirement. There is no
   obligation to publish the core or the sketches online.
3. **Attribution**: keep `LICENSE.md`, `README.md` and this file next
   to the core whenever it is re-zipped or handed to a customer.

When rebuilding the distributable core zip (see `CORE_REBUILD.md`),
always include: `LICENSE.md`, `README.md`, and this `ATTRIBUTION.md`.
