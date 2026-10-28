# Attribution: ESP32-P4 board core

The `esp32_mannual:esp32p4` core is **derived from Espressif's
arduino-esp32** (Boards Manager package `esp32` v3.3.0):

- Upstream: <https://github.com/espressif/arduino-esp32>
- Copyright: Espressif Systems (Shanghai) Co., Ltd. and contributors
- License: **GNU LGPL-2.1** (see the core's `LICENSE.md`); bundled
  ESP-IDF components are Apache-2.0.

Modifications by **AIoScout** (<https://aioscout.io>):

- [List what you changed here, e.g. "board variant definition for the
  ESP32-P4 Dev Module added under variants/", "pin mapping adjusted in
  pins_arduino.h", "flash partition layout changed in boards.txt"]

Obligations we comply with (LGPL-2.1):

1. **Same license for modified files**: files derived from LGPL-2.1
   code keep their LGPL-2.1 license and original copyright headers —
   do not delete those headers. New copyright lines may be added above
   them (e.g. `Copyright 2026 AIoScout`).
2. **Source availability**: the complete source of this core ships
   WITH the core (it is installed as source into the Arduino hardware
   folder), which satisfies the LGPL source requirement. There is no
   obligation to publish the core or the sketches online.
3. **Attribution**: keep `LICENSE.md`, this file, and the original
   `README.md` next to the core whenever it is re-zipped or handed to
   a customer.

When rebuilding the distributable core zip (see `CORE_REBUILD.md`),
always include: `LICENSE.md`, `README.md`, and this `ATTRIBUTION.md`.
