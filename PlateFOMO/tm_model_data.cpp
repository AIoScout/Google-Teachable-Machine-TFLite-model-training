/*
  PlateFOMO — Placeholder model data.

  ╔══════════════════════════════════════════════════════════════════════╗
  ║  REPLACE this file with your trained FOMO int8 TFLite model data.   ║
  ║                                                                      ║
  ║  To export the model as a C array:                                   ║
  ║    xxd -i your_fomo_model.tflite > tm_model_data.cpp                 ║
  ║                                                                      ║
  ║  Then edit the generated arrays to match:                            ║
  ║    const unsigned char g_platefomo_model_data[] = { ... };           ║
  ║    const int g_platefomo_model_data_len = <size>;                    ║
  ╚══════════════════════════════════════════════════════════════════════╝

  If using Edge Impulse, the export includes a header + source pair.
  Rename / edit them to expose `g_platefomo_model_data` and
  `g_platefomo_model_data_len` as declared in tm_model_data.h.
*/

#include "tm_model_data.h"

// Dummy model — replace this entire array with your real model bytes.
// This is intentionally an invalid model — it causes an early schema
// version mismatch error in setup(), reminding you to replace it.
const unsigned char g_platefomo_model_data[] = {
  0x00, 0x00, 0x00, 0x00,
};
const int g_platefomo_model_data_len =
    sizeof(g_platefomo_model_data) / sizeof(g_platefomo_model_data[0]);
