/*
  PlateFOMO — Model data header.

  Replace the placeholder arrays in tm_model_data.cpp with your trained
  FOMO int8 TFLite model, exported as a C byte array.

  To generate the C array from a .tflite file:
    xxd -i model.tflite > tm_model_data.cpp
    (then edit to use the same symbol names as below)
*/

#ifndef PLATEFOMO_TM_MODEL_DATA_H_
#define PLATEFOMO_TM_MODEL_DATA_H_

extern const unsigned char g_platefomo_model_data[];
extern const int g_platefomo_model_data_len;

#endif  // PLATEFOMO_TM_MODEL_DATA_H_
