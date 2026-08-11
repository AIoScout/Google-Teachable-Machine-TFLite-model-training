/*
  PlateFOMO — Class labels for FOMO character recognition.
  Edit this file to match your trained model's classes.
*/

#include "model_settings.h"

// Default: 0-9 + A-Z (36 classes).
// Replace with your actual charset, e.g. Chinese provinces + alphanumeric.
const char* kFomoClassLabels[kFomoNumClasses] = {
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
    "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T",
    "U", "V", "W", "X", "Y", "Z",
};
