/*
  PlateFOMO — Op resolver for the FOMO model.

  FOMO (MobileNetV2 backbone truncated to output-grid resolution):
    - Conv2D
    - DepthwiseConv2D
    - AveragePool2D
    - Reshape
    - Add (residual connections in MobileNetV2 blocks)
    - Pad (optional, depends on backbone variant)

  No FullyConnected, no Softmax — FOMO is fully convolutional.

  If using Edge Impulse's generated model, replace this file with their
  auto-generated resolver or use the fallback MicroMutableOpResolver below.
*/

#ifndef PLATEFOMO_MODEL_RESOLVER_H_
#define PLATEFOMO_MODEL_RESOLVER_H_

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/c/common.h"

// ── Manual op resolver ───────────────────────────────────────────────────────
// Returns a MicroMutableOpResolver pre-populated with the ops that FOMO needs.
// Call this if you don't have an auto-generated resolver from Edge Impulse.

inline tflite::MicroMutableOpResolver<12>& FomoOpResolver() {
  // 12 slots — enough for MobileNetV2 backbone + standard FOMO head
  static tflite::MicroMutableOpResolver<12> resolver;
  static bool initialized = false;

  if (!initialized) {
    // MobileNetV2 core ops (order of addition doesn't matter)
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddAveragePool2D();
    resolver.AddReshape();
    resolver.AddAdd();               // residual adds in inverted bottleneck
    resolver.AddPad();               // "same" padding fallback
    resolver.AddRelu6();             // optional — MobileNetV2 uses ReLU6
    resolver.AddMul();               // optional — batch-norm folding may insert Mul
    resolver.AddFullyConnected();    // optional — some FOMO variants use FC head

    // FOMO head ops (may include upsampling for higher-res grids)
    // resolver.AddResizeNearestNeighbor();  // if FPN-like upsampling is used

    initialized = true;
    MicroPrintf("FomoOpResolver: 9+ ops registered");
  }

  return resolver;
}

// Alternative: if you have an Edge Impulse auto-generated resolver, include
// it instead of using the manual one above.
//
// #include "edge-impulse-sdk/classifier/ei_model_resolver.h"
// Then in setup() call the generated registration function.

#endif  // PLATEFOMO_MODEL_RESOLVER_H_
