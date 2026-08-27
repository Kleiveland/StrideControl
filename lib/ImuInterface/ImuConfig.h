#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "ImuTypes.h"

namespace stridecontrol {

struct ImuConfig {
  TwoWire* wire = &Wire;
  int sdaPin = 47;
  int sclPin = 48;
  uint32_t i2cFrequencyHz = 100000;
  uint8_t i2cAddress = 0x6A;

  ImuAccelRange accelRange = ImuAccelRange::G8;
  ImuGyroRange gyroRange = ImuGyroRange::Dps500;
  // Note: V1 supports only ImuOutputDataRate::Hz104.
  ImuOutputDataRate outputDataRate = ImuOutputDataRate::Hz104;

  // FIFO and execution budget limits (Initial tuning values)
  uint16_t fifoWatermark = 16;
  uint16_t maximumFifoEntriesPerUpdate = 16;
  uint32_t maximumUpdateTimeUs = 10000;  // 10 ms maximum time budget per update()

  // Timing and timeouts
  uint32_t dataStaleTimeoutMs = 250;
  uint32_t resetTimeoutMs = 500;
  uint32_t stabilizationWindowMs = 1000; // Continuous stable duration required
  uint32_t maximumGyroPairAgeUs = 15000;  // ~1.5 ODR periods at 104 Hz

  // Stability and filter thresholds (Initial tuning values)
  float accelerationNormToleranceG = 0.10f;
  float maximumStableGyroDps = 2.0f;
  float maximumStableAngleVarianceDeg2 = 0.05f;
  float shortAngleFilterAlpha = 0.10f;

  ImuAxisMapping axisMapping{};
};

}  // namespace stridecontrol
