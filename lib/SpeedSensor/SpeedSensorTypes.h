#pragma once
#include <Arduino.h>

namespace stridecontrol {

enum class SpeedSensorStatus : uint8_t {
  Uninitialized,
  Ready,
  AwaitingFirstPulse,
  AwaitingInterval,
  Measuring,
  TimedOut,
  HardwareError
};

struct SpeedSensorState {
  bool initialized = false;
  bool signalPresent = false;
  bool measurementValid = false;
  float frequencyHz = 0.0f;
  float speedKmh = 0.0f;
  uint32_t pulseIntervalUs = 0;
  uint32_t lastPulseTimestampUs = 0;
  uint32_t lastPulseAgeMs = 0;
  uint64_t acceptedPulseCount = 0;
  uint64_t rejectedGlitchCount = 0;
  uint64_t rejectedLockoutCount = 0;
  SpeedSensorStatus status = SpeedSensorStatus::Uninitialized;
};

const char* speedSensorStatusName(SpeedSensorStatus status);

}  // namespace stridecontrol

