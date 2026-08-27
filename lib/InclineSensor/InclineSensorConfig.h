#pragma once
#include <hal/gpio_types.h>
#include <stdint.h>

namespace stridecontrol {

struct InclineSensorConfig {
  gpio_num_t inputPin = GPIO_NUM_14;
  bool useInternalPullup = false;
  uint32_t minimumPulseIntervalUs = 1000;
  uint8_t movementStartPulseCount = 4;
  uint32_t movementStartWindowMs = 30;
  uint32_t movementStopTimeoutMs = 100;
  uint32_t maximumMovementTimeMs = 60000;
  float minimumInclinePct = 0.0f;
  float maximumInclinePct = 15.0f;
};

} // namespace stridecontrol

