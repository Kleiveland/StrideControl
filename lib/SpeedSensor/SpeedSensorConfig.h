#pragma once
#include <Arduino.h>

namespace stridecontrol {

struct SpeedSensorConfig {
  gpio_num_t inputPin = GPIO_NUM_3;
  bool useInternalPullup = false;
  uint32_t glitchRejectUs = 300;
  uint32_t pulseLockoutUs = 2000;
  uint32_t pulseTimeoutMs = 2000;
  float kmhPerHz = 1.1148f;

  SpeedSensorConfig() = default;
  SpeedSensorConfig(gpio_num_t pin, bool pullup, uint32_t glitch, uint32_t lockout, uint32_t timeout, float factor)
      : inputPin(pin),
        useInternalPullup(pullup),
        glitchRejectUs(glitch),
        pulseLockoutUs(lockout),
        pulseTimeoutMs(timeout),
        kmhPerHz(factor) {}
};

}  // namespace stridecontrol
