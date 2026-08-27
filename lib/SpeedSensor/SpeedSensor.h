#pragma once
#include <Arduino.h>
#include "SpeedSensorConfig.h"
#include "SpeedSensorTypes.h"

namespace stridecontrol {

class SpeedSensor {
 public:
  SpeedSensor();
  ~SpeedSensor();

  SpeedSensor(const SpeedSensor&) = delete;
  SpeedSensor& operator=(const SpeedSensor&) = delete;

  bool begin(const SpeedSensorConfig& config = SpeedSensorConfig{});
  void end();
  void update();
  SpeedSensorState getState() const;
  bool isReady() const;

  SpeedSensorConfig configSnapshot() const;
  static const char* version();

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace stridecontrol
