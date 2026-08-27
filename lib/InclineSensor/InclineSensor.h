#pragma once
#include <Arduino.h>
#include "InclineSensorConfig.h"
#include "InclineSensorTypes.h"

namespace stridecontrol {

class InclineSensor {
 public:
  InclineSensor();
  ~InclineSensor();

  InclineSensor(const InclineSensor&) = delete;
  InclineSensor& operator=(const InclineSensor&) = delete;

  bool begin(const InclineSensorConfig& config = InclineSensorConfig{},
             const InclineCalibration& calibration = InclineCalibration{});
  void end();
  void update();

  bool setExpectedDirection(InclineDirection direction);
  bool confirmHomedAtZero();
  bool restorePosition(float inclinePct, bool trusted);

  InclineState getState() const;
  bool applyCalibration(const InclineCalibration& calibration);
  InclineCalibration getCalibration() const;
  InclineSensorConfig configSnapshot() const;

  bool isReady() const;
  static const char* version();

 private:
  struct Impl;
  Impl* impl_;
};

} // namespace stridecontrol

