#pragma once
#include <Arduino.h>
#include "SpeedSensorConfig.h"
#include "SpeedSensorTypes.h"

namespace stridecontrol {

enum class SpeedObservationMode : uint8_t {
  HardwareInterrupt,   ///< Production: attachInterrupt active, uses hardware micros()
  SoftwareObservation  ///< Firmware-HIL / Simulation: observeEdge active, uses scenario time
};

class SpeedSensor {
 public:
  SpeedSensor();
  ~SpeedSensor();

  SpeedSensor(const SpeedSensor&) = delete;
  SpeedSensor& operator=(const SpeedSensor&) = delete;

  bool begin(const SpeedSensorConfig& config = SpeedSensorConfig{},
             SpeedObservationMode mode = SpeedObservationMode::HardwareInterrupt);
  void end();
  void update();

  bool observeEdge(uint32_t edgeTimestampUs);
  bool evaluate(uint32_t nowUs32);

  SpeedSensorState getState() const;
  bool isReady() const;
  SpeedObservationMode getObservationMode() const;

  SpeedSensorConfig configSnapshot() const;
  static const char* version();

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace stridecontrol
