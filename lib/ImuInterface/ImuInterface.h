#pragma once
#include <Arduino.h>
#include "ImuConfig.h"
#include "ImuTypes.h"

namespace stridecontrol {

class ImuInterface {
 public:
  ImuInterface();

  /**
   * @brief Destructor for ImuInterface.
   *
   * Safely performs hardware teardown and releases synchronization primitives.
   * Note: Callers must ensure all external tasks have stopped invoking methods
   * on this instance prior to destroying it.
   */
  ~ImuInterface();

  ImuInterface(const ImuInterface&) = delete;
  ImuInterface& operator=(const ImuInterface&) = delete;

  bool begin(const ImuConfig& config = ImuConfig{}, ImuObservationMode mode = ImuObservationMode::HardwareI2c);
  bool begin(ImuObservationMode mode);
  void end();
  void update();

  ImuObservationMode getObservationMode() const;
  bool observeSamples(const ImuSample* samples, size_t count, const ImuState* simulatedState = nullptr);

  ImuState getState() const;

  size_t readSamples(ImuSample* destination, size_t capacity);

  bool setCurrentAngleAsZero();
  bool applyZeroAngleDeg(float zeroAngleDeg);
  float getZeroAngleDeg() const;

  ImuConfig configSnapshot() const;
  bool isReady() const;

  static const char* version();

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace stridecontrol
