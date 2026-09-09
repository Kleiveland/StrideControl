#pragma once
#include <stdint.h>

namespace stridecontrol {

struct InclineCalibration {
  float pulsesPerPercentUp = 3086.0f;
  float pulsesPerPercentDown = 2943.0f;
};

enum class InclineDirection : uint8_t {
  Unknown,
  Up,
  Down
};

enum class InclineObservationMode : uint8_t {
  HardwareInterrupt,
  SoftwareObservation
};

struct InclinePulseObservation {
  uint32_t pulseCount = 0;
  InclineDirection direction = InclineDirection::Unknown;
  bool signalValid = true;
};

enum class InclineStatus : uint8_t {
  Uninitialized,
  Ready,
  Stationary,
  QualifyingMovement,
  MovingUp,
  MovingDown,
  MovingDirectionUnknown,
  MotionTimeout,
  PositionLimitReached,
  HardwareError
};

struct InclineState {
  bool initialized = false;
  bool moving = false;
  bool homed = false;
  bool positionTrusted = false;
  bool signalPresent = false;
  InclineDirection expectedDirection = InclineDirection::Unknown;
  float estimatedInclinePct = 0.0f;
  uint32_t movementPulseCount = 0;
  uint64_t acceptedPulseCount = 0;
  uint64_t rejectedPulseCount = 0;
  uint32_t lastPulseTimestampUs = 0;
  uint32_t lastPulseAgeMs = 0;
  uint32_t movementStartedMs = 0;
  uint32_t movementDurationMs = 0;
  uint32_t snapshotTimestampUs = 0;
  InclineStatus status = InclineStatus::Uninitialized;
};

const char* inclineStatusName(InclineStatus status);
const char* inclineDirectionName(InclineDirection direction);

} // namespace stridecontrol

