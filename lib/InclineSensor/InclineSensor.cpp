#include "InclineSensor.h"
#include <atomic>
#include <cmath>
#include <algorithm>
#include <driver/gpio.h>

namespace stridecontrol {

const char* inclineStatusName(InclineStatus status) {
  switch (status) {
    case InclineStatus::Uninitialized: return "UNINITIALIZED";
    case InclineStatus::Ready: return "READY";
    case InclineStatus::Stationary: return "STATIONARY";
    case InclineStatus::QualifyingMovement: return "QUALIFYING_MOVEMENT";
    case InclineStatus::MovingUp: return "MOVING_UP";
    case InclineStatus::MovingDown: return "MOVING_DOWN";
    case InclineStatus::MovingDirectionUnknown: return "MOVING_DIRECTION_UNKNOWN";
    case InclineStatus::MotionTimeout: return "MOTION_TIMEOUT";
    case InclineStatus::PositionLimitReached: return "POSITION_LIMIT_REACHED";
    case InclineStatus::HardwareError: return "HARDWARE_ERROR";
    default: return "UNKNOWN";
  }
}

const char* inclineDirectionName(InclineDirection direction) {
  switch (direction) {
    case InclineDirection::Unknown: return "UNKNOWN";
    case InclineDirection::Up: return "UP";
    case InclineDirection::Down: return "DOWN";
    default: return "UNKNOWN";
  }
}

struct PositionResult {
  float value;
  bool exceededLimit;
};

static PositionResult calculatePosition(
    float baselineInclinePct,
    uint64_t pulseDelta,
    InclineDirection direction,
    const InclineCalibration& calibration,
    const InclineSensorConfig& config) {

  constexpr float EPSILON = 0.0001f;
  PositionResult res{};
  float delta = 0.0f;

  if (direction == InclineDirection::Up && calibration.pulsesPerPercentUp > 0.0f) {
    delta = static_cast<float>(pulseDelta) / calibration.pulsesPerPercentUp;
  } else if (direction == InclineDirection::Down && calibration.pulsesPerPercentDown > 0.0f) {
    delta = -static_cast<float>(pulseDelta) / calibration.pulsesPerPercentDown;
  }

  const float rawIncline = baselineInclinePct + delta;

  if (rawIncline < (config.minimumInclinePct - EPSILON)) {
    res.value = config.minimumInclinePct;
    res.exceededLimit = true;
  } else if (rawIncline > (config.maximumInclinePct + EPSILON)) {
    res.value = config.maximumInclinePct;
    res.exceededLimit = true;
  } else {
    res.value = std::max(config.minimumInclinePct, std::min(config.maximumInclinePct, rawIncline));
    res.exceededLimit = false;
  }

  return res;
}

static bool isValidCalibration(const InclineCalibration& cal) {
  if (std::isnan(cal.pulsesPerPercentUp) || std::isnan(cal.pulsesPerPercentDown)) return false;
  if (std::isinf(cal.pulsesPerPercentUp) || std::isinf(cal.pulsesPerPercentDown)) return false;
  if (cal.pulsesPerPercentUp <= 0.0f || cal.pulsesPerPercentDown <= 0.0f) return false;
  if (cal.pulsesPerPercentUp < 100.0f || cal.pulsesPerPercentUp > 10000.0f) return false;
  if (cal.pulsesPerPercentDown < 100.0f || cal.pulsesPerPercentDown > 10000.0f) return false;
  return true;
}

struct IsrData {
  uint64_t acceptedPulseCount = 0;
  uint64_t rejectedPulseCount = 0;
  uint32_t lastAcceptedPulseUs = 0;
  uint32_t previousAcceptedPulseUs = 0;
  uint32_t burstStartUs = 0;
  uint64_t burstStartPulseCount = 0;
  uint32_t isrSequence = 0;
  bool hasAcceptedPulse = false;
  bool burstArmed = false;
};

struct ModelSnapshot {
  float baselineInclinePct = 0.0f;
  uint64_t baselineAcceptedPulseCount = 0;
  InclineDirection baselineDirection = InclineDirection::Unknown;
  InclineCalibration calibration;
  bool initialized = false;
  bool moving = false;
  uint32_t modelGeneration = 0;
  uint64_t minimumQualifiedBurstPulseCount = 0;
  uint32_t movementStartedMs = 0;
  uint64_t movementPulseBaselineCount = 0;
};

struct InclineSensor::Impl {
  InclineSensorConfig config;
  InclineCalibration calibration;

  portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;
  IsrData isrData;

  portMUX_TYPE modelMux = portMUX_INITIALIZER_UNLOCKED;
  float baselineInclinePct = 0.0f;
  uint64_t baselineAcceptedPulseCount = 0;
  InclineDirection baselineDirection = InclineDirection::Unknown;

  bool initialized = false;
  bool moving = false;
  bool homed = false;
  bool positionTrusted = false;
  bool signalPresent = false;
  InclineDirection expectedDirection = InclineDirection::Unknown;
  
  uint32_t movementPulseCount = 0;
  uint32_t movementStartedMs = 0;
  uint32_t movementDurationMs = 0;
  uint64_t movementPulseBaselineCount = 0;
  
  bool positionLimitLatched = false;
  bool motionTimeoutLatched = false;
  bool hardwareErrorLatched = false;

  uint32_t modelGeneration = 0;
  uint64_t minimumQualifiedBurstPulseCount = 0;

  std::atomic<bool> ready{false};
  std::atomic<bool> isrAttached{false};

  static Impl* activeInstance;
  static void IRAM_ATTR isrHandler(void* arg);

  bool isIsrSnapshotCurrent(uint32_t snapSeq, uint32_t snapLastPulseUs) {
    bool current = false;
    portENTER_CRITICAL(&isrMux);
    if (isrData.isrSequence == snapSeq && isrData.lastAcceptedPulseUs == snapLastPulseUs) {
      current = true;
    }
    portEXIT_CRITICAL(&isrMux);
    return current;
  }

  bool disarmCandidateBurstIfUnchanged(uint32_t snapSeq, uint32_t snapBurstStartUs, uint64_t snapBurstStartPulseCount) {
    bool disarmed = false;
    portENTER_CRITICAL(&isrMux);
    if (isrData.isrSequence == snapSeq &&
        isrData.burstStartUs == snapBurstStartUs &&
        isrData.burstStartPulseCount == snapBurstStartPulseCount) {
      isrData.burstArmed = false;
      disarmed = true;
    }
    portEXIT_CRITICAL(&isrMux);
    return disarmed;
  }
};

InclineSensor::Impl* InclineSensor::Impl::activeInstance = nullptr;

void IRAM_ATTR InclineSensor::Impl::isrHandler(void* arg) {
  Impl* impl = static_cast<Impl*>(arg);
  if (!impl) return;

  const uint32_t nowUs = micros();

  portENTER_CRITICAL_ISR(&impl->isrMux);

  if (!impl->isrData.hasAcceptedPulse) {
    impl->isrData.hasAcceptedPulse = true;
    impl->isrData.burstStartUs = nowUs;
    impl->isrData.burstStartPulseCount = impl->isrData.acceptedPulseCount;
    impl->isrData.burstArmed = true;
    
    impl->isrData.lastAcceptedPulseUs = nowUs;
    impl->isrData.acceptedPulseCount++;
    impl->isrData.isrSequence++;
  } else {
    const uint32_t deltaUs = nowUs - impl->isrData.lastAcceptedPulseUs;
    if (deltaUs < impl->config.minimumPulseIntervalUs) {
      impl->isrData.rejectedPulseCount++;
    } else {
      if (deltaUs >= (impl->config.movementStopTimeoutMs * 1000UL)) {
        impl->isrData.burstStartUs = nowUs;
        impl->isrData.burstStartPulseCount = impl->isrData.acceptedPulseCount;
        impl->isrData.burstArmed = true;
      }
      impl->isrData.previousAcceptedPulseUs = impl->isrData.lastAcceptedPulseUs;
      impl->isrData.lastAcceptedPulseUs = nowUs;
      impl->isrData.acceptedPulseCount++;
      impl->isrData.isrSequence++;
    }
  }

  portEXIT_CRITICAL_ISR(&impl->isrMux);
}

InclineSensor::InclineSensor() : impl_(new Impl()) {}

InclineSensor::~InclineSensor() {
  end();
  delete impl_;
}

bool InclineSensor::begin(const InclineSensorConfig& config, const InclineCalibration& calibration) {
  if (impl_->ready.load()) {
    return true; // Already initialized
  }

  if (Impl::activeInstance != nullptr && Impl::activeInstance != impl_) {
    return false; // Another instance is active
  }

  // Validate config
  if (config.minimumPulseIntervalUs == 0) return false;
  if (config.movementStartPulseCount == 0) return false;
  if (config.movementStartWindowMs == 0) return false;
  if (config.movementStopTimeoutMs <= config.movementStartWindowMs) return false;
  if (config.maximumMovementTimeMs <= config.movementStopTimeoutMs) return false;
  if (std::isnan(config.minimumInclinePct) || std::isnan(config.maximumInclinePct)) return false;
  if (std::isinf(config.minimumInclinePct) || std::isinf(config.maximumInclinePct)) return false;
  if (config.maximumInclinePct <= config.minimumInclinePct) return false;
  if (!isValidCalibration(calibration)) return false;
  if (!GPIO_IS_VALID_GPIO(config.inputPin)) return false;
  if (digitalPinToInterrupt(config.inputPin) < 0) return false;

  impl_->config = config;
  impl_->calibration = calibration;

  impl_->isrData = IsrData{};

  impl_->baselineInclinePct = 0.0f; // Safe value
  impl_->baselineAcceptedPulseCount = 0;
  impl_->baselineDirection = InclineDirection::Unknown;
  
  impl_->initialized = false;
  impl_->moving = false;
  impl_->homed = false;
  impl_->positionTrusted = false;
  impl_->signalPresent = false;
  impl_->expectedDirection = InclineDirection::Unknown;
  
  impl_->movementPulseCount = 0;
  impl_->movementStartedMs = 0;
  impl_->movementDurationMs = 0;
  impl_->movementPulseBaselineCount = 0;
  
  impl_->positionLimitLatched = false;
  impl_->motionTimeoutLatched = false;
  impl_->hardwareErrorLatched = false;

  impl_->modelGeneration = 0;
  impl_->minimumQualifiedBurstPulseCount = 0;

  pinMode(config.inputPin, config.useInternalPullup ? INPUT_PULLUP : INPUT);
  Impl::activeInstance = impl_;

  attachInterruptArg(
      digitalPinToInterrupt(config.inputPin),
      Impl::isrHandler,
      impl_,
      FALLING
  );

  impl_->isrAttached.store(true);
  impl_->initialized = true;
  impl_->ready.store(true);

  return true;
}

void InclineSensor::end() {
  impl_->ready.store(false);

  if (impl_->isrAttached.load()) {
    detachInterrupt(digitalPinToInterrupt(impl_->config.inputPin));
    impl_->isrAttached.store(false);
  }

  if (Impl::activeInstance == impl_) {
    Impl::activeInstance = nullptr;
  }

  portENTER_CRITICAL(&impl_->modelMux);
  impl_->initialized = false;
  impl_->moving = false;
  impl_->signalPresent = false;
  impl_->positionTrusted = false;
  impl_->modelGeneration++;
  portEXIT_CRITICAL(&impl_->modelMux);
}

bool InclineSensor::applyCalibration(const InclineCalibration& newCal) {
  if (!impl_->ready.load()) return false;
  if (!isValidCalibration(newCal)) return false;

  constexpr uint8_t MAX_SETTER_ATTEMPTS = 3;

  for (uint8_t attempt = 0; attempt < MAX_SETTER_ATTEMPTS; ++attempt) {
    IsrData isrSnap{};
    portENTER_CRITICAL(&impl_->isrMux);
    isrSnap = impl_->isrData;
    portEXIT_CRITICAL(&impl_->isrMux);

    ModelSnapshot modelSnap{};
    portENTER_CRITICAL(&impl_->modelMux);
    modelSnap.baselineInclinePct = impl_->baselineInclinePct;
    modelSnap.baselineAcceptedPulseCount = impl_->baselineAcceptedPulseCount;
    modelSnap.baselineDirection = impl_->baselineDirection;
    modelSnap.calibration = impl_->calibration;
    modelSnap.moving = impl_->moving;
    modelSnap.modelGeneration = impl_->modelGeneration;
    portEXIT_CRITICAL(&impl_->modelMux);

    PositionResult posRes{};
    if (modelSnap.moving && isrSnap.acceptedPulseCount >= modelSnap.baselineAcceptedPulseCount) {
      const uint64_t delta = isrSnap.acceptedPulseCount - modelSnap.baselineAcceptedPulseCount;
      posRes = calculatePosition(modelSnap.baselineInclinePct, delta, modelSnap.baselineDirection,
                                 modelSnap.calibration, impl_->config);
    } else {
      posRes.value = modelSnap.baselineInclinePct;
      posRes.exceededLimit = false;
    }

    bool committed = false;
    portENTER_CRITICAL(&impl_->modelMux);
    if (impl_->modelGeneration == modelSnap.modelGeneration &&
        impl_->baselineAcceptedPulseCount == modelSnap.baselineAcceptedPulseCount) {
      
      impl_->baselineInclinePct = posRes.value;
      impl_->baselineAcceptedPulseCount = isrSnap.acceptedPulseCount;
      impl_->calibration = newCal;
      
      if (posRes.exceededLimit) {
        impl_->positionLimitLatched = true;
        impl_->positionTrusted = false;
      }

      impl_->modelGeneration++;
      committed = true;
    }
    portEXIT_CRITICAL(&impl_->modelMux);

    if (committed) return true;
  }
  return false;
}

bool InclineSensor::setExpectedDirection(InclineDirection newDirection) {
  if (!impl_->ready.load()) return false;

  constexpr uint8_t MAX_SETTER_ATTEMPTS = 3;

  for (uint8_t attempt = 0; attempt < MAX_SETTER_ATTEMPTS; ++attempt) {
    IsrData isrSnap{};
    portENTER_CRITICAL(&impl_->isrMux);
    isrSnap = impl_->isrData;
    portEXIT_CRITICAL(&impl_->isrMux);

    ModelSnapshot modelSnap{};
    portENTER_CRITICAL(&impl_->modelMux);
    modelSnap.baselineInclinePct = impl_->baselineInclinePct;
    modelSnap.baselineAcceptedPulseCount = impl_->baselineAcceptedPulseCount;
    modelSnap.baselineDirection = impl_->baselineDirection;
    modelSnap.calibration = impl_->calibration;
    modelSnap.moving = impl_->moving;
    modelSnap.modelGeneration = impl_->modelGeneration;
    portEXIT_CRITICAL(&impl_->modelMux);

    PositionResult posRes{};
    if (modelSnap.moving && isrSnap.acceptedPulseCount >= modelSnap.baselineAcceptedPulseCount) {
      const uint64_t delta = isrSnap.acceptedPulseCount - modelSnap.baselineAcceptedPulseCount;
      posRes = calculatePosition(modelSnap.baselineInclinePct, delta, modelSnap.baselineDirection,
                                 modelSnap.calibration, impl_->config);
    } else {
      posRes.value = modelSnap.baselineInclinePct;
      posRes.exceededLimit = false;
    }

    bool committed = false;
    portENTER_CRITICAL(&impl_->modelMux);
    if (impl_->modelGeneration == modelSnap.modelGeneration &&
        impl_->baselineAcceptedPulseCount == modelSnap.baselineAcceptedPulseCount &&
        impl_->baselineDirection == modelSnap.baselineDirection) {

      impl_->baselineInclinePct = posRes.value;
      impl_->baselineAcceptedPulseCount = isrSnap.acceptedPulseCount;
      impl_->baselineDirection = newDirection;
      impl_->expectedDirection = newDirection;

      if (!modelSnap.moving) {
        impl_->minimumQualifiedBurstPulseCount = isrSnap.acceptedPulseCount;
        if (isrSnap.burstArmed) {
          impl_->positionTrusted = false;
        }
      }

      if (posRes.exceededLimit) {
        impl_->positionLimitLatched = true;
        impl_->positionTrusted = false;
      }

      impl_->modelGeneration++;
      committed = true;
    }
    portEXIT_CRITICAL(&impl_->modelMux);

    if (committed) {
      if (!modelSnap.moving && isrSnap.burstArmed) {
        impl_->disarmCandidateBurstIfUnchanged(isrSnap.isrSequence, isrSnap.burstStartUs, isrSnap.burstStartPulseCount);
      }
      return true;
    }
  }

  return false;
}

bool InclineSensor::confirmHomedAtZero() {
  if (!impl_ || !impl_->ready.load()) return false;

  const uint32_t nowUs = micros();

  // Caller Precondition: The treadmill incline mechanism must be physically stationary.
  // Reject if movement is active, recent pulses exist, or a candidate burst is armed.
  IsrData isrSnap{};
  portENTER_CRITICAL(&impl_->isrMux);
  isrSnap = impl_->isrData;
  portEXIT_CRITICAL(&impl_->isrMux);

  const bool signalActive = isrSnap.hasAcceptedPulse &&
      ((nowUs - isrSnap.lastAcceptedPulseUs) < (impl_->config.movementStopTimeoutMs * 1000UL));
  if (isrSnap.burstArmed || signalActive) {
    return false;
  }

  ModelSnapshot modelSnap{};
  portENTER_CRITICAL(&impl_->modelMux);
  modelSnap.initialized = impl_->initialized;
  modelSnap.moving = impl_->moving;
  modelSnap.modelGeneration = impl_->modelGeneration;
  const InclineDirection currentExpDir = impl_->expectedDirection;
  portEXIT_CRITICAL(&impl_->modelMux);

  if (!modelSnap.initialized || modelSnap.moving) {
    return false;
  }

  if (!impl_->isIsrSnapshotCurrent(isrSnap.isrSequence, isrSnap.lastAcceptedPulseUs)) {
    return false;
  }

  bool committed = false;
  portENTER_CRITICAL(&impl_->modelMux);
  if (impl_->modelGeneration == modelSnap.modelGeneration && !impl_->moving) {
    impl_->estimatedInclinePct = 0.0f;
    impl_->baselineInclinePct = 0.0f;
    impl_->baselineAcceptedPulseCount = isrSnap.acceptedPulseCount;
    impl_->baselineDirection = currentExpDir;
    impl_->homed = true;
    impl_->positionTrusted = true;
    impl_->positionLimitLatched = false;
    impl_->motionTimeoutLatched = false;
    impl_->minimumQualifiedBurstPulseCount = isrSnap.acceptedPulseCount;
    impl_->modelGeneration++;
    committed = true;
  }
  portEXIT_CRITICAL(&impl_->modelMux);

  if (committed) {
    impl_->disarmCandidateBurstIfUnchanged(isrSnap.isrSequence, isrSnap.burstStartUs, isrSnap.burstStartPulseCount);
    return true;
  }

  return false;
}

bool InclineSensor::restorePosition(float inclinePct, bool trusted) {
  if (!impl_ || !impl_->ready.load()) return false;
  if (std::isnan(inclinePct) || std::isinf(inclinePct)) return false;
  if (inclinePct < impl_->config.minimumInclinePct || inclinePct > impl_->config.maximumInclinePct) return false;

  const uint32_t nowUs = micros();

  // Caller Precondition: The treadmill incline mechanism must be physically stationary.
  // Reject if movement is active, recent pulses exist, or a candidate burst is armed.
  IsrData isrSnap{};
  portENTER_CRITICAL(&impl_->isrMux);
  isrSnap = impl_->isrData;
  portEXIT_CRITICAL(&impl_->isrMux);

  const bool signalActive = isrSnap.hasAcceptedPulse &&
      ((nowUs - isrSnap.lastAcceptedPulseUs) < (impl_->config.movementStopTimeoutMs * 1000UL));
  if (isrSnap.burstArmed || signalActive) {
    return false;
  }

  ModelSnapshot modelSnap{};
  portENTER_CRITICAL(&impl_->modelMux);
  modelSnap.initialized = impl_->initialized;
  modelSnap.moving = impl_->moving;
  modelSnap.modelGeneration = impl_->modelGeneration;
  const InclineDirection currentExpDir = impl_->expectedDirection;
  portEXIT_CRITICAL(&impl_->modelMux);

  if (!modelSnap.initialized || modelSnap.moving) {
    return false;
  }

  if (!impl_->isIsrSnapshotCurrent(isrSnap.isrSequence, isrSnap.lastAcceptedPulseUs)) {
    return false;
  }

  bool committed = false;
  portENTER_CRITICAL(&impl_->modelMux);
  if (impl_->modelGeneration == modelSnap.modelGeneration && !impl_->moving) {
    impl_->estimatedInclinePct = inclinePct;
    impl_->baselineInclinePct = inclinePct;
    impl_->baselineAcceptedPulseCount = isrSnap.acceptedPulseCount;
    impl_->baselineDirection = currentExpDir;
    impl_->positionTrusted = trusted;
    impl_->homed = false;
    impl_->positionLimitLatched = false;
    if (trusted) {
      impl_->motionTimeoutLatched = false;
    }
    impl_->minimumQualifiedBurstPulseCount = isrSnap.acceptedPulseCount;
    impl_->modelGeneration++;
    committed = true;
  }
  portEXIT_CRITICAL(&impl_->modelMux);

  if (committed) {
    impl_->disarmCandidateBurstIfUnchanged(isrSnap.isrSequence, isrSnap.burstStartUs, isrSnap.burstStartPulseCount);
    return true;
  }

  return false;
}

void InclineSensor::update() {
  if (!impl_->ready.load()) return;

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  IsrData isrSnap{};
  portENTER_CRITICAL(&impl_->isrMux);
  isrSnap = impl_->isrData;
  portEXIT_CRITICAL(&impl_->isrMux);

  ModelSnapshot modelSnap{};
  portENTER_CRITICAL(&impl_->modelMux);
  modelSnap.baselineInclinePct = impl_->baselineInclinePct;
  modelSnap.baselineAcceptedPulseCount = impl_->baselineAcceptedPulseCount;
  modelSnap.baselineDirection = impl_->baselineDirection;
  modelSnap.calibration = impl_->calibration;
  modelSnap.moving = impl_->moving;
  modelSnap.modelGeneration = impl_->modelGeneration;
  modelSnap.minimumQualifiedBurstPulseCount = impl_->minimumQualifiedBurstPulseCount;
  modelSnap.movementStartedMs = impl_->movementStartedMs;
  modelSnap.movementPulseBaselineCount = impl_->movementPulseBaselineCount;
  portEXIT_CRITICAL(&impl_->modelMux);

  bool stopProcessed = false;

  if (modelSnap.moving) {
    const uint32_t silenceUs = nowUs - isrSnap.lastAcceptedPulseUs;
    const uint64_t delta = (isrSnap.acceptedPulseCount >= modelSnap.baselineAcceptedPulseCount) ? (isrSnap.acceptedPulseCount - modelSnap.baselineAcceptedPulseCount) : 0;
    const PositionResult posRes = calculatePosition(modelSnap.baselineInclinePct, delta, modelSnap.baselineDirection, modelSnap.calibration, impl_->config);

    if (silenceUs >= (impl_->config.movementStopTimeoutMs * 1000UL)) {
      const uint32_t calcMovementPulseCount = static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX, isrSnap.acceptedPulseCount - modelSnap.movementPulseBaselineCount));
      
      if (impl_->isIsrSnapshotCurrent(isrSnap.isrSequence, isrSnap.lastAcceptedPulseUs)) {
        bool stopCommitted = false;
        portENTER_CRITICAL(&impl_->modelMux);
        if (impl_->modelGeneration == modelSnap.modelGeneration &&
            impl_->moving == true &&
            impl_->baselineAcceptedPulseCount == modelSnap.baselineAcceptedPulseCount) {
          
          impl_->moving = false;
          impl_->signalPresent = false;
          impl_->baselineInclinePct = posRes.value;
          impl_->baselineAcceptedPulseCount = isrSnap.acceptedPulseCount;
          impl_->movementPulseCount = calcMovementPulseCount;
          impl_->movementDurationMs = (nowMs >= impl_->movementStartedMs) ? (nowMs - impl_->movementStartedMs) : 0;
          
          if (posRes.exceededLimit && !impl_->positionLimitLatched) {
            impl_->positionLimitLatched = true;
            impl_->positionTrusted = false;
          }
          impl_->modelGeneration++;
          stopCommitted = true;
        }
        portEXIT_CRITICAL(&impl_->modelMux);

        if (stopCommitted) {
          stopProcessed = true;
          // Post-commit check to prevent pulse loss.
          if (impl_->isIsrSnapshotCurrent(isrSnap.isrSequence, isrSnap.lastAcceptedPulseUs)) {
             impl_->disarmCandidateBurstIfUnchanged(isrSnap.isrSequence, isrSnap.burstStartUs, isrSnap.burstStartPulseCount);
          }
        }
      }
    } else {
      // Active movement continuing: check and latch position limits if exceeded
      if (posRes.exceededLimit) {
        portENTER_CRITICAL(&impl_->modelMux);
        if (impl_->modelGeneration == modelSnap.modelGeneration &&
            impl_->moving == true &&
            impl_->baselineAcceptedPulseCount == modelSnap.baselineAcceptedPulseCount) {
          if (!impl_->positionLimitLatched) {
            impl_->positionLimitLatched = true;
            impl_->positionTrusted = false;
            impl_->modelGeneration++;
          }
        }
        portEXIT_CRITICAL(&impl_->modelMux);
      }
    }
  } else {
    if (isrSnap.burstArmed) {
      if (isrSnap.burstStartPulseCount < modelSnap.minimumQualifiedBurstPulseCount) {
        impl_->disarmCandidateBurstIfUnchanged(isrSnap.isrSequence, isrSnap.burstStartUs, isrSnap.burstStartPulseCount);
      } else {
        const uint32_t pulseSpanUs = isrSnap.lastAcceptedPulseUs - isrSnap.burstStartUs;
        const uint32_t qualificationAgeUs = nowUs - isrSnap.burstStartUs;
        const uint64_t burstPulseCount = isrSnap.acceptedPulseCount - isrSnap.burstStartPulseCount;
        
        if (burstPulseCount >= impl_->config.movementStartPulseCount && pulseSpanUs <= (impl_->config.movementStartWindowMs * 1000UL)) {
          // Qualification Success
          portENTER_CRITICAL(&impl_->modelMux);
          if (impl_->modelGeneration == modelSnap.modelGeneration && impl_->moving == false) {
            impl_->moving = true;
            impl_->movementStartedMs = nowMs;
            impl_->movementPulseBaselineCount = isrSnap.burstStartPulseCount;
            impl_->baselineAcceptedPulseCount = isrSnap.burstStartPulseCount;
            impl_->modelGeneration++;
          }
          portEXIT_CRITICAL(&impl_->modelMux);
        } else if (qualificationAgeUs > (impl_->config.movementStartWindowMs * 1000UL)) {
          // Qualification Failure
          if (impl_->disarmCandidateBurstIfUnchanged(isrSnap.isrSequence, isrSnap.burstStartUs, isrSnap.burstStartPulseCount)) {
            portENTER_CRITICAL(&impl_->modelMux);
            if (impl_->modelGeneration == modelSnap.modelGeneration && impl_->moving == false) {
              impl_->positionTrusted = false;
              impl_->modelGeneration++;
            }
            portEXIT_CRITICAL(&impl_->modelMux);
          }
        }
      }
    }
  }

  // Motion Timeout Check - latched only on transition
  portENTER_CRITICAL(&impl_->modelMux);
  if (impl_->moving && !stopProcessed) {
    const bool timeoutExceeded = (nowMs - impl_->movementStartedMs) > impl_->config.maximumMovementTimeMs;
    if (!impl_->motionTimeoutLatched && timeoutExceeded) {
      impl_->motionTimeoutLatched = true;
      impl_->positionTrusted = false;
      impl_->modelGeneration++;
    }
  }
  portEXIT_CRITICAL(&impl_->modelMux);
}

InclineState InclineSensor::getState() const {
  InclineState state{};
  if (!impl_) {
    state.status = InclineStatus::Uninitialized;
    return state;
  }
  
  if (!impl_->ready.load()) {
    state.status = InclineStatus::Uninitialized;
    return state;
  }

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  IsrData isrSnap{};
  portENTER_CRITICAL(&impl_->isrMux);
  isrSnap = impl_->isrData;
  portEXIT_CRITICAL(&impl_->isrMux);

  ModelSnapshot modelSnap{};
  portENTER_CRITICAL(&impl_->modelMux);
  state.initialized = impl_->initialized;
  state.moving = impl_->moving;
  state.homed = impl_->homed;
  state.positionTrusted = impl_->positionTrusted;
  state.expectedDirection = impl_->expectedDirection;

  modelSnap.baselineInclinePct = impl_->baselineInclinePct;
  modelSnap.baselineAcceptedPulseCount = impl_->baselineAcceptedPulseCount;
  modelSnap.baselineDirection = impl_->baselineDirection;
  modelSnap.calibration = impl_->calibration;
  modelSnap.moving = impl_->moving;
  modelSnap.movementPulseBaselineCount = impl_->movementPulseBaselineCount;
  modelSnap.movementStartedMs = impl_->movementStartedMs;

  const uint32_t movementPulseCount = impl_->movementPulseCount;
  const uint32_t movementDurationMs = impl_->movementDurationMs;
  const bool limitLatched = impl_->positionLimitLatched;
  const bool timeoutLatched = impl_->motionTimeoutLatched;
  const bool hwError = impl_->hardwareErrorLatched;
  portEXIT_CRITICAL(&impl_->modelMux);

  // Lock-free calculations outside all critical sections
  float calcIncline = modelSnap.baselineInclinePct;
  if (state.moving && isrSnap.acceptedPulseCount >= modelSnap.baselineAcceptedPulseCount) {
    const uint64_t delta = isrSnap.acceptedPulseCount - modelSnap.baselineAcceptedPulseCount;
    PositionResult res = calculatePosition(modelSnap.baselineInclinePct, delta, modelSnap.baselineDirection, modelSnap.calibration, impl_->config);
    calcIncline = res.value;
  }
  state.estimatedInclinePct = calcIncline;

  state.movementPulseCount = state.moving ? static_cast<uint32_t>(std::min<uint64_t>(UINT32_MAX, isrSnap.acceptedPulseCount - modelSnap.movementPulseBaselineCount)) : movementPulseCount;
  state.movementStartedMs = modelSnap.movementStartedMs;
  state.movementDurationMs = state.moving ? (nowMs >= modelSnap.movementStartedMs ? (nowMs - modelSnap.movementStartedMs) : 0) : movementDurationMs;

  state.acceptedPulseCount = isrSnap.acceptedPulseCount;
  state.rejectedPulseCount = isrSnap.rejectedPulseCount;
  state.lastPulseTimestampUs = isrSnap.lastAcceptedPulseUs;
  state.lastPulseAgeMs = isrSnap.hasAcceptedPulse ? ((nowUs - isrSnap.lastAcceptedPulseUs) / 1000) : 0;
  state.snapshotTimestampUs = nowUs;

  state.signalPresent = isrSnap.hasAcceptedPulse && ((nowUs - isrSnap.lastAcceptedPulseUs) < (impl_->config.movementStopTimeoutMs * 1000UL));

  if (hwError) {
    state.status = InclineStatus::HardwareError;
  } else if (timeoutLatched) {
    state.status = InclineStatus::MotionTimeout;
  } else if (limitLatched) {
    state.status = InclineStatus::PositionLimitReached;
  } else if (state.moving) {
    if (state.expectedDirection == InclineDirection::Unknown) {
      state.status = InclineStatus::MovingDirectionUnknown;
    } else if (state.expectedDirection == InclineDirection::Up) {
      state.status = InclineStatus::MovingUp;
    } else {
      state.status = InclineStatus::MovingDown;
    }
  } else if (isrSnap.burstArmed) {
    state.status = InclineStatus::QualifyingMovement;
  } else if (state.initialized) {
    state.status = InclineStatus::Stationary;
  } else {
    state.status = InclineStatus::Ready;
  }

  return state;
}

InclineCalibration InclineSensor::getCalibration() const {
  if (!impl_) return InclineCalibration{};
  portENTER_CRITICAL(&impl_->modelMux);
  InclineCalibration cal = impl_->calibration;
  portEXIT_CRITICAL(&impl_->modelMux);
  return cal;
}

InclineSensorConfig InclineSensor::configSnapshot() const {
  if (!impl_) return InclineSensorConfig{};
  return impl_->config;
}

bool InclineSensor::isReady() const {
  return impl_ != nullptr && impl_->ready.load();
}

const char* InclineSensor::version() {
  return "1.0.0";
}

} // namespace stridecontrol
