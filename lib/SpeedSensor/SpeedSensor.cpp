#include "SpeedSensor.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

namespace stridecontrol {

struct SpeedSensor::Impl {
  SpeedSensorConfig config{};
  SpeedObservationMode mode = SpeedObservationMode::HardwareInterrupt;
  std::atomic<bool> ready{false};
  portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

  struct IsrData {
    uint64_t acceptedPulseCount = 0;
    uint64_t rejectedGlitchCount = 0;
    uint64_t rejectedLockoutCount = 0;
    uint32_t lastAcceptedTimestampUs = 0;
    uint32_t lastPulseIntervalUs = 0;
    uint32_t isrSequence = 0;
    bool hasAcceptedPulse = false;
    bool hasValidInterval = false;
    bool awaitingReference = true;

    void reset() {
      acceptedPulseCount = 0;
      rejectedGlitchCount = 0;
      rejectedLockoutCount = 0;
      lastAcceptedTimestampUs = 0;
      lastPulseIntervalUs = 0;
      isrSequence = 0;
      hasAcceptedPulse = false;
      hasValidInterval = false;
      awaitingReference = true;
    }
  };

  struct EvaluationResult {
    SpeedSensorState state{};
    bool requestReferenceRearm = false;
    uint32_t observedSequence = 0;
    uint32_t observedTimestampUs = 0;
  };

  IsrData isrData{};
  SpeedSensorState state{};

  static Impl* activeInstance;

  // Algorithmic edge processing core (caller must hold isrMux)
  void handlePulseEdgeLocked(uint32_t timestampUs) {
    if (isrData.awaitingReference) {
      isrData.hasAcceptedPulse = true;
      isrData.lastAcceptedTimestampUs = timestampUs;
      isrData.hasValidInterval = false;
      isrData.awaitingReference = false;
      isrData.acceptedPulseCount++;
      isrData.isrSequence++;
    } else {
      const uint32_t elapsed = timestampUs - isrData.lastAcceptedTimestampUs;
      if (elapsed < config.glitchRejectUs) {
        isrData.rejectedGlitchCount++;
      } else if (elapsed < config.pulseLockoutUs) {
        isrData.rejectedLockoutCount++;
      } else {
        isrData.lastPulseIntervalUs = elapsed;
        isrData.lastAcceptedTimestampUs = timestampUs;
        isrData.hasValidInterval = true;
        isrData.acceptedPulseCount++;
        isrData.isrSequence++;
      }
    }
  }

  // Pure snapshot evaluation math: zero side-effects, referentially transparent
  EvaluationResult evaluateSnapshot(const IsrData& snap, uint32_t nowUs) const {
    EvaluationResult result{};
    result.state.initialized = true;
    result.state.acceptedPulseCount = snap.acceptedPulseCount;
    result.state.rejectedGlitchCount = snap.rejectedGlitchCount;
    result.state.rejectedLockoutCount = snap.rejectedLockoutCount;
    result.state.lastPulseTimestampUs = snap.lastAcceptedTimestampUs;
    result.state.pulseIntervalUs = snap.lastPulseIntervalUs;
    result.observedSequence = snap.isrSequence;
    result.observedTimestampUs = snap.lastAcceptedTimestampUs;

    if (snap.hasAcceptedPulse) {
      result.state.lastPulseAgeMs = (nowUs - snap.lastAcceptedTimestampUs) / 1000;
    } else {
      result.state.lastPulseAgeMs = 0;
    }

    if (!snap.hasAcceptedPulse) {
      result.state.signalPresent = false;
      result.state.measurementValid = false;
      result.state.frequencyHz = 0.0f;
      result.state.speedKmh = 0.0f;
      result.state.status = SpeedSensorStatus::AwaitingFirstPulse;
    } else {
      const uint32_t ageUs = nowUs - snap.lastAcceptedTimestampUs;
      const uint32_t timeoutUs = config.pulseTimeoutMs * 1000UL;
      if (ageUs > timeoutUs) {
        result.state.signalPresent = false;
        result.state.measurementValid = false;
        result.state.frequencyHz = 0.0f;
        result.state.speedKmh = 0.0f;
        result.state.status = SpeedSensorStatus::TimedOut;
        result.requestReferenceRearm = true;
      } else if (snap.awaitingReference || !snap.hasValidInterval) {
        result.state.signalPresent = true;
        result.state.measurementValid = false;
        result.state.frequencyHz = 0.0f;
        result.state.speedKmh = 0.0f;
        result.state.status = SpeedSensorStatus::AwaitingInterval;
      } else {
        result.state.signalPresent = true;
        result.state.measurementValid = true;
        result.state.frequencyHz = 1000000.0f / static_cast<float>(snap.lastPulseIntervalUs);
        result.state.speedKmh = result.state.frequencyHz * config.kmhPerHz;
        result.state.status = SpeedSensorStatus::Measuring;
      }
    }

    return result;
  }

  static void IRAM_ATTR isrRouter() {
    if (activeInstance) {
      const uint32_t now = micros();
      portENTER_CRITICAL_ISR(&activeInstance->isrMux);
      activeInstance->handlePulseEdgeLocked(now);
      portEXIT_CRITICAL_ISR(&activeInstance->isrMux);
    }
  }
};

SpeedSensor::Impl* SpeedSensor::Impl::activeInstance = nullptr;

SpeedSensor::SpeedSensor() : impl_(new Impl{}) {}

SpeedSensor::~SpeedSensor() {
  end();
  delete impl_;
}

bool SpeedSensor::begin(const SpeedSensorConfig& config, SpeedObservationMode mode) {
  if (impl_->ready.load()) return true;
  if (mode == SpeedObservationMode::HardwareInterrupt) {
    if (Impl::activeInstance != nullptr && Impl::activeInstance != impl_) return false;
    if (config.inputPin < 0 || !GPIO_IS_VALID_GPIO(config.inputPin)) return false;
  }
  if (config.glitchRejectUs == 0) return false;
  if (config.pulseLockoutUs < config.glitchRejectUs) return false;
  if (config.pulseTimeoutMs == 0 || (config.pulseTimeoutMs * 1000UL) <= config.pulseLockoutUs) return false;
  if (!std::isfinite(config.kmhPerHz) || config.kmhPerHz < 0.1f || config.kmhPerHz > 10.0f) {
    return false;
  }

  impl_->config = config;
  impl_->mode = mode;
  impl_->isrData.reset();
  impl_->state = {};
  impl_->state.status = SpeedSensorStatus::AwaitingFirstPulse;

  if (mode == SpeedObservationMode::HardwareInterrupt) {
    pinMode(config.inputPin, config.useInternalPullup ? INPUT_PULLUP : INPUT);
    Impl::activeInstance = impl_;
    attachInterrupt(digitalPinToInterrupt(config.inputPin), Impl::isrRouter, FALLING);
  }

  impl_->ready.store(true);
  impl_->state.initialized = true;
  return true;
}

void SpeedSensor::end() {
  if (!impl_ || !impl_->ready.load()) return;
  impl_->ready.store(false);
  if (impl_->mode == SpeedObservationMode::HardwareInterrupt) {
    detachInterrupt(digitalPinToInterrupt(impl_->config.inputPin));
    if (Impl::activeInstance == impl_) Impl::activeInstance = nullptr;
  }

  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state.initialized = false;
  impl_->state.signalPresent = false;
  impl_->state.measurementValid = false;
  impl_->state.frequencyHz = 0.0f;
  impl_->state.speedKmh = 0.0f;
  impl_->state.status = SpeedSensorStatus::Uninitialized;
  portEXIT_CRITICAL(&impl_->stateMux);
}

void SpeedSensor::update() {
  if (!impl_ || !impl_->ready.load()) return;
  if (impl_->mode != SpeedObservationMode::HardwareInterrupt) return;

  // Step 1: Copy IsrData under isrMux
  Impl::IsrData snap{};
  portENTER_CRITICAL(&impl_->isrMux);
  snap = impl_->isrData;
  portEXIT_CRITICAL(&impl_->isrMux);

  // Step 2: Sample authoritative hardware clock AFTER copy (eliminates underflow race)
  const uint32_t now = micros();

  // Step 3: Pure mathematical evaluation
  const Impl::EvaluationResult result = impl_->evaluateSnapshot(snap, now);

  // Step 4: Conditional rearm under isrMux (if timeout triggered)
  if (result.requestReferenceRearm) {
    portENTER_CRITICAL(&impl_->isrMux);
    if (impl_->isrData.isrSequence == result.observedSequence &&
        impl_->isrData.lastAcceptedTimestampUs == result.observedTimestampUs) {
      impl_->isrData.hasValidInterval = false;
      impl_->isrData.awaitingReference = true;
    }
    portEXIT_CRITICAL(&impl_->isrMux);
  }

  // Step 5: Publish SpeedSensorState under stateMux
  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state = result.state;
  portEXIT_CRITICAL(&impl_->stateMux);
}

bool SpeedSensor::observeEdge(uint32_t edgeTimestampUs) {
  if (!impl_ || !impl_->ready.load()) return false;
  if (impl_->mode != SpeedObservationMode::SoftwareObservation) return false;

  portENTER_CRITICAL(&impl_->isrMux);
  impl_->handlePulseEdgeLocked(edgeTimestampUs);
  portEXIT_CRITICAL(&impl_->isrMux);
  return true;
}

bool SpeedSensor::evaluate(uint32_t nowUs32) {
  if (!impl_ || !impl_->ready.load()) return false;
  if (impl_->mode != SpeedObservationMode::SoftwareObservation) return false;

  // Step 1: Copy IsrData under isrMux
  Impl::IsrData snap{};
  portENTER_CRITICAL(&impl_->isrMux);
  snap = impl_->isrData;
  portEXIT_CRITICAL(&impl_->isrMux);

  // Step 2: Authoritative clock is passed scenario microsecond timestamp
  const uint32_t now = nowUs32;

  // Step 3: Pure mathematical evaluation
  const Impl::EvaluationResult result = impl_->evaluateSnapshot(snap, now);

  // Step 4: Conditional rearm under isrMux (if timeout triggered)
  if (result.requestReferenceRearm) {
    portENTER_CRITICAL(&impl_->isrMux);
    if (impl_->isrData.isrSequence == result.observedSequence &&
        impl_->isrData.lastAcceptedTimestampUs == result.observedTimestampUs) {
      impl_->isrData.hasValidInterval = false;
      impl_->isrData.awaitingReference = true;
    }
    portEXIT_CRITICAL(&impl_->isrMux);
  }

  // Step 5: Publish SpeedSensorState under stateMux
  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state = result.state;
  portEXIT_CRITICAL(&impl_->stateMux);
  return true;
}

SpeedSensorState SpeedSensor::getState() const {
  SpeedSensorState snap{};
  if (impl_) {
    portENTER_CRITICAL(&impl_->stateMux);
    snap = impl_->state;
    portEXIT_CRITICAL(&impl_->stateMux);
  }
  return snap;
}

bool SpeedSensor::isReady() const {
  return impl_ && impl_->ready.load();
}

SpeedObservationMode SpeedSensor::getObservationMode() const {
  return impl_ ? impl_->mode : SpeedObservationMode::HardwareInterrupt;
}

SpeedSensorConfig SpeedSensor::configSnapshot() const {
  if (!impl_) return SpeedSensorConfig{};
  return impl_->config;
}

const char* SpeedSensor::version() {
  return "SpeedSensor/0.3.0 (ESP32-S3 Isolated PC817 + SoftwareObservation)";
}

const char* speedSensorStatusName(SpeedSensorStatus status) {
  switch (status) {
    case SpeedSensorStatus::Uninitialized: return "UNINITIALIZED";
    case SpeedSensorStatus::Ready: return "READY";
    case SpeedSensorStatus::AwaitingFirstPulse: return "AWAITING_FIRST_PULSE";
    case SpeedSensorStatus::AwaitingInterval: return "AWAITING_INTERVAL";
    case SpeedSensorStatus::Measuring: return "MEASURING";
    case SpeedSensorStatus::TimedOut: return "TIMED_OUT";
    default: return "HARDWARE_ERROR";
  }
}

}  // namespace stridecontrol
