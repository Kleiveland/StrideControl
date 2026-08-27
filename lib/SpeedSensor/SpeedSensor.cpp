#include "SpeedSensor.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

namespace stridecontrol {

struct SpeedSensor::Impl {
  SpeedSensorConfig config{};
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

  IsrData isrData{};
  SpeedSensorState state{};

  static Impl* activeInstance;

  static void IRAM_ATTR isrRouter() {
    if (activeInstance) activeInstance->handlePulseIsr();
  }

  void IRAM_ATTR handlePulseIsr() {
    const uint32_t now = micros();
    portENTER_CRITICAL_ISR(&isrMux);
    if (isrData.awaitingReference) {
      isrData.hasAcceptedPulse = true;
      isrData.lastAcceptedTimestampUs = now;
      isrData.hasValidInterval = false;
      isrData.awaitingReference = false;
      isrData.acceptedPulseCount++;
      isrData.isrSequence++;
    } else {
      const uint32_t elapsed = now - isrData.lastAcceptedTimestampUs;
      if (elapsed < config.glitchRejectUs) {
        isrData.rejectedGlitchCount++;
      } else if (elapsed < config.pulseLockoutUs) {
        isrData.rejectedLockoutCount++;
      } else {
        isrData.lastPulseIntervalUs = elapsed;
        isrData.lastAcceptedTimestampUs = now;
        isrData.hasValidInterval = true;
        isrData.acceptedPulseCount++;
        isrData.isrSequence++;
      }
    }
    portEXIT_CRITICAL_ISR(&isrMux);
  }
};

SpeedSensor::Impl* SpeedSensor::Impl::activeInstance = nullptr;

SpeedSensor::SpeedSensor() : impl_(new Impl{}) {}

SpeedSensor::~SpeedSensor() {
  end();
  delete impl_;
}

bool SpeedSensor::begin(const SpeedSensorConfig& config) {
  if (impl_->ready.load()) return true;
  if (Impl::activeInstance != nullptr && Impl::activeInstance != impl_) return false;

  if (config.inputPin < 0 || !GPIO_IS_VALID_GPIO(config.inputPin)) return false;
  if (config.glitchRejectUs == 0) return false;
  if (config.pulseLockoutUs < config.glitchRejectUs) return false;
  if (config.pulseTimeoutMs == 0 || (config.pulseTimeoutMs * 1000UL) <= config.pulseLockoutUs) return false;
  if (!std::isfinite(config.kmhPerHz) || config.kmhPerHz < 0.1f || config.kmhPerHz > 10.0f) {
    return false;
  }

  impl_->config = config;
  impl_->isrData.reset();
  impl_->state = {};
  impl_->state.status = SpeedSensorStatus::AwaitingFirstPulse;

  pinMode(config.inputPin, config.useInternalPullup ? INPUT_PULLUP : INPUT);
  Impl::activeInstance = impl_;
  attachInterrupt(digitalPinToInterrupt(config.inputPin), Impl::isrRouter, FALLING);

  impl_->ready.store(true);
  impl_->state.initialized = true;
  return true;
}

void SpeedSensor::end() {
  if (!impl_ || !impl_->ready.load()) return;
  impl_->ready.store(false);
  detachInterrupt(digitalPinToInterrupt(impl_->config.inputPin));
  if (Impl::activeInstance == impl_) Impl::activeInstance = nullptr;

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

  Impl::IsrData snap{};
  portENTER_CRITICAL(&impl_->isrMux);
  snap = impl_->isrData;
  portEXIT_CRITICAL(&impl_->isrMux);

  const uint32_t now = micros();
  SpeedSensorState nextState{};
  nextState.initialized = true;
  nextState.acceptedPulseCount = snap.acceptedPulseCount;
  nextState.rejectedGlitchCount = snap.rejectedGlitchCount;
  nextState.rejectedLockoutCount = snap.rejectedLockoutCount;
  nextState.lastPulseTimestampUs = snap.lastAcceptedTimestampUs;
  nextState.pulseIntervalUs = snap.lastPulseIntervalUs;

  if (snap.hasAcceptedPulse) {
    nextState.lastPulseAgeMs = (now - snap.lastAcceptedTimestampUs) / 1000;
  } else {
    nextState.lastPulseAgeMs = 0;
  }

  if (!snap.hasAcceptedPulse) {
    nextState.signalPresent = false;
    nextState.measurementValid = false;
    nextState.frequencyHz = 0.0f;
    nextState.speedKmh = 0.0f;
    nextState.status = SpeedSensorStatus::AwaitingFirstPulse;
  } else {
    const uint32_t ageUs = now - snap.lastAcceptedTimestampUs;
    const uint32_t timeoutUs = impl_->config.pulseTimeoutMs * 1000UL;
    if (ageUs > timeoutUs) {
      nextState.signalPresent = false;
      nextState.measurementValid = false;
      nextState.frequencyHz = 0.0f;
      nextState.speedKmh = 0.0f;
      nextState.status = SpeedSensorStatus::TimedOut;

      portENTER_CRITICAL(&impl_->isrMux);
      if (impl_->isrData.isrSequence == snap.isrSequence &&
          impl_->isrData.lastAcceptedTimestampUs == snap.lastAcceptedTimestampUs) {
        impl_->isrData.hasValidInterval = false;
        impl_->isrData.awaitingReference = true;
      }
      portEXIT_CRITICAL(&impl_->isrMux);
    } else if (snap.awaitingReference || !snap.hasValidInterval) {
      nextState.signalPresent = true;
      nextState.measurementValid = false;
      nextState.frequencyHz = 0.0f;
      nextState.speedKmh = 0.0f;
      nextState.status = SpeedSensorStatus::AwaitingInterval;
    } else {
      nextState.signalPresent = true;
      nextState.measurementValid = true;
      nextState.frequencyHz = 1000000.0f / static_cast<float>(snap.lastPulseIntervalUs);
      nextState.speedKmh = nextState.frequencyHz * impl_->config.kmhPerHz;
      nextState.status = SpeedSensorStatus::Measuring;
    }
  }

  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state = nextState;
  portEXIT_CRITICAL(&impl_->stateMux);
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

SpeedSensorConfig SpeedSensor::configSnapshot() const {
  if (!impl_) return SpeedSensorConfig{};
  return impl_->config;
}

const char* SpeedSensor::version() {
  return "SpeedSensor/0.2.0 (ESP32-S3 Isolated PC817)";
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
