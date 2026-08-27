#include "ImuInterface.h"
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>
#include <math.h>
#include <string.h>
#include "vendor/lsm6dsox_reg.h"

namespace stridecontrol {

namespace {

constexpr const char* kImuInterfaceVersion = "1.0.0";
constexpr size_t kRingBufferCapacity = 128;
constexpr size_t kStabilityWindowSize = 16;
constexpr size_t kMaxBatchEntries = 32;
constexpr float kRadToDeg = 57.29577951308232f;

// Mutex wait timeouts (Implementation choices)
constexpr TickType_t kLifecycleMutexTimeoutTicks = pdMS_TO_TICKS(1000);  // 1000 ms for begin() and end()
constexpr TickType_t kCalibrationMutexTimeoutTicks = pdMS_TO_TICKS(100); // 100 ms for zero calibration
constexpr TickType_t kUpdateMutexTimeoutTicks = 0;                       // 0 ms (non-blocking skip) for update()

struct ImuBusContext {
  TwoWire* wire = nullptr;
  uint8_t address = 0x6A;
  uint32_t i2cReadErrors = 0;
  uint32_t i2cWriteErrors = 0;
};

int32_t platformWrite(void* handle, uint8_t reg, const uint8_t* buf, uint16_t len) {
  if (handle == nullptr || (buf == nullptr && len > 0)) {
    return -1;
  }
  auto* ctx = static_cast<ImuBusContext*>(handle);
  if (ctx->wire == nullptr) {
    return -1;
  }

  ctx->wire->beginTransmission(ctx->address);
  ctx->wire->write(reg);
  if (len > 0 && buf != nullptr) {
    ctx->wire->write(buf, len);
  }
  uint8_t res = ctx->wire->endTransmission();
  if (res != 0) {
    ctx->i2cWriteErrors++;
    return -1;
  }
  return 0;
}

int32_t platformRead(void* handle, uint8_t reg, uint8_t* buf, uint16_t len) {
  if (handle == nullptr || buf == nullptr || len == 0) {
    return -1;
  }
  auto* ctx = static_cast<ImuBusContext*>(handle);
  if (ctx->wire == nullptr) {
    return -1;
  }

  ctx->wire->beginTransmission(ctx->address);
  ctx->wire->write(reg);
  uint8_t res = ctx->wire->endTransmission(false);
  if (res != 0) {
    ctx->i2cReadErrors++;
    return -1;
  }

  uint8_t bytesReceived = ctx->wire->requestFrom(ctx->address, static_cast<uint8_t>(len));
  if (bytesReceived != len) {
    ctx->i2cReadErrors++;
    return -1;
  }

  for (uint16_t i = 0; i < len; ++i) {
    buf[i] = ctx->wire->read();
  }
  return 0;
}

void platformDelay(uint32_t millisec) {
  delay(millisec);
}

bool isValidAxisMapping(const ImuAxisMapping& m) {
  if (m.longitudinalAxis == m.lateralAxis ||
      m.lateralAxis == m.verticalAxis ||
      m.longitudinalAxis == m.verticalAxis) {
    return false;
  }
  if ((m.longitudinalSign != 1 && m.longitudinalSign != -1) ||
      (m.lateralSign != 1 && m.lateralSign != -1) ||
      (m.verticalSign != 1 && m.verticalSign != -1)) {
    return false;
  }
  return true;
}

bool validateConfiguration(const ImuConfig& config) {
  if (config.wire == nullptr) return false;
  if (config.sdaPin < 0 || config.sclPin < 0) return false;
  if (config.i2cFrequencyHz < 10000 || config.i2cFrequencyHz > 400000) return false;
  if (config.i2cAddress != 0x6A && config.i2cAddress != 0x6B) return false;
  if (config.outputDataRate != ImuOutputDataRate::Hz104) return false;
  if (config.fifoWatermark == 0 || config.fifoWatermark > 512) return false;
  if (config.maximumFifoEntriesPerUpdate == 0 || config.maximumFifoEntriesPerUpdate > kMaxBatchEntries) return false;
  if (config.maximumUpdateTimeUs == 0) return false;
  if (config.dataStaleTimeoutMs == 0) return false;
  if (config.resetTimeoutMs == 0) return false;
  if (config.stabilizationWindowMs == 0) return false;
  if (config.maximumGyroPairAgeUs == 0) return false;
  if (config.accelerationNormToleranceG <= 0.0f || isnan(config.accelerationNormToleranceG) || isinf(config.accelerationNormToleranceG)) return false;
  if (config.maximumStableGyroDps <= 0.0f || isnan(config.maximumStableGyroDps) || isinf(config.maximumStableGyroDps)) return false;
  if (config.maximumStableAngleVarianceDeg2 <= 0.0f || isnan(config.maximumStableAngleVarianceDeg2) || isinf(config.maximumStableAngleVarianceDeg2)) return false;
  if (config.shortAngleFilterAlpha <= 0.0f || config.shortAngleFilterAlpha > 1.0f || isnan(config.shortAngleFilterAlpha) || isinf(config.shortAngleFilterAlpha)) return false;
  if (!isValidAxisMapping(config.axisMapping)) return false;
  return true;
}

void transformAxes(float rawX, float rawY, float rawZ,
                   const ImuAxisMapping& mapping,
                   float* outLongitudinal,
                   float* outLateral,
                   float* outVertical) {
  const float raw[3] = {rawX, rawY, rawZ};

  const auto getComponent = [&](ImuAxis axis, int8_t sign) -> float {
    uint8_t idx = static_cast<uint8_t>(axis);
    if (idx > 2) idx = 0;
    return raw[idx] * static_cast<float>(sign);
  };

  if (outLongitudinal != nullptr) {
    *outLongitudinal = getComponent(mapping.longitudinalAxis, mapping.longitudinalSign);
  }
  if (outLateral != nullptr) {
    *outLateral = getComponent(mapping.lateralAxis, mapping.lateralSign);
  }
  if (outVertical != nullptr) {
    *outVertical = getComponent(mapping.verticalAxis, mapping.verticalSign);
  }
}

uint32_t getOdrPeriodUs(ImuOutputDataRate odr) {
  switch (odr) {
    case ImuOutputDataRate::Hz12_5: return 80000;
    case ImuOutputDataRate::Hz26:   return 38462;
    case ImuOutputDataRate::Hz52:   return 19231;
    case ImuOutputDataRate::Hz104:  return 9615;
    case ImuOutputDataRate::Hz208:  return 4808;
    default:                        return 9615;
  }
}

float getAccelSensitivityG(ImuAccelRange range) {
  switch (range) {
    case ImuAccelRange::G2:  return 0.061f * 0.001f;
    case ImuAccelRange::G4:  return 0.122f * 0.001f;
    case ImuAccelRange::G8:  return 0.244f * 0.001f;
    case ImuAccelRange::G16: return 0.488f * 0.001f;
    default:                 return 0.244f * 0.001f;
  }
}

float getAccelFullScaleG(ImuAccelRange range) {
  switch (range) {
    case ImuAccelRange::G2:  return 2.0f;
    case ImuAccelRange::G4:  return 4.0f;
    case ImuAccelRange::G8:  return 8.0f;
    case ImuAccelRange::G16: return 16.0f;
    default:                 return 8.0f;
  }
}

float getGyroSensitivityDps(ImuGyroRange range) {
  switch (range) {
    case ImuGyroRange::Dps125:  return 4.375f * 0.001f;
    case ImuGyroRange::Dps250:  return 8.75f * 0.001f;
    case ImuGyroRange::Dps500:  return 17.50f * 0.001f;
    case ImuGyroRange::Dps1000: return 35.0f * 0.001f;
    case ImuGyroRange::Dps2000: return 70.0f * 0.001f;
    default:                    return 17.50f * 0.001f;
  }
}

float getGyroFullScaleDps(ImuGyroRange range) {
  switch (range) {
    case ImuGyroRange::Dps125:  return 125.0f;
    case ImuGyroRange::Dps250:  return 250.0f;
    case ImuGyroRange::Dps500:  return 500.0f;
    case ImuGyroRange::Dps1000: return 1000.0f;
    case ImuGyroRange::Dps2000: return 2000.0f;
    default:                    return 500.0f;
  }
}

ImuStatus computeStatus(bool initialized, bool connected, bool hardwareError,
                        bool busError, bool fifoOverrun, bool invalidData,
                        bool dataStale) {
  if (hardwareError) {
    return ImuStatus::HardwareError;
  }
  if (!connected) {
    return ImuStatus::SensorDisconnected;
  }
  if (!initialized) {
    return ImuStatus::Uninitialized;
  }
  if (busError) {
    return ImuStatus::BusError;
  }
  if (fifoOverrun) {
    return ImuStatus::FifoOverrun;
  }
  if (invalidData) {
    return ImuStatus::InvalidData;
  }
  if (dataStale) {
    return ImuStatus::DataStale;
  }
  return ImuStatus::Ready;
}

struct FifoRawRecord {
  uint8_t tag = 0;
  int16_t raw[3] = {0, 0, 0};
};

struct ReconstructedGyro {
  uint32_t timestampUs = 0;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  bool valid = false;
};

class MutexLock {
 public:
  explicit MutexLock(SemaphoreHandle_t mutex, TickType_t waitTicks)
      : mutex_(mutex), locked_(false) {
    if (mutex_ != nullptr && xSemaphoreTake(mutex_, waitTicks) == pdTRUE) {
      locked_ = true;
    }
  }

  ~MutexLock() {
    if (locked_ && mutex_ != nullptr) {
      xSemaphoreGive(mutex_);
    }
  }

  bool isLocked() const {
    return locked_;
  }

 private:
  SemaphoreHandle_t mutex_;
  bool locked_;
};

}  // namespace

const char* imuStatusName(ImuStatus status) {
  switch (status) {
    case ImuStatus::Uninitialized:      return "UNINITIALIZED";
    case ImuStatus::Initializing:       return "INITIALIZING";
    case ImuStatus::Ready:              return "READY";
    case ImuStatus::DataStale:          return "DATA_STALE";
    case ImuStatus::FifoOverrun:        return "FIFO_OVERRUN";
    case ImuStatus::BusError:           return "BUS_ERROR";
    case ImuStatus::SensorDisconnected: return "SENSOR_DISCONNECTED";
    case ImuStatus::InvalidData:        return "INVALID_DATA";
    case ImuStatus::HardwareError:      return "HARDWARE_ERROR";
    default:                            return "UNKNOWN";
  }
}

const char* imuStabilityName(ImuStability stability) {
  switch (stability) {
    case ImuStability::Unknown:  return "UNKNOWN";
    case ImuStability::Settling: return "SETTLING";
    case ImuStability::Unstable: return "UNSTABLE";
    case ImuStability::Stable:   return "STABLE";
    default:                     return "UNKNOWN";
  }
}

struct ImuInterface::Impl {
  // FreeRTOS Task Mutex serializing all hardware operations and model mutations
  SemaphoreHandle_t opMutex = nullptr;

  // Spinlocks for atomic read snapshot and buffer access
  mutable portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
  mutable portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

  // Configuration and Driver Context
  ImuConfig config{};
  ImuBusContext busCtx{};
  stmdev_ctx_t stCtx{};

  // Lifecycle flags (owned by opMutex)
  bool initialized = false;
  bool connected = false;

  // Atomic state snapshot (protected exclusively by stateMux)
  ImuState state{};

  // Ring buffer & drop count (protected exclusively by bufferMux)
  ImuSample ringBuffer[kRingBufferCapacity];
  size_t ringHead = 0;
  size_t ringTail = 0;
  size_t ringCount = 0;
  uint32_t droppedSampleCount_ = 0;

  // Single Authoritative Zero-Angle Calibration (owned by opMutex)
  float authoritativeZeroAngleDeg_ = 0.0f;

  // Internal private telemetry & filter state (owned by opMutex)
  bool filterInitialized_ = false;
  float filteredAngle_ = 0.0f;
  float angleWindow_[kStabilityWindowSize];
  size_t angleWindowCount_ = 0;
  size_t angleWindowHead_ = 0;
  uint32_t stableAccumulatedUs_ = 0;
  uint32_t sequenceCounter_ = 0;

  // Persistent reconstructed gyro tracking across batches (owned by opMutex)
  ReconstructedGyro latestGyro_{};

  // Reconstructed timestamp sequence anchors (owned by opMutex)
  uint32_t lastXlTimestampUs_ = 0;
  uint32_t lastGyTimestampUs_ = 0;
  uint32_t lastSampleTimeUs_ = 0;

  // Clipping transition state & internal counters (owned by opMutex)
  bool prevAccelClipped_ = false;
  bool prevGyroClipped_ = false;
  uint32_t accelClippingEvents_ = 0;
  uint32_t gyroClippingEvents_ = 0;

  // Cumulative diagnostic counters (owned by opMutex)
  uint32_t fifoOverrunCount_ = 0;
  uint32_t unsupportedTagCount_ = 0;
  uint32_t invalidSampleCount_ = 0;

  Impl() {
    memset(ringBuffer, 0, sizeof(ringBuffer));
    memset(angleWindow_, 0, sizeof(angleWindow_));
    stCtx.write_reg = platformWrite;
    stCtx.read_reg = platformRead;
    stCtx.mdelay = platformDelay;
    stCtx.handle = &busCtx;

    opMutex = xSemaphoreCreateMutex();
  }

  ~Impl() {
    if (opMutex != nullptr) {
      vSemaphoreDelete(opMutex);
      opMutex = nullptr;
    }
  }

  void pushSample(const ImuSample& sample) {
    portENTER_CRITICAL(&bufferMux);
    if (ringCount >= kRingBufferCapacity) {
      ringTail = (ringTail + 1) % kRingBufferCapacity;
      ringCount--;
      droppedSampleCount_++;
    }
    ringBuffer[ringHead] = sample;
    ringHead = (ringHead + 1) % kRingBufferCapacity;
    ringCount++;
    portEXIT_CRITICAL(&bufferMux);
  }

  void resetRuntimeStateLocked() {
    // Reset ring buffer under bufferMux
    portENTER_CRITICAL(&bufferMux);
    ringHead = 0;
    ringTail = 0;
    ringCount = 0;
    droppedSampleCount_ = 0;
    portEXIT_CRITICAL(&bufferMux);

    // Reset internal working states
    filterInitialized_ = false;
    filteredAngle_ = 0.0f;
    angleWindowCount_ = 0;
    angleWindowHead_ = 0;
    memset(angleWindow_, 0, sizeof(angleWindow_));
    stableAccumulatedUs_ = 0;
    sequenceCounter_ = 0;
    latestGyro_ = ReconstructedGyro{};
    lastXlTimestampUs_ = 0;
    lastGyTimestampUs_ = 0;
    lastSampleTimeUs_ = 0;
    prevAccelClipped_ = false;
    prevGyroClipped_ = false;
    accelClippingEvents_ = 0;
    gyroClippingEvents_ = 0;
    fifoOverrunCount_ = 0;
    unsupportedTagCount_ = 0;
    invalidSampleCount_ = 0;
    busCtx.i2cReadErrors = 0;
    busCtx.i2cWriteErrors = 0;
  }

  void endLocked() {
    if (initialized) {
      lsm6dsox_fifo_mode_set(&stCtx, LSM6DSOX_BYPASS_MODE);
      lsm6dsox_xl_data_rate_set(&stCtx, LSM6DSOX_XL_ODR_OFF);
      lsm6dsox_gy_data_rate_set(&stCtx, LSM6DSOX_GY_ODR_OFF);
    }

    initialized = false;
    connected = false;
    resetRuntimeStateLocked();

    portENTER_CRITICAL(&stateMux);
    state = ImuState{};
    state.status = ImuStatus::Uninitialized;
    portEXIT_CRITICAL(&stateMux);
  }
};

ImuInterface::ImuInterface()
    : impl_(new Impl()) {}

ImuInterface::~ImuInterface() {
  if (impl_ != nullptr) {
    if (impl_->opMutex != nullptr) {
      // Acquire exclusive ownership of opMutex before hardware teardown and mutex deletion.
      if (xSemaphoreTake(impl_->opMutex, portMAX_DELAY) == pdTRUE) {
        impl_->endLocked();
        xSemaphoreGive(impl_->opMutex);
      }
    }
    delete impl_;
    impl_ = nullptr;
  }
}

bool ImuInterface::begin(const ImuConfig& config) {
  if (impl_ == nullptr || impl_->opMutex == nullptr) {
    return false;
  }

  MutexLock lock(impl_->opMutex, kLifecycleMutexTimeoutTicks);
  if (!lock.isLocked()) {
    return false;
  }

  // Idempotent guard
  if (impl_->initialized && impl_->connected) {
    return true;
  }

  // Configuration validation before touching hardware
  if (!validateConfiguration(config)) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state = ImuState{};
    impl_->state.status = ImuStatus::HardwareError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Reset runtime state before starting initialization attempt
  impl_->resetRuntimeStateLocked();
  impl_->config = config;
  impl_->busCtx.wire = config.wire;
  impl_->busCtx.address = config.i2cAddress;

  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state = ImuState{};
  impl_->state.status = ImuStatus::Initializing;
  portEXIT_CRITICAL(&impl_->stateMux);

  // Initialize TwoWire I2C
  config.wire->begin(config.sdaPin, config.sclPin, config.i2cFrequencyHz);

  // Verify WHO_AM_I
  uint8_t whoAmI = 0;
  if (lsm6dsox_device_id_get(&impl_->stCtx, &whoAmI) != 0 || whoAmI != LSM6DSOX_ID) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->connected = false;
    impl_->initialized = false;
    impl_->state.connected = false;
    impl_->state.initialized = false;
    impl_->state.i2cReadErrorCount = impl_->busCtx.i2cReadErrors;
    impl_->state.i2cWriteErrorCount = impl_->busCtx.i2cWriteErrors;
    impl_->state.status = ImuStatus::SensorDisconnected;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Software reset
  if (lsm6dsox_reset_set(&impl_->stCtx, PROPERTY_ENABLE) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.i2cReadErrorCount = impl_->busCtx.i2cReadErrors;
    impl_->state.i2cWriteErrorCount = impl_->busCtx.i2cWriteErrors;
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Poll reset completion with timeout
  const uint32_t resetStartMs = millis();
  uint8_t resetDone = 1;
  while (resetDone != 0) {
    if ((millis() - resetStartMs) > config.resetTimeoutMs) {
      portENTER_CRITICAL(&impl_->stateMux);
      impl_->state.status = ImuStatus::HardwareError;
      portEXIT_CRITICAL(&impl_->stateMux);
      return false;
    }
    delay(5);
    if (lsm6dsox_reset_get(&impl_->stCtx, &resetDone) != 0) {
      portENTER_CRITICAL(&impl_->stateMux);
      impl_->state.i2cReadErrorCount = impl_->busCtx.i2cReadErrors;
      impl_->state.i2cWriteErrorCount = impl_->busCtx.i2cWriteErrors;
      impl_->state.status = ImuStatus::BusError;
      portEXIT_CRITICAL(&impl_->stateMux);
      return false;
    }
  }

  // Disable I3C
  if (lsm6dsox_i3c_disable_set(&impl_->stCtx, LSM6DSOX_I3C_DISABLE) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Enable Block Data Update (BDU)
  if (lsm6dsox_block_data_update_set(&impl_->stCtx, PROPERTY_ENABLE) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Configure Accelerometer full scale
  lsm6dsox_fs_xl_t fsXl = LSM6DSOX_8g;
  switch (config.accelRange) {
    case ImuAccelRange::G2:  fsXl = LSM6DSOX_2g; break;
    case ImuAccelRange::G4:  fsXl = LSM6DSOX_4g; break;
    case ImuAccelRange::G8:  fsXl = LSM6DSOX_8g; break;
    case ImuAccelRange::G16: fsXl = LSM6DSOX_16g; break;
  }
  if (lsm6dsox_xl_full_scale_set(&impl_->stCtx, fsXl) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Configure Gyroscope full scale
  lsm6dsox_fs_g_t fsG = LSM6DSOX_500dps;
  switch (config.gyroRange) {
    case ImuGyroRange::Dps125:  fsG = LSM6DSOX_125dps; break;
    case ImuGyroRange::Dps250:  fsG = LSM6DSOX_250dps; break;
    case ImuGyroRange::Dps500:  fsG = LSM6DSOX_500dps; break;
    case ImuGyroRange::Dps1000: fsG = LSM6DSOX_1000dps; break;
    case ImuGyroRange::Dps2000: fsG = LSM6DSOX_2000dps; break;
  }
  if (lsm6dsox_gy_full_scale_set(&impl_->stCtx, fsG) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Configure Accelerometer and Gyroscope ODR (104 Hz)
  if (lsm6dsox_xl_data_rate_set(&impl_->stCtx, LSM6DSOX_XL_ODR_104Hz) != 0 ||
      lsm6dsox_gy_data_rate_set(&impl_->stCtx, LSM6DSOX_GY_ODR_104Hz) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Configure FIFO batching at 104 Hz
  if (lsm6dsox_fifo_xl_batch_set(&impl_->stCtx, LSM6DSOX_XL_BATCHED_AT_104Hz) != 0 ||
      lsm6dsox_fifo_gy_batch_set(&impl_->stCtx, LSM6DSOX_GY_BATCHED_AT_104Hz) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Configure FIFO watermark
  if (lsm6dsox_fifo_watermark_set(&impl_->stCtx, config.fifoWatermark) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Configure FIFO Continuous/Stream mode
  if (lsm6dsox_fifo_mode_set(&impl_->stCtx, LSM6DSOX_STREAM_MODE) != 0) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state.status = ImuStatus::BusError;
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  // Drain initial stale FIFO entries
  uint8_t dummyBuf[7];
  for (int i = 0; i < 32; ++i) {
    uint16_t numEntries = 0;
    if (lsm6dsox_fifo_data_level_get(&impl_->stCtx, &numEntries) != 0 || numEntries == 0) {
      break;
    }
    lsm6dsox_fifo_out_raw_get(&impl_->stCtx, dummyBuf);
  }

  // Commit clean initial Ready state
  impl_->initialized = true;
  impl_->connected = true;
  impl_->lastSampleTimeUs_ = micros();

  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state.initialized = true;
  impl_->state.connected = true;
  impl_->state.status = ImuStatus::Ready;
  impl_->state.dataValid = false;
  impl_->state.dataStale = false;
  portEXIT_CRITICAL(&impl_->stateMux);

  return true;
}

void ImuInterface::end() {
  if (impl_ == nullptr || impl_->opMutex == nullptr) {
    return;
  }

  MutexLock lock(impl_->opMutex, kLifecycleMutexTimeoutTicks);
  if (!lock.isLocked()) {
    return;
  }

  impl_->endLocked();
}

void ImuInterface::update() {
  if (impl_ == nullptr || impl_->opMutex == nullptr) {
    return;
  }

  // Non-blocking update: if operation mutex is busy, skip this cycle promptly
  MutexLock lock(impl_->opMutex, kUpdateMutexTimeoutTicks);
  if (!lock.isLocked()) {
    return;
  }

  if (!impl_->initialized || !impl_->connected) {
    return;
  }

  const uint32_t startUs = micros();
  const uint32_t odrPeriodUs = getOdrPeriodUs(impl_->config.outputDataRate);
  const float accelSensitivity = getAccelSensitivityG(impl_->config.accelRange);
  const float gyroSensitivity = getGyroSensitivityDps(impl_->config.gyroRange);
  const float accelClipLimit = getAccelFullScaleG(impl_->config.accelRange) * 0.98f;
  const float gyroClipLimit = getGyroFullScaleDps(impl_->config.gyroRange) * 0.98f;

  bool localBusError = false;
  bool localFifoOverrun = false;

  // 1. Read FIFO Status
  lsm6dsox_fifo_status2_t fifoStatus2{};
  if (lsm6dsox_fifo_status_get(&impl_->stCtx, &fifoStatus2) != 0) {
    localBusError = true;
  } else if (fifoStatus2.fifo_ovr_ia != 0 || fifoStatus2.over_run_latched != 0) {
    localFifoOverrun = true;
    impl_->fifoOverrunCount_++;
  }

  // 2. Read number of unread FIFO words
  uint16_t fifoLevel = 0;
  if (!localBusError) {
    if (lsm6dsox_fifo_data_level_get(&impl_->stCtx, &fifoLevel) != 0) {
      localBusError = true;
    }
  }

  // 3. Drain bounded batch into fixed temporary storage
  FifoRawRecord batch[kMaxBatchEntries];
  size_t batchCount = 0;
  size_t accelCountInBatch = 0;
  size_t gyroCountInBatch = 0;

  if (!localBusError) {
    while (batchCount < impl_->config.maximumFifoEntriesPerUpdate &&
           batchCount < kMaxBatchEntries &&
           fifoLevel > 0) {
      // Soft time-budget check before starting next transaction
      if ((micros() - startUs) >= impl_->config.maximumUpdateTimeUs) {
        break;
      }

      uint8_t rawEntry[7];
      if (lsm6dsox_fifo_out_raw_get(&impl_->stCtx, rawEntry) != 0) {
        localBusError = true;
        break;
      }

      const uint8_t tag = (rawEntry[0] >> 3) & 0x1F;
      batch[batchCount].tag = tag;
      batch[batchCount].raw[0] = static_cast<int16_t>(static_cast<uint16_t>(rawEntry[1]) | (static_cast<uint16_t>(rawEntry[2]) << 8));
      batch[batchCount].raw[1] = static_cast<int16_t>(static_cast<uint16_t>(rawEntry[3]) | (static_cast<uint16_t>(rawEntry[4]) << 8));
      batch[batchCount].raw[2] = static_cast<int16_t>(static_cast<uint16_t>(rawEntry[5]) | (static_cast<uint16_t>(rawEntry[6]) << 8));

      if (tag == LSM6DSOX_XL_NC_TAG) {
        accelCountInBatch++;
      } else if (tag == LSM6DSOX_GYRO_NC_TAG) {
        gyroCountInBatch++;
      } else if (tag != LSM6DSOX_TIMESTAMP_TAG) {
        impl_->unsupportedTagCount_++;
      }

      batchCount++;
      fifoLevel--;
    }
  }

  // Working telemetry values for snapshot update
  bool sampleEmitted = false;
  ImuSample latestEmittedSample{};
  bool latestAccelClipped = impl_->prevAccelClipped_;
  bool latestGyroClipped = impl_->prevGyroClipped_;

  // 4. Reconstruct Gyro and Accel streams with reconstructed measurement timestamps
  if (!localBusError && (accelCountInBatch > 0 || gyroCountInBatch > 0)) {
    const uint32_t batchTimeUs = micros();

    // First, reconstruct all Gyroscope timestamps and store them in chronological order
    ReconstructedGyro batchGyros[kMaxBatchEntries];
    size_t batchGyroCount = 0;
    size_t gyIdx = 0;

    for (size_t i = 0; i < batchCount; ++i) {
      if (batch[i].tag == LSM6DSOX_GYRO_NC_TAG) {
        const uint32_t gyOffsetUs = static_cast<uint32_t>(gyroCountInBatch - 1 - gyIdx) * odrPeriodUs;
        uint32_t gyTimestampUs = batchTimeUs - gyOffsetUs;
        if (gyTimestampUs <= impl_->lastGyTimestampUs_) {
          gyTimestampUs = impl_->lastGyTimestampUs_ + 1;
        }
        impl_->lastGyTimestampUs_ = gyTimestampUs;

        const float rawGx = static_cast<float>(batch[i].raw[0]) * gyroSensitivity;
        const float rawGy = static_cast<float>(batch[i].raw[1]) * gyroSensitivity;
        const float rawGz = static_cast<float>(batch[i].raw[2]) * gyroSensitivity;

        batchGyros[batchGyroCount].timestampUs = gyTimestampUs;
        batchGyros[batchGyroCount].x = rawGx;
        batchGyros[batchGyroCount].y = rawGy;
        batchGyros[batchGyroCount].z = rawGz;
        batchGyros[batchGyroCount].valid = true;

        // Update latest uncoupled gyro
        impl_->latestGyro_ = batchGyros[batchGyroCount];

        // Gyro clipping check
        const bool gyClipped = (fabsf(rawGx) >= gyroClipLimit ||
                                fabsf(rawGy) >= gyroClipLimit ||
                                fabsf(rawGz) >= gyroClipLimit);
        if (gyClipped && !impl_->prevGyroClipped_) {
          impl_->gyroClippingEvents_++;
          impl_->invalidSampleCount_++;
        }
        impl_->prevGyroClipped_ = gyClipped;
        latestGyroClipped = gyClipped;

        batchGyroCount++;
        gyIdx++;
      }
    }

    // Next, reconstruct Accelerometer timestamps and pair with reconstructed gyro
    size_t xlIdx = 0;
    for (size_t i = 0; i < batchCount; ++i) {
      if (batch[i].tag == LSM6DSOX_XL_NC_TAG) {
        const uint32_t xlOffsetUs = static_cast<uint32_t>(accelCountInBatch - 1 - xlIdx) * odrPeriodUs;
        uint32_t xlTimestampUs = batchTimeUs - xlOffsetUs;
        if (xlTimestampUs <= impl_->lastXlTimestampUs_) {
          xlTimestampUs = impl_->lastXlTimestampUs_ + 1;
        }
        impl_->lastXlTimestampUs_ = xlTimestampUs;
        impl_->lastSampleTimeUs_ = xlTimestampUs;

        const float rawXlX = static_cast<float>(batch[i].raw[0]) * accelSensitivity;
        const float rawXlY = static_cast<float>(batch[i].raw[1]) * accelSensitivity;
        const float rawXlZ = static_cast<float>(batch[i].raw[2]) * accelSensitivity;

        ImuSample sample{};
        sample.timestampUs = xlTimestampUs;
        sample.sequence = ++impl_->sequenceCounter_;
        sample.timestampEstimated = true;

        transformAxes(rawXlX, rawXlY, rawXlZ, impl_->config.axisMapping,
                      &sample.accelLongitudinalG,
                      &sample.accelLateralG,
                      &sample.accelVerticalG);
        sample.accelValid = true;

        // Accelerometer clipping check
        const bool xlClipped = (fabsf(rawXlX) >= accelClipLimit ||
                                fabsf(rawXlY) >= accelClipLimit ||
                                fabsf(rawXlZ) >= accelClipLimit);
        if (xlClipped && !impl_->prevAccelClipped_) {
          impl_->accelClippingEvents_++;
          impl_->invalidSampleCount_++;
        }
        impl_->prevAccelClipped_ = xlClipped;
        latestAccelClipped = xlClipped;
        if (xlClipped) {
          sample.accelValid = false;
        }

        // Pair with nearest reconstructed gyroscope measurement
        if (impl_->latestGyro_.valid) {
          const uint32_t gyroAgeUs = (xlTimestampUs >= impl_->latestGyro_.timestampUs)
                                         ? (xlTimestampUs - impl_->latestGyro_.timestampUs)
                                         : (impl_->latestGyro_.timestampUs - xlTimestampUs);
          sample.gyroAgeUs = gyroAgeUs;

          if (gyroAgeUs <= impl_->config.maximumGyroPairAgeUs && !latestGyroClipped) {
            transformAxes(impl_->latestGyro_.x, impl_->latestGyro_.y, impl_->latestGyro_.z,
                          impl_->config.axisMapping,
                          &sample.gyroLongitudinalDps,
                          &sample.gyroLateralDps,
                          &sample.gyroVerticalDps);
            sample.gyroValid = true;
          } else {
            sample.gyroValid = false;
          }
        } else {
          sample.gyroValid = false;
          sample.gyroAgeUs = 0xFFFFFFFF;
        }

        // Push completed sample into ring buffer (under bufferMux)
        impl_->pushSample(sample);

        sampleEmitted = true;
        latestEmittedSample = sample;
        xlIdx++;
      }
    }
  }

  // 5. Working Deck Angle & Stability Calculations (outside stateMux, serialized by opMutex)
  float rawDeckAngle = 0.0f;
  float filteredDeckAngle = impl_->filteredAngle_;
  float relativeDeckAngle = 0.0f;
  bool deckAngleValid = false;
  float angleVariance = 0.0f;
  ImuStability stability = ImuStability::Unknown;

  if (sampleEmitted) {
    const float longG = latestEmittedSample.accelLongitudinalG;
    const float latG = latestEmittedSample.accelLateralG;
    const float vertG = latestEmittedSample.accelVerticalG;
    const float accelNormSq = (longG * longG) + (latG * latG) + (vertG * vertG);
    const float accelNorm = sqrtf(accelNormSq);

    const float gyroLongDps = latestEmittedSample.gyroLongitudinalDps;
    const float gyroLatDps = latestEmittedSample.gyroLateralDps;
    const float gyroVertDps = latestEmittedSample.gyroVerticalDps;
    const float gyroNormSq = (gyroLongDps * gyroLongDps) + (gyroLatDps * gyroLatDps) + (gyroVertDps * gyroVertDps);
    const float gyroNorm = sqrtf(gyroNormSq);

    const float normDiff = fabsf(accelNorm - 1.0f);
    const bool normValid = (normDiff <= impl_->config.accelerationNormToleranceG);
    const bool valuesFinite = !isnan(longG) && !isnan(vertG) && !isinf(longG) && !isinf(vertG);
    const bool accelNotClipped = !latestAccelClipped;

    if (latestEmittedSample.accelValid && normValid && valuesFinite && accelNotClipped) {
      rawDeckAngle = atan2f(-longG, vertG) * kRadToDeg;
      deckAngleValid = true;

      if (!impl_->filterInitialized_) {
        filteredDeckAngle = rawDeckAngle;
        impl_->filteredAngle_ = rawDeckAngle;
        impl_->filterInitialized_ = true;
      } else {
        const float alpha = impl_->config.shortAngleFilterAlpha;
        filteredDeckAngle = impl_->filteredAngle_ + alpha * (rawDeckAngle - impl_->filteredAngle_);
        impl_->filteredAngle_ = filteredDeckAngle;
      }
      relativeDeckAngle = filteredDeckAngle - impl_->authoritativeZeroAngleDeg_;

      // Update 16-sample rolling angle variance window
      impl_->angleWindow_[impl_->angleWindowHead_] = rawDeckAngle;
      impl_->angleWindowHead_ = (impl_->angleWindowHead_ + 1) % kStabilityWindowSize;
      if (impl_->angleWindowCount_ < kStabilityWindowSize) {
        impl_->angleWindowCount_++;
      }

      float sum = 0.0f;
      for (size_t i = 0; i < impl_->angleWindowCount_; ++i) {
        sum += impl_->angleWindow_[i];
      }
      const float mean = sum / static_cast<float>(impl_->angleWindowCount_);
      float varSum = 0.0f;
      for (size_t i = 0; i < impl_->angleWindowCount_; ++i) {
        const float diff = impl_->angleWindow_[i] - mean;
        varSum += diff * diff;
      }
      angleVariance = (impl_->angleWindowCount_ > 1) ? (varSum / static_cast<float>(impl_->angleWindowCount_ - 1)) : 0.0f;

      // Evaluate stability
      const bool gyroStable = latestEmittedSample.gyroValid && (gyroNorm <= impl_->config.maximumStableGyroDps);
      const bool varianceStable = (impl_->angleWindowCount_ >= kStabilityWindowSize) &&
                                  (angleVariance <= impl_->config.maximumStableAngleVarianceDeg2);
      const bool clippingFree = !latestAccelClipped && !latestGyroClipped;

      if (normValid && gyroStable && varianceStable && clippingFree) {
        impl_->stableAccumulatedUs_ += odrPeriodUs;
        if ((impl_->stableAccumulatedUs_ / 1000) >= impl_->config.stabilizationWindowMs) {
          stability = ImuStability::Stable;
        } else {
          stability = ImuStability::Settling;
        }
      } else {
        impl_->stableAccumulatedUs_ = 0;
        stability = ImuStability::Unstable;
      }
    } else {
      deckAngleValid = false;
      impl_->stableAccumulatedUs_ = 0;
      stability = ImuStability::Unstable;
    }
  }

  // 6. Staleness Check
  const uint32_t nowUs = micros();
  const uint32_t dataAgeUs = nowUs - impl_->lastSampleTimeUs_;
  const uint32_t dataAgeMs = dataAgeUs / 1000;
  const bool dataStale = (impl_->lastSampleTimeUs_ == 0) || (dataAgeMs > impl_->config.dataStaleTimeoutMs);

  if (dataStale) {
    deckAngleValid = false;
    stability = ImuStability::Unknown;
    impl_->stableAccumulatedUs_ = 0;
  }

  // 7. Get Current Dropped Count from bufferMux (without holding stateMux)
  uint32_t currentDropped = 0;
  portENTER_CRITICAL(&impl_->bufferMux);
  currentDropped = impl_->droppedSampleCount_;
  portEXIT_CRITICAL(&impl_->bufferMux);

  // 8. Atomic State Commit under stateMux
  portENTER_CRITICAL(&impl_->stateMux);

  impl_->state.initialized = impl_->initialized;
  impl_->state.connected = impl_->connected && !localBusError;

  if (sampleEmitted) {
    impl_->state.accelLongitudinalG = latestEmittedSample.accelLongitudinalG;
    impl_->state.accelLateralG = latestEmittedSample.accelLateralG;
    impl_->state.accelVerticalG = latestEmittedSample.accelVerticalG;
    impl_->state.accelerationMagnitudeG = sqrtf((latestEmittedSample.accelLongitudinalG * latestEmittedSample.accelLongitudinalG) +
                                                (latestEmittedSample.accelLateralG * latestEmittedSample.accelLateralG) +
                                                (latestEmittedSample.accelVerticalG * latestEmittedSample.accelVerticalG));

    impl_->state.gyroLongitudinalDps = latestEmittedSample.gyroLongitudinalDps;
    impl_->state.gyroLateralDps = latestEmittedSample.gyroLateralDps;
    impl_->state.gyroVerticalDps = latestEmittedSample.gyroVerticalDps;
    impl_->state.gyroMagnitudeDps = sqrtf((latestEmittedSample.gyroLongitudinalDps * latestEmittedSample.gyroLongitudinalDps) +
                                          (latestEmittedSample.gyroLateralDps * latestEmittedSample.gyroLateralDps) +
                                          (latestEmittedSample.gyroVerticalDps * latestEmittedSample.gyroVerticalDps));

    impl_->state.rawDeckAngleDeg = rawDeckAngle;
    impl_->state.filteredDeckAngleDeg = filteredDeckAngle;
    impl_->state.zeroAngleDeg = impl_->authoritativeZeroAngleDeg_;
    impl_->state.relativeDeckAngleDeg = relativeDeckAngle;
    impl_->state.deckAngleValid = deckAngleValid;
    impl_->state.angleVarianceDeg2 = angleVariance;
    impl_->state.stability = stability;
    impl_->state.stabilityDurationMs = impl_->stableAccumulatedUs_ / 1000;

    impl_->state.sampleTimestampUs = latestEmittedSample.timestampUs;
    impl_->state.sampleSequence = latestEmittedSample.sequence;
    impl_->state.dataValid = latestEmittedSample.accelValid && !dataStale;
  } else if (dataStale) {
    impl_->state.dataValid = false;
    impl_->state.deckAngleValid = false;
    impl_->state.stability = ImuStability::Unknown;
    impl_->state.stabilityDurationMs = 0;
  }

  impl_->state.dataStale = dataStale;
  impl_->state.accelerometerClipped = latestAccelClipped;
  impl_->state.gyroscopeClipped = latestGyroClipped;

  impl_->state.unreadFifoEntryCount = fifoLevel;
  impl_->state.lastBatchEntryCount = static_cast<uint16_t>(batchCount);

  impl_->state.i2cReadErrorCount = impl_->busCtx.i2cReadErrors;
  impl_->state.i2cWriteErrorCount = impl_->busCtx.i2cWriteErrors;
  impl_->state.fifoOverrunCount = impl_->fifoOverrunCount_;
  impl_->state.unsupportedTagCount = impl_->unsupportedTagCount_;
  impl_->state.invalidSampleCount = impl_->invalidSampleCount_;
  impl_->state.droppedSampleCount = currentDropped;

  impl_->state.snapshotTimestampUs = nowUs;
  impl_->state.dataAgeMs = dataAgeMs;

  const bool invalidData = (!impl_->state.dataValid && !dataStale) || latestAccelClipped || latestGyroClipped;
  impl_->state.status = computeStatus(impl_->initialized,
                                      impl_->state.connected,
                                      false,
                                      localBusError,
                                      localFifoOverrun,
                                      invalidData,
                                      dataStale);

  portEXIT_CRITICAL(&impl_->stateMux);
}

ImuState ImuInterface::getState() const {
  ImuState snapshot{};
  if (impl_ == nullptr) {
    return snapshot;
  }
  portENTER_CRITICAL(&impl_->stateMux);
  snapshot = impl_->state;
  portEXIT_CRITICAL(&impl_->stateMux);
  return snapshot;
}

size_t ImuInterface::readSamples(ImuSample* destination, size_t capacity) {
  if (impl_ == nullptr || destination == nullptr || capacity == 0) {
    return 0;
  }

  size_t copied = 0;
  portENTER_CRITICAL(&impl_->bufferMux);
  while (copied < capacity && impl_->ringCount > 0) {
    destination[copied] = impl_->ringBuffer[impl_->ringTail];
    impl_->ringTail = (impl_->ringTail + 1) % kRingBufferCapacity;
    impl_->ringCount--;
    copied++;
  }
  portEXIT_CRITICAL(&impl_->bufferMux);

  return copied;
}

bool ImuInterface::setCurrentAngleAsZero() {
  if (impl_ == nullptr || impl_->opMutex == nullptr) {
    return false;
  }

  MutexLock lock(impl_->opMutex, kCalibrationMutexTimeoutTicks);
  if (!lock.isLocked()) {
    return false;
  }

  if (!impl_->initialized || !impl_->connected) {
    return false;
  }

  // Evaluate preconditions against current working telemetry
  if (!impl_->filterInitialized_ || !impl_->state.deckAngleValid || impl_->state.dataStale || !impl_->state.dataValid) {
    return false;
  }
  if (impl_->state.stability != ImuStability::Stable ||
      (impl_->stableAccumulatedUs_ / 1000) < impl_->config.stabilizationWindowMs) {
    return false;
  }
  if (impl_->prevAccelClipped_ || impl_->prevGyroClipped_) {
    return false;
  }

  impl_->authoritativeZeroAngleDeg_ = impl_->filteredAngle_;

  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state.zeroAngleDeg = impl_->authoritativeZeroAngleDeg_;
  impl_->state.relativeDeckAngleDeg = 0.0f;
  portEXIT_CRITICAL(&impl_->stateMux);

  return true;
}

bool ImuInterface::applyZeroAngleDeg(float zeroAngleDeg) {
  if (impl_ == nullptr || impl_->opMutex == nullptr || isnan(zeroAngleDeg) || isinf(zeroAngleDeg)) {
    return false;
  }
  if (zeroAngleDeg < -45.0f || zeroAngleDeg > 45.0f) {
    return false;
  }

  MutexLock lock(impl_->opMutex, kCalibrationMutexTimeoutTicks);
  if (!lock.isLocked()) {
    return false;
  }

  impl_->authoritativeZeroAngleDeg_ = zeroAngleDeg;

  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state.zeroAngleDeg = zeroAngleDeg;
  if (impl_->state.deckAngleValid) {
    impl_->state.relativeDeckAngleDeg = impl_->filteredAngle_ - zeroAngleDeg;
  }
  portEXIT_CRITICAL(&impl_->stateMux);

  return true;
}

float ImuInterface::getZeroAngleDeg() const {
  if (impl_ == nullptr) {
    return 0.0f;
  }
  portENTER_CRITICAL(&impl_->stateMux);
  const float zero = impl_->state.zeroAngleDeg;
  portEXIT_CRITICAL(&impl_->stateMux);
  return zero;
}

ImuConfig ImuInterface::configSnapshot() const {
  if (impl_ == nullptr) {
    return ImuConfig{};
  }
  portENTER_CRITICAL(&impl_->stateMux);
  const ImuConfig cfg = impl_->config;
  portEXIT_CRITICAL(&impl_->stateMux);
  return cfg;
}

bool ImuInterface::isReady() const {
  if (impl_ == nullptr) {
    return false;
  }
  portENTER_CRITICAL(&impl_->stateMux);
  const bool ready = impl_->initialized && impl_->connected && (impl_->state.status == ImuStatus::Ready);
  portEXIT_CRITICAL(&impl_->stateMux);
  return ready;
}

const char* ImuInterface::version() {
  return kImuInterfaceVersion;
}

}  // namespace stridecontrol
