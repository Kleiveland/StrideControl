#pragma once
#include <stdint.h>

namespace stridecontrol {

enum class ImuAxis : uint8_t {
  X = 0,
  Y = 1,
  Z = 2
};

struct ImuAxisMapping {
  ImuAxis longitudinalAxis = ImuAxis::X;
  int8_t longitudinalSign = 1;

  ImuAxis lateralAxis = ImuAxis::Y;
  int8_t lateralSign = 1;

  ImuAxis verticalAxis = ImuAxis::Z;
  int8_t verticalSign = 1;
};

enum class ImuAccelRange : uint8_t {
  G2 = 0,
  G4 = 1,
  G8 = 2,
  G16 = 3
};

enum class ImuGyroRange : uint8_t {
  Dps125 = 0,
  Dps250 = 1,
  Dps500 = 2,
  Dps1000 = 3,
  Dps2000 = 4
};

enum class ImuOutputDataRate : uint8_t {
  PowerDown = 0,
  Hz12_5 = 1,
  Hz26 = 2,
  Hz52 = 3,
  Hz104 = 4,
  Hz208 = 5
};

enum class ImuStability : uint8_t {
  Unknown = 0,
  Settling = 1,
  Unstable = 2,
  Stable = 3
};

enum class ImuStatus : uint8_t {
  Uninitialized = 0,
  Initializing = 1,
  Ready = 2,
  DataStale = 3,
  FifoOverrun = 4,
  BusError = 5,
  SensorDisconnected = 6,
  InvalidData = 7,
  HardwareError = 8
};

enum class ImuObservationMode : uint8_t {
  HardwareI2c = 0,         ///< Production: LSM6DSOX I2C hardware FIFO communication
  SoftwareObservation = 1  ///< Simulation: Synthetic sample ingestion via observeSamples(), zero I2C
};

struct ImuSample {
  uint32_t timestampUs = 0;
  uint32_t sequence = 0;

  float accelLongitudinalG = 0.0f;
  float accelLateralG = 0.0f;
  float accelVerticalG = 0.0f;

  float gyroLongitudinalDps = 0.0f;
  float gyroLateralDps = 0.0f;
  float gyroVerticalDps = 0.0f;

  uint32_t gyroAgeUs = 0;

  bool accelValid = false;
  bool gyroValid = false;
  bool timestampEstimated = true;
};

struct ImuState {
  bool initialized = false;
  bool connected = false;
  bool dataValid = false;
  bool dataStale = false;

  float accelLongitudinalG = 0.0f;
  float accelLateralG = 0.0f;
  float accelVerticalG = 0.0f;
  float accelerationMagnitudeG = 0.0f;

  float gyroLongitudinalDps = 0.0f;
  float gyroLateralDps = 0.0f;
  float gyroVerticalDps = 0.0f;
  float gyroMagnitudeDps = 0.0f;

  float rawDeckAngleDeg = 0.0f;
  float filteredDeckAngleDeg = 0.0f;
  float relativeDeckAngleDeg = 0.0f;
  float zeroAngleDeg = 0.0f;

  bool deckAngleValid = false;
  ImuStability stability = ImuStability::Unknown;
  uint32_t stabilityDurationMs = 0;
  float angleVarianceDeg2 = 0.0f;

  bool accelerometerClipped = false;
  bool gyroscopeClipped = false;

  uint16_t unreadFifoEntryCount = 0;
  uint16_t lastBatchEntryCount = 0;

  uint32_t i2cReadErrorCount = 0;
  uint32_t i2cWriteErrorCount = 0;
  uint32_t fifoOverrunCount = 0;
  uint32_t invalidSampleCount = 0;
  uint32_t unsupportedTagCount = 0;
  uint32_t droppedSampleCount = 0;

  uint32_t sampleTimestampUs = 0;
  uint32_t snapshotTimestampUs = 0;
  uint32_t dataAgeMs = 0;
  uint32_t sampleSequence = 0;

  ImuStatus status = ImuStatus::Uninitialized;
};

const char* imuStatusName(ImuStatus status);
const char* imuStabilityName(ImuStability stability);

}  // namespace stridecontrol

