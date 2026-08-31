#pragma once

#include <cstdint>
#include "../SpeedSensor/SpeedSensorTypes.h"
#include "../InclineSensor/InclineSensorTypes.h"
#include "../ImuInterface/ImuTypes.h"
#include "../RunnerDynamics/RunnerDynamicsTypes.h"
#include "../InclineVerifier/InclineVerifierTypes.h"
#include "../DiagnosticsService/DiagnosticsServiceTypes.h"
#include "../BluetoothTypes/BluetoothTypes.h"

namespace stridecontrol {

/**
 * @brief Pure passive Data Transfer Object (DTO) capturing the physical
 *        hardware sensor state, verification state, system health, and BLE telemetry.
 *
 * Designed with pure value semantics, zero dynamic heap allocation,
 * and zero runtime logic for safe, fast cross-core duplication.
 */
struct ApplicationSnapshot {
    uint32_t timestampMs = 0;
    uint32_t sequenceNumber = 0;

    SpeedSensorState speed{};
    InclineState incline{};
    ImuState imu{};
    RunnerDynamicsState runner{};
    InclineVerifierState inclineVerifier{};
    SystemHealthSnapshot health{};

    HeartRateState heartRate{};
    BleState ble{};
};

} // namespace stridecontrol
