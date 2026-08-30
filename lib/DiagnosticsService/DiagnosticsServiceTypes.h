#pragma once

#include <cstddef>
#include <cstdint>
#include <bitset>

namespace stridecontrol {

/**
 * @brief Categorized severity levels for system faults.
 */
enum class FaultSeverity : uint8_t {
    None = 0,
    Notice,
    Warning,
    Critical,
    Fatal
};

/**
 * @brief Authoritative system fault identifier codes.
 *
 * Count must remain the final element to size static arrays and bitsets.
 */
enum class FaultCode : uint16_t {
    None = 0,

    // Speed Sensor Subsystem
    SpeedSensorTimeout,
    SpeedSensorSignalLoss,
    SpeedSensorHardwareError,

    // IMU Subsystem
    ImuCommunicationError,
    ImuDataStale,
    ImuFifoOverrun,
    ImuSensorDisconnected,
    ImuHardwareError,

    // Incline Sensor Subsystem
    InclineSensorTimeout,
    InclineSensorPositionLimit,
    InclineSensorHardwareError,

    // Incline Verifier Subsystem
    InclineVerifierDivergence,
    InclineVerifierMismatch,

    // Runner Dynamics Subsystem
    RunnerDynamicsDiscontinuity,

    // Communication Interfaces
    CsafeTimeout,
    CsafeFrameError,
    ConsoleTimeout,
    ConsoleCommandError,

    // Motor, Power & Safety Subsystem
    MotorInterlockFault,
    EmergencyStopActive,
    OverTemperatureWarning,
    PowerSupplyFault,
    SystemWatchdogWarning,

    Count
};

constexpr size_t kEventHistorySize = 32;
constexpr size_t kFaultCodeCount = static_cast<size_t>(FaultCode::Count);

/**
 * @brief State transition log entry for circular event buffer.
 */
struct FaultEvent {
    uint32_t timestampMs = 0;
    FaultCode code = FaultCode::None;
    FaultSeverity severity = FaultSeverity::None;
    bool isActive = false;
};

/**
 * @brief Snapshot of active system health and recent event log.
 */
struct SystemHealthSnapshot {
    bool isHealthy = true;
    uint32_t activeFaultCount = 0;
    FaultSeverity highestSeverity = FaultSeverity::None;
    uint32_t totalLoggedEvents = 0;
    uint32_t snapshotTimestampMs = 0;

    std::bitset<kFaultCodeCount> activeFaultBits{};

    // Chronological event log: oldest to newest
    FaultEvent recentEvents[kEventHistorySize]{};
    size_t recentEventCount = 0;
};

/**
 * @brief Compile-time static mapping from FaultCode to inherent FaultSeverity.
 */
constexpr FaultSeverity getFaultSeverity(FaultCode code) {
    return (code == FaultCode::None) ? FaultSeverity::None :
           (code == FaultCode::InclineVerifierDivergence ||
            code == FaultCode::RunnerDynamicsDiscontinuity ||
            code == FaultCode::ImuDataStale) ? FaultSeverity::Notice :
           (code == FaultCode::SpeedSensorTimeout ||
            code == FaultCode::SpeedSensorSignalLoss ||
            code == FaultCode::ImuFifoOverrun ||
            code == FaultCode::InclineSensorTimeout ||
            code == FaultCode::InclineSensorPositionLimit ||
            code == FaultCode::InclineVerifierMismatch ||
            code == FaultCode::CsafeTimeout ||
            code == FaultCode::CsafeFrameError ||
            code == FaultCode::ConsoleTimeout ||
            code == FaultCode::ConsoleCommandError ||
            code == FaultCode::OverTemperatureWarning ||
            code == FaultCode::SystemWatchdogWarning) ? FaultSeverity::Warning :
           (code == FaultCode::SpeedSensorHardwareError ||
            code == FaultCode::ImuCommunicationError ||
            code == FaultCode::ImuSensorDisconnected ||
            code == FaultCode::InclineSensorHardwareError ||
            code == FaultCode::MotorInterlockFault ||
            code == FaultCode::PowerSupplyFault) ? FaultSeverity::Critical :
           (code == FaultCode::ImuHardwareError ||
            code == FaultCode::EmergencyStopActive) ? FaultSeverity::Fatal :
           FaultSeverity::None;
}

const char* faultCodeToString(FaultCode code);
const char* faultSeverityToString(FaultSeverity severity);

} // namespace stridecontrol
