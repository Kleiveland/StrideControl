#include "DiagnosticsService.h"

namespace stridecontrol {

const char* faultSeverityToString(FaultSeverity severity) {
    switch (severity) {
        case FaultSeverity::None:     return "NONE";
        case FaultSeverity::Notice:   return "NOTICE";
        case FaultSeverity::Warning:  return "WARNING";
        case FaultSeverity::Critical: return "CRITICAL";
        case FaultSeverity::Fatal:    return "FATAL";
    }
    return "UNKNOWN";
}

const char* faultCodeToString(FaultCode code) {
    switch (code) {
        case FaultCode::None:                        return "NONE";
        case FaultCode::SpeedSensorTimeout:          return "SPEED_SENSOR_TIMEOUT";
        case FaultCode::SpeedSensorSignalLoss:        return "SPEED_SENSOR_SIGNAL_LOSS";
        case FaultCode::SpeedSensorHardwareError:     return "SPEED_SENSOR_HARDWARE_ERROR";
        case FaultCode::ImuCommunicationError:       return "IMU_COMMUNICATION_ERROR";
        case FaultCode::ImuDataStale:                return "IMU_DATA_STALE";
        case FaultCode::ImuFifoOverrun:              return "IMU_FIFO_OVERRUN";
        case FaultCode::ImuSensorDisconnected:       return "IMU_SENSOR_DISCONNECTED";
        case FaultCode::ImuHardwareError:            return "IMU_HARDWARE_ERROR";
        case FaultCode::InclineSensorTimeout:        return "INCLINE_SENSOR_TIMEOUT";
        case FaultCode::InclineSensorPositionLimit:  return "INCLINE_SENSOR_POSITION_LIMIT";
        case FaultCode::InclineSensorHardwareError:  return "INCLINE_SENSOR_HARDWARE_ERROR";
        case FaultCode::InclineVerifierDivergence:   return "INCLINE_VERIFIER_DIVERGENCE";
        case FaultCode::InclineVerifierMismatch:     return "INCLINE_VERIFIER_MISMATCH";
        case FaultCode::RunnerDynamicsDiscontinuity: return "RUNNER_DYNAMICS_DISCONTINUITY";
        case FaultCode::CsafeTimeout:                return "CSAFE_TIMEOUT";
        case FaultCode::CsafeFrameError:             return "CSAFE_FRAME_ERROR";
        case FaultCode::ConsoleTimeout:              return "CONSOLE_TIMEOUT";
        case FaultCode::ConsoleCommandError:         return "CONSOLE_COMMAND_ERROR";
        case FaultCode::MotorInterlockFault:         return "MOTOR_INTERLOCK_FAULT";
        case FaultCode::EmergencyStopActive:         return "EMERGENCY_STOP_ACTIVE";
        case FaultCode::OverTemperatureWarning:      return "OVER_TEMPERATURE_WARNING";
        case FaultCode::PowerSupplyFault:            return "POWER_SUPPLY_FAULT";
        case FaultCode::SystemWatchdogWarning:       return "SYSTEM_WATCHDOG_WARNING";
        case FaultCode::Count:                       return "COUNT";
    }
    return "UNKNOWN";
}

DiagnosticsService::DiagnosticsService() = default;

void DiagnosticsService::begin() {
    portENTER_CRITICAL(&lock_);
    initialized_ = true;
    activeFaultCount_ = 0;
    totalLoggedEvents_ = 0;
    bufferHead_ = 0;
    bufferCount_ = 0;
    activeFaults_.reset();
    for (size_t i = 0; i < kEventHistorySize; ++i) {
        eventBuffer_[i] = FaultEvent{};
    }
    portEXIT_CRITICAL(&lock_);
}

void DiagnosticsService::end() {
    portENTER_CRITICAL(&lock_);
    initialized_ = false;
    activeFaultCount_ = 0;
    totalLoggedEvents_ = 0;
    bufferHead_ = 0;
    bufferCount_ = 0;
    activeFaults_.reset();
    for (size_t i = 0; i < kEventHistorySize; ++i) {
        eventBuffer_[i] = FaultEvent{};
    }
    portEXIT_CRITICAL(&lock_);
}

void DiagnosticsService::reportFault(FaultCode code, uint32_t timestampMs) {
    if (code == FaultCode::None || code >= FaultCode::Count) {
        return;
    }
    const size_t idx = static_cast<size_t>(code);
    const FaultSeverity severity = getFaultSeverity(code);

    portENTER_CRITICAL(&lock_);
    if (!initialized_) {
        portEXIT_CRITICAL(&lock_);
        return;
    }

    // Strict idempotency: only log transition on rising edge
    if (activeFaults_.test(idx)) {
        portEXIT_CRITICAL(&lock_);
        return;
    }

    activeFaults_.set(idx, true);
    activeFaultCount_++;

    FaultEvent evt;
    evt.timestampMs = timestampMs;
    evt.code = code;
    evt.severity = severity;
    evt.isActive = true;

    eventBuffer_[bufferHead_] = evt;
    bufferHead_ = (bufferHead_ + 1) % kEventHistorySize;
    if (bufferCount_ < kEventHistorySize) {
        bufferCount_++;
    }
    totalLoggedEvents_++;

    portEXIT_CRITICAL(&lock_);
}

void DiagnosticsService::clearFault(FaultCode code, uint32_t timestampMs) {
    if (code == FaultCode::None || code >= FaultCode::Count) {
        return;
    }
    const size_t idx = static_cast<size_t>(code);
    const FaultSeverity severity = getFaultSeverity(code);

    portENTER_CRITICAL(&lock_);
    if (!initialized_) {
        portEXIT_CRITICAL(&lock_);
        return;
    }

    // Strict idempotency: only log transition on falling edge
    if (!activeFaults_.test(idx)) {
        portEXIT_CRITICAL(&lock_);
        return;
    }

    activeFaults_.set(idx, false);
    if (activeFaultCount_ > 0) {
        activeFaultCount_--;
    }

    FaultEvent evt;
    evt.timestampMs = timestampMs;
    evt.code = code;
    evt.severity = severity;
    evt.isActive = false;

    eventBuffer_[bufferHead_] = evt;
    bufferHead_ = (bufferHead_ + 1) % kEventHistorySize;
    if (bufferCount_ < kEventHistorySize) {
        bufferCount_++;
    }
    totalLoggedEvents_++;

    portEXIT_CRITICAL(&lock_);
}

bool DiagnosticsService::hasActiveFaults() const {
    portENTER_CRITICAL(&lock_);
    const bool active = (initialized_ && activeFaultCount_ > 0);
    portEXIT_CRITICAL(&lock_);
    return active;
}

uint32_t DiagnosticsService::getActiveFaultCount() const {
    portENTER_CRITICAL(&lock_);
    const uint32_t count = initialized_ ? activeFaultCount_ : 0;
    portEXIT_CRITICAL(&lock_);
    return count;
}

bool DiagnosticsService::isFaultActive(FaultCode code) const {
    if (code == FaultCode::None || code >= FaultCode::Count) {
        return false;
    }
    const size_t idx = static_cast<size_t>(code);
    portENTER_CRITICAL(&lock_);
    const bool active = (initialized_ && activeFaults_.test(idx));
    portEXIT_CRITICAL(&lock_);
    return active;
}

FaultSeverity DiagnosticsService::getHighestActiveSeverity() const {
    portENTER_CRITICAL(&lock_);
    if (!initialized_ || activeFaultCount_ == 0) {
        portEXIT_CRITICAL(&lock_);
        return FaultSeverity::None;
    }

    FaultSeverity highest = FaultSeverity::None;
    for (size_t i = 1; i < kFaultCodeCount; ++i) {
        if (activeFaults_.test(i)) {
            const FaultSeverity sev = getFaultSeverity(static_cast<FaultCode>(i));
            if (static_cast<uint8_t>(sev) > static_cast<uint8_t>(highest)) {
                highest = sev;
            }
        }
    }
    portEXIT_CRITICAL(&lock_);
    return highest;
}

SystemHealthSnapshot DiagnosticsService::getSnapshot() const {
    return getSnapshot(0);
}

SystemHealthSnapshot DiagnosticsService::getSnapshot(uint32_t timestampMs) const {
    SystemHealthSnapshot snap;

    portENTER_CRITICAL(&lock_);
    snap.isHealthy = (initialized_ && activeFaultCount_ == 0);
    snap.activeFaultCount = initialized_ ? activeFaultCount_ : 0;
    snap.totalLoggedEvents = totalLoggedEvents_;
    snap.snapshotTimestampMs = timestampMs;
    snap.activeFaultBits = activeFaults_;

    FaultSeverity highest = FaultSeverity::None;
    if (initialized_ && activeFaultCount_ > 0) {
        for (size_t i = 1; i < kFaultCodeCount; ++i) {
            if (activeFaults_.test(i)) {
                const FaultSeverity sev = getFaultSeverity(static_cast<FaultCode>(i));
                if (static_cast<uint8_t>(sev) > static_cast<uint8_t>(highest)) {
                    highest = sev;
                }
            }
        }
    }
    snap.highestSeverity = highest;

    snap.recentEventCount = bufferCount_;
    if (bufferCount_ > 0) {
        size_t readIdx = 0;
        if (bufferCount_ == kEventHistorySize) {
            readIdx = bufferHead_;
        } else {
            readIdx = 0;
        }

        for (size_t i = 0; i < bufferCount_; ++i) {
            snap.recentEvents[i] = eventBuffer_[readIdx];
            readIdx = (readIdx + 1) % kEventHistorySize;
        }
    }
    portEXIT_CRITICAL(&lock_);

    return snap;
}

const char* DiagnosticsService::version() {
    return "1.0.0";
}

} // namespace stridecontrol
