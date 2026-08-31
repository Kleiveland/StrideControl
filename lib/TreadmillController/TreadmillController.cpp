#include "TreadmillController.h"
#include <cmath>

namespace stridecontrol {

TreadmillController::TreadmillController(
    ConsoleInterface& console,
    SpeedCalibration& calibration,
    DiagnosticsService& diagnostics
)
    : console_(console),
      calibration_(calibration),
      diagnostics_(diagnostics) {}

bool TreadmillController::begin() {
    if (initialized_) {
        return true;
    }

    if (!console_.isReady()) {
        snapshot_ = TreadmillControllerSnapshot{};
        snapshot_.state = TreadmillControllerState::Faulted;
        snapshot_.ready = false;
        snapshot_.busy = false;
        if (!consoleFaultReported_) {
            diagnostics_.reportFault(FaultCode::ConsoleCommandError, 0);
            consoleFaultReported_ = true;
        }
        return false;
    }

    initialized_ = true;
    snapshot_ = TreadmillControllerSnapshot{};
    snapshot_.state = TreadmillControllerState::Idle;
    snapshot_.ready = true;
    snapshot_.busy = false;

    if (consoleFaultReported_) {
        diagnostics_.clearFault(FaultCode::ConsoleCommandError, 0);
        consoleFaultReported_ = false;
    }

    return true;
}

void TreadmillController::end() {
    if (initialized_ && snapshot_.busy) {
        console_.abortActiveCommand();
    }

    initialized_ = false;
    if (consoleFaultReported_) {
        diagnostics_.clearFault(FaultCode::ConsoleCommandError, 0);
        consoleFaultReported_ = false;
    }

    snapshot_ = TreadmillControllerSnapshot{};
    snapshot_.state = TreadmillControllerState::Uninitialized;
    snapshot_.ready = false;
    snapshot_.busy = false;
}

bool TreadmillController::submitSpeedTarget(
    float desiredPhysicalSpeedKmh,
    uint32_t requestTimestampMs
) {
    if (!initialized_ || snapshot_.state == TreadmillControllerState::Faulted) {
        return false;
    }

    if (snapshot_.busy) {
        return false;
    }

    if (!console_.isReady()) {
        snapshot_.state = TreadmillControllerState::Faulted;
        snapshot_.ready = false;
        snapshot_.busy = false;
        if (!consoleFaultReported_) {
            diagnostics_.reportFault(FaultCode::ConsoleCommandError, requestTimestampMs);
            consoleFaultReported_ = true;
        }
        return false;
    }

    if (!std::isfinite(desiredPhysicalSpeedKmh)) {
        return false;
    }

    if (desiredPhysicalSpeedKmh <= 0.0f) {
        return submitStop(requestTimestampMs);
    }

    // Apply maximum achievable speed ceiling
    const float maxSpeed = calibration_.getMaxAchievableSpeedKmh();
    float targetSpeed = desiredPhysicalSpeedKmh;
    if (targetSpeed > maxSpeed) {
        targetSpeed = maxSpeed;
    }

    const SpeedCalibrationResult calResult = calibration_.calculateCommand(targetSpeed);

    if (calResult.status == SpeedCalibrationResultStatus::InvalidInput) {
        return false;
    }

    if (calResult.status == SpeedCalibrationResultStatus::Stopped) {
        return submitStop(requestTimestampMs);
    }

    uint32_t reqId = ++nextRequestId_;
    if (reqId == 0) {
        reqId = ++nextRequestId_;
    }

    TreadmillCommand cmd{};
    cmd.requestId = reqId;
    cmd.type = CommandType::SetSpeed;
    cmd.value = calResult.treadmillCommandKmh;
    cmd.button = ButtonId::Unknown;

    if (!console_.submit(cmd, 0)) {
        return false;
    }

    snapshot_.acceptedPhysicalSpeedTargetKmh = targetSpeed;
    snapshot_.physicalSpeedTargetValid = true;
    snapshot_.correctedConsoleSpeedKmh = calResult.treadmillCommandKmh;
    snapshot_.correctedConsoleSpeedValid = true;
    snapshot_.lastCalibrationStatus = calResult.status;

    snapshot_.activeRequestId = reqId;
    snapshot_.activeRequestValid = true;
    snapshot_.activeCommandType = CommandType::SetSpeed;

    snapshot_.state = TreadmillControllerState::WaitingForResult;
    snapshot_.busy = true;
    snapshot_.lastUpdateTimestampMs = requestTimestampMs;

    return true;
}

bool TreadmillController::submitInclineTarget(
    float targetInclinePct,
    uint32_t requestTimestampMs
) {
    if (!initialized_ || snapshot_.state == TreadmillControllerState::Faulted) {
        return false;
    }

    if (snapshot_.busy) {
        return false;
    }

    if (!console_.isReady()) {
        snapshot_.state = TreadmillControllerState::Faulted;
        snapshot_.ready = false;
        snapshot_.busy = false;
        if (!consoleFaultReported_) {
            diagnostics_.reportFault(FaultCode::ConsoleCommandError, requestTimestampMs);
            consoleFaultReported_ = true;
        }
        return false;
    }

    if (!std::isfinite(targetInclinePct)) {
        return false;
    }

    // Strict whole-percentage incline rule [0.0f, 15.0f]
    if (targetInclinePct < 0.0f || targetInclinePct > 15.0f || std::floor(targetInclinePct) != targetInclinePct) {
        return false;
    }

    uint32_t reqId = ++nextRequestId_;
    if (reqId == 0) {
        reqId = ++nextRequestId_;
    }

    TreadmillCommand cmd{};
    cmd.requestId = reqId;
    cmd.type = CommandType::SetIncline;
    cmd.value = targetInclinePct;
    cmd.button = ButtonId::Unknown;

    if (!console_.submit(cmd, 0)) {
        return false;
    }

    snapshot_.acceptedInclineTargetPct = targetInclinePct;
    snapshot_.inclineTargetValid = true;

    snapshot_.activeRequestId = reqId;
    snapshot_.activeRequestValid = true;
    snapshot_.activeCommandType = CommandType::SetIncline;

    snapshot_.state = TreadmillControllerState::WaitingForResult;
    snapshot_.busy = true;
    snapshot_.lastUpdateTimestampMs = requestTimestampMs;

    return true;
}

bool TreadmillController::submitStop(uint32_t requestTimestampMs) {
    if (!initialized_ || snapshot_.state == TreadmillControllerState::Faulted) {
        return false;
    }

    if (!console_.isReady()) {
        snapshot_.state = TreadmillControllerState::Faulted;
        snapshot_.ready = false;
        snapshot_.busy = false;
        if (!consoleFaultReported_) {
            diagnostics_.reportFault(FaultCode::ConsoleCommandError, requestTimestampMs);
            consoleFaultReported_ = true;
        }
        return false;
    }

    // Stop has explicit priority: abort any active in-flight macro before submission
    if (snapshot_.busy || console_.isActive()) {
        console_.abortActiveCommand();
    }

    uint32_t reqId = ++nextRequestId_;
    if (reqId == 0) {
        reqId = ++nextRequestId_;
    }

    TreadmillCommand cmd{};
    cmd.requestId = reqId;
    cmd.type = CommandType::PressButton;
    cmd.value = 0.0f;
    cmd.button = ButtonId::Stop;

    if (!console_.submit(cmd, 0)) {
        return false;
    }

    snapshot_.acceptedPhysicalSpeedTargetKmh = 0.0f;
    snapshot_.physicalSpeedTargetValid = true;
    snapshot_.correctedConsoleSpeedKmh = 0.0f;
    snapshot_.correctedConsoleSpeedValid = true;
    snapshot_.lastCalibrationStatus = SpeedCalibrationResultStatus::Stopped;

    snapshot_.activeRequestId = reqId;
    snapshot_.activeRequestValid = true;
    snapshot_.activeCommandType = CommandType::PressButton;

    snapshot_.state = TreadmillControllerState::WaitingForResult;
    snapshot_.busy = true;
    snapshot_.lastUpdateTimestampMs = requestTimestampMs;

    return true;
}

void TreadmillController::update(uint32_t nowMs) {
    if (!initialized_) {
        return;
    }

    snapshot_.lastUpdateTimestampMs = nowMs;

    if (!console_.isReady()) {
        if (snapshot_.state != TreadmillControllerState::Faulted) {
            snapshot_.state = TreadmillControllerState::Faulted;
            snapshot_.ready = false;
            snapshot_.busy = false;
            snapshot_.activeRequestValid = false;
            if (!consoleFaultReported_) {
                diagnostics_.reportFault(FaultCode::ConsoleCommandError, nowMs);
                consoleFaultReported_ = true;
            }
        }
        return;
    } else if (snapshot_.state == TreadmillControllerState::Faulted) {
        // Automatic recovery on healthy readiness
        snapshot_.state = TreadmillControllerState::Idle;
        snapshot_.ready = true;
        if (consoleFaultReported_) {
            diagnostics_.clearFault(FaultCode::ConsoleCommandError, nowMs);
            consoleFaultReported_ = false;
        }
    }

    CommandEvent event{};
    while (console_.receiveCommandEvent(event, 0)) {
        // Correlate against active request ID
        if (snapshot_.activeRequestValid && event.requestId == snapshot_.activeRequestId) {
            switch (event.status) {
                case CommandStatus::Completed:
                    snapshot_.latestOutcome = event.outcome;
                    snapshot_.latestOutcomeValid = true;
                    snapshot_.latestCommandStatus = event.status;
                    snapshot_.latestCommandStatusValid = true;
                    snapshot_.state = TreadmillControllerState::Completed;
                    snapshot_.busy = false;
                    snapshot_.activeRequestValid = false;
                    break;

                case CommandStatus::Failed:
                case CommandStatus::Rejected:
                case CommandStatus::Aborted:
                    snapshot_.latestOutcome = event.outcome;
                    snapshot_.latestOutcomeValid = true;
                    snapshot_.latestCommandStatus = event.status;
                    snapshot_.latestCommandStatusValid = true;
                    snapshot_.state = TreadmillControllerState::CommandFailed;
                    snapshot_.busy = false;
                    snapshot_.activeRequestValid = false;

                    if (event.outcome == ConsoleOutcome::WatchdogTimeout ||
                        event.outcome == ConsoleOutcome::PhaseTimeout ||
                        event.outcome == ConsoleOutcome::IdleTimeout) {
                        diagnostics_.reportFault(FaultCode::ConsoleTimeout, nowMs);
                    } else if (event.outcome == ConsoleOutcome::HardwareError ||
                               event.outcome == ConsoleOutcome::SyncFailure) {
                        diagnostics_.reportFault(FaultCode::ConsoleCommandError, nowMs);
                    }
                    break;

                case CommandStatus::Queued:
                case CommandStatus::Started:
                case CommandStatus::StepStarted:
                case CommandStatus::StepConfirmed:
                case CommandStatus::Retrying:
                case CommandStatus::RecoveryRequired:
                    snapshot_.state = TreadmillControllerState::WaitingForResult;
                    snapshot_.busy = true;
                    break;
            }
        }
    }
}

TreadmillControllerSnapshot TreadmillController::getSnapshot() const {
    return snapshot_;
}

bool TreadmillController::isReady() const {
    return initialized_ && snapshot_.ready && snapshot_.state != TreadmillControllerState::Faulted && console_.isReady();
}

bool TreadmillController::isBusy() const {
    return snapshot_.busy;
}

const char* TreadmillController::version() {
    return "TreadmillController/1.1.0";
}

} // namespace stridecontrol
