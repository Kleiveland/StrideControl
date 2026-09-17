#include "SystemManager.h"

namespace stridecontrol {

SystemManager::SystemManager(
    ConsoleInterface& console,
    SpeedCalibration& calibration,
    DiagnosticsService& diagnostics
)
    : controlRuntime_(console, calibration, diagnostics) {}

bool SystemManager::begin(uint32_t nowMs) {
    state_ = SystemState::Initializing;
    return controlRuntime_.begin();
}

bool SystemManager::begin(ApplicationOrchestrator* orchestrator, uint32_t nowMs) {
    orchestrator_ = orchestrator;
    initialized_ = (orchestrator_ != nullptr);
    return begin(nowMs);
}

void SystemManager::end() {
    state_ = SystemState::ShuttingDown;
    controlRuntime_.end();
    initialized_ = false;
    orchestrator_ = nullptr;
}

bool SystemManager::startControlTask(ApplicationOrchestrator* orchestrator) {
    orchestrator_ = orchestrator;
    initialized_ = (orchestrator_ != nullptr);
    return controlRuntime_.startControlTask(orchestrator);
}

bool SystemManager::stopControlTask(uint32_t timeoutMs) {
    return controlRuntime_.stopControlTask(timeoutMs);
}

void SystemManager::update() {
    if (!initialized_ || orchestrator_ == nullptr) {
        state_ = SystemState::Initializing;
        return;
    }

    const ApplicationSnapshot snap = orchestrator_->getSnapshot();
    if (snap.timestampMs == 0 && snap.sequenceNumber == 0) {
        state_ = SystemState::Initializing;
        return;
    }

    // 1. Fault Evaluation (Critical or Fatal triggers Faulted state latch)
    if (snap.health.highestSeverity == FaultSeverity::Critical ||
        snap.health.highestSeverity == FaultSeverity::Fatal) {
        state_ = SystemState::Faulted;
        return;
    }

    // 2. Initializing -> Operational, once sensors are confirmed ready
    if (state_ == SystemState::Initializing) {
        if (!snap.speed.initialized || !snap.incline.initialized) {
            return; // still initializing
        }
    }

    // 3. Degraded: a non-fatal issue is present. Own health only - never derived from raw
    //    belt speed, which is CsafeMachineState's and WorkoutSessionState's responsibility.
    const bool csafeCommsIssue = snap.csafe.initialized &&
                                  (!snap.csafe.online || !snap.csafe.machineStateFresh);
    const bool degraded = controlRuntime_.isConnectionWarningActive() ||
                          snap.health.highestSeverity == FaultSeverity::Warning ||
                          csafeCommsIssue;

    state_ = degraded ? SystemState::Degraded : SystemState::Operational;
}

SystemState SystemManager::getState() const {
    return state_;
}

bool SystemManager::isFaulted() const {
    return state_ == SystemState::Faulted;
}

InclineVerificationCommandInput SystemManager::getInclineVerificationCommandInput() const {
    return controlRuntime_.getInclineVerificationCommandInput();
}

CommandExecutionStatus SystemManager::getCommandExecutionStatus() const {
    return controlRuntime_.getCommandExecutionStatus();
}

const char* SystemManager::version() {
    return "1.0.0";
}

} // namespace stridecontrol

