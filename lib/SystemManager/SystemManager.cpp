#include "SystemManager.h"

namespace stridecontrol {

SystemManager::SystemManager() = default;

void SystemManager::begin(ApplicationOrchestrator* orchestrator) {
    orchestrator_ = orchestrator;
    state_ = SystemState::Initializing;
    initialized_ = (orchestrator_ != nullptr);
    previousSpeedKmh_ = 0.0f;
}

void SystemManager::update() {
    if (!initialized_ || orchestrator_ == nullptr) {
        state_ = SystemState::Initializing;
        return;
    }

    ApplicationSnapshot snap = orchestrator_->getSnapshot();
    if (snap.timestampMs == 0 && snap.sequenceNumber == 0) {
        state_ = SystemState::Initializing;
        return;
    }

    // 1. Fault Evaluation (Critical or Fatal triggers Faulted state latch)
    if (snap.health.highestSeverity == FaultSeverity::Critical ||
        snap.health.highestSeverity == FaultSeverity::Fatal) {
        state_ = SystemState::Faulted;
        previousSpeedKmh_ = snap.speed.speedKmh;
        return;
    }

    // Recovering from Faulted state
    if (state_ == SystemState::Faulted) {
        if (snap.speed.speedKmh > 0.1f) {
            state_ = SystemState::Stopping;
        } else {
            state_ = SystemState::Ready;
        }
    }

    // 2. Operational State Machine Transitions
    const float currentSpeed = snap.speed.speedKmh;

    switch (state_) {
        case SystemState::Initializing:
            if (snap.speed.initialized && snap.incline.initialized) {
                state_ = (currentSpeed > 0.1f) ? SystemState::Running : SystemState::Ready;
            }
            break;

        case SystemState::Ready:
            if (currentSpeed > 0.1f) {
                state_ = SystemState::Starting;
            }
            break;

        case SystemState::Starting:
            if (currentSpeed >= 0.5f) {
                state_ = SystemState::Running;
            } else if (currentSpeed <= 0.05f) {
                state_ = SystemState::Ready;
            }
            break;

        case SystemState::Running:
            if (currentSpeed < previousSpeedKmh_ && currentSpeed < 0.5f) {
                state_ = SystemState::Stopping;
            } else if (currentSpeed <= 0.05f) {
                state_ = SystemState::Ready;
            }
            break;

        case SystemState::Stopping:
            if (currentSpeed <= 0.05f) {
                state_ = SystemState::Ready;
            } else if (currentSpeed > previousSpeedKmh_ && currentSpeed >= 0.5f) {
                state_ = SystemState::Running;
            }
            break;

        case SystemState::Faulted:
            // Latch handled above
            break;

        case SystemState::SafetyStop:
            // Contract gap: ApplicationSnapshot contains no physical safety-stop hardware line.
            break;
    }

    previousSpeedKmh_ = currentSpeed;
}

SystemState SystemManager::getState() const {
    return state_;
}

bool SystemManager::isFaulted() const {
    return state_ == SystemState::Faulted;
}

bool SystemManager::isSafetyStop() const {
    return state_ == SystemState::SafetyStop;
}

const char* SystemManager::version() {
    return "1.0.0";
}

} // namespace stridecontrol

