#include "WorkoutDispatcher.h"

namespace stridecontrol {

void WorkoutDispatcher::begin() {
    staged_ = StagedTargets{};
}

void WorkoutDispatcher::update(
    WorkoutSession& session,
    IWorkoutTargetSink& treadmill,
    uint32_t nowMs
) {
    // 1. INGEST NEWEST DOMAIN INTENT FIRST
    const WorkoutCommandIntent intent = session.getPendingCommandIntent();

    // 2. FIELD-WISE PREEMPTION
    bool intentHadAssertedTargets = false;

    if (intent.hasSpeedTarget) {
        staged_.pendingSpeed = true;
        staged_.speedKmh = intent.targetSpeedKmh;
        intentHadAssertedTargets = true;
    }

    if (intent.hasInclineTarget) {
        staged_.pendingIncline = true;
        staged_.inclinePct = static_cast<float>(intent.targetInclinePct);
        intentHadAssertedTargets = true;
    }

    // 3. CLEAR ONLY AFTER SAFE STAGING
    if (intentHadAssertedTargets) {
        session.clearPendingCommandIntent();
    }

    // 4. READINESS
    if (!treadmill.isReady()) {
        return;
    }

    // 5. BUSY SERIALIZATION
    if (treadmill.isBusy()) {
        return;
    }

    // 6. INCLINE DELIVERY
    if (staged_.pendingIncline) {
        const bool delivered = treadmill.submitInclineTarget(staged_.inclinePct, nowMs);
        if (delivered) {
            staged_.pendingIncline = false;
        }
        return;
    }

    // 7. SPEED DELIVERY (Only when pendingIncline is false)
    if (staged_.pendingSpeed) {
        const bool delivered = treadmill.submitSpeedTarget(staged_.speedKmh, nowMs);
        if (delivered) {
            staged_.pendingSpeed = false;
        }
        return;
    }
}

bool WorkoutDispatcher::hasPendingTargets() const {
    return staged_.pendingSpeed || staged_.pendingIncline;
}

StagedTargets WorkoutDispatcher::getStagedTargets() const {
    return staged_;
}

const char* WorkoutDispatcher::version() {
    return "WorkoutDispatcher/1.0.0";
}

} // namespace stridecontrol

