#include "WorkoutDispatcher.h"

namespace stridecontrol {

void WorkoutDispatcher::begin() {
    clearAllTargets();
}

void WorkoutDispatcher::clearSpeedTarget() {
    staged_.pendingSpeed = false;
    staged_.speedKmh = 0.0f;
    staged_.speedOrigin = TargetOrigin::None;
    staged_.speedSessionGeneration = 0;
    staged_.speedIntentSequence = 0;
    staged_.speedStepIndex = 0;
    staged_.speedIsPreFire = false;
    staged_.speedStagedTimestampMs = 0;
}

void WorkoutDispatcher::clearInclineTarget() {
    staged_.pendingIncline = false;
    staged_.inclinePct = 0.0f;
    staged_.inclineOrigin = TargetOrigin::None;
    staged_.inclineSessionGeneration = 0;
    staged_.inclineIntentSequence = 0;
    staged_.inclineStepIndex = 0;
    staged_.inclineIsPreFire = false;
    staged_.inclineStagedTimestampMs = 0;
}

void WorkoutDispatcher::clearWorkoutTargets() {
    if (staged_.speedOrigin == TargetOrigin::WorkoutGenerated ||
        staged_.speedOrigin == TargetOrigin::SessionManualAdjustment) {
        clearSpeedTarget();
    }
    if (staged_.inclineOrigin == TargetOrigin::WorkoutGenerated ||
        staged_.inclineOrigin == TargetOrigin::SessionManualAdjustment) {
        clearInclineTarget();
    }
}

void WorkoutDispatcher::clearForNewSession() {
    if (staged_.speedOrigin == TargetOrigin::WorkoutGenerated ||
        staged_.speedOrigin == TargetOrigin::SessionManualAdjustment ||
        staged_.speedOrigin == TargetOrigin::StandaloneManual) {
        clearSpeedTarget();
    }
    if (staged_.inclineOrigin == TargetOrigin::WorkoutGenerated ||
        staged_.inclineOrigin == TargetOrigin::SessionManualAdjustment ||
        staged_.inclineOrigin == TargetOrigin::StandaloneManual) {
        clearInclineTarget();
    }
}

void WorkoutDispatcher::clearAllTargets() {
    clearSpeedTarget();
    clearInclineTarget();
}

void WorkoutDispatcher::stageSpeedTarget(float speedKmh, const TargetContext& ctx) {
    staged_.pendingSpeed = true;
    staged_.speedKmh = speedKmh;
    staged_.speedOrigin = ctx.origin;
    staged_.speedSessionGeneration = ctx.sessionGeneration;
    staged_.speedIntentSequence = 0;
    staged_.speedStepIndex = ctx.stepIndex;
    staged_.speedIsPreFire = false;
    staged_.speedStagedTimestampMs = ctx.timestampMs;
}

void WorkoutDispatcher::stageInclineTarget(float inclinePct, const TargetContext& ctx) {
    staged_.pendingIncline = true;
    staged_.inclinePct = inclinePct;
    staged_.inclineOrigin = ctx.origin;
    staged_.inclineSessionGeneration = ctx.sessionGeneration;
    staged_.inclineIntentSequence = 0;
    staged_.inclineStepIndex = ctx.stepIndex;
    staged_.inclineIsPreFire = false;
    staged_.inclineStagedTimestampMs = ctx.timestampMs;
}

void WorkoutDispatcher::stepSpeedTarget(float deltaKmh, float currentSpeedKmh, const TargetContext& ctx) {
    if (!staged_.pendingSpeed) {
        staged_.speedKmh = currentSpeedKmh;
    }
    staged_.pendingSpeed = true;
    staged_.speedKmh += deltaKmh;
    if (staged_.speedKmh < 0.0f) staged_.speedKmh = 0.0f;
    staged_.speedOrigin = ctx.origin;
    staged_.speedSessionGeneration = ctx.sessionGeneration;
    staged_.speedIntentSequence = 0;
    staged_.speedStepIndex = ctx.stepIndex;
    staged_.speedIsPreFire = false;
    staged_.speedStagedTimestampMs = ctx.timestampMs;
}

void WorkoutDispatcher::stepInclineTarget(float deltaPct, float currentInclinePct, const TargetContext& ctx) {
    if (!staged_.pendingIncline) {
        staged_.inclinePct = currentInclinePct;
    }
    staged_.pendingIncline = true;
    staged_.inclinePct += deltaPct;
    if (staged_.inclinePct < 0.0f) staged_.inclinePct = 0.0f;
    staged_.inclineOrigin = ctx.origin;
    staged_.inclineSessionGeneration = ctx.sessionGeneration;
    staged_.inclineIntentSequence = 0;
    staged_.inclineStepIndex = ctx.stepIndex;
    staged_.inclineIsPreFire = false;
    staged_.inclineStagedTimestampMs = ctx.timestampMs;
}

void WorkoutDispatcher::stageSpeedTarget(float speedKmh, uint32_t nowMs, TargetOrigin origin) {
    TargetContext ctx;
    ctx.origin = origin;
    ctx.timestampMs = nowMs;
    stageSpeedTarget(speedKmh, ctx);
}

void WorkoutDispatcher::stageInclineTarget(float inclinePct, uint32_t nowMs, TargetOrigin origin) {
    TargetContext ctx;
    ctx.origin = origin;
    ctx.timestampMs = nowMs;
    stageInclineTarget(inclinePct, ctx);
}

void WorkoutDispatcher::stepSpeedTarget(float deltaKmh, float currentSpeedKmh, uint32_t nowMs, TargetOrigin origin) {
    TargetContext ctx;
    ctx.origin = origin;
    ctx.timestampMs = nowMs;
    stepSpeedTarget(deltaKmh, currentSpeedKmh, ctx);
}

void WorkoutDispatcher::stepInclineTarget(float deltaPct, float currentInclinePct, uint32_t nowMs, TargetOrigin origin) {
    TargetContext ctx;
    ctx.origin = origin;
    ctx.timestampMs = nowMs;
    stepInclineTarget(deltaPct, currentInclinePct, ctx);
}

void WorkoutDispatcher::validateRetainedTargets(const WorkoutSession& session, uint32_t nowMs) {
    (void)nowMs;
    const bool sessionActive = session.isActive();
    const uint32_t currentGeneration = session.getSessionGeneration();
    const uint8_t currentStep = session.getCurrentStepIndex();
    const bool preFireActive = session.isPreFireActive();
    const uint8_t preFireTargetStep = session.getPreFireTargetStepIndex();

    // 1. Validate staged speed target
    if (staged_.pendingSpeed) {
        bool valid = true;
        if (staged_.speedOrigin == TargetOrigin::WorkoutGenerated) {
            if (!sessionActive || staged_.speedSessionGeneration != currentGeneration) {
                valid = false;
            } else if (staged_.speedIsPreFire) {
                if (currentStep == staged_.speedStepIndex) {
                    valid = true;
                } else if (preFireActive && preFireTargetStep != UINT8_MAX && staged_.speedStepIndex == preFireTargetStep) {
                    valid = true;
                } else {
                    valid = false;
                }
            } else {
                if (staged_.speedStepIndex != currentStep) {
                    valid = false;
                }
            }
        } else if (staged_.speedOrigin == TargetOrigin::SessionManualAdjustment) {
            if (!sessionActive || staged_.speedSessionGeneration != currentGeneration) {
                valid = false;
            }
        } else if (staged_.speedOrigin == TargetOrigin::StandaloneManual) {
            if (sessionActive) {
                valid = false;
            }
        }

        if (!valid) {
            clearSpeedTarget();
        }
    }

    // 2. Validate staged incline target
    if (staged_.pendingIncline) {
        bool valid = true;
        if (staged_.inclineOrigin == TargetOrigin::WorkoutGenerated) {
            if (!sessionActive || staged_.inclineSessionGeneration != currentGeneration) {
                valid = false;
            } else if (staged_.inclineIsPreFire) {
                if (currentStep == staged_.inclineStepIndex) {
                    valid = true;
                } else if (preFireActive && preFireTargetStep != UINT8_MAX && staged_.inclineStepIndex == preFireTargetStep) {
                    valid = true;
                } else {
                    valid = false;
                }
            } else {
                if (staged_.inclineStepIndex != currentStep) {
                    valid = false;
                }
            }
        } else if (staged_.inclineOrigin == TargetOrigin::SessionManualAdjustment) {
            if (!sessionActive || staged_.inclineSessionGeneration != currentGeneration) {
                valid = false;
            }
        } else if (staged_.inclineOrigin == TargetOrigin::StandaloneManual) {
            if (sessionActive) {
                valid = false;
            }
        }

        if (!valid) {
            clearInclineTarget();
        }
    }
}

void WorkoutDispatcher::update(
    WorkoutSession& session,
    IWorkoutTargetSink& treadmill,
    uint32_t nowMs
) {
    // 1. INGEST NEWEST DOMAIN INTENT FIRST
    const WorkoutCommandIntent intent = session.getPendingCommandIntent();

    // 2. FIELD-WISE PREEMPTION WITH EXPLICIT PROVENANCE
    bool intentHadAssertedTargets = false;

    if (intent.hasSpeedTarget) {
        if (!staged_.pendingSpeed ||
            intent.sessionGeneration != staged_.speedSessionGeneration ||
            intent.intentSequence >= staged_.speedIntentSequence) {
            staged_.pendingSpeed = true;
            staged_.speedKmh = intent.targetSpeedKmh;
            staged_.speedOrigin = TargetOrigin::WorkoutGenerated;
            staged_.speedSessionGeneration = intent.sessionGeneration;
            staged_.speedIntentSequence = intent.intentSequence;
            staged_.speedStepIndex = intent.stepIndex;
            staged_.speedIsPreFire = intent.isPreFire;
            staged_.speedStagedTimestampMs = (intent.timestampMs != 0) ? intent.timestampMs : nowMs;
            intentHadAssertedTargets = true;
        }
    }

    if (intent.hasInclineTarget) {
        if (!staged_.pendingIncline ||
            intent.sessionGeneration != staged_.inclineSessionGeneration ||
            intent.intentSequence >= staged_.inclineIntentSequence) {
            staged_.pendingIncline = true;
            staged_.inclinePct = static_cast<float>(intent.targetInclinePct);
            staged_.inclineOrigin = TargetOrigin::WorkoutGenerated;
            staged_.inclineSessionGeneration = intent.sessionGeneration;
            staged_.inclineIntentSequence = intent.intentSequence;
            staged_.inclineStepIndex = intent.stepIndex;
            staged_.inclineIsPreFire = intent.isPreFire;
            staged_.inclineStagedTimestampMs = (intent.timestampMs != 0) ? intent.timestampMs : nowMs;
            intentHadAssertedTargets = true;
        }
    }

    // 3. CLEAR ONLY AFTER SAFE STAGING
    if (intentHadAssertedTargets) {
        session.clearPendingCommandIntent();
    }

    // 4. VALIDATE RETAINED TARGETS AGAINST ACTIVE SESSION STATE
    validateRetainedTargets(session, nowMs);

    // 5. READINESS
    if (!treadmill.isReady()) {
        return;
    }

    // 6. BUSY SERIALIZATION
    if (treadmill.isBusy()) {
        return;
    }

    // 7. INCLINE DELIVERY
    if (staged_.pendingIncline) {
        const bool delivered = treadmill.submitInclineTarget(staged_.inclinePct, nowMs);
        if (delivered) {
            clearInclineTarget();
        }
        return;
    }

    // 8. SPEED DELIVERY (Only when pendingIncline is false)
    if (staged_.pendingSpeed) {
        const bool delivered = treadmill.submitSpeedTarget(staged_.speedKmh, nowMs);
        if (delivered) {
            clearSpeedTarget();
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
    return "WorkoutDispatcher/2.0.0";
}

} // namespace stridecontrol

