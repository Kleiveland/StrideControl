#include "WorkoutSession.h"
#include <cmath>
#include <algorithm>

namespace stridecontrol {

const char* workoutSessionStateName(WorkoutSessionState state) {
    switch (state) {
        case WorkoutSessionState::Uninitialized: return "Uninitialized";
        case WorkoutSessionState::Idle: return "Idle";
        case WorkoutSessionState::Armed: return "Armed";
        case WorkoutSessionState::Running: return "Running";
        case WorkoutSessionState::Suspended: return "Suspended";
        case WorkoutSessionState::CompletionPending: return "CompletionPending";
        case WorkoutSessionState::Completed: return "Completed";
        case WorkoutSessionState::Aborted: return "Aborted";
        default: return "Unknown";
    }
}

bool WorkoutSession::begin(const WorkoutSessionConfig& config) {
    config_ = config;
    initialized_ = true;
    workout_ = nullptr;

    snapshot_ = WorkoutSessionSnapshot{};
    snapshot_.state = WorkoutSessionState::Idle;
    snapshot_.initialized = true;

    pendingIntent_ = WorkoutCommandIntent{};

    lastUpdateTimestampMs_ = 0;
    totalElapsedTimeMs_ = 0;
    activeRunningTimeMs_ = 0;
    totalValidatedDistanceKm_ = 0.0;
    stepElapsedMs_ = 0;
    stepElapsedValidatedDistanceKm_ = 0.0;
    runtimeStepTargetDurationMs_ = 0;
    distanceAtStepEntryKm_ = 0.0;
    lastRunnerDistanceKm_ = -1.0;
    partialDragCount_ = 0;
    isPartialDragCurrent_ = false;
    isRestExtendedCurrent_ = false;
    restExtensionSecondsTotal_ = 0;

    physicalStopCount_ = 0;
    continuationWindowExpiresMs_ = 0;
    isEmergencyStopped_ = false;
    eStopRestartPending_ = false;

    pendingShiftPrompt_ = false;
    netWorkSpeedDeltaKmh_ = 0.0f;
    speedAdjustmentShiftAppliedKmh_ = 0.0f;
    speedAdjustmentPromptExpiresMs_ = 0;

    acknowledgedHasSpeed_ = false;
    acknowledgedSpeedTargetKmh_ = 0.0f;
    acknowledgedHasIncline_ = false;
    acknowledgedInclineTargetPct_ = 0;
    restartReissuePending_ = false;
    lowSpeedDebounceActive_ = false;
    lowSpeedStartMs_ = 0;
    distanceOvershootCarryKm_ = 0.0;

    return true;
}

void WorkoutSession::end() {
    initialized_ = false;
    workout_ = nullptr;
    snapshot_ = WorkoutSessionSnapshot{};
    snapshot_.state = WorkoutSessionState::Uninitialized;
    pendingIntent_ = WorkoutCommandIntent{};
}

bool WorkoutSession::armWorkout(const ExpandedWorkout* workout, uint32_t nowMs) {
    if (!initialized_ || workout == nullptr || workout->totalSteps == 0) {
        return false;
    }

    workout_ = workout;

    snapshot_ = WorkoutSessionSnapshot{};
    snapshot_.state = WorkoutSessionState::Armed;
    snapshot_.initialized = true;
    snapshot_.active = true;
    snapshot_.suspended = false;
    snapshot_.completionPending = false;
    snapshot_.workoutId = workout->workoutId;
    snapshot_.currentStepIndex = 0;
    snapshot_.totalStepCount = workout->totalSteps;
    snapshot_.currentStep = workout->steps[0];
    snapshot_.currentRole = workout->steps[0].role;
    snapshot_.currentRep = workout->steps[0].repNumber;
    snapshot_.totalRepsInGroup = workout->steps[0].totalRepsInGroup;

    totalElapsedTimeMs_ = 0;
    activeRunningTimeMs_ = 0;
    totalValidatedDistanceKm_ = 0.0;
    stepElapsedMs_ = 0;
    stepElapsedValidatedDistanceKm_ = 0.0;
    distanceAtStepEntryKm_ = 0.0;
    lastRunnerDistanceKm_ = -1.0;
    partialDragCount_ = 0;
    isPartialDragCurrent_ = false;
    isRestExtendedCurrent_ = false;
    restExtensionSecondsTotal_ = 0;

    physicalStopCount_ = 0;
    continuationWindowExpiresMs_ = 0;
    isEmergencyStopped_ = false;
    eStopRestartPending_ = false;

    pendingShiftPrompt_ = false;
    netWorkSpeedDeltaKmh_ = 0.0f;
    speedAdjustmentShiftAppliedKmh_ = 0.0f;
    speedAdjustmentPromptExpiresMs_ = 0;

    acknowledgedHasSpeed_ = false;
    acknowledgedSpeedTargetKmh_ = 0.0f;
    acknowledgedHasIncline_ = false;
    acknowledgedInclineTargetPct_ = 0;
    restartReissuePending_ = false;
    lowSpeedDebounceActive_ = false;
    lowSpeedStartMs_ = 0;
    distanceOvershootCarryKm_ = 0.0;

    snapshot_.totalElapsedTimeMs = 0;
    snapshot_.activeRunningTimeMs = 0;
    snapshot_.totalValidatedDistanceKm = 0.0;
    snapshot_.stepElapsedMs = 0;
    snapshot_.stepElapsedValidatedDistanceKm = 0.0;
    snapshot_.stepProgressFraction = 0.0f;
    snapshot_.stepRemainingFraction = 1.0f;

    snapshot_.snapshotTimestampMs = nowMs;
    lastUpdateTimestampMs_ = nowMs;

    pendingIntent_ = WorkoutCommandIntent{};

    return true;
}

void WorkoutSession::startStep(uint8_t stepIndex, uint32_t nowMs, double currentRunnerDistanceKm) {
    if (workout_ == nullptr || stepIndex >= workout_->totalSteps) {
        snapshot_.state = WorkoutSessionState::Completed;
        snapshot_.active = false;
        snapshot_.completionPending = false;
        snapshot_.suspended = false;
        clearPendingCommandIntent();
        return;
    }

    // Force prompt closure if active when exiting REST
    if (snapshot_.speedAdjustmentPromptActive) {
        rejectSpeedAdjustmentShift();
    }

    snapshot_.currentStepIndex = stepIndex;
    snapshot_.currentStep = workout_->steps[stepIndex];
    snapshot_.currentRole = workout_->steps[stepIndex].role;
    snapshot_.currentRep = workout_->steps[stepIndex].repNumber;
    snapshot_.totalRepsInGroup = workout_->steps[stepIndex].totalRepsInGroup;

    stepStartTimestampMs_ = nowMs;
    stepElapsedMs_ = 0;
    isPartialDragCurrent_ = false;
    isRestExtendedCurrent_ = false;

    // Check speed adjustment prompt on entering a REST step
    if (snapshot_.currentRole == StepRole::REST && pendingShiftPrompt_) {
        snapshot_.speedAdjustmentPromptActive = true;
        snapshot_.suggestedSpeedDeltaKmh = netWorkSpeedDeltaKmh_;
        snapshot_.speedAdjustmentPromptExpiresMs = nowMs + config_.speedAdjustmentPromptDurationMs;
        speedAdjustmentPromptExpiresMs_ = snapshot_.speedAdjustmentPromptExpiresMs;
    } else if (snapshot_.currentRole != StepRole::REST) {
        snapshot_.speedAdjustmentPromptActive = false;
    }

    if (snapshot_.currentStep.durationType == DurationType::TIME_SECONDS) {
        runtimeStepTargetDurationMs_ = snapshot_.currentStep.durationValue * 1000;
        snapshot_.stepRemainingMs = runtimeStepTargetDurationMs_;
        snapshot_.stepRemainingValidatedDistanceKm = 0.0;
        distanceAtStepEntryKm_ = currentRunnerDistanceKm;
        snapshot_.stepElapsedValidatedDistanceKm = 0.0;
        snapshot_.stepElapsedMs = 0;
        snapshot_.stepProgressFraction = 0.0f;
        snapshot_.stepRemainingFraction = 1.0f;
    } else {
        runtimeStepTargetDurationMs_ = 0;
        snapshot_.stepRemainingMs = 0;
        double targetKm = static_cast<double>(snapshot_.currentStep.durationValue) / 1000.0;
        distanceAtStepEntryKm_ = currentRunnerDistanceKm - distanceOvershootCarryKm_;
        distanceOvershootCarryKm_ = 0.0;
        double initialElapsed = (currentRunnerDistanceKm >= distanceAtStepEntryKm_)
                                    ? (currentRunnerDistanceKm - distanceAtStepEntryKm_)
                                    : 0.0;
        snapshot_.stepElapsedValidatedDistanceKm = initialElapsed;
        snapshot_.stepRemainingValidatedDistanceKm = (initialElapsed < targetKm) ? (targetKm - initialElapsed) : 0.0;
        snapshot_.stepElapsedMs = 0;
        snapshot_.stepProgressFraction = (targetKm > 0.0) ? static_cast<float>(initialElapsed / targetKm) : 0.0f;
        if (snapshot_.stepProgressFraction > 1.0f) snapshot_.stepProgressFraction = 1.0f;
        snapshot_.stepRemainingFraction = 1.0f - snapshot_.stepProgressFraction;
    }

    emitStepCommandIntent(snapshot_.currentStep, false);
}

void WorkoutSession::emitStepCommandIntent(const ExpandedStep& step, bool forceReissue) {
    bool shouldHaveSpeed = (step.speedMode == SpeedMode::FIXED);
    float effectiveSpeed = 0.0f;
    if (shouldHaveSpeed) {
        effectiveSpeed = step.targetSpeedKmh;
        if (step.role == StepRole::WORK) {
            effectiveSpeed += speedAdjustmentShiftAppliedKmh_;
            if (effectiveSpeed < 0.5f) effectiveSpeed = 0.5f;
            if (effectiveSpeed > 25.0f) effectiveSpeed = 25.0f;
        }
    }

    if (forceReissue || (shouldHaveSpeed != acknowledgedHasSpeed_) ||
        (shouldHaveSpeed && std::abs(effectiveSpeed - acknowledgedSpeedTargetKmh_) >= 0.01f)) {
        pendingIntent_.hasSpeedTarget = shouldHaveSpeed;
        pendingIntent_.targetSpeedKmh = shouldHaveSpeed ? effectiveSpeed : 0.0f;
        acknowledgedHasSpeed_ = shouldHaveSpeed;
        acknowledgedSpeedTargetKmh_ = effectiveSpeed;
    }

    bool shouldHaveIncline = step.setIncline;
    uint8_t effectiveIncline = shouldHaveIncline ? step.targetInclinePct : 0;

    if (forceReissue || (shouldHaveIncline != acknowledgedHasIncline_) ||
        (shouldHaveIncline && effectiveIncline != acknowledgedInclineTargetPct_)) {
        pendingIntent_.hasInclineTarget = shouldHaveIncline;
        pendingIntent_.targetInclinePct = effectiveIncline;
        acknowledgedHasIncline_ = shouldHaveIncline;
        acknowledgedInclineTargetPct_ = effectiveIncline;
    }

    snapshot_.hasSpeedTarget = shouldHaveSpeed;
    snapshot_.targetSpeedKmh = shouldHaveSpeed ? effectiveSpeed : 0.0f;
    snapshot_.hasInclineTarget = shouldHaveIncline;
    snapshot_.targetInclinePct = effectiveIncline;
}

void WorkoutSession::advanceStep(uint32_t nowMs, double currentRunnerDistanceKm) {
    if (workout_ == nullptr) return;

    // Force prompt closure if active when exiting step
    if (snapshot_.speedAdjustmentPromptActive) {
        rejectSpeedAdjustmentShift();
    }

    if (snapshot_.currentStepIndex + 1 >= workout_->totalSteps) {
        if (snapshot_.currentStep.role == StepRole::COOLDOWN) {
            // Nedjogg completion: enter CompletionPending, do NOT stop or auto-complete
            snapshot_.state = WorkoutSessionState::CompletionPending;
            snapshot_.completionPending = true;
            snapshot_.stepRemainingMs = 0;
            snapshot_.stepRemainingValidatedDistanceKm = 0.0;
            snapshot_.stepProgressFraction = 1.0f;
            snapshot_.stepRemainingFraction = 0.0f;
            return;
        } else {
            snapshot_.state = WorkoutSessionState::Completed;
            snapshot_.active = false;
            snapshot_.completionPending = false;
            snapshot_.suspended = false;
            clearPendingCommandIntent();
            return;
        }
    }

    startStep(snapshot_.currentStepIndex + 1, nowMs, currentRunnerDistanceKm);
}

void WorkoutSession::registerPhysicalStop(uint32_t nowMs) {
    physicalStopCount_++;
    snapshot_.physicalStopCount = physicalStopCount_;

    if (physicalStopCount_ == 1) {
        // 1x Stop: Suspends session (Pause)
        suspend(nowMs);
        continuationWindowExpiresMs_ = 0;
        snapshot_.continuationWindowActive = false;
    } else if (physicalStopCount_ == 2) {
        // 2x Stop: Resets machine targets, preserves 10-second continuation window
        suspend(nowMs);
        clearPendingCommandIntent();
        restartReissuePending_ = true;
        continuationWindowExpiresMs_ = nowMs + config_.continuationWindowDurationMs;
        snapshot_.continuationWindowActive = true;
        snapshot_.continuationWindowRemainingMs = config_.continuationWindowDurationMs;
    } else if (physicalStopCount_ >= 3) {
        // 3x Stop: Finalizes workout and triggers summary
        finalizeSession(nowMs);
    }
}

void WorkoutSession::registerEmergencyStop(uint32_t nowMs) {
    isEmergencyStopped_ = true;
    eStopRestartPending_ = true;
    restartReissuePending_ = true;
    snapshot_.isEmergencyStopped = true;
    suspend(nowMs);
    clearPendingCommandIntent();
}

void WorkoutSession::registerEmergencyStopCleared() {
    isEmergencyStopped_ = false;
    snapshot_.isEmergencyStopped = false;
}

void WorkoutSession::reportWorkSpeedAdjustment(float actualSpeedKmh) {
    if (snapshot_.state == WorkoutSessionState::Running && snapshot_.currentStep.role == StepRole::WORK) {
        float plannedSpeed = snapshot_.currentStep.targetSpeedKmh + speedAdjustmentShiftAppliedKmh_;
        netWorkSpeedDeltaKmh_ = actualSpeedKmh - plannedSpeed;
        if (std::abs(netWorkSpeedDeltaKmh_) >= 0.05f) {
            pendingShiftPrompt_ = true;
        }
    }
}

void WorkoutSession::acceptSpeedAdjustmentShift() {
    if (!snapshot_.speedAdjustmentPromptActive) {
        return;
    }
    speedAdjustmentShiftAppliedKmh_ += netWorkSpeedDeltaKmh_;
    snapshot_.appliedWorkSpeedShiftKmh = speedAdjustmentShiftAppliedKmh_;
    snapshot_.speedAdjustmentPromptActive = false;
    pendingShiftPrompt_ = false;
    netWorkSpeedDeltaKmh_ = 0.0f;
}

void WorkoutSession::rejectSpeedAdjustmentShift() {
    snapshot_.speedAdjustmentPromptActive = false;
    pendingShiftPrompt_ = false;
    netWorkSpeedDeltaKmh_ = 0.0f;
}

void WorkoutSession::update(
    const ApplicationSnapshot& applicationSnapshot,
    uint32_t nowMs
) {
    if (!initialized_ || workout_ == nullptr) {
        return;
    }

    // Check continuation-window timeout
    if (snapshot_.continuationWindowActive) {
        if (nowMs >= continuationWindowExpiresMs_) {
            snapshot_.continuationWindowActive = false;
            snapshot_.continuationWindowRemainingMs = 0;
            finalizeSession(nowMs);
            return;
        } else {
            snapshot_.continuationWindowRemainingMs = continuationWindowExpiresMs_ - nowMs;
        }
    }

    // Check speed adjustment prompt timeout
    if (snapshot_.speedAdjustmentPromptActive) {
        if (nowMs >= speedAdjustmentPromptExpiresMs_) {
            rejectSpeedAdjustmentShift();
        }
    }

    // Monotonic time elapsed calculation with unsigned rollover safety
    const uint32_t dtMs = (lastUpdateTimestampMs_ > 0)
                              ? (nowMs - lastUpdateTimestampMs_)
                              : 0;
    lastUpdateTimestampMs_ = nowMs;

    snapshot_.snapshotTimestampMs = nowMs;
    snapshot_.snapshotSequence++;

    const bool beltMoving = (applicationSnapshot.speed.speedKmh >= config_.beltMovingThresholdKmh);
    const double currentRunnerDist = applicationSnapshot.runner.validatedDistanceKm;
    double distDelta = 0.0;
    if (lastRunnerDistanceKm_ >= 0.0) {
        if (currentRunnerDist >= lastRunnerDistanceKm_) {
            distDelta = currentRunnerDist - lastRunnerDistanceKm_;
        }
    }
    lastRunnerDistanceKm_ = currentRunnerDist;

    // 1. Armed -> Transition to Running once belt movement begins
    if (snapshot_.state == WorkoutSessionState::Armed) {
        if (beltMoving) {
            snapshot_.state = WorkoutSessionState::Running;
            snapshot_.suspended = false;
            startStep(0, nowMs, currentRunnerDist);
        }
        return;
    }

    // 2. Suspended state -> Check for automatic continuation after normal physical restart
    if (snapshot_.state == WorkoutSessionState::Suspended) {
        if (beltMoving && !isEmergencyStopped_) {
            physicalStopCount_ = 0;
            snapshot_.physicalStopCount = 0;
            snapshot_.continuationWindowActive = false;
            snapshot_.continuationWindowRemainingMs = 0;
            resume(nowMs);
        }
        return;
    }

    // 3. CompletionPending -> Training activity preserved while waiting for physical stop or finalization
    if (snapshot_.state == WorkoutSessionState::CompletionPending) {
        if (beltMoving) {
            totalElapsedTimeMs_ += dtMs;
            if (applicationSnapshot.runner.speedCreditEnabled) {
                activeRunningTimeMs_ += dtMs;
            }
            if (distDelta > 0.0) {
                totalValidatedDistanceKm_ += distDelta;
            }
        }
        snapshot_.totalElapsedTimeMs = totalElapsedTimeMs_;
        snapshot_.activeRunningTimeMs = activeRunningTimeMs_;
        snapshot_.totalValidatedDistanceKm = totalValidatedDistanceKm_;
        return;
    }

    // 4. Running state
    if (snapshot_.state == WorkoutSessionState::Running) {
        // Physical belt stop -> automatic suspension with 1000ms debounce
        if (!beltMoving) {
            if (!lowSpeedDebounceActive_) {
                lowSpeedDebounceActive_ = true;
                lowSpeedStartMs_ = nowMs;
            } else if ((nowMs - lowSpeedStartMs_) >= 1000) {
                lowSpeedDebounceActive_ = false;
                suspend(nowMs);
                return;
            }
        } else {
            lowSpeedDebounceActive_ = false;
        }

        // Accumulate active workout totals
        totalElapsedTimeMs_ += dtMs;
        if (applicationSnapshot.runner.speedCreditEnabled) {
            activeRunningTimeMs_ += dtMs;
        }
        if (distDelta > 0.0) {
            totalValidatedDistanceKm_ += distDelta;
        }

        // Active step progression
        stepElapsedMs_ += dtMs;
        if (currentRunnerDist >= distanceAtStepEntryKm_) {
            stepElapsedValidatedDistanceKm_ = currentRunnerDist - distanceAtStepEntryKm_;
        } else {
            stepElapsedValidatedDistanceKm_ = 0.0;
        }

        const ExpandedStep& curStep = snapshot_.currentStep;

        if (curStep.durationType == DurationType::TIME_SECONDS) {
            uint32_t targetMs = runtimeStepTargetDurationMs_;
            if (targetMs > 0) {
                snapshot_.stepElapsedMs = stepElapsedMs_;
                snapshot_.stepRemainingMs = (stepElapsedMs_ < targetMs) ? (targetMs - stepElapsedMs_) : 0;
                snapshot_.stepProgressFraction = static_cast<float>(stepElapsedMs_) / static_cast<float>(targetMs);
                if (snapshot_.stepProgressFraction > 1.0f) snapshot_.stepProgressFraction = 1.0f;
                snapshot_.stepRemainingFraction = 1.0f - snapshot_.stepProgressFraction;

                if (stepElapsedMs_ >= targetMs) {
                    advanceStep(nowMs, currentRunnerDist);
                }
            }
        } else if (curStep.durationType == DurationType::METERS) {
            while (snapshot_.state == WorkoutSessionState::Running &&
                   snapshot_.currentStep.durationType == DurationType::METERS) {
                double stepTargetKm = static_cast<double>(snapshot_.currentStep.durationValue) / 1000.0;
                if (stepTargetKm <= 0.0) break;
                double elapsedKm = (currentRunnerDist >= distanceAtStepEntryKm_)
                                       ? (currentRunnerDist - distanceAtStepEntryKm_)
                                       : 0.0;
                if (elapsedKm >= stepTargetKm) {
                    double overshootKm = elapsedKm - stepTargetKm;
                    distanceOvershootCarryKm_ = overshootKm;
                    advanceStep(nowMs, currentRunnerDist);
                } else {
                    snapshot_.stepElapsedValidatedDistanceKm = elapsedKm;
                    snapshot_.stepRemainingValidatedDistanceKm = stepTargetKm - elapsedKm;
                    snapshot_.stepProgressFraction = static_cast<float>(elapsedKm / stepTargetKm);
                    if (snapshot_.stepProgressFraction > 1.0f) snapshot_.stepProgressFraction = 1.0f;
                    snapshot_.stepRemainingFraction = 1.0f - snapshot_.stepProgressFraction;
                    break;
                }
            }
        }

        snapshot_.totalElapsedTimeMs = totalElapsedTimeMs_;
        snapshot_.activeRunningTimeMs = activeRunningTimeMs_;
        snapshot_.totalValidatedDistanceKm = totalValidatedDistanceKm_;
        snapshot_.isPartialDrag = isPartialDragCurrent_;
        snapshot_.partialDragCount = partialDragCount_;
        snapshot_.isRestExtended = isRestExtendedCurrent_;
        snapshot_.restExtensionSeconds = restExtensionSecondsTotal_;
        snapshot_.appliedWorkSpeedShiftKmh = speedAdjustmentShiftAppliedKmh_;
        return;
    }
}

bool WorkoutSession::suspend(uint32_t nowMs) {
    if (!initialized_ || !snapshot_.active || snapshot_.state != WorkoutSessionState::Running) {
        return false;
    }
    snapshot_.state = WorkoutSessionState::Suspended;
    snapshot_.suspended = true;
    lastUpdateTimestampMs_ = nowMs;
    lowSpeedDebounceActive_ = false;
    clearPendingCommandIntent();
    return true;
}

bool WorkoutSession::resume(uint32_t nowMs) {
    if (!initialized_ || snapshot_.state != WorkoutSessionState::Suspended) {
        return false;
    }
    snapshot_.state = WorkoutSessionState::Running;
    snapshot_.suspended = false;
    lastUpdateTimestampMs_ = nowMs; // Freezes out paused time
    lowSpeedDebounceActive_ = false;
    emitStepCommandIntent(snapshot_.currentStep, restartReissuePending_);
    restartReissuePending_ = false;
    return true;
}

bool WorkoutSession::cutDrag(uint32_t nowMs) {
    if (!initialized_ || !snapshot_.active || snapshot_.state != WorkoutSessionState::Running ||
        snapshot_.currentStep.role != StepRole::WORK || workout_ == nullptr) {
        return false;
    }

    isPartialDragCurrent_ = true;
    partialDragCount_++;
    snapshot_.isPartialDrag = true;
    snapshot_.partialDragCount = partialDragCount_;

    // Advance to the next REST (Hvile) step, or next valid step if no REST exists
    uint8_t targetStepIndex = snapshot_.currentStepIndex + 1;
    for (uint8_t i = snapshot_.currentStepIndex + 1; i < workout_->totalSteps; ++i) {
        if (workout_->steps[i].role == StepRole::REST) {
            targetStepIndex = i;
            break;
        }
    }

    if (targetStepIndex >= workout_->totalSteps) {
        advanceStep(nowMs, lastRunnerDistanceKm_);
    } else {
        startStep(targetStepIndex, nowMs, lastRunnerDistanceKm_);
    }

    return true;
}

bool WorkoutSession::extendRest(uint32_t extensionSeconds) {
    if (!initialized_ || !snapshot_.active || snapshot_.state != WorkoutSessionState::Running ||
        snapshot_.currentStep.role != StepRole::REST || extensionSeconds == 0) {
        return false;
    }

    runtimeStepTargetDurationMs_ += (extensionSeconds * 1000);
    snapshot_.stepRemainingMs += (extensionSeconds * 1000);
    isRestExtendedCurrent_ = true;
    restExtensionSecondsTotal_ += extensionSeconds;
    snapshot_.isRestExtended = true;
    snapshot_.restExtensionSeconds = restExtensionSecondsTotal_;

    return true;
}

bool WorkoutSession::advanceToNextStep(uint32_t nowMs) {
    if (!initialized_ || !snapshot_.active) {
        return false;
    }
    advanceStep(nowMs, lastRunnerDistanceKm_);
    return true;
}

bool WorkoutSession::abortSession(uint32_t nowMs) {
    if (!initialized_ || !snapshot_.active) {
        return false;
    }
    snapshot_.state = WorkoutSessionState::Aborted;
    snapshot_.active = false;
    snapshot_.suspended = false;
    snapshot_.completionPending = false;
    snapshot_.speedAdjustmentPromptActive = false;
    pendingShiftPrompt_ = false;
    netWorkSpeedDeltaKmh_ = 0.0f;
    lowSpeedDebounceActive_ = false;
    clearPendingCommandIntent();
    return true;
}

bool WorkoutSession::finalizeSession(uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    snapshot_.state = WorkoutSessionState::Completed;
    snapshot_.active = false;
    snapshot_.suspended = false;
    snapshot_.completionPending = false;
    snapshot_.speedAdjustmentPromptActive = false;
    pendingShiftPrompt_ = false;
    netWorkSpeedDeltaKmh_ = 0.0f;
    lowSpeedDebounceActive_ = false;
    clearPendingCommandIntent();
    return true;
}

WorkoutSessionSnapshot WorkoutSession::getSnapshot() const {
    return snapshot_;
}

WorkoutCommandIntent WorkoutSession::getPendingCommandIntent() const {
    return pendingIntent_;
}

void WorkoutSession::clearPendingCommandIntent() {
    pendingIntent_ = WorkoutCommandIntent{};
}

bool WorkoutSession::isActive() const {
    return snapshot_.active;
}

bool WorkoutSession::isSuspended() const {
    return snapshot_.suspended;
}

const char* WorkoutSession::version() {
    return "WorkoutSession/2.0.0";
}

} // namespace stridecontrol


