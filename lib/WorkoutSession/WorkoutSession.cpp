#include "WorkoutSession.h"
#include <cmath>
#include <algorithm>

namespace stridecontrol {

const char* workoutSessionStateName(WorkoutSessionState state) {
    switch (state) {
        case WorkoutSessionState::Uninitialized: return "Uninitialized";
        case WorkoutSessionState::Idle: return "Idle";
        case WorkoutSessionState::Armed: return "Armed";
        case WorkoutSessionState::Warmup: return "Warmup";
        case WorkoutSessionState::WorkRamping: return "WorkRamping";
        case WorkoutSessionState::WorkActive: return "WorkActive";
        case WorkoutSessionState::RecoveryRamping: return "RecoveryRamping";
        case WorkoutSessionState::RecoveryActive: return "RecoveryActive";
        case WorkoutSessionState::Cooldown: return "Cooldown";
        case WorkoutSessionState::Suspended: return "Suspended";
        case WorkoutSessionState::AwaitingResumeDecision: return "AwaitingResumeDecision";
        case WorkoutSessionState::Completed: return "Completed";
        case WorkoutSessionState::Failed: return "Failed";
        default: return "Unknown";
    }
}

const char* workoutStepTypeName(WorkoutStepType type) {
    switch (type) {
        case WorkoutStepType::Warmup: return "Warmup";
        case WorkoutStepType::Work: return "Work";
        case WorkoutStepType::Recovery: return "Recovery";
        case WorkoutStepType::Cooldown: return "Cooldown";
        default: return "Unknown";
    }
}

const char* workoutStepGoalTypeName(WorkoutStepGoalType goalType) {
    switch (goalType) {
        case WorkoutStepGoalType::Duration: return "Duration";
        case WorkoutStepGoalType::ValidatedRunnerDistance: return "ValidatedRunnerDistance";
        default: return "Unknown";
    }
}

const char* workoutResumeChoiceName(WorkoutResumeChoice choice) {
    switch (choice) {
        case WorkoutResumeChoice::None: return "None";
        case WorkoutResumeChoice::ResumeRemaining: return "ResumeRemaining";
        case WorkoutResumeChoice::ResumeWithReWarmup: return "ResumeWithReWarmup";
        case WorkoutResumeChoice::RestartCurrentStep: return "RestartCurrentStep";
        case WorkoutResumeChoice::SkipToRecovery: return "SkipToRecovery";
        case WorkoutResumeChoice::EndWorkout: return "EndWorkout";
        default: return "Unknown";
    }
}

const char* workoutResumeRecommendationName(WorkoutResumeRecommendation rec) {
    switch (rec) {
        case WorkoutResumeRecommendation::None: return "None";
        case WorkoutResumeRecommendation::ResumeRemaining: return "ResumeRemaining";
        case WorkoutResumeRecommendation::ShortReEntry: return "ShortReEntry";
        case WorkoutResumeRecommendation::ReWarmup: return "ReWarmup";
        case WorkoutResumeRecommendation::RestartCurrentStep: return "RestartCurrentStep";
        case WorkoutResumeRecommendation::SkipToRecovery: return "SkipToRecovery";
        case WorkoutResumeRecommendation::EndWorkout: return "EndWorkout";
        default: return "Unknown";
    }
}

WorkoutSession::WorkoutSession(
    TreadmillController& controller,
    DiagnosticsService& diagnostics
)
    : controller_(controller),
      diagnostics_(diagnostics) {}

bool WorkoutSession::begin(const WorkoutSessionConfig& config) {
    if (!validateConfig(config)) {
        return false;
    }

    config_ = config;
    initialized_ = true;

    snapshot_ = WorkoutSessionSnapshot{};
    snapshot_.state = WorkoutSessionState::Idle;
    snapshot_.initialized = true;

    return true;
}

void WorkoutSession::end() {
    initialized_ = false;
    plan_ = WorkoutPlan{};
    snapshot_ = WorkoutSessionSnapshot{};
    snapshot_.state = WorkoutSessionState::Uninitialized;
}

bool WorkoutSession::loadPlan(const WorkoutPlan& plan) {
    if (!initialized_ || !validatePlan(plan)) {
        return false;
    }

    plan_ = plan;
    snapshot_.planId = plan.planId;
    snapshot_.totalStepCount = plan.stepCount;
    snapshot_.currentStepIndex = 0;
    snapshot_.state = WorkoutSessionState::Idle;

    return true;
}

bool WorkoutSession::armWorkout(uint32_t nowMs) {
    if (!initialized_ || plan_.stepCount == 0) {
        return false;
    }

    snapshot_.state = WorkoutSessionState::Armed;
    snapshot_.active = true;
    snapshot_.suspended = false;
    snapshot_.decisionRequired = false;
    snapshot_.currentStepIndex = 0;
    snapshot_.currentStep = plan_.steps[0];

    totalElapsedTimeMs_ = 0;
    activeRunningTimeMs_ = 0;
    totalValidatedDistanceKm_ = 0.0;
    stepElapsedMs_ = 0;
    stepElapsedValidatedDistanceKm_ = 0.0;
    lastRunnerDistanceKm_ = 0.0;

    snapshot_.totalElapsedTimeMs = 0;
    snapshot_.activeRunningTimeMs = 0;
    snapshot_.totalValidatedDistanceKm = 0.0;

    snapshot_.snapshotTimestampMs = nowMs;
    lastUpdateTimestampMs_ = nowMs;

    commandSubmittedForStep_ = false;
    inReWarmupStep_ = false;

    return true;
}

void WorkoutSession::update(
    const ApplicationSnapshot& applicationSnapshot,
    const TreadmillControllerSnapshot& controllerSnapshot,
    uint32_t nowMs
) {
    if (!initialized_) {
        return;
    }

    const uint32_t dt = (lastUpdateTimestampMs_ > 0 && nowMs >= lastUpdateTimestampMs_)
                            ? (nowMs - lastUpdateTimestampMs_)
                            : 0;
    lastUpdateTimestampMs_ = nowMs;

    snapshot_.snapshotTimestampMs = nowMs;
    snapshot_.snapshotSequence++;

    // Mirror physical and runner telemetry
    snapshot_.measuredBeltSpeedKmh = applicationSnapshot.speed.speedKmh;
    snapshot_.runnerQualifiedSpeedKmh = applicationSnapshot.runner.runnerSpeedKmh;
    snapshot_.instantaneousCadenceSpm = applicationSnapshot.runner.instantaneousCadenceSpm;
    snapshot_.cadenceValid = applicationSnapshot.runner.cadenceValid;
    snapshot_.runnerOnSideRails = applicationSnapshot.runner.onSideRails;
    snapshot_.runnerSpeedCreditEnabled = applicationSnapshot.runner.speedCreditEnabled;
    snapshot_.runnerPresence = applicationSnapshot.runner.presence;

    const bool beltMoving = (applicationSnapshot.speed.speedKmh >= config_.beltMovingThresholdKmh);
    const double currentRunnerDist = applicationSnapshot.runner.validatedDistanceKm;
    const double distDelta = (lastRunnerDistanceKm_ > 0.0 && currentRunnerDist >= lastRunnerDistanceKm_)
                                 ? (currentRunnerDist - lastRunnerDistanceKm_)
                                 : 0.0;
    lastRunnerDistanceKm_ = currentRunnerDist;

    // 1. Handle Armed state -> Activate on belt motion
    if (snapshot_.state == WorkoutSessionState::Armed) {
        if (beltMoving) {
            startStep(0, nowMs);
        }
        return;
    }

    // 2. Handle active states
    const bool isStepState = (snapshot_.state == WorkoutSessionState::Warmup ||
                              snapshot_.state == WorkoutSessionState::WorkRamping ||
                              snapshot_.state == WorkoutSessionState::WorkActive ||
                              snapshot_.state == WorkoutSessionState::RecoveryRamping ||
                              snapshot_.state == WorkoutSessionState::RecoveryActive ||
                              snapshot_.state == WorkoutSessionState::Cooldown);

    if (isStepState) {
        // Interruption check: Belt stopped (speedKmh == 0.0f)
        if (!beltMoving) {
            snapshot_.state = WorkoutSessionState::Suspended;
            snapshot_.suspended = true;
            snapshot_.lastSuspensionTimestampMs = nowMs;
            suspensionStartTimestampMs_ = nowMs;
            return;
        }

        const WorkoutStep& curStep = snapshot_.currentStep;

        // A. Speed Ramping phase
        if (snapshot_.state == WorkoutSessionState::WorkRamping ||
            snapshot_.state == WorkoutSessionState::RecoveryRamping) {

            if (!commandSubmittedForStep_) {
                controller_.submitSpeedTarget(curStep.targetPhysicalSpeedKmh, nowMs);
                controller_.submitInclineTarget(curStep.targetInclinePct, nowMs);
                commandSubmittedForStep_ = true;
                stepStartTimestampMs_ = nowMs;
                snapshot_.expectedRampTimeMs = calculateExpectedRampTimeMs(
                    applicationSnapshot.speed.speedKmh, curStep.targetPhysicalSpeedKmh);
                snapshot_.speedGateReached = false;
                snapshot_.speedGateTimedOut = false;
            }

            const float speedDiff = std::abs(applicationSnapshot.speed.speedKmh - curStep.targetPhysicalSpeedKmh);
            if (speedDiff <= config_.targetSpeedToleranceKmh) {
                snapshot_.speedGateReached = true;
                stepSpeedGateTimestampMs_ = nowMs;
                if (curStep.type == WorkoutStepType::Work) {
                    snapshot_.state = WorkoutSessionState::WorkActive;
                } else if (curStep.type == WorkoutStepType::Recovery) {
                    snapshot_.state = WorkoutSessionState::RecoveryActive;
                }
            } else if (nowMs - stepStartTimestampMs_ > config_.speedGateTimeoutMs) {
                snapshot_.speedGateTimedOut = true;
                if (curStep.type == WorkoutStepType::Work) {
                    snapshot_.state = WorkoutSessionState::WorkActive;
                } else if (curStep.type == WorkoutStepType::Recovery) {
                    snapshot_.state = WorkoutSessionState::RecoveryActive;
                }
            }
        }

        // B. Active progress tracking
        totalElapsedTimeMs_ += dt;
        if (applicationSnapshot.runner.speedCreditEnabled) {
            activeRunningTimeMs_ += dt;
        }
        if (distDelta > 0.0) {
            totalValidatedDistanceKm_ += distDelta;
            stepElapsedValidatedDistanceKm_ += distDelta;
        }

        // Step wall-clock duration advances continuously
        stepElapsedMs_ += dt;

        snapshot_.totalElapsedTimeMs = totalElapsedTimeMs_;
        snapshot_.activeRunningTimeMs = activeRunningTimeMs_;
        snapshot_.totalValidatedDistanceKm = totalValidatedDistanceKm_;
        snapshot_.stepElapsedMs = stepElapsedMs_;
        snapshot_.stepElapsedValidatedDistanceKm = stepElapsedValidatedDistanceKm_;

        // C. Step completion evaluation
        if (curStep.goalType == WorkoutStepGoalType::Duration) {
            if (curStep.targetDurationMs > 0) {
                snapshot_.stepRemainingMs = (stepElapsedMs_ < curStep.targetDurationMs)
                                                ? (curStep.targetDurationMs - stepElapsedMs_)
                                                : 0;
                snapshot_.stepProgressFraction = static_cast<float>(stepElapsedMs_) /
                                                 static_cast<float>(curStep.targetDurationMs);
                snapshot_.stepRemainingFraction = 1.0f - snapshot_.stepProgressFraction;

                if (stepElapsedMs_ >= curStep.targetDurationMs) {
                    advanceStep(nowMs);
                }
            }
        } else if (curStep.goalType == WorkoutStepGoalType::ValidatedRunnerDistance) {
            if (curStep.targetDistanceKm > 0.0) {
                snapshot_.stepRemainingValidatedDistanceKm = (stepElapsedValidatedDistanceKm_ < curStep.targetDistanceKm)
                                                                 ? (curStep.targetDistanceKm - stepElapsedValidatedDistanceKm_)
                                                                 : 0.0;
                snapshot_.stepProgressFraction = static_cast<float>(stepElapsedValidatedDistanceKm_ / curStep.targetDistanceKm);
                snapshot_.stepRemainingFraction = 1.0f - snapshot_.stepProgressFraction;

                if (stepElapsedValidatedDistanceKm_ >= curStep.targetDistanceKm) {
                    advanceStep(nowMs);
                }
            }
        }
        return;
    }

    // 3. Handle Suspended state
    if (snapshot_.state == WorkoutSessionState::Suspended) {
        if (beltMoving) {
            // Motion restarted -> Await resume choice
            snapshot_.state = WorkoutSessionState::AwaitingResumeDecision;
            snapshot_.decisionRequired = true;
            const uint32_t pauseDuration = (nowMs >= suspensionStartTimestampMs_)
                                               ? (nowMs - suspensionStartTimestampMs_)
                                               : 0;
            snapshot_.currentPauseDurationMs = pauseDuration;
            evaluateRecommendations(pauseDuration);
        }
        return;
    }
}

bool WorkoutSession::applyResumeChoice(
    WorkoutResumeChoice choice,
    uint32_t nowMs
) {
    if (!initialized_ || snapshot_.state != WorkoutSessionState::AwaitingResumeDecision) {
        return false;
    }

    snapshot_.decisionRequired = false;
    snapshot_.suspended = false;

    switch (choice) {
        case WorkoutResumeChoice::ResumeRemaining: {
            commandSubmittedForStep_ = false;
            const WorkoutStep& curStep = snapshot_.currentStep;
            if (curStep.type == WorkoutStepType::Work) {
                snapshot_.state = WorkoutSessionState::WorkRamping;
            } else if (curStep.type == WorkoutStepType::Recovery) {
                snapshot_.state = WorkoutSessionState::RecoveryRamping;
            } else if (curStep.type == WorkoutStepType::Warmup) {
                snapshot_.state = WorkoutSessionState::Warmup;
            } else {
                snapshot_.state = WorkoutSessionState::Cooldown;
            }
            controller_.submitSpeedTarget(curStep.targetPhysicalSpeedKmh, nowMs);
            controller_.submitInclineTarget(curStep.targetInclinePct, nowMs);
            return true;
        }

        case WorkoutResumeChoice::ResumeWithReWarmup: {
            inReWarmupStep_ = true;
            snapshot_.state = WorkoutSessionState::Warmup;
            controller_.submitSpeedTarget(config_.reWarmupSpeedKmh, nowMs);
            controller_.submitInclineTarget(config_.reWarmupInclinePct, nowMs);
            return true;
        }

        case WorkoutResumeChoice::RestartCurrentStep: {
            startStep(snapshot_.currentStepIndex, nowMs, true);
            return true;
        }

        case WorkoutResumeChoice::SkipToRecovery: {
            // Find next Recovery step in plan
            for (uint8_t i = snapshot_.currentStepIndex + 1; i < plan_.stepCount; ++i) {
                if (plan_.steps[i].type == WorkoutStepType::Recovery) {
                    startStep(i, nowMs);
                    return true;
                }
            }
            // If no subsequent recovery step, advance to next step normally
            advanceStep(nowMs);
            return true;
        }

        case WorkoutResumeChoice::EndWorkout: {
            snapshot_.state = WorkoutSessionState::Completed;
            snapshot_.active = false;
            controller_.submitStop(nowMs);
            return true;
        }

        default:
            return false;
    }
}

bool WorkoutSession::skipCurrentStep(uint32_t nowMs) {
    if (!initialized_ || !snapshot_.active || snapshot_.suspended) {
        return false;
    }
    advanceStep(nowMs);
    return true;
}

bool WorkoutSession::extendCurrentStep(uint32_t extensionMs) {
    if (!initialized_ || !snapshot_.active || extensionMs == 0) {
        return false;
    }
    if (snapshot_.currentStep.goalType == WorkoutStepGoalType::Duration) {
        snapshot_.currentStep.targetDurationMs += extensionMs;
        return true;
    }
    return false;
}

void WorkoutSession::startStep(uint8_t stepIndex, uint32_t nowMs, bool isRestart) {
    if (stepIndex >= plan_.stepCount) {
        snapshot_.state = WorkoutSessionState::Completed;
        snapshot_.active = false;
        controller_.submitStop(nowMs);
        return;
    }

    snapshot_.currentStepIndex = stepIndex;
    snapshot_.currentStep = plan_.steps[stepIndex];

    if (isRestart) {
        stepElapsedMs_ = 0;
        stepElapsedValidatedDistanceKm_ = 0.0;
    }

    commandSubmittedForStep_ = false;

    const WorkoutStep& curStep = snapshot_.currentStep;

    if (curStep.type == WorkoutStepType::Work) {
        snapshot_.state = WorkoutSessionState::WorkRamping;
    } else if (curStep.type == WorkoutStepType::Recovery) {
        snapshot_.state = WorkoutSessionState::RecoveryRamping;
    } else if (curStep.type == WorkoutStepType::Warmup) {
        snapshot_.state = WorkoutSessionState::Warmup;
    } else if (curStep.type == WorkoutStepType::Cooldown) {
        snapshot_.state = WorkoutSessionState::Cooldown;
    }

    controller_.submitSpeedTarget(curStep.targetPhysicalSpeedKmh, nowMs);
    controller_.submitInclineTarget(curStep.targetInclinePct, nowMs);
    commandSubmittedForStep_ = true;
    stepStartTimestampMs_ = nowMs;
}

void WorkoutSession::advanceStep(uint32_t nowMs) {
    stepElapsedMs_ = 0;
    stepElapsedValidatedDistanceKm_ = 0.0;
    startStep(snapshot_.currentStepIndex + 1, nowMs);
}

void WorkoutSession::evaluateRecommendations(uint32_t pauseDurationMs) {
    snapshot_.choiceResumeRemainingAvailable = true;
    snapshot_.choiceEndWorkoutAvailable = true;
    snapshot_.choiceResumeWithReWarmupAvailable = (pauseDurationMs >= config_.immediateResumeThresholdMs);
    snapshot_.choiceRestartStepAvailable = (snapshot_.currentStep.type == WorkoutStepType::Work);
    snapshot_.choiceSkipToRecoveryAvailable = (snapshot_.currentStep.type == WorkoutStepType::Work);

    if (pauseDurationMs < config_.immediateResumeThresholdMs) {
        snapshot_.recommendation = WorkoutResumeRecommendation::ResumeRemaining;
    } else if (pauseDurationMs < config_.shortReEntryThresholdMs) {
        if (snapshot_.currentStep.type == WorkoutStepType::Work &&
            snapshot_.stepRemainingFraction <= config_.skipToRecoveryRemainingFraction) {
            snapshot_.recommendation = WorkoutResumeRecommendation::SkipToRecovery;
        } else {
            snapshot_.recommendation = WorkoutResumeRecommendation::ShortReEntry;
        }
    } else if (pauseDurationMs < config_.reWarmupThresholdMs) {
        snapshot_.recommendation = WorkoutResumeRecommendation::ReWarmup;
    } else {
        snapshot_.recommendation = WorkoutResumeRecommendation::RestartCurrentStep;
    }
}

uint32_t WorkoutSession::calculateExpectedRampTimeMs(float fromSpeedKmh, float toSpeedKmh) const {
    const float diff = std::abs(toSpeedKmh - fromSpeedKmh);
    const float rate = (toSpeedKmh >= fromSpeedKmh)
                           ? config_.assumedAccelerationKmhPerSec
                           : config_.assumedDecelerationKmhPerSec;
    if (rate <= 0.0f) {
        return config_.commandLatencyMs;
    }
    const float sec = (diff / rate) + (static_cast<float>(config_.commandLatencyMs) / 1000.0f);
    return static_cast<uint32_t>(sec * 1000.0f);
}

bool WorkoutSession::validatePlan(const WorkoutPlan& plan) const {
    if (plan.stepCount == 0 || plan.stepCount > kMaxWorkoutSteps) {
        return false;
    }

    for (size_t i = 0; i < plan.stepCount; ++i) {
        const auto& step = plan.steps[i];
        if (!std::isfinite(step.targetPhysicalSpeedKmh) ||
            step.targetPhysicalSpeedKmh < 0.0f ||
            step.targetPhysicalSpeedKmh > 25.0f) {
            return false;
        }
        if (!std::isfinite(step.targetInclinePct) ||
            step.targetInclinePct < 0.0f ||
            step.targetInclinePct > 15.0f ||
            std::floor(step.targetInclinePct) != step.targetInclinePct) {
            return false;
        }
        if (step.goalType == WorkoutStepGoalType::Duration && step.targetDurationMs == 0) {
            return false;
        }
        if (step.goalType == WorkoutStepGoalType::ValidatedRunnerDistance &&
            (!std::isfinite(step.targetDistanceKm) || step.targetDistanceKm <= 0.0)) {
            return false;
        }
        if (step.type != WorkoutStepType::Recovery && step.restType != WorkoutRestType::None) {
            return false;
        }
    }

    return true;
}

bool WorkoutSession::validateConfig(const WorkoutSessionConfig& config) const {
    if (config.immediateResumeThresholdMs == 0 ||
        config.shortReEntryThresholdMs == 0 ||
        config.reWarmupThresholdMs == 0 ||
        config.extendedReWarmupThresholdMs == 0) {
        return false;
    }
    if (!std::isfinite(config.skipToRecoveryRemainingFraction) ||
        config.skipToRecoveryRemainingFraction < 0.0f ||
        config.skipToRecoveryRemainingFraction > 1.0f) {
        return false;
    }
    if (config.assumedAccelerationKmhPerSec <= 0.0f ||
        config.assumedDecelerationKmhPerSec <= 0.0f ||
        config.beltMovingThresholdKmh <= 0.0f ||
        config.targetSpeedToleranceKmh <= 0.0f) {
        return false;
    }
    return true;
}

WorkoutSessionSnapshot WorkoutSession::getSnapshot() const {
    return snapshot_;
}

bool WorkoutSession::isActive() const {
    return snapshot_.active;
}

bool WorkoutSession::isSuspended() const {
    return snapshot_.suspended;
}

const char* WorkoutSession::version() {
    return "WorkoutSession/1.1.0";
}

} // namespace stridecontrol
