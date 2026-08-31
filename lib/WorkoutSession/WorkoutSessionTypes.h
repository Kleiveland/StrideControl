#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include "../RunnerDynamics/RunnerDynamicsTypes.h"

namespace stridecontrol {

constexpr size_t kMaxWorkoutSteps = 96;

/**
 * @brief State machine operational states for WorkoutSession.
 */
enum class WorkoutSessionState : uint8_t {
    Uninitialized,          ///< begin() not yet called or end() completed
    Idle,                   ///< Initialized, no plan loaded or armed
    Armed,                  ///< Plan loaded, waiting for belt movement to activate
    Warmup,                 ///< Executing warmup step
    WorkRamping,            ///< Transitioning speed/incline to work step target
    WorkActive,             ///< Actively executing work step
    RecoveryRamping,        ///< Transitioning speed/incline to recovery target
    RecoveryActive,         ///< Actively executing recovery step
    Cooldown,               ///< Executing cooldown step
    Suspended,              ///< Interrupted / belt stopped, progression frozen
    AwaitingResumeDecision, ///< Belt restarted, awaiting user UI resume choice
    Completed,              ///< All plan steps completed successfully
    Failed                  ///< Controller or configuration failure
};

/**
 * @brief Structural step classification within a structured workout plan.
 */
enum class WorkoutStepType : uint8_t {
    Warmup,
    Work,
    Recovery,
    Cooldown
};

/**
 * @brief Goal criteria defining when a step reaches completion.
 */
enum class WorkoutStepGoalType : uint8_t {
    Duration,
    ValidatedRunnerDistance
};

/**
 * @brief Rest tactic intended during recovery steps.
 */
enum class WorkoutRestType : uint8_t {
    None,
    ActiveRecovery,
    SideRailStand
};

/**
 * @brief Available user action choices upon resuming a suspended session.
 */
enum class WorkoutResumeChoice : uint8_t {
    None,
    ResumeRemaining,
    ResumeWithReWarmup,
    RestartCurrentStep,
    SkipToRecovery,
    EndWorkout
};

/**
 * @brief Advisory system recommendation generated upon resume motion.
 */
enum class WorkoutResumeRecommendation : uint8_t {
    None,
    ResumeRemaining,
    ShortReEntry,
    ReWarmup,
    RestartCurrentStep,
    SkipToRecovery,
    EndWorkout
};

/**
 * @brief Single discrete step in a structured workout profile.
 */
struct WorkoutStep {
    WorkoutStepType type = WorkoutStepType::Work;
    WorkoutStepGoalType goalType = WorkoutStepGoalType::Duration;
    WorkoutRestType restType = WorkoutRestType::None;

    float targetPhysicalSpeedKmh = 0.0f;
    float targetInclinePct = 0.0f;

    uint32_t targetDurationMs = 0;
    double targetDistanceKm = 0.0;
};

/**
 * @brief Complete structured workout plan.
 */
struct WorkoutPlan {
    uint32_t planId = 0;
    uint8_t stepCount = 0;
    std::array<WorkoutStep, kMaxWorkoutSteps> steps{};
};

/**
 * @brief Tunable session policy configuration.
 */
struct WorkoutSessionConfig {
    uint32_t immediateResumeThresholdMs = 60000;         // 0–60s pause -> ResumeRemaining
    uint32_t shortReEntryThresholdMs = 300000;           // 1–5 min pause -> ShortReEntry
    uint32_t reWarmupThresholdMs = 900000;               // 5–15 min pause -> ReWarmup
    uint32_t extendedReWarmupThresholdMs = 1800000;      // >15 min pause -> Restart or Cold End
    float skipToRecoveryRemainingFraction = 0.30f;       // <30% remaining in Work -> Recommend SkipToRecovery
    float beltMovingThresholdKmh = 0.5f;                 // Minimum speed to qualify belt motion
    float targetSpeedToleranceKmh = 0.3f;                // Speed gate tolerance window
    uint32_t speedGateTimeoutMs = 15000;                 // Timeout waiting for motor to reach speed gate
    float assumedAccelerationKmhPerSec = 1.5f;           // Estimated acceleration rate for ramp calculation
    float assumedDecelerationKmhPerSec = 2.0f;           // Estimated deceleration rate
    uint32_t commandLatencyMs = 200;                     // Console queue and communication delay
    uint32_t reWarmupDurationMs = 120000;                // Default duration for re-warmup step (2 min)
    float reWarmupSpeedKmh = 6.0f;                       // Default easy speed for re-warmup
    float reWarmupInclinePct = 0.0f;                     // Default incline for re-warmup
};

/**
 * @brief Copy-safe, immutable snapshot of the active workout session state.
 */
struct WorkoutSessionSnapshot {
    WorkoutSessionState state = WorkoutSessionState::Uninitialized;
    bool initialized = false;
    bool active = false;
    bool suspended = false;
    bool decisionRequired = false;

    // Plan & Step progress
    uint32_t planId = 0;
    uint8_t currentStepIndex = 0;
    uint8_t totalStepCount = 0;

    WorkoutStep currentStep{};

    // Metrics for active step
    uint32_t stepElapsedMs = 0;
    uint32_t stepRemainingMs = 0;
    double stepElapsedValidatedDistanceKm = 0.0;
    double stepRemainingValidatedDistanceKm = 0.0;
    float stepProgressFraction = 0.0f;
    float stepRemainingFraction = 1.0f;

    // Total workout metrics
    uint32_t totalElapsedTimeMs = 0;         // Wall-clock session time
    uint32_t activeRunningTimeMs = 0;        // Accumulated strictly when speedCreditEnabled == true
    double totalValidatedDistanceKm = 0.0;   // Accumulated strictly via RunnerDynamics

    // Physical & Runner context
    float measuredBeltSpeedKmh = 0.0f;
    float runnerQualifiedSpeedKmh = 0.0f;
    float instantaneousCadenceSpm = 0.0f;
    bool cadenceValid = false;
    bool runnerOnSideRails = false;
    bool runnerSpeedCreditEnabled = false;
    RunnerPresence runnerPresence = RunnerPresence::Unknown;

    // Heart rate context (only when available)
    uint8_t heartRateBpm = 0;
    bool heartRateValid = false;

    // Speed gate & ramp tracking
    bool speedGateReached = false;
    bool speedGateTimedOut = false;
    uint32_t expectedRampTimeMs = 0;

    // Suspension & Resume Context
    uint32_t lastSuspensionTimestampMs = 0;
    uint32_t currentPauseDurationMs = 0;
    WorkoutResumeRecommendation recommendation = WorkoutResumeRecommendation::None;

    // Bitmask of available resume choices
    bool choiceResumeRemainingAvailable = false;
    bool choiceResumeWithReWarmupAvailable = false;
    bool choiceRestartStepAvailable = false;
    bool choiceSkipToRecoveryAvailable = false;
    bool choiceEndWorkoutAvailable = false;

    // Diagnostics & Sequence
    uint32_t snapshotTimestampMs = 0;
    uint32_t snapshotSequence = 0;
};

const char* workoutSessionStateName(WorkoutSessionState state);
const char* workoutStepTypeName(WorkoutStepType type);
const char* workoutStepGoalTypeName(WorkoutStepGoalType goalType);
const char* workoutResumeChoiceName(WorkoutResumeChoice choice);
const char* workoutResumeRecommendationName(WorkoutResumeRecommendation rec);

} // namespace stridecontrol
