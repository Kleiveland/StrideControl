#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include "../WorkoutEngine/WorkoutExecutionTypes.h"
#include "../RunnerDynamics/RunnerDynamicsTypes.h"

namespace stridecontrol {

/**
 * @brief State machine operational states for WorkoutSession.
 */
enum class WorkoutSessionState : uint8_t {
    Uninitialized = 0,      ///< begin() not yet called or end() completed
    Idle = 1,               ///< Initialized, no plan loaded or armed
    Armed = 2,              ///< Plan loaded, waiting for belt movement to activate
    Running = 3,            ///< Actively executing workout steps with belt movement
    Suspended = 4,          ///< Interrupted / belt stopped, progression frozen
    CompletionPending = 5,  ///< Nedjogg/cooldown finished, awaiting physical stop or finalize
    Completed = 6,          ///< Workout finished and finalized
    Aborted = 7             ///< Explicitly cancelled by user
};

/**
 * @brief Command intent emitted by WorkoutSession on step entries or state transitions.
 * Forwarded to TreadmillController by the owning application layer.
 *
 * @note WorkoutSession NEVER requests physical treadmill stop (stop is console-only).
 */
struct WorkoutCommandIntent {
    bool hasSpeedTarget = false;
    float targetSpeedKmh = 0.0f;
    bool hasInclineTarget = false;
    uint8_t targetInclinePct = 0;
};

/**
 * @brief Tunable session policy configuration.
 */
struct WorkoutSessionConfig {
    float beltMovingThresholdKmh = 0.5f;                 // Minimum speed to qualify belt motion
    uint32_t defaultRestExtensionSeconds = 30;          // Standard Hvile extension in seconds
    uint32_t continuationWindowDurationMs = 10000;      // 10-second continuation window after 2x stop
    uint32_t speedAdjustmentPromptDurationMs = 15000;   // 15-second prompt duration during REST step
};

/**
 * @brief Copy-safe, immutable snapshot of the active workout session state.
 */
struct WorkoutSessionSnapshot {
    WorkoutSessionState state = WorkoutSessionState::Uninitialized;
    bool initialized = false;
    bool active = false;
    bool suspended = false;
    bool completionPending = false;

    // Plan & Step progress
    uint16_t workoutId = 0;
    uint8_t armedUserId = 0;
    uint8_t currentStepIndex = 0;
    uint8_t totalStepCount = 0;

    ExpandedStep currentStep{};
    StepRole currentRole = StepRole::WORK;
    uint16_t currentRep = 0;
    uint16_t totalRepsInGroup = 0;

    // Metrics for active step
    uint32_t stepElapsedMs = 0;
    uint32_t stepRemainingMs = 0;
    double stepElapsedValidatedDistanceKm = 0.0;
    double stepRemainingValidatedDistanceKm = 0.0;
    float stepProgressFraction = 0.0f;
    float stepRemainingFraction = 1.0f;

    // Total workout metrics
    uint32_t totalElapsedTimeMs = 0;         // Wall-clock active workout time
    uint32_t activeRunningTimeMs = 0;        // Accumulated strictly when runner dynamics speed credit enabled
    double totalValidatedDistanceKm = 0.0;   // Accumulated strictly via RunnerDynamics

    // Target command intent
    bool hasSpeedTarget = false;
    float targetSpeedKmh = 0.0f;
    bool hasInclineTarget = false;
    uint8_t targetInclinePct = 0;

    // Special runtime modifiers
    bool isPartialDrag = false;
    uint8_t partialDragCount = 0;
    bool isRestExtended = false;
    uint32_t restExtensionSeconds = 0;

    // Stop hierarchy & continuation
    uint8_t physicalStopCount = 0;
    bool continuationWindowActive = false;
    uint32_t continuationWindowRemainingMs = 0;
    bool isEmergencyStopped = false;

    // Remaining-drag speed adjustment
    bool speedAdjustmentPromptActive = false;
    float suggestedSpeedDeltaKmh = 0.0f;
    float appliedWorkSpeedShiftKmh = 0.0f;
    uint32_t speedAdjustmentPromptExpiresMs = 0;

    // Diagnostics & Sequence
    uint32_t snapshotTimestampMs = 0;
    uint32_t snapshotSequence = 0;
};

const char* workoutSessionStateName(WorkoutSessionState state);

} // namespace stridecontrol


