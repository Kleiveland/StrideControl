#pragma once

#include <cstdint>
#include <cstddef>
#include "../ApplicationSnapshot/ApplicationSnapshot.h"
#include "../WorkoutEngine/WorkoutExecutionTypes.h"
#include "WorkoutSessionTypes.h"

namespace stridecontrol {

/**
 * @brief Deterministic, non-allocating workout and interval runtime execution engine.
 *
 * Consumes an immutable ExpandedWorkout (owned with persistent lifetime by WorkoutEngine)
 * and executes the workout state machine against monotonic time and authoritative
 * RunnerDynamics distance.
 *
 * Produces command intents for speed and incline targets without direct hardware dispatch.
 * WorkoutSession never requests physical treadmill stops (physical stop is console-only).
 */
class WorkoutSession {
public:
    WorkoutSession() = default;
    ~WorkoutSession() = default;

    WorkoutSession(const WorkoutSession&) = delete;
    WorkoutSession& operator=(const WorkoutSession&) = delete;

    bool begin(const WorkoutSessionConfig& config = WorkoutSessionConfig{});
    void end();

    /**
     * @brief Arm a workout session from an immutable ExpandedWorkout reference.
     * @note The caller must ensure the ExpandedWorkout pointer points to the persistent
     *       ExpandedWorkout member owned by WorkoutEngine.
     */
    bool armWorkout(const ExpandedWorkout* workout, uint32_t nowMs, uint8_t userId = 0);
    bool startFreeRun(uint32_t nowMs, uint8_t userId);
    void setDesiredGuiMode(uint8_t userId, bool isManual);

    void update(
        const ApplicationSnapshot& applicationSnapshot,
        uint32_t nowMs
    );

    bool suspend(uint32_t nowMs);
    bool resume(uint32_t nowMs);

    // Stop hierarchy handlers
    void registerPhysicalStop(uint32_t nowMs);
    void registerEmergencyStop(uint32_t nowMs);
    void registerEmergencyStopCleared();

    // Speed adjustment shift handlers
    void reportWorkSpeedAdjustment(float actualSpeedKmh);
    void acceptSpeedAdjustmentShift();
    void rejectSpeedAdjustmentShift();

    bool cutDrag(uint32_t nowMs);
    bool extendRest(uint32_t extensionSeconds = 30);
    bool skipToNextDrag(uint32_t nowMs);
    bool advanceToNextStep(uint32_t nowMs);
    bool abortSession(uint32_t nowMs);
    bool finalizeSession(uint32_t nowMs);

    WorkoutSessionSnapshot getSnapshot() const;
    WorkoutCommandIntent getPendingCommandIntent() const;
    void clearPendingCommandIntent();

    bool isActive() const;
    bool isSuspended() const;

    static const char* version();

private:
    void startStep(uint8_t stepIndex, uint32_t nowMs, double currentRunnerDistanceKm);
    void advanceStep(uint32_t nowMs, double currentRunnerDistanceKm);
    void emitStepCommandIntent(const ExpandedStep& step, bool forceReissue = false);

    WorkoutSessionConfig config_{};
    const ExpandedWorkout* workout_ = nullptr;
    WorkoutSessionSnapshot snapshot_{};
    WorkoutCommandIntent pendingIntent_{};

    bool initialized_ = false;
    uint32_t lastUpdateTimestampMs_ = 0;

    // Step state tracking
    uint32_t stepStartTimestampMs_ = 0;
    uint32_t stepElapsedMs_ = 0;
    uint32_t runtimeStepTargetDurationMs_ = 0;
    double distanceAtStepEntryKm_ = 0.0;
    double stepElapsedValidatedDistanceKm_ = 0.0;
    double lastRunnerDistanceKm_ = -1.0;

    // Total workout metrics
    uint32_t totalElapsedTimeMs_ = 0;
    uint32_t activeRunningTimeMs_ = 0;
    double totalValidatedDistanceKm_ = 0.0;
    double totalElevationMeters_ = 0.0;
    uint32_t heartRateSampleSum_ = 0;
    uint32_t heartRateSampleCount_ = 0;
    uint8_t heartRateMaxBpm_ = 0;
    bool heartRateEverValid_ = false;

    // Runtime modifiers
    uint8_t partialDragCount_ = 0;
    bool isPartialDragCurrent_ = false;
    bool isRestExtendedCurrent_ = false;
    uint32_t restExtensionSecondsTotal_ = 0;

    // Stop hierarchy tracking
    uint8_t physicalStopCount_ = 0;
    uint32_t continuationWindowExpiresMs_ = 0;
    bool isEmergencyStopped_ = false;
    bool eStopRestartPending_ = false;

    // Remaining-drag speed adjustment
    bool pendingShiftPrompt_ = false;
    float netWorkSpeedDeltaKmh_ = 0.0f;
    float speedAdjustmentShiftAppliedKmh_ = 0.0f;
    uint16_t speedAdjustmentShiftSegmentId_ = UINT16_MAX; // Sentinel: no segment tracked yet
    uint32_t actualStepDurationsMs_[MAX_EXPANDED_WORKOUT_STEPS] = {};
    bool hasPriorStep_ = false; // False only before the very first step of a fresh session
    uint32_t speedAdjustmentPromptExpiresMs_ = 0;

    // Intent latching & de-duplication
    bool acknowledgedHasSpeed_ = false;
    float acknowledgedSpeedTargetKmh_ = 0.0f;
    bool acknowledgedHasIncline_ = false;
    uint8_t acknowledgedInclineTargetPct_ = 0;
    bool restartReissuePending_ = false;

    // Low-speed sensor debounce
    bool lowSpeedDebounceActive_ = false;
    uint32_t lowSpeedStartMs_ = 0;

    // Distance overshoot rollover
    double distanceOvershootCarryKm_ = 0.0;

    // Manual "free run" support
    ExpandedWorkout freeRunWorkout_{};
    bool beltHasStoppedSinceSuspend_ = false;
    uint8_t desiredGuiUserId_ = 0;
    bool desiredGuiIsManual_ = false;

    static constexpr uint16_t kFreeRunWorkoutId = 65535;
    static constexpr uint32_t kFreeRunDurationSeconds = 36000; // 10 hours - effectively indefinite
};

} // namespace stridecontrol


