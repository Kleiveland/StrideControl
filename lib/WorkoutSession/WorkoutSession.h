#pragma once

#include <cstdint>
#include <cstddef>
#include "../TreadmillController/TreadmillController.h"
#include "../DiagnosticsService/DiagnosticsService.h"
#include "../ApplicationSnapshot/ApplicationSnapshot.h"
#include "WorkoutSessionTypes.h"

namespace stridecontrol {

/**
 * @brief Deterministic, non-blocking workout and interval state engine.
 *
 * Owns workout plan loading, step transitions, interval wall-clock countdowns,
 * runner-qualified distance tracking, motion-gated activation, and structured resume recommendations.
 */
class WorkoutSession {
public:
    WorkoutSession(
        TreadmillController& controller,
        DiagnosticsService& diagnostics
    );

    ~WorkoutSession() = default;

    WorkoutSession(const WorkoutSession&) = delete;
    WorkoutSession& operator=(const WorkoutSession&) = delete;

    bool begin(const WorkoutSessionConfig& config = WorkoutSessionConfig{});
    void end();

    bool loadPlan(const WorkoutPlan& plan);
    bool armWorkout(uint32_t nowMs);

    void update(
        const ApplicationSnapshot& applicationSnapshot,
        const TreadmillControllerSnapshot& controllerSnapshot,
        uint32_t nowMs
    );

    bool applyResumeChoice(
        WorkoutResumeChoice choice,
        uint32_t nowMs
    );

    bool skipCurrentStep(uint32_t nowMs);
    bool extendCurrentStep(uint32_t extensionMs);

    WorkoutSessionSnapshot getSnapshot() const;

    bool isActive() const;
    bool isSuspended() const;

    static const char* version();

private:
    bool validatePlan(const WorkoutPlan& plan) const;
    bool validateConfig(const WorkoutSessionConfig& config) const;
    void startStep(uint8_t stepIndex, uint32_t nowMs, bool isRestart = false);
    void advanceStep(uint32_t nowMs);
    void evaluateRecommendations(uint32_t pauseDurationMs);
    uint32_t calculateExpectedRampTimeMs(float fromSpeedKmh, float toSpeedKmh) const;

    TreadmillController& controller_;
    DiagnosticsService& diagnostics_;

    WorkoutSessionConfig config_{};
    WorkoutPlan plan_{};
    WorkoutSessionSnapshot snapshot_{};

    bool initialized_ = false;
    uint32_t lastUpdateTimestampMs_ = 0;

    // Step state tracking
    uint32_t stepStartTimestampMs_ = 0;
    uint32_t stepSpeedGateTimestampMs_ = 0;
    double lastRunnerDistanceKm_ = 0.0;
    uint32_t stepElapsedMs_ = 0;
    double stepElapsedValidatedDistanceKm_ = 0.0;
    bool commandSubmittedForStep_ = false;

    // Workout time metrics
    uint32_t totalElapsedTimeMs_ = 0;
    uint32_t activeRunningTimeMs_ = 0;
    double totalValidatedDistanceKm_ = 0.0;

    // Suspension tracking
    uint32_t suspensionStartTimestampMs_ = 0;

    // Re-warmup step tracking
    bool inReWarmupStep_ = false;
};

} // namespace stridecontrol
