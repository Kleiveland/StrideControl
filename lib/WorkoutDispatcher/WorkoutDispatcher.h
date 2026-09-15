#pragma once

#include <cstdint>
#include "IWorkoutTargetSink.h"
#include "../WorkoutSession/WorkoutSession.h"

namespace stridecontrol {

/**
 * @brief Staged physical command targets awaiting sequential dispatch.
 */
struct StagedTargets {
    bool pendingSpeed = false;
    float speedKmh = 0.0f;
    bool pendingIncline = false;
    float inclinePct = 0.0f;

    // Target identity & provenance
    TargetOrigin speedOrigin = TargetOrigin::None;
    uint32_t speedSessionGeneration = 0;
    uint32_t speedIntentSequence = 0;
    uint8_t speedStepIndex = 0;
    bool speedIsPreFire = false;
    uint32_t speedStagedTimestampMs = 0;

    TargetOrigin inclineOrigin = TargetOrigin::None;
    uint32_t inclineSessionGeneration = 0;
    uint32_t inclineIntentSequence = 0;
    uint8_t inclineStepIndex = 0;
    bool inclineIsPreFire = false;
    uint32_t inclineStagedTimestampMs = 0;
};

/**
 * @brief Context used to stamp provenance and session identity onto direct physical targets.
 */
struct TargetContext {
    TargetOrigin origin = TargetOrigin::StandaloneManual;
    uint32_t sessionGeneration = 0;
    uint8_t stepIndex = 0;
    uint32_t timestampMs = 0;
};

/**
 * @brief Transport and reconciliation dispatcher between WorkoutSession and target sink.
 *
 * Implements bounded, non-allocating command serialization.
 */
class WorkoutDispatcher {
public:
    WorkoutDispatcher() = default;
    ~WorkoutDispatcher() = default;

    WorkoutDispatcher(const WorkoutDispatcher&) = delete;
    WorkoutDispatcher& operator=(const WorkoutDispatcher&) = delete;

    void begin();

    void update(
        WorkoutSession& session,
        IWorkoutTargetSink& treadmill,
        uint32_t nowMs
    );

    bool hasPendingTargets() const;

    void clearSpeedTarget();
    void clearInclineTarget();
    void clearWorkoutTargets();
    void clearForNewSession();
    void clearAllTargets();

    void stageSpeedTarget(float speedKmh, const TargetContext& ctx);
    void stageInclineTarget(float inclinePct, const TargetContext& ctx);
    void stepSpeedTarget(float deltaKmh, float currentSpeedKmh, const TargetContext& ctx);
    void stepInclineTarget(float deltaPct, float currentInclinePct, const TargetContext& ctx);

    void stageSpeedTarget(float speedKmh, uint32_t nowMs = 0, TargetOrigin origin = TargetOrigin::StandaloneManual);
    void stageInclineTarget(float inclinePct, uint32_t nowMs = 0, TargetOrigin origin = TargetOrigin::StandaloneManual);
    void stepSpeedTarget(float deltaKmh, float currentSpeedKmh = 0.0f, uint32_t nowMs = 0, TargetOrigin origin = TargetOrigin::StandaloneManual);
    void stepInclineTarget(float deltaPct, float currentInclinePct = 0.0f, uint32_t nowMs = 0, TargetOrigin origin = TargetOrigin::StandaloneManual);

    StagedTargets getStagedTargets() const;

    static const char* version();

private:
    void validateRetainedTargets(const WorkoutSession& session, uint32_t nowMs);

    StagedTargets staged_{};
};

} // namespace stridecontrol

