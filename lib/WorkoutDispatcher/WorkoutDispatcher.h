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

    void stageSpeedTarget(float speedKmh) {
        staged_.pendingSpeed = true;
        staged_.speedKmh = speedKmh;
    }

    void stageInclineTarget(float inclinePct) {
        staged_.pendingIncline = true;
        staged_.inclinePct = inclinePct;
    }

    void stepSpeedTarget(float deltaKmh, float currentSpeedKmh = 0.0f) {
        if (!staged_.pendingSpeed) {
            staged_.speedKmh = currentSpeedKmh;
        }
        staged_.pendingSpeed = true;
        staged_.speedKmh += deltaKmh;
        if (staged_.speedKmh < 0.0f) staged_.speedKmh = 0.0f;
    }

    void stepInclineTarget(float deltaPct, float currentInclinePct = 0.0f) {
        if (!staged_.pendingIncline) {
            staged_.inclinePct = currentInclinePct;
        }
        staged_.pendingIncline = true;
        staged_.inclinePct += deltaPct;
        if (staged_.inclinePct < 0.0f) staged_.inclinePct = 0.0f;
    }

    StagedTargets getStagedTargets() const;

    static const char* version();

private:
    StagedTargets staged_{};
};

} // namespace stridecontrol

