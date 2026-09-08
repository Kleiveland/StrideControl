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

    StagedTargets getStagedTargets() const;

    static const char* version();

private:
    StagedTargets staged_{};
};

} // namespace stridecontrol

