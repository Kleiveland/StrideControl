#pragma once

#include <cstdint>
#include "../WorkoutSession/WorkoutSession.h"
#include "../WorkoutDispatcher/WorkoutDispatcher.h"
#include "../WorkoutDispatcher/IWorkoutTargetSink.h"
#include "../ApplicationSnapshot/ApplicationSnapshot.h"

namespace stridecontrol {

/**
 * @brief Deterministic Core 0 control coordinator harness.
 *
 * Sequences the control tick: updates WorkoutSession with the latest
 * ApplicationSnapshot, then triggers WorkoutDispatcher to process intent
 * ingestion, field-wise preemption, and serialized sink delivery.
 */
class ControlCoordinator {
public:
    ControlCoordinator() = default;
    ~ControlCoordinator() = default;

    ControlCoordinator(const ControlCoordinator&) = delete;
    ControlCoordinator& operator=(const ControlCoordinator&) = delete;

    void tick(
        WorkoutSession& session,
        WorkoutDispatcher& dispatcher,
        IWorkoutTargetSink& sink,
        const ApplicationSnapshot& snapshot,
        uint32_t nowMs
    );

    static const char* version();
};

} // namespace stridecontrol

