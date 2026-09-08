#include "ControlCoordinator.h"

namespace stridecontrol {

void ControlCoordinator::tick(
    WorkoutSession& session,
    WorkoutDispatcher& dispatcher,
    IWorkoutTargetSink& sink,
    const ApplicationSnapshot& snapshot,
    uint32_t nowMs
) {
    // 1. Advance session state against latest physical/telemetry snapshot
    session.update(snapshot, nowMs);

    // 2. Ingest domain command intent and dispatch/reconcile with target sink
    dispatcher.update(session, sink, nowMs);
}

const char* ControlCoordinator::version() {
    return "ControlCoordinator/1.0.0";
}

} // namespace stridecontrol

