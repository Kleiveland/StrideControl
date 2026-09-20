#include "ControlCoordinator.h"
#include "../DiagnosticsLog/DiagnosticsLog.h"

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

    // 2. Safe defaults on fresh session start (only when unset)
    const WorkoutSessionSnapshot sessionSnap = session.getSnapshot();
    if (sessionSnap.freshSessionStarted) {
        TargetContext ctx;
        ctx.origin = TargetOrigin::WorkoutGenerated;
        ctx.sessionGeneration = sessionSnap.sessionGeneration;
        ctx.stepIndex = sessionSnap.currentStepIndex;
        ctx.timestampMs = nowMs;

        if (sessionSnap.workoutId == WorkoutSession::kFreeRunWorkoutId) {
            // Manual mode, fresh start only: send speed=1.0 AND incline=0.0 unconditionally
            dispatcher.stageInclineTarget(0.0f, ctx);
            dispatcher.stageSpeedTarget(1.0f, ctx);
            DiagnosticsLog::instance().addEntryf(
                "[FreshStart] Manual mode: staged safe defaults speed=1.0 km/h, incline=0.0%%");
        } else {
            // Interval mode, fresh start only, Warmup step specifically:
            // If the Warmup step's own configured target speed is 0/unset: send speed=1.0
            if (sessionSnap.currentStep.targetSpeedKmh < 0.5f) {
                dispatcher.stageSpeedTarget(1.0f, ctx);
                DiagnosticsLog::instance().addEntryf(
                    "[FreshStart] Interval mode: Warmup speed unset -> staged default speed=1.0 km/h");
            }
            // If the Warmup step's own incline is unset (!setIncline): send incline=0.0
            if (!sessionSnap.currentStep.setIncline) {
                dispatcher.stageInclineTarget(0.0f, ctx);
                DiagnosticsLog::instance().addEntryf(
                    "[FreshStart] Interval mode: Warmup incline unset -> staged default incline=0.0%%");
            }
        }
    }

    // 3. Ingest domain command intent and dispatch/reconcile with target sink
    dispatcher.update(session, sink, nowMs);
}

const char* ControlCoordinator::version() {
    return "ControlCoordinator/1.0.0";
}

} // namespace stridecontrol

