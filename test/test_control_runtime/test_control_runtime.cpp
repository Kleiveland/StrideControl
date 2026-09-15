#include <unity.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <cmath>
#include <cstring>
#include "WorkoutSession.h"
#include "WorkoutExpander.h"
#include "WorkoutDispatcher.h"
#include "TreadmillSimulator.h"
#include "TreadmillController.h"
#include "TreadmillControllerAdapter.h"
#include "ControlCoordinator.h"
#include "ControlRuntime.h"
#include "ConsoleInterface.h"
#include "SpeedCalibration.h"
#include "DiagnosticsService.h"

using namespace stridecontrol;

static ConsoleInterface s_console;
static SpeedCalibration s_calibration;
static DiagnosticsService s_diagnostics;
static ExpandedWorkout s_testWorkout;
static ExpandedWorkout s_testWorkout2;

class MockOrchestrator : public ApplicationOrchestrator {
public:
    void setSnapshot(const ApplicationSnapshot& snap) {
        portENTER_CRITICAL(&mux_);
        snap_ = snap;
        portEXIT_CRITICAL(&mux_);
    }

    ApplicationSnapshot getSnapshot() const override {
        portENTER_CRITICAL(&mux_);
        ApplicationSnapshot s = snap_;
        portEXIT_CRITICAL(&mux_);
        return s;
    }

private:
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    ApplicationSnapshot snap_{};
};

static MockOrchestrator s_mockOrchestrator;

static void initTestWorkout(ExpandedWorkout& ew) {
    memset(&ew, 0, sizeof(ExpandedWorkout));
    ew.workoutId = 303;
    strncpy(ew.workoutName, "RUNTIME TEST", sizeof(ew.workoutName) - 1);
    ew.totalSteps = 2;

    // Step 0: Warmup, FIXED 10.0 km/h, 2% incline
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WARMUP;
    ew.steps[0].durationType = DurationType::TIME_SECONDS;
    ew.steps[0].durationValue = 60;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 10.0f;
    ew.steps[0].setIncline = true;
    ew.steps[0].targetInclinePct = 2;

    // Step 1: Work, FIXED 14.0 km/h, 4% incline
    ew.steps[1].stepIndex = 1;
    ew.steps[1].role = StepRole::WORK;
    ew.steps[1].durationType = DurationType::TIME_SECONDS;
    ew.steps[1].durationValue = 120;
    ew.steps[1].speedMode = SpeedMode::FIXED;
    ew.steps[1].targetSpeedKmh = 14.0f;
    ew.steps[1].setIncline = true;
    ew.steps[1].targetInclinePct = 4;
}

void setUp() {
    initTestWorkout(s_testWorkout);
    initTestWorkout(s_testWorkout2);
}

void tearDown() {
    s_console.end();
}

static void setCsafeSnapshotState(ApplicationSnapshot& snap, CsafeMachineState state, bool online = true, bool fresh = true) {
    snap.csafe.initialized = true;
    snap.csafe.online = online;
    snap.csafe.linkStatus = online ? CsafeLinkStatus::Online : CsafeLinkStatus::TimedOut;
    snap.csafe.qualifiedState = state;
    snap.csafe.reportedState = state;
    snap.csafe.machineStateFresh = fresh;
    snap.csafe.rawStateByte = static_cast<uint8_t>(state);
    snap.csafe.stateNibble = static_cast<uint8_t>(state) & 0x0F;
}

static ApplicationSnapshot makeAuthoritativeSnapshot(uint32_t nowMs, float speedKmh, double distanceKm) {
    ApplicationSnapshot snap{};
    snap.timestampMs = nowMs;
    snap.sequenceNumber = 10;

    snap.speed.initialized = true;
    snap.speed.speedKmh = speedKmh;
    if (speedKmh > 0.1f) {
        snap.speed.signalPresent = true;
        snap.speed.measurementValid = true;
        snap.speed.status = SpeedSensorStatus::Measuring;
    } else {
        snap.speed.signalPresent = false;
        snap.speed.measurementValid = false;
        snap.speed.status = SpeedSensorStatus::TimedOut; // Standstill exemption
    }

    snap.incline.initialized = true;
    snap.incline.status = InclineStatus::Ready;
    snap.incline.estimatedInclinePct = 2.0f;

    snap.runner.initialized = true;
    snap.runner.status = (speedKmh > 0.1f) ? RunnerDynamicsStatus::Tracking : RunnerDynamicsStatus::Ready;
    snap.runner.validatedDistanceKm = distanceKm;
    snap.runner.speedCreditEnabled = true;
    snap.runner.distanceValid = true;
    snap.runner.distancePauseReason = (speedKmh > 0.1f) ? RunnerDistancePauseReason::None : RunnerDistancePauseReason::BeltStopped;

    snap.health.isHealthy = true;
    snap.health.highestSeverity = FaultSeverity::None;

    if (speedKmh > 0.1f) {
        setCsafeSnapshotState(snap, CsafeMachineState::InUse, true, true);
    } else {
        setCsafeSnapshotState(snap, CsafeMachineState::Ready, true, true);
    }

    return snap;
}

// 1. Exactly one ControlRuntime owner path
void test_control_runtime_single_owner_path() {
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);

    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_FALSE(runtime.isSessionActive());
    TEST_ASSERT_FALSE(runtime.isSessionSuspended());

    // Accessors reflect single internal instance
    WorkoutSessionSnapshot sSnap = runtime.getSessionSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Idle, sSnap.state);

    StagedTargets targets = runtime.getStagedTargets();
    TEST_ASSERT_FALSE(targets.pendingSpeed);
    TEST_ASSERT_FALSE(targets.pendingIncline);

    runtime.end();
}

// 2. Snapshot age rollover safety
void test_snapshot_age_rollover_safety() {
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1000, 0.0f, 0.0);

    // Fresh snapshot
    TEST_ASSERT_TRUE(ControlRuntime::isSnapshotAuthoritative(snap, 1100)); // age 100ms <= 200ms

    // Stale snapshot
    TEST_ASSERT_FALSE(ControlRuntime::isSnapshotAuthoritative(snap, 1250)); // age 250ms > 200ms

    // Rollover near UINT32_MAX
    const uint32_t rolloverTimestamp = 0xFFFFFFF0; // 4294967280
    snap.timestampMs = rolloverTimestamp;

    // 26ms later across rollover (wraps to 10)
    TEST_ASSERT_TRUE(ControlRuntime::isSnapshotAuthoritative(snap, 10)); // age 26ms <= 200ms

    // 250ms later across rollover (wraps to 234)
    TEST_ASSERT_FALSE(ControlRuntime::isSnapshotAuthoritative(snap, 234)); // age 250ms > 200ms
}

// 3. Operational stopped snapshot acceptance (Standstill Exemption)
void test_operational_stopped_snapshot_acceptance() {
    // Belt stopped: measurementValid is false, status is TimedOut
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1000, 0.0f, 0.0);
    snap.speed.measurementValid = false;
    snap.speed.status = SpeedSensorStatus::TimedOut;

    TEST_ASSERT_TRUE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));

    // Also valid with AwaitingFirstPulse at initial startup
    snap.speed.status = SpeedSensorStatus::AwaitingFirstPulse;
    TEST_ASSERT_TRUE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));

    // But HardwareError must be rejected
    snap.speed.status = SpeedSensorStatus::HardwareError;
    TEST_ASSERT_FALSE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));
}

// 4. Stale moving snapshot rejection
void test_stale_moving_snapshot_rejection() {
    // Belt moving at 10 km/h with valid measuring status
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1000, 10.0f, 0.5);
    TEST_ASSERT_TRUE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));

    // Moving belt with lost pulses (status transitioned to TimedOut)
    snap.speed.status = SpeedSensorStatus::TimedOut;
    snap.speed.measurementValid = false;
    TEST_ASSERT_FALSE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));
}

// 5. Critical and fatal system health rejection
void test_critical_fatal_health_rejection() {
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1000, 5.0f, 0.1);
    TEST_ASSERT_TRUE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));

    // Warning severity allowed
    snap.health.highestSeverity = FaultSeverity::Warning;
    TEST_ASSERT_TRUE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));

    // Critical rejected
    snap.health.highestSeverity = FaultSeverity::Critical;
    TEST_ASSERT_FALSE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));

    // Fatal rejected
    snap.health.highestSeverity = FaultSeverity::Fatal;
    TEST_ASSERT_FALSE(ControlRuntime::isSnapshotAuthoritative(snap, 1050));
}

// 6. No session tick when snapshot is unauthorized
void test_no_session_tick_when_unauthorized() {
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    runtime.begin();

    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));
    TEST_ASSERT_EQUAL(WorkoutSessionState::Armed, runtime.getSessionSnapshot().state);

    // Pass default unpopulated snapshot
    ApplicationSnapshot emptySnap{};
    runtime.update(emptySnap, 1050);

    // Session remains in Armed, did not advance
    TEST_ASSERT_EQUAL(WorkoutSessionState::Armed, runtime.getSessionSnapshot().state);
    TEST_ASSERT_EQUAL_UINT32(1, runtime.getLostAuthorityCount());

    runtime.end();
}

// 7. No dispatcher delivery when snapshot is unauthorized
void test_no_dispatcher_delivery_when_unauthorized() {
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    runtime.begin();

    runtime.armWorkout(&s_testWorkout, 1000);

    // Provide valid snapshot to activate session and emit intent
    ApplicationSnapshot validSnap = makeAuthoritativeSnapshot(1000, 1.0f, 0.0);
    runtime.update(validSnap, 1000);

    // Targets were staged
    TEST_ASSERT_TRUE(runtime.isSessionActive());
    StagedTargets targets = runtime.getStagedTargets();
    TEST_ASSERT_TRUE(targets.pendingSpeed || targets.pendingIncline);

    // Now send unauthorized snapshot
    ApplicationSnapshot staleSnap = validSnap;
    staleSnap.timestampMs = 500; // 600ms old at nowMs=1100 -> unauthorized!
    runtime.update(staleSnap, 1100);

    // Dispatcher was not ticked for delivery, staged targets preserved
    StagedTargets targetsAfter = runtime.getStagedTargets();
    TEST_ASSERT_EQUAL(targets.pendingSpeed, targetsAfter.pendingSpeed);
    TEST_ASSERT_EQUAL(targets.pendingIncline, targetsAfter.pendingIncline);

    runtime.end();
}

// 8. Existing staged targets survive authority loss without being delivered
void test_staged_targets_survive_authority_loss() {
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    runtime.begin();

    runtime.armWorkout(&s_testWorkout, 1000);

    // Valid tick stages targets
    ApplicationSnapshot validSnap = makeAuthoritativeSnapshot(1000, 1.0f, 0.0);
    runtime.update(validSnap, 1000);
    TEST_ASSERT_TRUE(runtime.getStagedTargets().pendingSpeed);

    // 5 consecutive unauthorized ticks
    for (uint32_t t = 1100; t <= 1500; t += 100) {
        ApplicationSnapshot badSnap{};
        runtime.update(badSnap, t);
        // Staged speed target survives!
        TEST_ASSERT_TRUE(runtime.getStagedTargets().pendingSpeed);
        TEST_ASSERT_EQUAL_FLOAT(10.0f, runtime.getStagedTargets().speedKmh);
    }

    runtime.end();
}

// 9. Session time does not jump after authority recovery
void test_session_time_freeze_after_authority_loss() {
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    runtime.begin();

    runtime.armWorkout(&s_testWorkout, 1000);

    // Run session normally for 5 seconds (1000 -> 6000)
    for (uint32_t t = 1000; t <= 6000; t += 1000) {
        ApplicationSnapshot snap = makeAuthoritativeSnapshot(t, 10.0f, 0.01 * (t - 1000));
        runtime.update(snap, t);
    }
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, runtime.getSessionSnapshot().state);
    uint32_t elapsedBefore = runtime.getSessionSnapshot().totalElapsedTimeMs;
    TEST_ASSERT_EQUAL_UINT32(5000, elapsedBefore);

    // 10 seconds of telemetry blackout (6000 -> 16000)
    for (uint32_t t = 7000; t <= 16000; t += 1000) {
        ApplicationSnapshot badSnap{}; // Unauthorized
        runtime.update(badSnap, t);
    }
    // Lost snapshot policy suspended the session to freeze time
    TEST_ASSERT_TRUE(runtime.isSessionSuspended());

    // Authority recovers at t = 17000 with moving belt
    ApplicationSnapshot recoveredSnap = makeAuthoritativeSnapshot(17000, 10.0f, 0.05);
    runtime.update(recoveredSnap, 17000);

    // Run for 1 more second (17000 -> 18000)
    ApplicationSnapshot nextSnap = makeAuthoritativeSnapshot(18000, 10.0f, 0.06);
    runtime.update(nextSnap, 18000);

    // Total elapsed time must be 6000 ms, NOT 17000 ms!
    uint32_t elapsedAfter = runtime.getSessionSnapshot().totalElapsedTimeMs;
    TEST_ASSERT_EQUAL_UINT32(6000, elapsedAfter);

    runtime.end();
}

// 10. WorkoutEngine cannot be rebound or reset while session is active
void test_workout_engine_rebinding_guards() {
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    runtime.begin();

    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));

    // Activate session
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1000, 1.0f, 0.0);
    runtime.update(snap, 1000);
    TEST_ASSERT_TRUE(runtime.isSessionActive());
    TEST_ASSERT_FALSE(runtime.canSafelyModifyWorkoutEngine());

    // Rebinding while active must be rejected
    TEST_ASSERT_FALSE(runtime.armWorkout(&s_testWorkout2, 1050));

    // Abort session
    TEST_ASSERT_TRUE(runtime.abortWorkout(1100));
    TEST_ASSERT_FALSE(runtime.isSessionActive());
    TEST_ASSERT_TRUE(runtime.canSafelyModifyWorkoutEngine());

    // Rebinding now allowed
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout2, 1200));

    runtime.end();
}

// 11. Controller, session, and dispatcher tick ordering
void test_controller_session_dispatcher_ordering() {
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    runtime.begin();

    runtime.armWorkout(&s_testWorkout, 1000);

    // Valid tick
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1000, 1.0f, 0.0);
    runtime.update(snap, 1000);

    // Controller updated, session transitioned to Running, dispatcher staged targets
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, runtime.getSessionSnapshot().state);
    TEST_ASSERT_TRUE(runtime.getStagedTargets().pendingIncline || runtime.getStagedTargets().pendingSpeed);

    runtime.end();
}

// 12. No physical stop submission is introduced
void test_no_physical_stop_submission() {
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    runtime.begin();

    runtime.armWorkout(&s_testWorkout, 1000);

    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1000, 1.0f, 0.0);
    runtime.update(snap, 1000);

    // Finalize workout
    runtime.finalizeWorkout(2000);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Completed, runtime.getSessionSnapshot().state);

    // Controller snapshot must not show a forced physical stop command from session
    TreadmillControllerSnapshot cSnap = runtime.getControllerSnapshot();
    TEST_ASSERT_FALSE(cSnap.activeRequestValid);

    runtime.end();
}

// 13. Real controller delivery path when controller.isReady() == true
void test_real_controller_ready_delivery_path() {
    // Initialize ConsoleInterface so controller.isReady() returns true
    s_console.begin();
    TEST_ASSERT_TRUE(s_console.isReady());

    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.isControllerReady());
    TEST_ASSERT_FALSE(runtime.isControllerBusy());

    // Arm workout with moving belt to start step 0 (2% incline, 10 km/h)
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1000, 1.0f, 0.0);

    // Initial update: session enters Running and emits intent
    // Dispatcher stages both, sees controller ready & unbusy, delivers Incline first
    runtime.update(snap, 1000);

    // Verify incline was accepted and controller became busy
    TEST_ASSERT_TRUE(runtime.isControllerBusy());
    StagedTargets targetsAfterIncline = runtime.getStagedTargets();
    TEST_ASSERT_FALSE(targetsAfterIncline.pendingIncline); // Incline delivered!
    TEST_ASSERT_TRUE(targetsAfterIncline.pendingSpeed);    // Speed waiting for controller to clear busy

    // Wait for console background command to finish / fail, clearing busy and allowing speed delivery
    uint32_t t = 1020;
    while (t < 2000) {
        StagedTargets cur = runtime.getStagedTargets();
        if (!cur.pendingIncline && !cur.pendingSpeed) {
            break; // Both delivered!
        }
        delay(20);
        t += 20;
        snap.timestampMs = t;
        runtime.update(snap, t);
    }

    // Confirm both targets were delivered and staged targets are completely empty
    StagedTargets finalTargets = runtime.getStagedTargets();
    TEST_ASSERT_FALSE(finalTargets.pendingIncline);
    TEST_ASSERT_FALSE(finalTargets.pendingSpeed);

    runtime.end();
    s_console.end();
}

// 14. Real FreeRTOS task lifecycle pinned to Core 0
void test_control_task_freertos_lifecycle_core0() {
    uint32_t nowMs = millis();
    s_mockOrchestrator.setSnapshot(makeAuthoritativeSnapshot(nowMs, 0.0f, 0.0));

    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());

    TEST_ASSERT_FALSE(runtime.isTaskRunning());
    TEST_ASSERT_TRUE(runtime.startControlTask(&s_mockOrchestrator));
    delay(10);
    TEST_ASSERT_TRUE(runtime.isTaskRunning());

    // Allow multiple 20ms ticks to run on Core 0
    delay(140);

    uint32_t loops = runtime.getLoopCount();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(5, loops);
    TEST_ASSERT_EQUAL_UINT32(0, runtime.getOverrunCount());
    TEST_ASSERT_EQUAL_UINT32(0, runtime.getDeadlineMissCount());

    // Clean teardown and verify task is no longer running
    TEST_ASSERT_TRUE(runtime.stopControlTask(1000));
    TEST_ASSERT_FALSE(runtime.isTaskRunning());

    runtime.end();
}

// 15. Asynchronous orchestrator snapshot ingestion driving session without caching
void test_control_task_orchestrator_snapshot_ingestion() {
    uint32_t t0 = millis();
    ApplicationSnapshot snap0 = makeAuthoritativeSnapshot(t0, 0.0f, 0.0);
    snap0.sequenceNumber = 100;
    s_mockOrchestrator.setSnapshot(snap0);

    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, t0));

    // Session starts in Armed
    TEST_ASSERT_EQUAL(WorkoutSessionState::Armed, runtime.getSessionSnapshot().state);

    // Start background task on Core 0
    TEST_ASSERT_TRUE(runtime.startControlTask(&s_mockOrchestrator));

    // Let task tick a few times while belt is stopped (standstill)
    delay(60);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Armed, runtime.getSessionSnapshot().state);

    // Asynchronously update mock orchestrator with moving belt and advancing sequenceNumber
    uint32_t t1 = millis();
    ApplicationSnapshot snap1 = makeAuthoritativeSnapshot(t1, 10.0f, 0.05);
    snap1.sequenceNumber = 101;
    s_mockOrchestrator.setSnapshot(snap1);

    // Wait for Core 0 task to asynchronously observe snap1 and advance session
    delay(80);

    // Session was actively transitioned from Armed to Running by Core 0 task!
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, runtime.getSessionSnapshot().state);
    TEST_ASSERT_TRUE(runtime.isSessionActive());

    // Next, asynchronously publish an expired/stale snapshot (300ms old > 200ms max age)
    uint32_t t2 = millis();
    ApplicationSnapshot snapStale = snap1;
    snapStale.sequenceNumber = 102;
    snapStale.timestampMs = t2 - 300; // 300ms old -> unauthorized!
    s_mockOrchestrator.setSnapshot(snapStale);

    // Wait for Core 0 task to process stale snapshot
    delay(80);

    // Task must have detected authority loss and suspended the session to freeze time
    TEST_ASSERT_TRUE(runtime.isSessionSuspended());
    TEST_ASSERT_GREATER_THAN_UINT32(0, runtime.getLostAuthorityCount());

    // Clean stop
    TEST_ASSERT_TRUE(runtime.stopControlTask(1000));
    runtime.end();
}

// CSAFE Stop Hierarchy Test 1: InUse -> Paused triggers 1st physical stop (session suspended)
void test_csafe_first_stop_inuse_to_paused_suspends_session() {
    s_console.begin(ConsoleExecutionMode::SoftwareSink);
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));

    // First snapshot: Moving belt, InUse state -> session transitions to Running
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1020, 10.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::InUse);
    runtime.update(snap, 1020);

    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, runtime.getSessionSnapshot().state);
    TEST_ASSERT_EQUAL(0, runtime.getSessionSnapshot().physicalStopCount);

    // Stop 1 transition: CSAFE InUse -> Paused, belt decelerates
    snap = makeAuthoritativeSnapshot(1040, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Paused);
    runtime.update(snap, 1040);

    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, runtime.getSessionSnapshot().state);
    TEST_ASSERT_EQUAL(1, runtime.getSessionSnapshot().physicalStopCount);
    TEST_ASSERT_FALSE(runtime.getSessionSnapshot().continuationWindowActive);

    runtime.end();
}

// CSAFE Stop Hierarchy Test 2: Paused -> Ready triggers 2nd physical stop (continuation window active, 30s)
void test_csafe_second_stop_paused_to_ready_enters_continuation() {
    s_console.begin(ConsoleExecutionMode::SoftwareSink);
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));

    // Move belt -> Running
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1020, 10.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::InUse);
    runtime.update(snap, 1020);

    // 1st stop: InUse -> Paused
    snap = makeAuthoritativeSnapshot(1040, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Paused);
    runtime.update(snap, 1040);
    TEST_ASSERT_EQUAL(1, runtime.getSessionSnapshot().physicalStopCount);

    // 2nd stop: Paused -> Ready
    snap = makeAuthoritativeSnapshot(1060, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Ready);
    runtime.update(snap, 1060);

    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, runtime.getSessionSnapshot().state);
    TEST_ASSERT_EQUAL(2, runtime.getSessionSnapshot().physicalStopCount);
    TEST_ASSERT_TRUE(runtime.getSessionSnapshot().continuationWindowActive);
    TEST_ASSERT_EQUAL_UINT32(30000, runtime.getSessionSnapshot().continuationWindowRemainingMs);

    runtime.end();
}

// CSAFE Stop Hierarchy Test 3: 3rd stop button press when in continuation and CSAFE Ready finalizes session
void test_third_stop_physical_button_event_finalizes_session() {
    s_console.begin(ConsoleExecutionMode::SoftwareSink);
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));

    // Run -> Stop 1 -> Stop 2
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1020, 10.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::InUse);
    runtime.update(snap, 1020);

    snap = makeAuthoritativeSnapshot(1040, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Paused);
    runtime.update(snap, 1040);

    snap = makeAuthoritativeSnapshot(1060, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Ready);
    runtime.update(snap, 1060);
    TEST_ASSERT_TRUE(runtime.getSessionSnapshot().continuationWindowActive);

    // Inject 3rd physical stop button press into console
    PhysicalButtonEvent stopEv{ButtonId::Stop, PhysicalButtonAction::Pressed, 1080, 0, 0xFF, 0xFF};
    TEST_ASSERT_TRUE(s_console.injectPhysicalButtonEvent(stopEv));

    // Update with CSAFE still Ready
    snap = makeAuthoritativeSnapshot(1080, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Ready);
    runtime.update(snap, 1080);

    TEST_ASSERT_EQUAL(3, runtime.getSessionSnapshot().physicalStopCount);
    TEST_ASSERT_TRUE(runtime.getSessionSnapshot().state == WorkoutSessionState::Completed ||
                     runtime.getSessionSnapshot().state == WorkoutSessionState::Aborted);

    runtime.end();
}

// CSAFE Stop Hierarchy Test 4: Generic API stop does NOT increment physicalStopCount or register physical stop
void test_generic_api_stop_does_not_call_register_physical_stop() {
    s_console.begin(ConsoleExecutionMode::SoftwareSink);
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));

    // Running state
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1020, 10.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::InUse);
    runtime.update(snap, 1020);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, runtime.getSessionSnapshot().state);

    // Stage generic API stop command
    ControlCommand stopCmd{};
    stopCmd.type = ControlCommandType::Stop;
    stopCmd.timestampMs = 1040;
    TEST_ASSERT_TRUE(runtime.stageCommand(stopCmd));

    // When the control task drains the command while CSAFE is still InUse (before hardware decels),
    // session must NOT register a physical stop
    runtime.update(snap, 1040);
    TEST_ASSERT_EQUAL(0, runtime.getSessionSnapshot().physicalStopCount);

    runtime.end();
}

// CSAFE Stop Hierarchy Test 5: Initial connection and reconnection baselining doesn't cause spurious transitions
void test_csafe_initial_and_reconnection_baselining_no_spurious_stop() {
    s_console.begin(ConsoleExecutionMode::SoftwareSink);
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));

    // Initial snapshot: Ready state (fresh connection baseline, no transition)
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1020, 0.0f, 0.0);
    setCsafeSnapshotState(snap, CsafeMachineState::Ready);
    runtime.update(snap, 1020);
    TEST_ASSERT_EQUAL(0, runtime.getSessionSnapshot().physicalStopCount);

    // Move to InUse
    snap = makeAuthoritativeSnapshot(1040, 10.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::InUse);
    runtime.update(snap, 1040);
    TEST_ASSERT_EQUAL(0, runtime.getSessionSnapshot().physicalStopCount);

    // Drop connection (safety key pull or UART silence: online = false)
    snap = makeAuthoritativeSnapshot(1060, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Unknown, false, false);
    runtime.update(snap, 1060);
    TEST_ASSERT_EQUAL(0, runtime.getSessionSnapshot().physicalStopCount);

    // Reconnect in Ready state -> must re-baseline, NOT trigger Paused->Ready transition
    snap = makeAuthoritativeSnapshot(1080, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Ready, true, true);
    runtime.update(snap, 1080);
    TEST_ASSERT_EQUAL(0, runtime.getSessionSnapshot().physicalStopCount);

    runtime.end();
}

// CSAFE Stop Hierarchy Test 6: Continuation window countdown and expiration after 30 seconds
void test_continuation_window_expires_after_30s() {
    s_console.begin(ConsoleExecutionMode::SoftwareSink);
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));

    // Run -> Stop 1 -> Stop 2
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1020, 10.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::InUse);
    runtime.update(snap, 1020);

    snap = makeAuthoritativeSnapshot(1040, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Paused);
    runtime.update(snap, 1040);

    snap = makeAuthoritativeSnapshot(1060, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Ready);
    runtime.update(snap, 1060);
    TEST_ASSERT_TRUE(runtime.getSessionSnapshot().continuationWindowActive);
    TEST_ASSERT_EQUAL_UINT32(30000, runtime.getSessionSnapshot().continuationWindowRemainingMs);

    // Advance 15 seconds: continuation window still active with ~15000ms remaining
    snap = makeAuthoritativeSnapshot(16060, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Ready);
    runtime.update(snap, 16060);
    TEST_ASSERT_TRUE(runtime.getSessionSnapshot().continuationWindowActive);
    TEST_ASSERT_EQUAL_UINT32(15000, runtime.getSessionSnapshot().continuationWindowRemainingMs);

    // Advance past 30 seconds total (30001 ms elapsed): continuation window expires
    snap = makeAuthoritativeSnapshot(31061, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Ready);
    runtime.update(snap, 31061);
    TEST_ASSERT_FALSE(runtime.getSessionSnapshot().continuationWindowActive);
    TEST_ASSERT_EQUAL_UINT32(0, runtime.getSessionSnapshot().continuationWindowRemainingMs);

    runtime.end();
}

// 22. ControlCommandType::Resume stages physical speed target and does NOT bypass WorkoutSession confirmation
void test_control_command_resume_stages_speed_not_bypass_session() {
    s_console.begin(ConsoleExecutionMode::SoftwareSink);
    ControlRuntime runtime(s_console, s_calibration, s_diagnostics);
    TEST_ASSERT_TRUE(runtime.begin());
    TEST_ASSERT_TRUE(runtime.armWorkout(&s_testWorkout, 1000));

    // First snapshot: Moving belt, InUse state -> session transitions to Running
    ApplicationSnapshot snap = makeAuthoritativeSnapshot(1020, 10.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::InUse);
    runtime.update(snap, 1020);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, runtime.getSessionSnapshot().state);

    // Stop 1: InUse -> Paused -> Suspended
    snap = makeAuthoritativeSnapshot(1040, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Paused);
    runtime.update(snap, 1040);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, runtime.getSessionSnapshot().state);

    // Issue Resume API command
    ControlCommand resumeCmd{};
    resumeCmd.type = ControlCommandType::Resume;
    resumeCmd.timestampMs = 1060;
    TEST_ASSERT_TRUE(runtime.stageCommand(resumeCmd));

    // Update with CSAFE Starting, belt 0.0 km/h (audible countdown phase)
    snap = makeAuthoritativeSnapshot(1060, 0.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::Starting);
    runtime.update(snap, 1060);

    // Session MUST remain Suspended - resume command must not bypass confirmation!
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, runtime.getSessionSnapshot().state);

    // Treadmill finishes countdown: transitions to InUse with moving belt
    snap = makeAuthoritativeSnapshot(1100, 5.0f, 0.05);
    setCsafeSnapshotState(snap, CsafeMachineState::InUse);
    runtime.update(snap, 1100);

    // Now session confirms resume and enters Running
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, runtime.getSessionSnapshot().state);

    runtime.end();
}

void run_all_control_runtime_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_control_runtime_single_owner_path);
    RUN_TEST(test_snapshot_age_rollover_safety);
    RUN_TEST(test_operational_stopped_snapshot_acceptance);
    RUN_TEST(test_stale_moving_snapshot_rejection);
    RUN_TEST(test_critical_fatal_health_rejection);
    RUN_TEST(test_no_session_tick_when_unauthorized);
    RUN_TEST(test_no_dispatcher_delivery_when_unauthorized);
    RUN_TEST(test_staged_targets_survive_authority_loss);
    RUN_TEST(test_session_time_freeze_after_authority_loss);
    RUN_TEST(test_workout_engine_rebinding_guards);
    RUN_TEST(test_controller_session_dispatcher_ordering);
    RUN_TEST(test_no_physical_stop_submission);
    RUN_TEST(test_real_controller_ready_delivery_path);
    RUN_TEST(test_control_task_freertos_lifecycle_core0);
    RUN_TEST(test_control_task_orchestrator_snapshot_ingestion);
    RUN_TEST(test_csafe_first_stop_inuse_to_paused_suspends_session);
    RUN_TEST(test_csafe_second_stop_paused_to_ready_enters_continuation);
    RUN_TEST(test_third_stop_physical_button_event_finalizes_session);
    RUN_TEST(test_generic_api_stop_does_not_call_register_physical_stop);
    RUN_TEST(test_csafe_initial_and_reconnection_baselining_no_spurious_stop);
    RUN_TEST(test_continuation_window_expires_after_30s);
    RUN_TEST(test_control_command_resume_stages_speed_not_bypass_session);
    UNITY_END();
}

#if defined(ARDUINO)
void setup() {
    Serial.begin(115200);
    uint32_t startMs = millis();
    while (!Serial && (millis() - startMs) < 6000) {
        delay(50);
    }
    delay(1000);
    run_all_control_runtime_tests();
}
void loop() {}
#else
int main(int argc, char** argv) {
    run_all_control_runtime_tests();
    return 0;
}
#endif

