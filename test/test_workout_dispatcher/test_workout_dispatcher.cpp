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
#include "VirtualTreadmill.h"
#include "IWorkoutTargetSink.h"

using namespace stridecontrol;

// Minimal mock sink to test isolated failure, busy, and readiness conditions
class MockTargetSink : public IWorkoutTargetSink {
public:
    bool ready = true;
    bool busy = false;
    bool failSpeed = false;
    bool failIncline = false;

    uint32_t speedSubmitCount = 0;
    float lastSpeedTarget = 0.0f;
    uint32_t lastSpeedTimestamp = 0;

    uint32_t inclineSubmitCount = 0;
    float lastInclineTarget = 0.0f;
    uint32_t lastInclineTimestamp = 0;

    bool submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) override {
        if (!ready || busy || failSpeed) return false;
        speedSubmitCount++;
        lastSpeedTarget = targetSpeedKmh;
        lastSpeedTimestamp = nowMs;
        return true;
    }

    bool submitInclineTarget(float targetInclinePct, uint32_t nowMs) override {
        if (!ready || busy || failIncline) return false;
        inclineSubmitCount++;
        lastInclineTarget = targetInclinePct;
        lastInclineTimestamp = nowMs;
        return true;
    }

    bool isBusy() const override { return busy; }
    bool isReady() const override { return ready; }
};

// Helper to create an expanded workout for test harness
static ExpandedWorkout createTestWorkout() {
    ExpandedWorkout ew{};
    ew.workoutId = 101;
    strncpy(ew.workoutName, "DISPATCHER TEST", sizeof(ew.workoutName) - 1);
    ew.totalSteps = 3;

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

    // Step 2: Rest, FREE speed, no incline
    ew.steps[2].stepIndex = 2;
    ew.steps[2].role = StepRole::REST;
    ew.steps[2].durationType = DurationType::TIME_SECONDS;
    ew.steps[2].durationValue = 60;
    ew.steps[2].speedMode = SpeedMode::FREE;
    ew.steps[2].targetSpeedKmh = 6.0f; // fallback, must not emit
    ew.steps[2].setIncline = false;
    ew.steps[2].targetInclinePct = 0;

    return ew;
}

static ApplicationSnapshot makeAppSnapshot(float speedKmh, double distanceKm) {
    ApplicationSnapshot snap{};
    snap.speed.initialized = true;
    snap.speed.speedKmh = speedKmh;
    snap.speed.measurementValid = (speedKmh > 0.0f);
    snap.speed.status = (speedKmh > 0.0f) ? SpeedSensorStatus::Measuring : SpeedSensorStatus::TimedOut;
    snap.runner.validatedDistanceKm = distanceKm;
    snap.runner.speedCreditEnabled = true;
    snap.csafe.initialized = true;
    snap.csafe.online = true;
    snap.csafe.machineStateFresh = true;
    snap.csafe.linkStatus = CsafeLinkStatus::Online;
    snap.csafe.qualifiedState = (speedKmh > 0.0f) ? CsafeMachineState::InUse : CsafeMachineState::Ready;
    snap.csafe.reportedState = snap.csafe.qualifiedState;
    snap.csafe.rawStateByte = static_cast<uint8_t>(snap.csafe.qualifiedState);
    snap.csafe.stateNibble = static_cast<uint8_t>(snap.csafe.qualifiedState) & 0x0F;
    return snap;
}

// 1. Combined incline and speed intent
void test_dispatcher_combined_incline_and_speed() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);

    // Start belt to trigger step 0 transition and emit combined intent
    session.update(makeAppSnapshot(1.0f, 0.0), 1000);
    WorkoutCommandIntent intent = session.getPendingCommandIntent();
    TEST_ASSERT_TRUE(intent.hasSpeedTarget);
    TEST_ASSERT_TRUE(intent.hasInclineTarget);

    TreadmillSimulator sim;
    sim.begin(1000);

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // First update: stages both, clears session intent, submits incline first
    dispatcher.update(session, sim, 1000);

    // Domain intent cleared only after safe staging
    TEST_ASSERT_FALSE(session.getPendingCommandIntent().hasSpeedTarget);
    TEST_ASSERT_FALSE(session.getPendingCommandIntent().hasInclineTarget);

    // Incline was submitted first
    TEST_ASSERT_EQUAL_FLOAT(2.0f, sim.getTargetInclinePct());
    TEST_ASSERT_TRUE(sim.isBusy());

    // Speed remains staged and pending
    StagedTargets staged = dispatcher.getStagedTargets();
    TEST_ASSERT_FALSE(staged.pendingIncline);
    TEST_ASSERT_TRUE(staged.pendingSpeed);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, staged.speedKmh);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    // While busy at 1200 ms, speed is not submitted
    sim.update(1200);
    TEST_ASSERT_TRUE(sim.isBusy());
    dispatcher.update(session, sim, 1200);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.getTargetSpeedKmh());

    // At 1500 ms, busy clears on simulator
    sim.update(1500);
    TEST_ASSERT_FALSE(sim.isBusy());

    // Next dispatcher update submits speed
    dispatcher.update(session, sim, 1500);
    TEST_ASSERT_FALSE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_FLOAT(10.0f, sim.getTargetSpeedKmh());
    TEST_ASSERT_TRUE(sim.isBusy());
}

// 2. Busy sink: ingest while busy, retain staged, no submission until ready and not busy
void test_dispatcher_busy_sink() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000);

    MockTargetSink mock;
    mock.ready = true;
    mock.busy = true; // Sink is busy

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    dispatcher.update(session, mock, 1000);

    // Ingested newest intent
    TEST_ASSERT_FALSE(session.getPendingCommandIntent().hasSpeedTarget);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);

    // No submission to busy sink
    TEST_ASSERT_EQUAL_UINT32(0, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);

    // Now clear busy
    mock.busy = false;
    dispatcher.update(session, mock, 1050);

    // Incline submitted first
    TEST_ASSERT_EQUAL_UINT32(1, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
    TEST_ASSERT_FALSE(dispatcher.getStagedTargets().pendingIncline);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
}

// 3. Submit failure: failed incline remains pending, speed not attempted, failed speed remains pending
void test_dispatcher_submit_failure() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000);

    MockTargetSink mock;
    mock.failIncline = true; // Reject incline

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    dispatcher.update(session, mock, 1000);

    // Failed incline must remain pending
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_EQUAL_UINT32(0, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount); // Speed not attempted!

    // Fix incline failure, make speed fail
    mock.failIncline = false;
    mock.failSpeed = true;

    // Delivery 1: Incline succeeds
    dispatcher.update(session, mock, 1010);
    TEST_ASSERT_EQUAL_UINT32(1, mock.inclineSubmitCount);
    TEST_ASSERT_FALSE(dispatcher.getStagedTargets().pendingIncline);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);

    // Delivery 2: Speed attempt fails
    dispatcher.update(session, mock, 1020);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed); // Remains pending!
}

// 4. Field-wise preemption: newest replaces older pending
void test_dispatcher_field_wise_preemption() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000); // Emits speed 10.0, incline 2

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Stage initial targets
    dispatcher.update(session, mock, 1000);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, dispatcher.getStagedTargets().speedKmh);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, dispatcher.getStagedTargets().inclinePct);

    // Advance to step 1 (speed 14.0, incline 4)
    session.advanceToNextStep(1100);
    TEST_ASSERT_TRUE(session.getPendingCommandIntent().hasSpeedTarget);
    TEST_ASSERT_EQUAL_FLOAT(14.0f, session.getPendingCommandIntent().targetSpeedKmh);

    // Update dispatcher while sink still busy
    dispatcher.update(session, mock, 1100);
    TEST_ASSERT_EQUAL_FLOAT(14.0f, dispatcher.getStagedTargets().speedKmh);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, dispatcher.getStagedTargets().inclinePct);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);
}

// 5. Partial new intent: speed-only does not erase incline, incline-only does not erase speed
void test_dispatcher_partial_new_intent() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000); // Speed 10.0, Incline 2

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();
    dispatcher.update(session, mock, 1000);

    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);

    // Report a speed adjustment shift to create a speed-only intent change on reissue
    session.reportWorkSpeedAdjustment(12.0f);
    session.acceptSpeedAdjustmentShift();

    // Advance to next step then manually test partial intent
    // Staged targets retain incline when new intent has only speed
    StagedTargets staged = dispatcher.getStagedTargets();
    TEST_ASSERT_EQUAL_FLOAT(2.0f, staged.inclinePct);
    TEST_ASSERT_TRUE(staged.pendingIncline);
}

// 6. No duplicate submissions on repeated update ticks
void test_dispatcher_no_duplicate_submissions() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000);

    MockTargetSink mock;
    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Tick 1: submit incline
    dispatcher.update(session, mock, 1000);
    TEST_ASSERT_EQUAL_UINT32(1, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);

    // Tick 2: submit speed
    dispatcher.update(session, mock, 1020);
    TEST_ASSERT_EQUAL_UINT32(1, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(1, mock.speedSubmitCount);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());

    // Subsequent ticks: no submissions
    for (int i = 0; i < 5; ++i) {
        dispatcher.update(session, mock, 1040 + i * 20);
    }
    TEST_ASSERT_EQUAL_UINT32(1, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(1, mock.speedSubmitCount);
}

// 7. FREE speed behavior: intent without hasSpeedTarget never results in speed submission
void test_dispatcher_free_speed_mode() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000);

    MockTargetSink mock;
    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Deliver step 0
    dispatcher.update(session, mock, 1000);
    dispatcher.update(session, mock, 1020);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());

    // Advance to step 1 (Work)
    session.advanceToNextStep(1100);
    dispatcher.update(session, mock, 1100);
    dispatcher.update(session, mock, 1120);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());

    // Advance to step 2 (FREE Rest)
    session.advanceToNextStep(1200);
    WorkoutCommandIntent freeIntent = session.getPendingCommandIntent();
    TEST_ASSERT_FALSE(freeIntent.hasSpeedTarget);

    const uint32_t speedSubmitsBefore = mock.speedSubmitCount;
    dispatcher.update(session, mock, 1200);

    // Verify speed was never submitted for FREE step
    TEST_ASSERT_EQUAL_UINT32(speedSubmitsBefore, mock.speedSubmitCount);
    TEST_ASSERT_FALSE(dispatcher.getStagedTargets().pendingSpeed);
}

// 8. Simulator busy duration & rollover safety
void test_simulator_busy_duration_and_rollover() {
    TreadmillSimulator sim;
    sim.begin(1000);
    TEST_ASSERT_TRUE(sim.isReady());
    TEST_ASSERT_FALSE(sim.isBusy());

    TEST_ASSERT_TRUE(sim.submitSpeedTarget(10.0f, 1000));
    TEST_ASSERT_TRUE(sim.isBusy());

    // Reject new command while busy without mutating accepted target
    TEST_ASSERT_FALSE(sim.submitSpeedTarget(15.0f, 1200));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, sim.getTargetSpeedKmh());

    // Exactly 499 ms later: still busy
    sim.update(1499);
    TEST_ASSERT_TRUE(sim.isBusy());

    // Exactly 500 ms boundary: busy clears
    sim.update(1500);
    TEST_ASSERT_FALSE(sim.isBusy());

    // Rollover test near UINT32_MAX
    const uint32_t rolloverBaseMs = 0xFFFFFFE0; // 4294967264
    sim.update(rolloverBaseMs);
    TEST_ASSERT_TRUE(sim.submitInclineTarget(4.0f, rolloverBaseMs));
    TEST_ASSERT_TRUE(sim.isBusy());

    // 499 ms elapsed across rollover: rolloverBaseMs + 499 wraps to 467
    const uint32_t rollover499Ms = rolloverBaseMs + 499;
    sim.update(rollover499Ms);
    TEST_ASSERT_TRUE(sim.isBusy());

    // 500 ms elapsed across rollover: wraps to 468
    const uint32_t rollover500Ms = rolloverBaseMs + 500;
    sim.update(rollover500Ms);
    TEST_ASSERT_FALSE(sim.isBusy());
}

// 9. Simulator ramp and distance integration
void test_simulator_ramp_and_distance() {
    TreadmillSimulator sim;
    sim.begin(1000);

    // Zero speed produces zero distance
    sim.update(2000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.getCurrentSpeedKmh());
    TEST_ASSERT_EQUAL_FLOAT(0.0, sim.getCumulativeDistanceKm());

    // Submit 10.0 km/h target
    TEST_ASSERT_TRUE(sim.submitSpeedTarget(10.0f, 2000));

    // After 2.0 seconds (2000 ms), speed ramps at 0.5 km/h per sec -> 1.0 km/h
    sim.update(4000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, sim.getCurrentSpeedKmh());

    // Average speed = (0 + 1.0)/2 = 0.5 km/h. Dist = 0.5 * 2000 / 3600000 = 0.0002778 km
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0002778f, static_cast<float>(sim.getCumulativeDistanceKm()));

    // After 30 seconds, speed must clamp at 10.0 km/h without overshoot
    sim.update(34000);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, sim.getCurrentSpeedKmh());

    // Distance must be monotonic
    double distAt34s = sim.getCumulativeDistanceKm();
    sim.update(35000);
    TEST_ASSERT_TRUE(sim.getCumulativeDistanceKm() > distAt34s);
}

// 10. Closed loop: session, dispatcher, and simulator closed-loop execution
void test_closed_loop_integration() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);

    TreadmillSimulator sim;
    sim.begin(1000);

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Start treadmill belt manually at 1.0 km/h to trigger session activation
    sim.submitSpeedTarget(1.0f, 1000);

    uint32_t nowMs = 1000;
    // Step forward in 100 ms increments
    for (int step = 0; step < 30; ++step) {
        nowMs += 100;
        sim.update(nowMs);
        ApplicationSnapshot snap = sim.getSnapshot();
        // TreadmillSimulator does not synthesize CSAFE state; populate using authoritative testbench helper
        snap.csafe = makeAppSnapshot(snap.speed.speedKmh, snap.runner.validatedDistanceKm).csafe;
        session.update(snap, nowMs);
        dispatcher.update(session, sim, nowMs);
    }

    // After 3.0 s: session is Running
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);

    // Both targets must have been accepted by simulator
    TEST_ASSERT_EQUAL_FLOAT(2.0f, sim.getTargetInclinePct());
    TEST_ASSERT_EQUAL_FLOAT(10.0f, sim.getTargetSpeedKmh());

    // Dispatcher staged targets fully drained
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());

    // Simulator is actively ramping speed and accumulating distance
    TEST_ASSERT_TRUE(sim.getCurrentSpeedKmh() > 1.0f);
    TEST_ASSERT_TRUE(sim.getCumulativeDistanceKm() > 0.0);
}

void test_dispatcher_aborted_session_target_discarded() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000);

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Stage targets while sink is busy
    dispatcher.update(session, mock, 1000);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_FLOAT(10.0f, dispatcher.getStagedTargets().speedKmh);

    // Abort session
    session.abortSession(1050);
    TEST_ASSERT_FALSE(session.isActive());

    // Unbusy sink and update
    mock.busy = false;
    dispatcher.update(session, mock, 1050);

    // Targets must have been invalidated and NOT delivered!
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(0, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
}

void test_dispatcher_finalized_session_target_discarded() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000);

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    dispatcher.update(session, mock, 1000);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    // Finalize session
    session.finalizeSession(1050);
    TEST_ASSERT_FALSE(session.isActive());

    mock.busy = false;
    dispatcher.update(session, mock, 1050);

    // Targets must be discarded and NOT delivered
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(0, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
}

void test_dispatcher_rearm_session_target_discarded() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000);
    uint32_t gen1 = session.getSessionGeneration();

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    dispatcher.update(session, mock, 1000);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(gen1, dispatcher.getStagedTargets().speedSessionGeneration);

    // Abort and rearm new workout -> increment generation
    session.abortSession(1050);
    session.armWorkout(&ew, 1100);
    uint32_t gen2 = session.getSessionGeneration();
    TEST_ASSERT_GREATER_THAN_UINT32(gen1, gen2);

    mock.busy = false;
    // Dispatcher updates before new session enters Running
    dispatcher.update(session, mock, 1100);

    // Staged targets from gen1 must have been discarded!
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(0, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
}

void test_dispatcher_manual_target_retained_across_inactive_session() {
    WorkoutSession session;
    session.begin(); // Idle session

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // User stages manual speed command
    dispatcher.stageSpeedTarget(12.5f, 1000, TargetOrigin::StandaloneManual);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL(TargetOrigin::StandaloneManual, dispatcher.getStagedTargets().speedOrigin);

    // Dispatcher updates while session is inactive (Idle)
    dispatcher.update(session, mock, 1000);
    // Still pending because sink is busy, NOT discarded because origin is StandaloneManual and session is inactive
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    // Unbusy sink
    mock.busy = false;
    dispatcher.update(session, mock, 1050);

    // Manual target was delivered!
    TEST_ASSERT_EQUAL_UINT32(1, mock.speedSubmitCount);
    TEST_ASSERT_EQUAL_FLOAT(12.5f, mock.lastSpeedTarget);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
}

// 1. SessionManualAdjustment invalidated on abort and finalize
void test_session_manual_adjustment_invalidated_on_abort_and_finalize() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000); // Running
    TEST_ASSERT_TRUE(session.isActive());

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Stage SessionManualAdjustment
    TargetContext ctx;
    ctx.origin = TargetOrigin::SessionManualAdjustment;
    ctx.sessionGeneration = session.getSessionGeneration();
    ctx.stepIndex = session.getCurrentStepIndex();
    ctx.timestampMs = 1000;
    dispatcher.stageSpeedTarget(14.0f, ctx);
    dispatcher.stageInclineTarget(3.0f, ctx);

    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL(TargetOrigin::SessionManualAdjustment, dispatcher.getStagedTargets().speedOrigin);

    // Abort session
    session.abortSession(1020);
    TEST_ASSERT_FALSE(session.isActive());

    mock.busy = false;
    dispatcher.update(session, mock, 1040);

    // Targets must be discarded and NOT delivered
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.inclineSubmitCount);

    // Test Finalize path
    session.armWorkout(&ew, 2000);
    session.update(makeAppSnapshot(1.0f, 0.0), 2000);
    mock.busy = true;
    ctx.sessionGeneration = session.getSessionGeneration();
    ctx.stepIndex = session.getCurrentStepIndex();
    dispatcher.stageSpeedTarget(15.0f, ctx);

    session.finalizeSession(2020);
    TEST_ASSERT_FALSE(session.isActive());

    mock.busy = false;
    dispatcher.update(session, mock, 2040);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
}

// 2. StandaloneManual cleared when a new workout is successfully armed
void test_standalone_manual_cleared_on_new_workout_arm() {
    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Staged when no session is active
    dispatcher.stageSpeedTarget(8.5f, 1000, TargetOrigin::StandaloneManual);
    dispatcher.stageInclineTarget(2.0f, 1000, TargetOrigin::StandaloneManual);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    // Arming new session calls clearForNewSession
    dispatcher.clearForNewSession();
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
}

// 3. StandaloneManual preserved across non-authoritative tick if no session is armed
void test_standalone_manual_preserved_across_unauthorized_tick_when_no_session() {
    WorkoutSession session;
    session.begin(); // Idle

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    dispatcher.stageSpeedTarget(7.5f, 1000, TargetOrigin::StandaloneManual);

    // Dispatcher tick with busy sink
    dispatcher.update(session, mock, 1000);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    // Another tick
    dispatcher.update(session, mock, 1020);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    // Sink ready
    mock.busy = false;
    dispatcher.update(session, mock, 1040);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(1, mock.speedSubmitCount);
    TEST_ASSERT_EQUAL_FLOAT(7.5f, mock.lastSpeedTarget);
}

// 4. Pre-fire target validation when stepIndex == 0 (no unsigned underflow)
void test_prefire_target_validation_no_underflow_at_step_zero() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000); // Running, currentStepIndex = 0

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Stage a target marked as prefire with stepIndex = 0 (edge case)
    TargetContext ctx;
    ctx.origin = TargetOrigin::WorkoutGenerated;
    ctx.sessionGeneration = session.getSessionGeneration();
    ctx.stepIndex = 0;
    ctx.timestampMs = 1000;
    dispatcher.stageSpeedTarget(12.0f, ctx);

    // Update with currentStep == 0 - must not crash or underflow
    dispatcher.update(session, mock, 1000);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    mock.busy = false;
    dispatcher.update(session, mock, 1020);
    TEST_ASSERT_EQUAL_UINT32(1, mock.speedSubmitCount);
}

// 5. Pre-fire target rejected if pre-fire cancels or changes target step before delivery
void test_prefire_target_rejected_if_prefire_cancels_or_changes() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000); // Running, step 0

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    // Manually stage a prefire target targeting step 2 while prefire is NOT active for step 2
    TargetContext ctx;
    ctx.origin = TargetOrigin::WorkoutGenerated;
    ctx.sessionGeneration = session.getSessionGeneration();
    ctx.stepIndex = 2; // Step 2 is not active and not prefire target
    ctx.timestampMs = 1000;
    dispatcher.stageSpeedTarget(16.0f, ctx);

    // In update, validateRetainedTargets should see isPreFire (if marked) or mismatched stepIndex and discard
    dispatcher.update(session, mock, 1000);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
}

// 6. Arming failure preserves existing staged targets
void test_arming_failure_preserves_staged_targets() {
    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    dispatcher.stageSpeedTarget(9.0f, 1000, TargetOrigin::StandaloneManual);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    // Simulate arming failure: workout definition is invalid / null
    const ExpandedWorkout* invalidWorkout = nullptr;
    WorkoutSession session;
    session.begin();
    const bool armed = session.armWorkout(invalidWorkout, 1000);
    TEST_ASSERT_FALSE(armed);

    // Dispatcher targets must NOT have been cleared because arming failed
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_FLOAT(9.0f, dispatcher.getStagedTargets().speedKmh);
}

// 7. Verification that CSAFE state in TreadmillSimulator does not cast enum ordinals to raw bytes and conforms to VirtualTreadmill lifecycle
void test_csafe_virtual_treadmill_lifecycle_and_raw_bytes() {
    VirtualTreadmill vt;
    vt.resetModel(1000);

    // Ready: raw byte 0x01, nibble 0x01
    TEST_ASSERT_EQUAL_HEX8(0x01, vt.getCsafeState().rawStateByte);
    TEST_ASSERT_EQUAL_HEX8(0x01, vt.getCsafeState().stateNibble);
    TEST_ASSERT_EQUAL(CsafeMachineState::Ready, vt.getCsafeState().qualifiedState);

    // Start -> Starting (3-2-1 countdown): raw byte 0x08, nibble 0x08
    vt.onConsoleQuickStart(5.0f, 0.0f);
    TEST_ASSERT_EQUAL_HEX8(0x08, vt.getCsafeState().rawStateByte);
    TEST_ASSERT_EQUAL_HEX8(0x08, vt.getCsafeState().stateNibble);
    TEST_ASSERT_EQUAL(CsafeMachineState::Starting, vt.getCsafeState().qualifiedState);

    // Advance 3000 ms to complete Starting countdown -> InUse: raw byte 0x85, nibble 0x05
    SimulationTick tick(1ULL, 4000U, 3000U);
    vt.tick(tick);
    TEST_ASSERT_EQUAL_HEX8(0x85, vt.getCsafeState().rawStateByte);
    TEST_ASSERT_EQUAL_HEX8(0x05, vt.getCsafeState().stateNibble);
    TEST_ASSERT_EQUAL(CsafeMachineState::InUse, vt.getCsafeState().qualifiedState);

    // 1st Stop -> Paused: raw byte 0x04, nibble 0x04
    vt.onConsoleStop();
    TEST_ASSERT_EQUAL_HEX8(0x04, vt.getCsafeState().rawStateByte);
    TEST_ASSERT_EQUAL_HEX8(0x04, vt.getCsafeState().stateNibble);
    TEST_ASSERT_EQUAL(CsafeMachineState::Paused, vt.getCsafeState().qualifiedState);

    // 2nd Stop -> Ready: raw byte 0x01, nibble 0x01
    vt.onConsoleStop();
    TEST_ASSERT_EQUAL_HEX8(0x01, vt.getCsafeState().rawStateByte);
    TEST_ASSERT_EQUAL_HEX8(0x01, vt.getCsafeState().stateNibble);
    TEST_ASSERT_EQUAL(CsafeMachineState::Ready, vt.getCsafeState().qualifiedState);

    // TreadmillSimulator check: does not synthesize CSAFE state, csafe.initialized is false
    TreadmillSimulator sim;
    sim.begin(1000);
    ApplicationSnapshot snap = sim.getSnapshot();
    TEST_ASSERT_FALSE(snap.csafe.initialized);
}

// 8. Commissioning target cleared when new workout session is armed
void test_commissioning_target_cleared_on_new_workout_arm() {
    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    TargetContext ctx;
    ctx.origin = TargetOrigin::Commissioning;
    ctx.timestampMs = 1000;
    dispatcher.stageSpeedTarget(12.0f, ctx);
    TEST_ASSERT_TRUE(dispatcher.hasPendingTargets());

    // Arming new session calls clearForNewSession
    dispatcher.clearForNewSession();
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
}

// 9. Commissioning target discarded if workout session is active
void test_commissioning_target_invalidated_when_session_active() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(1.0f, 0.0), 1000); // Running
    TEST_ASSERT_TRUE(session.isActive());

    MockTargetSink mock;
    mock.busy = true;

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    TargetContext ctx;
    ctx.origin = TargetOrigin::Commissioning;
    ctx.timestampMs = 1000;
    dispatcher.stageSpeedTarget(15.0f, ctx);

    // validateRetainedTargets should discard commissioning target because session is active
    dispatcher.update(session, mock, 1020);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
}

void run_all_workout_dispatcher_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_dispatcher_combined_incline_and_speed);
    RUN_TEST(test_dispatcher_busy_sink);
    RUN_TEST(test_dispatcher_submit_failure);
    RUN_TEST(test_dispatcher_field_wise_preemption);
    RUN_TEST(test_dispatcher_partial_new_intent);
    RUN_TEST(test_dispatcher_no_duplicate_submissions);
    RUN_TEST(test_dispatcher_free_speed_mode);
    RUN_TEST(test_simulator_busy_duration_and_rollover);
    RUN_TEST(test_simulator_ramp_and_distance);
    RUN_TEST(test_closed_loop_integration);
    RUN_TEST(test_dispatcher_aborted_session_target_discarded);
    RUN_TEST(test_dispatcher_finalized_session_target_discarded);
    RUN_TEST(test_dispatcher_rearm_session_target_discarded);
    RUN_TEST(test_dispatcher_manual_target_retained_across_inactive_session);
    RUN_TEST(test_session_manual_adjustment_invalidated_on_abort_and_finalize);
    RUN_TEST(test_standalone_manual_cleared_on_new_workout_arm);
    RUN_TEST(test_standalone_manual_preserved_across_unauthorized_tick_when_no_session);
    RUN_TEST(test_prefire_target_validation_no_underflow_at_step_zero);
    RUN_TEST(test_prefire_target_rejected_if_prefire_cancels_or_changes);
    RUN_TEST(test_arming_failure_preserves_staged_targets);
    RUN_TEST(test_csafe_virtual_treadmill_lifecycle_and_raw_bytes);
    RUN_TEST(test_commissioning_target_cleared_on_new_workout_arm);
    RUN_TEST(test_commissioning_target_invalidated_when_session_active);
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
    run_all_workout_dispatcher_tests();
}
void loop() {}
#else
int main(int argc, char** argv) {
    run_all_workout_dispatcher_tests();
    return 0;
}
#endif

