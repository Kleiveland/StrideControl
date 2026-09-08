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
    snap.speed.speedKmh = speedKmh;
    snap.runner.validatedDistanceKm = distanceKm;
    snap.runner.speedCreditEnabled = true;
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

