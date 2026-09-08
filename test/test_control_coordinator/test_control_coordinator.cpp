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
#include "ConsoleInterface.h"
#include "SpeedCalibration.h"
#include "DiagnosticsService.h"

using namespace stridecontrol;

// Controllable mock sink for deterministic forwarding and failure injection
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

static ExpandedWorkout createCoordinatorTestWorkout() {
    ExpandedWorkout ew{};
    ew.workoutId = 202;
    strncpy(ew.workoutName, "COORDINATOR TEST", sizeof(ew.workoutName) - 1);
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
    ew.steps[2].targetSpeedKmh = 6.0f;
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

// 1. Adapter Forwarding: Verify all 4 methods forward faithfully to underlying controller
void test_adapter_forwarding() {
    ConsoleInterface console;
    SpeedCalibration calibration;
    DiagnosticsService diagnostics;
    TreadmillController controller(console, calibration, diagnostics);
    TreadmillControllerAdapter adapter(controller);

    // Prior to begin(), controller is uninitialized
    TEST_ASSERT_FALSE(adapter.isReady());
    TEST_ASSERT_EQUAL(controller.isReady(), adapter.isReady());
    TEST_ASSERT_FALSE(adapter.isBusy());
    TEST_ASSERT_EQUAL(controller.isBusy(), adapter.isBusy());

    // Submissions forward to controller and fail safely because controller is uninitialized
    TEST_ASSERT_FALSE(adapter.submitSpeedTarget(10.0f, 1000));
    TEST_ASSERT_FALSE(adapter.submitInclineTarget(2.0f, 1000));

    // After begin(), if console hardware is not ready, controller transitions to Faulted
    controller.begin();
    TEST_ASSERT_FALSE(adapter.isReady());
    TEST_ASSERT_EQUAL(controller.isReady(), adapter.isReady());
    TEST_ASSERT_FALSE(adapter.submitSpeedTarget(12.0f, 1000));
    TEST_ASSERT_FALSE(adapter.submitInclineTarget(3.0f, 1000));
}

// 2. Full Loop Coordination: Snapshot -> Session -> Coordinator -> Dispatcher -> Sink
void test_full_loop_coordination() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createCoordinatorTestWorkout();
    session.armWorkout(&ew, 1000);

    TreadmillSimulator sim;
    sim.begin(1000);

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    ControlCoordinator coordinator;

    // Tick 1: Belt moving snapshot triggers transition to Running and step 0 emission
    ApplicationSnapshot snap = makeAppSnapshot(1.0f, 0.0);
    coordinator.tick(session, dispatcher, sim, snap, 1000);

    // Session is Running
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);

    // Incline delivered first to simulator
    TEST_ASSERT_EQUAL_FLOAT(2.0f, sim.getTargetInclinePct());
    TEST_ASSERT_TRUE(sim.isBusy());

    // Speed remains staged and pending in dispatcher
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, dispatcher.getStagedTargets().speedKmh);
    TEST_ASSERT_FALSE(dispatcher.getStagedTargets().pendingIncline);

    // Intermediate ticks while simulator is busy
    for (uint32_t t = 1100; t <= 1400; t += 100) {
        sim.update(t);
        coordinator.tick(session, dispatcher, sim, sim.getSnapshot(), t);
        TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.getTargetSpeedKmh());
    }

    // At 1500 ms: simulator busy period expires
    sim.update(1500);
    TEST_ASSERT_FALSE(sim.isBusy());

    // Coordinator tick at 1500 ms delivers speed target
    coordinator.tick(session, dispatcher, sim, sim.getSnapshot(), 1500);
    TEST_ASSERT_FALSE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
    TEST_ASSERT_EQUAL_FLOAT(10.0f, sim.getTargetSpeedKmh());
    TEST_ASSERT_TRUE(sim.isBusy());
}

// 3. Preemption through Coordinator: Field-wise overwrite without losing unasserted fields
void test_preemption_through_coordinator() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createCoordinatorTestWorkout();
    session.armWorkout(&ew, 1000);

    MockTargetSink mock;
    mock.busy = true; // Block deliveries

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    ControlCoordinator coordinator;

    // Stage initial targets from Step 0 (speed 10.0, incline 2)
    coordinator.tick(session, dispatcher, mock, makeAppSnapshot(1.0f, 0.0), 1000);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, dispatcher.getStagedTargets().speedKmh);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, dispatcher.getStagedTargets().inclinePct);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);

    // Advance to Step 1 (speed 14.0, incline 4)
    session.advanceToNextStep(1100);

    // Coordinator tick with mock still busy
    coordinator.tick(session, dispatcher, mock, makeAppSnapshot(1.0f, 0.0), 1100);

    // Both fields overwritten with newest intent
    TEST_ASSERT_EQUAL_FLOAT(14.0f, dispatcher.getStagedTargets().speedKmh);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, dispatcher.getStagedTargets().inclinePct);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);

    // Partial preemption: unasserted fields are preserved
    session.reportWorkSpeedAdjustment(16.0f);
    session.acceptSpeedAdjustmentShift();

    // Verify un-overwritten incline remains intact
    TEST_ASSERT_EQUAL_FLOAT(4.0f, dispatcher.getStagedTargets().inclinePct);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);
}

// 4. Error / Reject Retention: Targets retained upon sink rejection
void test_error_reject_retention() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createCoordinatorTestWorkout();
    session.armWorkout(&ew, 1000);

    MockTargetSink mock;
    mock.failIncline = true; // Incline rejected

    WorkoutDispatcher dispatcher;
    dispatcher.begin();

    ControlCoordinator coordinator;

    // Tick: incline fails
    coordinator.tick(session, dispatcher, mock, makeAppSnapshot(1.0f, 0.0), 1000);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);
    TEST_ASSERT_EQUAL_UINT32(0, mock.inclineSubmitCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);

    // Repeated ticks retain both targets
    for (uint32_t t = 1020; t <= 1060; t += 20) {
        coordinator.tick(session, dispatcher, mock, makeAppSnapshot(1.0f, 0.0), t);
    }
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingIncline);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);

    // Incline cleared, speed fails
    mock.failIncline = false;
    mock.failSpeed = true;

    // Tick 1: Incline delivered
    coordinator.tick(session, dispatcher, mock, makeAppSnapshot(1.0f, 0.0), 1080);
    TEST_ASSERT_EQUAL_UINT32(1, mock.inclineSubmitCount);
    TEST_ASSERT_FALSE(dispatcher.getStagedTargets().pendingIncline);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);

    // Tick 2: Speed rejected, retained
    coordinator.tick(session, dispatcher, mock, makeAppSnapshot(1.0f, 0.0), 1100);
    TEST_ASSERT_EQUAL_UINT32(0, mock.speedSubmitCount);
    TEST_ASSERT_TRUE(dispatcher.getStagedTargets().pendingSpeed);

    // Speed failure resolved
    mock.failSpeed = false;
    coordinator.tick(session, dispatcher, mock, makeAppSnapshot(1.0f, 0.0), 1120);
    TEST_ASSERT_EQUAL_UINT32(1, mock.speedSubmitCount);
    TEST_ASSERT_FALSE(dispatcher.hasPendingTargets());
}

void run_all_control_coordinator_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_adapter_forwarding);
    RUN_TEST(test_full_loop_coordination);
    RUN_TEST(test_preemption_through_coordinator);
    RUN_TEST(test_error_reject_retention);
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
    run_all_control_coordinator_tests();
}
void loop() {}
#else
int main(int argc, char** argv) {
    run_all_control_coordinator_tests();
    return 0;
}
#endif

