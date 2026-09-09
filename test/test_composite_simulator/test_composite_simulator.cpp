#include <unity.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <cmath>
#include <cstring>
#include <memory>

#include "TreadmillSimulatorComposite.h"
#include "SpeedSensor.h"
#include "InclineSensor.h"
#include "ConsoleInterface.h"
#include "ImuInterface.h"
#include "RunnerDynamics.h"
#include "DiagnosticsService.h"
#include "ApplicationOrchestrator.h"
#include "WorkoutSession.h"
#include "WorkoutExpander.h"
#include "WorkoutDispatcher.h"
#include "ControlCoordinator.h"
#include "ControlRuntime.h"

using namespace stridecontrol;

// Test fixture encapsulating the entire composite simulator testbench
struct CompositeTestRig {
    SpeedSensor speedSensor;
    InclineSensor inclineSensor;
    ConsoleInterface console;
    ImuInterface imu;
    RunnerDynamics runnerDynamics;
    DiagnosticsService diagService;
    ApplicationOrchestrator orchestrator;

    std::unique_ptr<TreadmillSimulatorComposite> composite;

    CompositeTestRig() {
        speedSensor.begin(SpeedSensorConfig{}, SpeedObservationMode::SoftwareObservation);
        inclineSensor.begin(InclineSensorConfig{}, InclineCalibration{}, InclineObservationMode::SoftwareObservation);
        console.begin(ConsoleExecutionMode::SoftwareSink);
        imu.begin(ImuObservationMode::SoftwareObservation);
        runnerDynamics.begin();

        ApplicationOrchestratorDependencies deps{};
        deps.speedSensor = &speedSensor;
        deps.inclineSensor = &inclineSensor;
        deps.imuInterface = &imu;
        deps.runnerDynamics = &runnerDynamics;
        deps.diagnosticsService = &diagService;
        orchestrator.begin(deps, OrchestratorExecutionMode::ExternalStep);

        VirtualTreadmillConfig vtCfg;
        vtCfg.accelerationKmhPerSec = 4.0f; // brisk acceleration for fast test convergence
        vtCfg.normalDecelerationKmhPerSec = 4.0f; // brisk deceleration for fast test convergence
        composite = std::unique_ptr<TreadmillSimulatorComposite>(
            new TreadmillSimulatorComposite(speedSensor, inclineSensor, imu, vtCfg)
        );
    }

    ~CompositeTestRig() {
        orchestrator.end();
    }

    ApplicationSnapshot step(uint64_t tickIdx, uint32_t dtMs = 20) {
        uint64_t scenarioTimeUs = tickIdx * static_cast<uint64_t>(dtMs) * 1000ULL;
        SimulationTick simTick(tickIdx, scenarioTimeUs, dtMs * 1000UL);
        composite->tick(simTick, 0.0f);

        ApplicationTickContext appCtx(tickIdx, scenarioTimeUs, dtMs);
        orchestrator.step(appCtx);
        return orchestrator.getSnapshot();
    }
};

void test_mailbox_fifo_and_overflow() {
    StagedStimulusMailbox mailbox;
    TEST_ASSERT_EQUAL_UINT32(0, mailbox.getDroppedEventsCount());

    // Enqueue 16 discrete events
    for (uint8_t i = 1; i <= 16; ++i) {
        StagedDiscreteEvent ev{};
        ev.type = StagedDiscreteEvent::Type::SetSpeed;
        ev.paramValue = static_cast<float>(i);
        ev.timestampMs = i * 10;
        TEST_ASSERT_TRUE(mailbox.enqueueEvent(ev));
    }

    // 17th event must be rejected and increment dropped count
    StagedDiscreteEvent overflowEv{};
    overflowEv.type = StagedDiscreteEvent::Type::QuickStart;
    TEST_ASSERT_FALSE(mailbox.enqueueEvent(overflowEv));
    TEST_ASSERT_EQUAL_UINT32(1, mailbox.getDroppedEventsCount());

    // Drain and verify exact FIFO ordering
    StagedDiscreteEvent drained[16]{};
    StagedContinuousRunnerState runnerState{};
    bool runnerDirty = false;
    size_t count = mailbox.drainEvents(drained, 16, runnerState, runnerDirty);
    TEST_ASSERT_EQUAL_size_t(16, count);
    TEST_ASSERT_FALSE(runnerDirty);

    for (size_t i = 0; i < 16; ++i) {
        TEST_ASSERT_EQUAL(StagedDiscreteEvent::Type::SetSpeed, drained[i].type);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, static_cast<float>(i + 1), drained[i].paramValue);
    }
}

void test_closed_loop_quickstart_and_speed_ramp() {
    CompositeTestRig rig;
    rig.composite->stageRunner(VirtualRunnerMode::RunningOnBelt, 180);

    // 1. Stage QuickStart
    TEST_ASSERT_TRUE(rig.composite->stageQuickStart(100));

    // Run for 150 ticks (3.0 seconds)
    ApplicationSnapshot snap{};
    for (uint64_t i = 1; i <= 150; ++i) {
        snap = rig.step(i, 20);
    }

    // Belt must be at 1.0 km/h (physical QuickStart baseline, 1.1148 km/h discrete tacho quantum)
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 1.0f, snap.speed.speedKmh);
    TEST_ASSERT_TRUE(snap.speed.measurementValid);

    // 2. Stage Speed to 10.0 km/h
    TEST_ASSERT_TRUE(rig.composite->stageSpeedTarget(10.0f, 3100));

    // Run for another 150 ticks (3.0 seconds, accel = 4.0 km/h/s reaches 10 km/h in 2.25s)
    for (uint64_t i = 151; i <= 300; ++i) {
        snap = rig.step(i, 20);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, snap.speed.speedKmh);
    TEST_ASSERT_TRUE(snap.runner.activePresence);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 180.0f, snap.runner.smoothedCadenceSpm);
    TEST_ASSERT_TRUE(snap.runner.validatedDistanceKm > 0.005);
}

void test_incline_adjustment() {
    CompositeTestRig rig;
    rig.composite->stageRunner(VirtualRunnerMode::RunningOnBelt, 180);
    rig.composite->stageQuickStart(100);

    // Initial ticks to establish speed
    for (uint64_t i = 1; i <= 50; ++i) {
        rig.step(i, 20);
    }

    // Stage Incline to 3.0 %
    TEST_ASSERT_TRUE(rig.composite->stageInclineTarget(3.0f, 1100));

    // Run ticks until incline reaches target (at 0.35 %/s, 3% takes ~8.5s = 430 ticks)
    ApplicationSnapshot snap{};
    for (uint64_t i = 51; i <= 550; ++i) {
        snap = rig.step(i, 20);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.3f, 3.0f, snap.incline.estimatedInclinePct);
}

void test_side_rails_freeze_and_resume() {
    CompositeTestRig rig;
    rig.composite->stageRunner(VirtualRunnerMode::RunningOnBelt, 180);
    rig.composite->stageSpeedTarget(10.0f, 100);

    // 1. Establish Active presence on belt for 150 ticks (3.0s)
    ApplicationSnapshot snap{};
    for (uint64_t i = 1; i <= 150; ++i) {
        snap = rig.step(i, 20);
    }
    TEST_ASSERT_EQUAL(RunnerPresence::Active, snap.runner.presence);
    TEST_ASSERT_TRUE(snap.runner.speedCreditEnabled);

    // 2. Transition to OnSideRails (belt continues rolling at 10 km/h)
    rig.composite->stageRunner(VirtualRunnerMode::OnSideRails);

    // Step through 40 ticks (800ms > 750ms grace) to enter SideRailsCandidate
    for (uint64_t i = 151; i <= 190; ++i) {
        snap = rig.step(i, 20);
    }
    TEST_ASSERT_EQUAL(RunnerPresence::SideRailsCandidate, snap.runner.presence);
    TEST_ASSERT_FALSE(snap.runner.speedCreditEnabled);
    TEST_ASSERT_FALSE(snap.runner.distanceAccumulationEnabled);

    float distAtRailsCandidate = static_cast<float>(snap.runner.validatedDistanceKm);

    // Step another 90 ticks (1800ms > 2500ms timeout) to enter SideRails
    for (uint64_t i = 191; i <= 280; ++i) {
        snap = rig.step(i, 20);
    }
    TEST_ASSERT_EQUAL(RunnerPresence::SideRails, snap.runner.presence);
    // Distance must have remained completely frozen
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, distAtRailsCandidate, static_cast<float>(snap.runner.validatedDistanceKm));

    // 3. Return to RunningOnBelt at 180 SPM
    rig.composite->stageRunner(VirtualRunnerMode::RunningOnBelt, 180);

    // Run for 80 ticks (1.6s, covers 3 qualified resume steps)
    for (uint64_t i = 281; i <= 360; ++i) {
        snap = rig.step(i, 20);
    }
    TEST_ASSERT_EQUAL(RunnerPresence::Active, snap.runner.presence);
    TEST_ASSERT_TRUE(snap.runner.speedCreditEnabled);
    TEST_ASSERT_TRUE(snap.runner.distanceAccumulationEnabled);
    TEST_ASSERT_TRUE(snap.runner.validatedDistanceKm > distAtRailsCandidate);
}

void test_workout_session_coordination_and_stop() {
    CompositeTestRig rig;
    rig.composite->stageRunner(VirtualRunnerMode::RunningOnBelt, 180);

    WorkoutSession session;
    WorkoutDispatcher dispatcher;
    ControlCoordinator coordinator;
    session.begin();
    dispatcher.begin();

    // Create simple 2-step workout
    ExpandedWorkout ew{};
    ew.workoutId = 1;
    strncpy(ew.workoutName, "TESTBENCH WORKOUT", sizeof(ew.workoutName) - 1);
    ew.totalSteps = 2;
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WARMUP;
    ew.steps[0].durationType = DurationType::TIME_SECONDS;
    ew.steps[0].durationValue = 10;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 6.0f;

    ew.steps[1].stepIndex = 1;
    ew.steps[1].role = StepRole::WORK;
    ew.steps[1].durationType = DurationType::TIME_SECONDS;
    ew.steps[1].durationValue = 10;
    ew.steps[1].speedMode = SpeedMode::FIXED;
    ew.steps[1].targetSpeedKmh = 10.0f;

    TEST_ASSERT_TRUE(session.armWorkout(&ew, 1000));
    TEST_ASSERT_EQUAL(WorkoutSessionState::Armed, session.getSnapshot().state);

    // QuickStart triggers belt movement
    rig.composite->stageQuickStart(1020);

    ApplicationSnapshot snap{};
    for (uint64_t i = 1; i <= 150; ++i) {
        snap = rig.step(i, 20);
        coordinator.tick(session, dispatcher, *rig.composite, snap, 1000 + static_cast<uint32_t>(i * 20));
    }

    WorkoutSessionSnapshot sessSnap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, sessSnap.state);
    TEST_ASSERT_TRUE(sessSnap.activeRunningTimeMs > 0);

    // Stage physical Stop
    rig.composite->stageStop(4020);

    for (uint64_t i = 151; i <= 350; ++i) {
        snap = rig.step(i, 20);
        coordinator.tick(session, dispatcher, *rig.composite, snap, 1000 + static_cast<uint32_t>(i * 20));
    }

    // Speed ramps to 0.0 km/h
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, snap.speed.speedKmh);
}

void run_all_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_mailbox_fifo_and_overflow);
    RUN_TEST(test_closed_loop_quickstart_and_speed_ramp);
    RUN_TEST(test_incline_adjustment);
    RUN_TEST(test_side_rails_freeze_and_resume);
    RUN_TEST(test_workout_session_coordination_and_stop);
    UNITY_END();
}

#ifdef ARDUINO
static void testTask(void* pvParameters) {
    run_all_tests();
    vTaskDelete(NULL);
}

void setup() {
    delay(2000);
    xTaskCreatePinnedToCore(testTask, "testTask", 32768, NULL, 1, NULL, 1);
}
void loop() {
    vTaskDelay(pdMS_TO_TICKS(500));
}
#else
int main(int argc, char** argv) {
    run_all_tests();
    return 0;
}
#endif

