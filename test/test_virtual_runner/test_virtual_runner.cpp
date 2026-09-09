#include <unity.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <cmath>
#include <cstring>
#include <memory>

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
#include "SpeedSensor.h"
#include "InclineSensor.h"
#include "ImuInterface.h"
#include "RunnerDynamics.h"
#include "ApplicationOrchestrator.h"
#include "VirtualTreadmill.h"
#include "VirtualSpeedSensorAdapter.h"
#include "VirtualRunnerAdapter.h"

using namespace stridecontrol;

// Helper to create a multi-step expanded workout for testing
static ExpandedWorkout createTestExpandedWorkout() {
    ExpandedWorkout ew{};
    ew.workoutId = 42;
    strncpy(ew.workoutName, "INTERVAL TEST", sizeof(ew.workoutName) - 1);
    ew.totalSteps = 3;

    // Step 0: Work 1 (Time, 30s, FIXED 12.0 km/h)
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WORK;
    ew.steps[0].durationType = DurationType::TIME_SECONDS;
    ew.steps[0].durationValue = 30;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 12.0f;
    ew.steps[0].repNumber = 1;
    ew.steps[0].totalRepsInGroup = 1;

    // Step 1: Rest 1 (Time, 30s, FREE, 6.0 km/h fallback)
    ew.steps[1].stepIndex = 1;
    ew.steps[1].role = StepRole::REST;
    ew.steps[1].durationType = DurationType::TIME_SECONDS;
    ew.steps[1].durationValue = 30;
    ew.steps[1].speedMode = SpeedMode::FREE;
    ew.steps[1].targetSpeedKmh = 6.0f;
    ew.steps[1].repNumber = 1;
    ew.steps[1].totalRepsInGroup = 1;

    // Step 2: Work 2 (Time, 30s, FIXED 12.0 km/h)
    ew.steps[2].stepIndex = 2;
    ew.steps[2].role = StepRole::WORK;
    ew.steps[2].durationType = DurationType::TIME_SECONDS;
    ew.steps[2].durationValue = 30;
    ew.steps[2].speedMode = SpeedMode::FIXED;
    ew.steps[2].targetSpeedKmh = 12.0f;
    ew.steps[2].repNumber = 2;
    ew.steps[2].totalRepsInGroup = 1;

    return ew;
}

// Helper fixture for isolated runner dynamics stepping
struct RunnerTestRig {
    std::unique_ptr<ImuInterface> imu;
    std::unique_ptr<RunnerDynamics> dynamics;
    std::unique_ptr<VirtualRunnerAdapter> adapter;
    SpeedSensorState speedState{};
    CsafeState csafeState{};
    ImuSample sampleBuffer[64]{};

    RunnerTestRig() {
        imu = std::unique_ptr<ImuInterface>(new ImuInterface());
        dynamics = std::unique_ptr<RunnerDynamics>(new RunnerDynamics());
        imu->begin(ImuObservationMode::SoftwareObservation);
        dynamics->begin();
        adapter = std::unique_ptr<VirtualRunnerAdapter>(new VirtualRunnerAdapter(*imu));

        speedState.initialized = true;
        speedState.measurementValid = true;
        speedState.status = SpeedSensorStatus::Measuring;
        speedState.speedKmh = 10.0f;
        speedState.lastPulseAgeMs = 10;

        csafeState.initialized = true;
        csafeState.online = true;
        csafeState.machineStateFresh = true;
        csafeState.reportedState = CsafeMachineState::InUse;
    }

    void setSpeed(float kmh) {
        speedState.speedKmh = kmh;
        speedState.lastPulseAgeMs = 10;
        speedState.measurementValid = true;
        speedState.status = (kmh > 0.01f) ? SpeedSensorStatus::Measuring : SpeedSensorStatus::Ready;
    }

    RunnerDynamicsState step(uint64_t tickIdx, uint64_t scenarioTimeUs, uint32_t dtUs, float deckAngle = 0.0f) {
        SimulationTick tick(tickIdx, scenarioTimeUs, dtUs);
        adapter->processTick(tick, deckAngle);

        size_t count = imu->readSamples(sampleBuffer, 64);
        uint32_t nowMs = static_cast<uint32_t>(scenarioTimeUs / 1000ULL);
        speedState.lastPulseAgeMs = 10;
        speedState.measurementValid = true;
        speedState.status = (speedState.speedKmh > 0.01f) ? SpeedSensorStatus::Measuring : SpeedSensorStatus::Ready;
        dynamics->update(sampleBuffer, count, speedState, csafeState, nowMs);
        return dynamics->getState();
    }
};

void test_standstill_not_present() {
    RunnerTestRig rig;
    rig.setSpeed(0.0f);
    rig.adapter->setMode(VirtualRunnerMode::NotPresent);

    RunnerDynamicsState state{};
    for (uint64_t i = 1; i <= 50; ++i) {
        state = rig.step(i, i * 20000ULL, 20000);
    }

    TEST_ASSERT_TRUE(state.initialized);
    TEST_ASSERT_FALSE(state.cadenceValid);
    TEST_ASSERT_FALSE(state.activePresence);
    TEST_ASSERT_FALSE(state.speedCreditEnabled);
    TEST_ASSERT_FALSE(state.distanceAccumulationEnabled);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, static_cast<float>(state.validatedDistanceKm));
}

void test_running_on_belt_steady() {
    RunnerTestRig rig;
    rig.setSpeed(10.0f);
    rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
    rig.adapter->setCadenceSpm(180);

    RunnerDynamicsState state{};
    // Run for 175 ticks (3.5 seconds at 50Hz, active for > 2.0 sec)
    for (uint64_t i = 1; i <= 175; ++i) {
        state = rig.step(i, i * 20000ULL, 20000);
    }

    TEST_ASSERT_TRUE(state.activePresence);
    TEST_ASSERT_EQUAL(RunnerPresence::Active, state.presence);
    TEST_ASSERT_TRUE(state.cadenceValid);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 180.0f, state.smoothedCadenceSpm);
    TEST_ASSERT_TRUE(state.speedCreditEnabled);
    TEST_ASSERT_TRUE(state.distanceAccumulationEnabled);
    TEST_ASSERT_TRUE(state.validatedDistanceKm > 0.005); // > 5 meters in 3 sec at 10 km/h
}

void test_transition_to_side_rails() {
    RunnerTestRig rig;
    rig.setSpeed(10.0f);
    rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
    rig.adapter->setCadenceSpm(180);

    RunnerDynamicsState state{};
    uint64_t tick = 0;
    // Step 1: Run on belt for 2.0s (100 ticks) to establish Active
    for (uint64_t i = 1; i <= 100; ++i) {
        tick = i;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }
    TEST_ASSERT_EQUAL(RunnerPresence::Active, state.presence);

    // Step 2: Transition to SideRails (zero impacts, belt continues moving)
    rig.adapter->setMode(VirtualRunnerMode::OnSideRails);

    // Run for 40 ticks (800 ms > 750 ms grace period)
    for (uint64_t i = 1; i <= 40; ++i) {
        tick++;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }
    TEST_ASSERT_EQUAL(RunnerPresence::SideRailsCandidate, state.presence);
    TEST_ASSERT_FALSE(state.speedCreditEnabled);
    TEST_ASSERT_FALSE(state.distanceAccumulationEnabled);
    float distAtRailsCandidate = static_cast<float>(state.validatedDistanceKm);

    // Run for another 90 ticks (1800 ms; total step age > 2500 ms)
    for (uint64_t i = 1; i <= 90; ++i) {
        tick++;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }
    TEST_ASSERT_EQUAL(RunnerPresence::SideRails, state.presence);
    TEST_ASSERT_EQUAL(RunnerActivity::SideRails, state.activity);
    // Distance must have remained frozen during side rails
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, distAtRailsCandidate, static_cast<float>(state.validatedDistanceKm));
}

void test_side_rails_to_belt_resume() {
    RunnerTestRig rig;
    rig.setSpeed(10.0f);
    rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
    rig.adapter->setCadenceSpm(180);

    RunnerDynamicsState state{};
    uint64_t tick = 0;
    // Establish active
    for (uint64_t i = 1; i <= 100; ++i) {
        tick = i;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }

    // Move to side rails for 3.0 seconds
    rig.adapter->setMode(VirtualRunnerMode::OnSideRails);
    for (uint64_t i = 1; i <= 150; ++i) {
        tick++;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }
    TEST_ASSERT_EQUAL(RunnerPresence::SideRails, state.presence);
    double distBeforeResume = state.validatedDistanceKm;

    // Resume running on belt at 180 SPM
    rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
    for (uint64_t i = 1; i <= 80; ++i) { // 1.6s (covers 3 qualified resume steps)
        tick++;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }

    TEST_ASSERT_EQUAL(RunnerPresence::Active, state.presence);
    TEST_ASSERT_TRUE(state.speedCreditEnabled);
    TEST_ASSERT_TRUE(state.distanceAccumulationEnabled);
    TEST_ASSERT_TRUE(state.validatedDistanceKm > distBeforeResume);
}

void test_belt_running_no_runner() {
    RunnerTestRig rig;
    rig.setSpeed(8.0f);
    rig.adapter->setMode(VirtualRunnerMode::NotPresent);

    RunnerDynamicsState state{};
    // Run for 35 ticks (700 ms > 500 ms imuDataMaxAgeMs)
    for (uint64_t i = 1; i <= 35; ++i) {
        state = rig.step(i, i * 20000ULL, 20000);
    }

    TEST_ASSERT_FALSE(state.imuInputValid);
    TEST_ASSERT_EQUAL(RunnerDistancePauseReason::ImuInvalid, state.distancePauseReason);
}

void test_cadence_variation() {
    RunnerTestRig rig;
    rig.setSpeed(10.0f);
    rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
    rig.adapter->setCadenceSpm(160);

    RunnerDynamicsState state{};
    uint64_t tick = 0;
    // Step 1: Steady at 160 SPM for 120 ticks (2.4s)
    for (uint64_t i = 1; i <= 120; ++i) {
        tick = i;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }
    TEST_ASSERT_TRUE(state.cadenceValid);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 160.0f, state.smoothedCadenceSpm);

    // Step 2: Step cadence to 190 SPM for 150 ticks (3.0s)
    rig.adapter->setCadenceSpm(190);
    for (uint64_t i = 1; i <= 150; ++i) {
        tick++;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }
    TEST_ASSERT_TRUE(state.cadenceValid);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 190.0f, state.smoothedCadenceSpm);
}

void test_imu_signal_loss_fault() {
    RunnerTestRig rig;
    rig.setSpeed(10.0f);
    rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
    rig.adapter->setCadenceSpm(180);

    RunnerDynamicsState state{};
    uint64_t tick = 0;
    for (uint64_t i = 1; i <= 100; ++i) {
        tick = i;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }
    TEST_ASSERT_TRUE(state.activePresence);

    // Signal loss: mark IMU invalid
    rig.adapter->setSignalValid(false);
    for (uint64_t i = 1; i <= 10; ++i) {
        tick++;
        state = rig.step(tick, tick * 20000ULL, 20000);
    }

    TEST_ASSERT_FALSE(state.inputDataValid);
    TEST_ASSERT_FALSE(state.distanceAccumulationEnabled);
}

void test_deterministic_replay() {
    auto runProfile = []() -> RunnerDynamicsState {
        RunnerTestRig rig;
        rig.setSpeed(10.0f);
        rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
        rig.adapter->setCadenceSpm(180);

        RunnerDynamicsState st{};
        uint64_t tick = 0;
        // Phase 1: 80 ticks on belt
        for (uint64_t i = 1; i <= 80; ++i) {
            tick = i; st = rig.step(tick, tick * 20000ULL, 20000);
        }
        // Phase 2: 50 ticks on side rails
        rig.adapter->setMode(VirtualRunnerMode::OnSideRails);
        for (uint64_t i = 1; i <= 50; ++i) {
            tick++; st = rig.step(tick, tick * 20000ULL, 20000);
        }
        // Phase 3: 70 ticks on belt
        rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
        for (uint64_t i = 1; i <= 70; ++i) {
            tick++; st = rig.step(tick, tick * 20000ULL, 20000);
        }
        return st;
    };

    RunnerDynamicsState run1 = runProfile();
    RunnerDynamicsState run2 = runProfile();

    TEST_ASSERT_EQUAL_UINT32(run1.sessionStepCount, run2.sessionStepCount);
    TEST_ASSERT_EQUAL_UINT32(run1.detectedImpactCount, run2.detectedImpactCount);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, run1.smoothedCadenceSpm, run2.smoothedCadenceSpm);
    TEST_ASSERT_FLOAT_WITHIN(0.0000001f, static_cast<float>(run1.validatedDistanceKm), static_cast<float>(run2.validatedDistanceKm));
}

void test_orchestrator_snapshot_integration() {
    SpeedSensor speed;
    InclineSensor incline;
    ImuInterface imu;
    RunnerDynamics runner;
    DiagnosticsService diag;

    speed.begin(SpeedSensorConfig{}, SpeedObservationMode::SoftwareObservation);
    incline.begin(InclineSensorConfig{}, InclineCalibration{}, InclineObservationMode::SoftwareObservation);
    imu.begin(ImuObservationMode::SoftwareObservation);
    runner.begin();

    ApplicationOrchestratorDependencies deps{};
    deps.speedSensor = &speed;
    deps.inclineSensor = &incline;
    deps.imuInterface = &imu;
    deps.runnerDynamics = &runner;
    deps.diagnosticsService = &diag;

    ApplicationOrchestrator orchestrator;
    TEST_ASSERT_TRUE(orchestrator.begin(deps, OrchestratorExecutionMode::ExternalStep));

    VirtualTreadmillConfig vtCfg;
    vtCfg.accelerationKmhPerSec = 4.0f;
    VirtualTreadmill treadmill(vtCfg);
    treadmill.setTargetSpeedKmh(10.0f);
    VirtualSpeedSensorAdapter speedAdapter(speed);
    VirtualRunnerAdapter runnerAdapter(imu);
    runnerAdapter.setMode(VirtualRunnerMode::RunningOnBelt);
    runnerAdapter.setCadenceSpm(180);

    for (uint64_t i = 1; i <= 150; ++i) {
        SimulationTick tick(i, i * 20000ULL, 20000);
        treadmill.tick(tick);
        speedAdapter.processTacho(treadmill.getTachoOutput());
        runnerAdapter.processTick(tick, 0.0f);

        ApplicationTickContext ctx(i, i * 20000ULL, 20);
        TEST_ASSERT_TRUE(orchestrator.step(ctx));
    }

    ApplicationSnapshot snap = orchestrator.getSnapshot();
    TEST_ASSERT_EQUAL_UINT32(150, snap.sequenceNumber);
    TEST_ASSERT_TRUE(snap.runner.activePresence);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 180.0f, snap.runner.smoothedCadenceSpm);
    TEST_ASSERT_TRUE(snap.runner.validatedDistanceKm > 0.0);

    orchestrator.end(150);
}

void test_workout_interval_workflow() {
    RunnerTestRig rig;
    WorkoutSession session;
    session.begin();

    ExpandedWorkout ew = createTestExpandedWorkout();
    TEST_ASSERT_TRUE(session.armWorkout(&ew, 1000));
    TEST_ASSERT_EQUAL(WorkoutSessionState::Armed, session.getSnapshot().state);

    // Interval 1: Work at 12 km/h, RunningOnBelt for 2.0s (100 ticks)
    rig.setSpeed(12.0f);
    rig.adapter->setMode(VirtualRunnerMode::RunningOnBelt);
    rig.adapter->setCadenceSpm(180);

    uint64_t tick = 0;
    for (uint64_t i = 1; i <= 100; ++i) {
        tick = i;
        RunnerDynamicsState rState = rig.step(tick, tick * 20000ULL, 20000);

        ApplicationSnapshot snap{};
        snap.speed = rig.speedState;
        snap.runner = rState;
        session.update(snap, static_cast<uint32_t>(1000 + tick * 20));
    }

    WorkoutSessionSnapshot s1 = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, s1.state);
    TEST_ASSERT_TRUE(s1.activeRunningTimeMs > 0);
    TEST_ASSERT_TRUE(s1.totalValidatedDistanceKm > 0.0);

    // Interval 2: Rest at 6 km/h, OnSideRails
    rig.setSpeed(6.0f);
    rig.adapter->setMode(VirtualRunnerMode::OnSideRails);

    // Step through 40 ticks (800ms > 750ms grace) to enter SideRailsCandidate
    for (uint64_t i = 1; i <= 40; ++i) {
        tick++;
        RunnerDynamicsState rState = rig.step(tick, tick * 20000ULL, 20000);

        ApplicationSnapshot snap{};
        snap.speed = rig.speedState;
        snap.runner = rState;
        session.update(snap, static_cast<uint32_t>(1000 + tick * 20));
    }

    WorkoutSessionSnapshot sGraceEnd = session.getSnapshot();
    uint32_t runningTimeAtGraceEnd = sGraceEnd.activeRunningTimeMs;
    float distAtGraceEnd = static_cast<float>(sGraceEnd.totalValidatedDistanceKm);

    // Continue on side rails for another 60 ticks (1200ms)
    for (uint64_t i = 1; i <= 60; ++i) {
        tick++;
        RunnerDynamicsState rState = rig.step(tick, tick * 20000ULL, 20000);

        ApplicationSnapshot snap{};
        snap.speed = rig.speedState;
        snap.runner = rState;
        session.update(snap, static_cast<uint32_t>(1000 + tick * 20));
    }

    WorkoutSessionSnapshot s2 = session.getSnapshot();
    // Total elapsed time increased, but active running time and validated distance paused
    TEST_ASSERT_TRUE(s2.totalElapsedTimeMs > sGraceEnd.totalElapsedTimeMs);
    // Active running time should be frozen after grace period
    TEST_ASSERT_EQUAL_UINT32(runningTimeAtGraceEnd, s2.activeRunningTimeMs);
    // Validated distance should be frozen after grace period
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, distAtGraceEnd, static_cast<float>(s2.totalValidatedDistanceKm));
}

void run_all_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_standstill_not_present);
    RUN_TEST(test_running_on_belt_steady);
    RUN_TEST(test_transition_to_side_rails);
    RUN_TEST(test_side_rails_to_belt_resume);
    RUN_TEST(test_belt_running_no_runner);
    RUN_TEST(test_cadence_variation);
    RUN_TEST(test_imu_signal_loss_fault);
    RUN_TEST(test_deterministic_replay);
    RUN_TEST(test_orchestrator_snapshot_integration);
    RUN_TEST(test_workout_interval_workflow);
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
