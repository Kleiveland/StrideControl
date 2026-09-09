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
#include "VirtualTreadmill.h"
#include "VirtualSpeedSensorAdapter.h"
#include "ApplicationOrchestrator.h"

using namespace stridecontrol;

void test_adapter_edge_forwarding() {
    SpeedSensor sensor;
    SpeedSensorConfig cfg{};
    cfg.glitchRejectUs = 300;
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 2000;
    cfg.kmhPerHz = 1.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));
    VirtualSpeedSensorAdapter adapter(sensor);

    TachoOutput tacho{};
    tacho.edgeCount = 2;
    tacho.edgeTimestampUs[0] = 1000000;
    tacho.edgeTimestampUs[1] = 1050000; // 50ms interval -> 20 Hz
    tacho.pulsesThisTick = 2;
    tacho.signalValid = true;

    const uint8_t forwarded = adapter.processTacho(tacho);
    TEST_ASSERT_EQUAL_UINT8(2, forwarded);
    TEST_ASSERT_EQUAL_UINT32(1050000, adapter.getLastObservedEdgeUs());

    TEST_ASSERT_TRUE(sensor.evaluate(1050000));
    SpeedSensorState state = sensor.getState();
    TEST_ASSERT_TRUE(state.measurementValid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, state.speedKmh);
}

void test_adapter_glitch_and_lockout_helpers() {
    SpeedSensor sensor;
    SpeedSensorConfig cfg{};
    cfg.glitchRejectUs = 300;
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 2000;
    cfg.kmhPerHz = 1.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));
    VirtualSpeedSensorAdapter adapter(sensor);

    // Initial edge
    TEST_ASSERT_TRUE(adapter.injectEdge(1000000));

    // Glitch helper: default +150us (< 300us)
    TEST_ASSERT_TRUE(adapter.injectGlitchEdge(150));
    sensor.evaluate(1000150);
    TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(sensor.getState().rejectedGlitchCount));

    // Lockout helper: default +1000us (>= 300us and < 2000us)
    TEST_ASSERT_TRUE(adapter.injectLockoutEdge(1000));
    sensor.evaluate(1001000);
    TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(sensor.getState().rejectedLockoutCount));
}

void test_virtual_treadmill_to_speed_sensor_pipeline() {
    const float kmhFactor = 1.1148f;
    const double pulsesPerKm = 3600.0 / static_cast<double>(kmhFactor);

    VirtualTreadmillConfig vtCfg{};
    vtCfg.tachoPulsesPerKm = pulsesPerKm;
    vtCfg.accelerationKmhPerSec = 2.0f;
    VirtualTreadmill vt(vtCfg);
    vt.resetModelUs(0);

    SpeedSensor sensor;
    SpeedSensorConfig sCfg{};
    sCfg.kmhPerHz = kmhFactor;
    sCfg.glitchRejectUs = 300;
    sCfg.pulseLockoutUs = 2000;
    sCfg.pulseTimeoutMs = 2000;
    TEST_ASSERT_TRUE(sensor.begin(sCfg, SpeedObservationMode::SoftwareObservation));
    VirtualSpeedSensorAdapter adapter(sensor);

    vt.setTargetSpeedKmh(10.0f);

    uint64_t tickIndex = 0;
    uint64_t scenarioTimeUs = 0;
    const uint32_t dtUs = 20000; // 20ms @ 50 Hz

    // Step 300 ticks (6.0 seconds) -> Reaches 10.0 km/h
    for (uint32_t i = 0; i < 300; ++i) {
        tickIndex++;
        scenarioTimeUs += dtUs;
        SimulationTick simTick(tickIndex, scenarioTimeUs, dtUs);
        TEST_ASSERT_TRUE(vt.tick(simTick));

        adapter.processTacho(vt.getTachoOutput());
        TEST_ASSERT_TRUE(sensor.evaluate(static_cast<uint32_t>(scenarioTimeUs)));
        if ((i % 25) == 0) {
            yield();
        }
    }

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, vt.getActualSpeedKmh());
    SpeedSensorState state = sensor.getState();
    TEST_ASSERT_TRUE(state.measurementValid);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, state.speedKmh);
}

void test_orchestrator_external_step_lifecycle() {
    SpeedSensor sensor;
    SpeedSensorConfig sCfg{};
    sCfg.glitchRejectUs = 300;
    sCfg.pulseLockoutUs = 2000;
    sCfg.pulseTimeoutMs = 2000;
    TEST_ASSERT_TRUE(sensor.begin(sCfg, SpeedObservationMode::SoftwareObservation));

    auto orchestrator = std::unique_ptr<ApplicationOrchestrator>(new ApplicationOrchestrator());
    ApplicationOrchestratorDependencies deps{};
    deps.speedSensor = &sensor;

    TEST_ASSERT_TRUE(orchestrator->begin(deps, OrchestratorExecutionMode::ExternalStep));
    TEST_ASSERT_TRUE(orchestrator->isInitialized());
    TEST_ASSERT_TRUE(orchestrator->isRunning());
    TEST_ASSERT_TRUE(orchestrator->acceptsExternalSteps());
    TEST_ASSERT_FALSE(orchestrator->isWorkerTaskRunning());
    TEST_ASSERT_EQUAL(OrchestratorExecutionMode::ExternalStep, orchestrator->executionMode());

    // Monotonic external stepping
    ApplicationTickContext ctx1(1, 20000, 20);
    TEST_ASSERT_TRUE(orchestrator->step(ctx1));
    TEST_ASSERT_EQUAL_UINT32(1, orchestrator->getLoopCount());

    // Reject non-monotonic tick index
    ApplicationTickContext ctx_bad_index(1, 40000, 20);
    TEST_ASSERT_FALSE(orchestrator->step(ctx_bad_index));
    TEST_ASSERT_EQUAL_UINT32(1, orchestrator->getLoopCount());

    // Reject non-monotonic timestamp
    ApplicationTickContext ctx_bad_time(2, 20000, 20);
    TEST_ASSERT_FALSE(orchestrator->step(ctx_bad_time));
    TEST_ASSERT_EQUAL_UINT32(1, orchestrator->getLoopCount());

    // Valid next step
    ApplicationTickContext ctx2(2, 40000, 20);
    TEST_ASSERT_TRUE(orchestrator->step(ctx2));
    TEST_ASSERT_EQUAL_UINT32(2, orchestrator->getLoopCount());

    // Clean termination without blocking
    TEST_ASSERT_TRUE(orchestrator->end(100));
    TEST_ASSERT_FALSE(orchestrator->isRunning());
    TEST_ASSERT_FALSE(orchestrator->isInitialized());
    TEST_ASSERT_FALSE(orchestrator->acceptsExternalSteps());
}

void test_end_to_end_virtual_treadmill_orchestrator_step() {
    const float kmhFactor = 1.1148f;
    const double pulsesPerKm = 3600.0 / static_cast<double>(kmhFactor);

    VirtualTreadmillConfig vtCfg{};
    vtCfg.tachoPulsesPerKm = pulsesPerKm;
    vtCfg.accelerationKmhPerSec = 4.0f;
    auto vt = std::unique_ptr<VirtualTreadmill>(new VirtualTreadmill(vtCfg));
    vt->resetModelUs(0);

    auto sensor = std::unique_ptr<SpeedSensor>(new SpeedSensor());
    SpeedSensorConfig sCfg{};
    sCfg.kmhPerHz = kmhFactor;
    sCfg.glitchRejectUs = 300;
    sCfg.pulseLockoutUs = 2000;
    sCfg.pulseTimeoutMs = 2000;
    TEST_ASSERT_TRUE(sensor->begin(sCfg, SpeedObservationMode::SoftwareObservation));

    VirtualSpeedSensorAdapter adapter(*sensor);

    auto orchestrator = std::unique_ptr<ApplicationOrchestrator>(new ApplicationOrchestrator());
    ApplicationOrchestratorDependencies deps{};
    deps.speedSensor = sensor.get();
    TEST_ASSERT_TRUE(orchestrator->begin(deps, OrchestratorExecutionMode::ExternalStep));

    vt->setTargetSpeedKmh(8.0f);

    uint64_t tickIndex = 0;
    uint64_t scenarioTimeUs = 0;
    const uint32_t dtUs = 20000; // 20ms @ 50 Hz

    for (uint32_t i = 0; i < 200; ++i) { // 4.0 seconds
        tickIndex++;
        scenarioTimeUs += dtUs;

        SimulationTick simTick(tickIndex, scenarioTimeUs, dtUs);
        TEST_ASSERT_TRUE(vt->tick(simTick));

        adapter.processTacho(vt->getTachoOutput());

        ApplicationTickContext appCtx(tickIndex, scenarioTimeUs, 20);
        TEST_ASSERT_TRUE(orchestrator->step(appCtx));
        if ((i % 25) == 0) {
            yield();
        }
    }

    ApplicationSnapshot snap = orchestrator->getSnapshot();
    TEST_ASSERT_EQUAL_UINT32(200, snap.sequenceNumber);
    TEST_ASSERT_TRUE(snap.speed.measurementValid);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 8.0f, snap.speed.speedKmh);

    // Fault injection: TachoLostSignal causes timeout
    vt->setFault(VirtualTreadmillFault::TachoLostSignal, true);

    for (uint32_t i = 0; i < 125; ++i) { // 2.5 seconds (exceeds 2.0s timeout)
        tickIndex++;
        scenarioTimeUs += dtUs;

        SimulationTick simTick(tickIndex, scenarioTimeUs, dtUs);
        TEST_ASSERT_TRUE(vt->tick(simTick));

        adapter.processTacho(vt->getTachoOutput());

        ApplicationTickContext appCtx(tickIndex, scenarioTimeUs, 20);
        TEST_ASSERT_TRUE(orchestrator->step(appCtx));
        if ((i % 25) == 0) {
            yield();
        }
    }

    snap = orchestrator->getSnapshot();
    TEST_ASSERT_EQUAL(SpeedSensorStatus::TimedOut, snap.speed.status);
    TEST_ASSERT_FALSE(snap.speed.measurementValid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, snap.speed.speedKmh);

    TEST_ASSERT_TRUE(orchestrator->end(100));
}

void run_all_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_adapter_edge_forwarding);
    RUN_TEST(test_adapter_glitch_and_lockout_helpers);
    RUN_TEST(test_virtual_treadmill_to_speed_sensor_pipeline);
    RUN_TEST(test_orchestrator_external_step_lifecycle);
    RUN_TEST(test_end_to_end_virtual_treadmill_orchestrator_step);
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

