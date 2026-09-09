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
#include "InclineSensor.h"
#include "VirtualTreadmill.h"
#include "VirtualInclineAdapter.h"
#include "ApplicationOrchestrator.h"

using namespace stridecontrol;

void test_incline_initialization_modes() {
    InclineSensor sensorHw;
    InclineSensorConfig cfgHw{};
    cfgHw.inputPin = GPIO_NUM_2; // Valid GPIO on ESP32-S3
    cfgHw.useInternalPullup = true;
    InclineCalibration calHw{};

    // HardwareInterrupt mode begins with pin validation
    TEST_ASSERT_TRUE(sensorHw.begin(cfgHw, calHw, InclineObservationMode::HardwareInterrupt));
    TEST_ASSERT_TRUE(sensorHw.isReady());
    TEST_ASSERT_EQUAL(InclineObservationMode::HardwareInterrupt, sensorHw.getObservationMode());
    InclineState hwState = sensorHw.getState();
    TEST_ASSERT_TRUE(hwState.initialized);
    TEST_ASSERT_FALSE(hwState.homed);
    TEST_ASSERT_FALSE(hwState.positionTrusted);
    sensorHw.end();

    // SoftwareObservation mode begins without GPIO / ISR
    InclineSensor sensorSw;
    InclineSensorConfig cfgSw{};
    cfgSw.inputPin = static_cast<gpio_num_t>(99); // Invalid GPIO pin to prove hardware bypass
    InclineCalibration calSw{};

    TEST_ASSERT_TRUE(sensorSw.begin(cfgSw, calSw, InclineObservationMode::SoftwareObservation));
    TEST_ASSERT_TRUE(sensorSw.isReady());
    TEST_ASSERT_EQUAL(InclineObservationMode::SoftwareObservation, sensorSw.getObservationMode());
    InclineState swState = sensorSw.getState();
    TEST_ASSERT_TRUE(swState.initialized);
    TEST_ASSERT_TRUE(swState.homed);
    TEST_ASSERT_TRUE(swState.positionTrusted);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, swState.estimatedInclinePct);
    sensorSw.end();
}

void test_mode_misuse_rejection() {
    InclineSensor sensorHw;
    InclineSensorConfig cfgHw{};
    cfgHw.inputPin = GPIO_NUM_2;
    TEST_ASSERT_TRUE(sensorHw.begin(cfgHw, InclineCalibration{}, InclineObservationMode::HardwareInterrupt));

    // Hardware mode MUST reject software observations and evaluate calls
    InclinePulseObservation obs{};
    obs.pulseCount = 10;
    obs.direction = InclineDirection::Up;
    obs.signalValid = true;

    TEST_ASSERT_FALSE(sensorHw.observePulses(obs, 1000));
    TEST_ASSERT_FALSE(sensorHw.evaluate(1000));

    sensorHw.end();

    InclineSensor sensorSw;
    InclineSensorConfig cfgSw{};
    cfgSw.inputPin = GPIO_NUM_2;
    TEST_ASSERT_TRUE(sensorSw.begin(cfgSw, InclineCalibration{}, InclineObservationMode::SoftwareObservation));

    // Software mode ignores parameterless update()
    sensorSw.update();
    InclineState st = sensorSw.getState();
    TEST_ASSERT_EQUAL_UINT32(0, st.acceptedPulseCount);

    sensorSw.end();
}

void test_virtual_incline_adapter_forwarding() {
    InclineSensor sensor;
    InclineSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_2;
    TEST_ASSERT_TRUE(sensor.begin(cfg, InclineCalibration{}, InclineObservationMode::SoftwareObservation));

    VirtualInclineAdapter adapter(sensor);

    InclineFeedbackOutput feedback{};
    feedback.pulsesThisTick = 5;
    feedback.direction = InclineDirection::Up;
    feedback.signalValid = true;
    feedback.moving = true;

    TEST_ASSERT_TRUE(adapter.processFeedback(feedback, 50));
    TEST_ASSERT_EQUAL_UINT32(50, adapter.getLastObservedMs());

    TEST_ASSERT_TRUE(sensor.evaluate(50));
    InclineState st = sensor.getState();
    TEST_ASSERT_EQUAL_UINT32(5, static_cast<uint32_t>(st.acceptedPulseCount));
    TEST_ASSERT_EQUAL(InclineDirection::Up, st.expectedDirection);

    sensor.end();
}

void test_standstill_and_steady() {
    InclineSensor sensor;
    InclineSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_2;
    TEST_ASSERT_TRUE(sensor.begin(cfg, InclineCalibration{}, InclineObservationMode::SoftwareObservation));

    // Zero pulses, evaluate at 20ms and 100ms
    TEST_ASSERT_TRUE(sensor.evaluate(20));
    TEST_ASSERT_TRUE(sensor.evaluate(100));

    InclineState st = sensor.getState();
    TEST_ASSERT_EQUAL(InclineStatus::Stationary, st.status);
    TEST_ASSERT_FALSE(st.moving);
    TEST_ASSERT_TRUE(st.positionTrusted);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, st.estimatedInclinePct);

    sensor.end();
}

void test_transit_ramp_up_asymmetric() {
    InclineSensor sensor;
    InclineSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_2;
    cfg.movementStopTimeoutMs = 100;

    InclineCalibration cal{};
    cal.pulsesPerPercentUp = 3086.0f;
    cal.pulsesPerPercentDown = 2943.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, cal, InclineObservationMode::SoftwareObservation));
    VirtualInclineAdapter adapter(sensor);

    // Initial state: Stationary at 0.0%
    TEST_ASSERT_TRUE(sensor.evaluate(10));
    TEST_ASSERT_EQUAL(InclineStatus::Stationary, sensor.getState().status);

    // Inject 10 pulses up at t = 30ms -> should qualify movement
    InclinePulseObservation obs{};
    obs.pulseCount = 10;
    obs.direction = InclineDirection::Up;
    obs.signalValid = true;

    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 30));
    TEST_ASSERT_TRUE(sensor.evaluate(30));

    InclineState st = sensor.getState();
    TEST_ASSERT_TRUE(st.moving);
    TEST_ASSERT_EQUAL(InclineStatus::MovingUp, st.status);

    // Feed remaining pulses to reach 3086 total upward pulses over multiple ticks
    // 3086 - 10 = 3076 pulses. Split across 10 ticks (~307.6 pulses/tick)
    uint32_t nowMs = 30;
    uint32_t remaining = 3076;
    for (int i = 0; i < 10; ++i) {
        nowMs += 20;
        uint32_t chunk = (i == 9) ? remaining : 308;
        remaining -= chunk;

        obs.pulseCount = chunk;
        TEST_ASSERT_TRUE(adapter.injectObservation(obs, nowMs));
        TEST_ASSERT_TRUE(sensor.evaluate(nowMs));
    }

    st = sensor.getState();
    TEST_ASSERT_EQUAL_UINT32(3086, static_cast<uint32_t>(st.acceptedPulseCount));
    // Upward calibration: 3086 pulses / 3086.0 pulses/% = +1.000%
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.estimatedInclinePct);
    TEST_ASSERT_TRUE(st.moving);

    sensor.end();
}

void test_arrival_and_stop_settle() {
    InclineSensor sensor;
    InclineSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_2;
    cfg.movementStopTimeoutMs = 100;

    InclineCalibration cal{};
    cal.pulsesPerPercentUp = 3086.0f;
    cal.pulsesPerPercentDown = 2943.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, cal, InclineObservationMode::SoftwareObservation));
    VirtualInclineAdapter adapter(sensor);

    // Move to 1.0%
    InclinePulseObservation obs{};
    obs.pulseCount = 3086;
    obs.direction = InclineDirection::Up;
    obs.signalValid = true;

    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 50));
    TEST_ASSERT_TRUE(sensor.evaluate(50));
    TEST_ASSERT_TRUE(sensor.getState().moving);

    // Stop feeding pulses and advance time past 100ms stop timeout (t = 50ms + 120ms = 170ms)
    obs.pulseCount = 0;
    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 170));
    TEST_ASSERT_TRUE(sensor.evaluate(170));

    InclineState st = sensor.getState();
    TEST_ASSERT_FALSE(st.moving);
    TEST_ASSERT_EQUAL(InclineStatus::Stationary, st.status);
    TEST_ASSERT_TRUE(st.positionTrusted);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, st.estimatedInclinePct);

    sensor.end();
}

void test_transit_ramp_down_asymmetric() {
    InclineSensor sensor;
    InclineSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_2;
    cfg.movementStopTimeoutMs = 100;

    InclineCalibration cal{};
    cal.pulsesPerPercentUp = 3086.0f;
    cal.pulsesPerPercentDown = 2943.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, cal, InclineObservationMode::SoftwareObservation));
    VirtualInclineAdapter adapter(sensor);

    // Start by restoring position at 1.0%
    TEST_ASSERT_TRUE(sensor.restorePosition(1.0f, true));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, sensor.getState().estimatedInclinePct);

    // Move down with 2943 pulses (downward calibration: 2943 pulses / 2943.0 pulses/% = -1.000%)
    InclinePulseObservation obs{};
    obs.pulseCount = 2943;
    obs.direction = InclineDirection::Down;
    obs.signalValid = true;

    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 50));
    TEST_ASSERT_TRUE(sensor.evaluate(50));

    InclineState st = sensor.getState();
    TEST_ASSERT_TRUE(st.moving);
    TEST_ASSERT_EQUAL(InclineStatus::MovingDown, st.status);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.estimatedInclinePct);

    // Settle stop
    obs.pulseCount = 0;
    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 170));
    TEST_ASSERT_TRUE(sensor.evaluate(170));

    st = sensor.getState();
    TEST_ASSERT_FALSE(st.moving);
    TEST_ASSERT_EQUAL(InclineStatus::Stationary, st.status);
    TEST_ASSERT_TRUE(st.positionTrusted);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, st.estimatedInclinePct);

    sensor.end();
}

void test_limit_clamping() {
    InclineSensor sensor;
    InclineSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_2;
    cfg.minimumInclinePct = 0.0f;
    cfg.maximumInclinePct = 15.0f;
    cfg.movementStopTimeoutMs = 100;

    InclineCalibration cal{};
    cal.pulsesPerPercentUp = 1000.0f;
    cal.pulsesPerPercentDown = 1000.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, cal, InclineObservationMode::SoftwareObservation));
    VirtualInclineAdapter adapter(sensor);

    // Restore to 14.0%
    TEST_ASSERT_TRUE(sensor.restorePosition(14.0f, true));

    // Inject 2000 pulses Up (would reach 16.0% -> exceeds 15.0% max limit)
    InclinePulseObservation obs{};
    obs.pulseCount = 2000;
    obs.direction = InclineDirection::Up;
    obs.signalValid = true;

    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 50));
    TEST_ASSERT_TRUE(sensor.evaluate(50));
    TEST_ASSERT_TRUE(sensor.evaluate(60));

    InclineState st = sensor.getState();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, st.estimatedInclinePct);
    TEST_ASSERT_EQUAL(InclineStatus::PositionLimitReached, st.status);
    TEST_ASSERT_FALSE(st.positionTrusted);

    sensor.end();
}

void test_motor_stall_timeout() {
    InclineSensor sensor;
    InclineSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_2;
    cfg.maximumMovementTimeMs = 60000; // 60 seconds
    cfg.movementStopTimeoutMs = 100;

    TEST_ASSERT_TRUE(sensor.begin(cfg, InclineCalibration{}, InclineObservationMode::SoftwareObservation));
    VirtualInclineAdapter adapter(sensor);

    // Start movement at t = 100ms
    InclinePulseObservation obs{};
    obs.pulseCount = 10;
    obs.direction = InclineDirection::Up;
    obs.signalValid = true;

    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 100));
    TEST_ASSERT_TRUE(sensor.evaluate(100));
    TEST_ASSERT_TRUE(sensor.getState().moving);

    // Motor keeps sending pulses continuously until t = 60200ms (> 60000ms duration)
    obs.pulseCount = 1;
    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 60200));
    TEST_ASSERT_TRUE(sensor.evaluate(60200));

    InclineState st = sensor.getState();
    TEST_ASSERT_EQUAL(InclineStatus::MotionTimeout, st.status);
    TEST_ASSERT_FALSE(st.positionTrusted);

    sensor.end();
}

void test_pulse_lost_fault() {
    InclineSensor sensor;
    InclineSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_2;

    TEST_ASSERT_TRUE(sensor.begin(cfg, InclineCalibration{}, InclineObservationMode::SoftwareObservation));
    VirtualInclineAdapter adapter(sensor);

    // Fault injection: signalValid = false
    InclinePulseObservation obs{};
    obs.pulseCount = 0;
    obs.direction = InclineDirection::Unknown;
    obs.signalValid = false;

    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 100));
    TEST_ASSERT_TRUE(sensor.evaluate(100));

    InclineState st = sensor.getState();
    TEST_ASSERT_EQUAL(InclineStatus::HardwareError, st.status);
    TEST_ASSERT_FALSE(st.positionTrusted);

    sensor.end();
}

void test_orchestrator_external_step_integration() {
    auto orchestrator = std::unique_ptr<ApplicationOrchestrator>(new ApplicationOrchestrator());

    InclineSensor sensor;
    InclineSensorConfig inclineCfg{};
    inclineCfg.inputPin = GPIO_NUM_2;
    TEST_ASSERT_TRUE(sensor.begin(inclineCfg, InclineCalibration{}, InclineObservationMode::SoftwareObservation));

    VirtualInclineAdapter adapter(sensor);

    ApplicationOrchestratorDependencies deps{};
    deps.inclineSensor = &sensor;

    TEST_ASSERT_TRUE(orchestrator->begin(deps, OrchestratorExecutionMode::ExternalStep));
    TEST_ASSERT_TRUE(orchestrator->acceptsExternalSteps());

    // Step 1: ExternalStep execution at t = 20ms
    ApplicationTickContext ctx1(1, 20000, 20);
    TEST_ASSERT_TRUE(orchestrator->step(ctx1));

    ApplicationSnapshot snap1 = orchestrator->getSnapshot();
    TEST_ASSERT_EQUAL_UINT32(1, snap1.sequenceNumber);
    TEST_ASSERT_EQUAL(InclineStatus::Stationary, snap1.incline.status);
    TEST_ASSERT_TRUE(snap1.incline.positionTrusted);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, snap1.incline.estimatedInclinePct);

    // Step 2: Inject upward pulses via adapter and step at t = 40ms
    InclinePulseObservation obs{};
    obs.pulseCount = 100;
    obs.direction = InclineDirection::Up;
    obs.signalValid = true;
    TEST_ASSERT_TRUE(adapter.injectObservation(obs, 40));

    ApplicationTickContext ctx2(2, 40000, 20);
    TEST_ASSERT_TRUE(orchestrator->step(ctx2));

    ApplicationSnapshot snap2 = orchestrator->getSnapshot();
    TEST_ASSERT_EQUAL_UINT32(2, snap2.sequenceNumber);
    TEST_ASSERT_TRUE(snap2.incline.moving);
    TEST_ASSERT_EQUAL(InclineStatus::MovingUp, snap2.incline.status);
    TEST_ASSERT_TRUE(snap2.incline.positionTrusted);

    // Step 3: Test atomic rollback on step failure (e.g. non-monotonic tick)
    ApplicationTickContext invalidCtx(2, 40000, 20); // Duplicate tick index
    TEST_ASSERT_FALSE(orchestrator->step(invalidCtx));

    // Snapshot remains uncommitted at sequence 2
    ApplicationSnapshot snap3 = orchestrator->getSnapshot();
    TEST_ASSERT_EQUAL_UINT32(2, snap3.sequenceNumber);

    TEST_ASSERT_TRUE(orchestrator->end(100));
    sensor.end();
}

void run_all_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_incline_initialization_modes);
    RUN_TEST(test_mode_misuse_rejection);
    RUN_TEST(test_virtual_incline_adapter_forwarding);
    RUN_TEST(test_standstill_and_steady);
    RUN_TEST(test_transit_ramp_up_asymmetric);
    RUN_TEST(test_arrival_and_stop_settle);
    RUN_TEST(test_transit_ramp_down_asymmetric);
    RUN_TEST(test_limit_clamping);
    RUN_TEST(test_motor_stall_timeout);
    RUN_TEST(test_pulse_lost_fault);
    RUN_TEST(test_orchestrator_external_step_integration);
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

