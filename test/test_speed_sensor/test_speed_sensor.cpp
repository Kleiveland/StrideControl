#include <unity.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <cmath>
#include <cstring>
#include "SpeedSensor.h"

using namespace stridecontrol;

void test_speed_sensor_initialization_modes() {
    SpeedSensor sensor;
    TEST_ASSERT_FALSE(sensor.isReady());
    TEST_ASSERT_EQUAL(SpeedObservationMode::HardwareInterrupt, sensor.getObservationMode());

    SpeedSensorConfig cfg{};
    cfg.glitchRejectUs = 300;
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 2000;
    cfg.kmhPerHz = 1.1148f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));
    TEST_ASSERT_TRUE(sensor.isReady());
    TEST_ASSERT_EQUAL(SpeedObservationMode::SoftwareObservation, sensor.getObservationMode());

    SpeedSensorState state = sensor.getState();
    TEST_ASSERT_TRUE(state.initialized);
    TEST_ASSERT_FALSE(state.signalPresent);
    TEST_ASSERT_FALSE(state.measurementValid);
    TEST_ASSERT_EQUAL(SpeedSensorStatus::AwaitingFirstPulse, state.status);
    TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(state.acceptedPulseCount));
    TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(state.rejectedGlitchCount));
    TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(state.rejectedLockoutCount));
}

void test_speed_sensor_config_validation() {
    SpeedSensor sensor;
    SpeedSensorConfig cfg{};

    // 1. Zero glitch reject rejected
    cfg.glitchRejectUs = 0;
    TEST_ASSERT_FALSE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));

    // 2. Lockout < glitch rejected
    cfg.glitchRejectUs = 500;
    cfg.pulseLockoutUs = 400;
    TEST_ASSERT_FALSE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));

    // 3. Timeout == 0 or timeoutUs <= lockout rejected
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 1; // 1000us <= 2000us
    TEST_ASSERT_FALSE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));

    // 4. Invalid kmhPerHz rejected
    cfg.pulseTimeoutMs = 2000;
    cfg.kmhPerHz = 0.05f;
    TEST_ASSERT_FALSE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));
    cfg.kmhPerHz = 15.0f;
    TEST_ASSERT_FALSE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));
}

void test_glitch_rejection() {
    SpeedSensor sensor;
    SpeedSensorConfig cfg{};
    cfg.glitchRejectUs = 300;
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 2000;
    cfg.kmhPerHz = 1.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));

    const uint32_t t0 = 1000000; // 1s
    TEST_ASSERT_TRUE(sensor.observeEdge(t0));
    TEST_ASSERT_TRUE(sensor.evaluate(t0));

    SpeedSensorState state = sensor.getState();
    TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(state.acceptedPulseCount));
    TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(state.rejectedGlitchCount));
    TEST_ASSERT_EQUAL(SpeedSensorStatus::AwaitingInterval, state.status);

    // Inject glitch at t0 + 150us (< 300us)
    TEST_ASSERT_TRUE(sensor.observeEdge(t0 + 150));
    TEST_ASSERT_TRUE(sensor.evaluate(t0 + 150));

    state = sensor.getState();
    TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(state.acceptedPulseCount)); // not incremented
    TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(state.rejectedGlitchCount)); // incremented
    TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(state.rejectedLockoutCount));
    TEST_ASSERT_EQUAL(SpeedSensorStatus::AwaitingInterval, state.status);
}

void test_pulse_lockout_rejection() {
    SpeedSensor sensor;
    SpeedSensorConfig cfg{};
    cfg.glitchRejectUs = 300;
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 2000;
    cfg.kmhPerHz = 1.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));

    const uint32_t t0 = 1000000;
    TEST_ASSERT_TRUE(sensor.observeEdge(t0));

    // Inject edge at t0 + 1000us (>= 300us but < 2000us)
    TEST_ASSERT_TRUE(sensor.observeEdge(t0 + 1000));
    TEST_ASSERT_TRUE(sensor.evaluate(t0 + 1000));

    SpeedSensorState state = sensor.getState();
    TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(state.acceptedPulseCount));
    TEST_ASSERT_EQUAL_UINT32(0, static_cast<uint32_t>(state.rejectedGlitchCount));
    TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(state.rejectedLockoutCount));
}

void test_valid_interval_and_speed() {
    SpeedSensor sensor;
    SpeedSensorConfig cfg{};
    cfg.glitchRejectUs = 300;
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 2000;
    cfg.kmhPerHz = 1.25f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));

    const uint32_t t0 = 1000000;
    TEST_ASSERT_TRUE(sensor.observeEdge(t0));

    // Interval of 50,000us (50ms -> 20 Hz)
    const uint32_t dt = 50000;
    const uint32_t t1 = t0 + dt;
    TEST_ASSERT_TRUE(sensor.observeEdge(t1));
    TEST_ASSERT_TRUE(sensor.evaluate(t1));

    SpeedSensorState state = sensor.getState();
    TEST_ASSERT_EQUAL_UINT32(2, static_cast<uint32_t>(state.acceptedPulseCount));
    TEST_ASSERT_TRUE(state.signalPresent);
    TEST_ASSERT_TRUE(state.measurementValid);
    TEST_ASSERT_EQUAL(SpeedSensorStatus::Measuring, state.status);
    TEST_ASSERT_EQUAL_UINT32(50000, state.pulseIntervalUs);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, state.frequencyHz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, state.speedKmh); // 20 * 1.25
}

void test_timeout_and_rearm_cycle() {
    SpeedSensor sensor;
    SpeedSensorConfig cfg{};
    cfg.glitchRejectUs = 300;
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 2000; // 2.0s
    cfg.kmhPerHz = 1.0f;

    TEST_ASSERT_TRUE(sensor.begin(cfg, SpeedObservationMode::SoftwareObservation));

    const uint32_t t0 = 1000000;
    TEST_ASSERT_TRUE(sensor.observeEdge(t0));
    const uint32_t t1 = t0 + 100000; // 100ms interval
    TEST_ASSERT_TRUE(sensor.observeEdge(t1));
    TEST_ASSERT_TRUE(sensor.evaluate(t1));

    SpeedSensorState state = sensor.getState();
    TEST_ASSERT_EQUAL(SpeedSensorStatus::Measuring, state.status);
    TEST_ASSERT_TRUE(state.measurementValid);

    // Advance clock past 2.0s timeout (age = 2,000,001us)
    const uint32_t tTimeout = t1 + 2000001;
    TEST_ASSERT_TRUE(sensor.evaluate(tTimeout));

    state = sensor.getState();
    TEST_ASSERT_EQUAL(SpeedSensorStatus::TimedOut, state.status);
    TEST_ASSERT_FALSE(state.measurementValid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.speedKmh);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.frequencyHz);

    // Arrival of new edge after timeout: must act as reference edge (AwaitingInterval)
    const uint32_t t2 = tTimeout + 500000;
    TEST_ASSERT_TRUE(sensor.observeEdge(t2));
    TEST_ASSERT_TRUE(sensor.evaluate(t2));

    state = sensor.getState();
    TEST_ASSERT_EQUAL(SpeedSensorStatus::AwaitingInterval, state.status);
    TEST_ASSERT_FALSE(state.measurementValid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.speedKmh);

    // Subsequent edge forms new valid interval
    const uint32_t t3 = t2 + 80000; // 80ms interval -> 12.5 Hz
    TEST_ASSERT_TRUE(sensor.observeEdge(t3));
    TEST_ASSERT_TRUE(sensor.evaluate(t3));

    state = sensor.getState();
    TEST_ASSERT_EQUAL(SpeedSensorStatus::Measuring, state.status);
    TEST_ASSERT_TRUE(state.measurementValid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.5f, state.frequencyHz);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.5f, state.speedKmh);
}

void test_mode_isolation_and_teardown() {
    SpeedSensor sensor;
    SpeedSensorConfig cfg{};
    cfg.inputPin = GPIO_NUM_3;
    cfg.glitchRejectUs = 300;
    cfg.pulseLockoutUs = 2000;
    cfg.pulseTimeoutMs = 2000;
    cfg.kmhPerHz = 1.0f;

    // Hardware mode cannot observeEdge or evaluate
    TEST_ASSERT_TRUE(sensor.begin(cfg, SpeedObservationMode::HardwareInterrupt));
    TEST_ASSERT_FALSE(sensor.observeEdge(1000));
    TEST_ASSERT_FALSE(sensor.evaluate(1000));

    sensor.end();
    TEST_ASSERT_FALSE(sensor.isReady());
    TEST_ASSERT_EQUAL(SpeedSensorStatus::Uninitialized, sensor.getState().status);
}

void run_all_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_speed_sensor_initialization_modes);
    RUN_TEST(test_speed_sensor_config_validation);
    RUN_TEST(test_glitch_rejection);
    RUN_TEST(test_pulse_lockout_rejection);
    RUN_TEST(test_valid_interval_and_speed);
    RUN_TEST(test_timeout_and_rearm_cycle);
    RUN_TEST(test_mode_isolation_and_teardown);
    UNITY_END();
}

#ifdef ARDUINO
void setup() {
    delay(2000);
    run_all_tests();
}
void loop() {}
#else
int main(int argc, char** argv) {
    run_all_tests();
    return 0;
}
#endif

