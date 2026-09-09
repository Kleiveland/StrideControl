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
#include "VirtualTreadmill.h"

using namespace stridecontrol;

// Helper to advance time by N ticks at given deltaMs
static void advanceTicks(VirtualTreadmill& vt, uint32_t count, uint32_t deltaMs, uint64_t& tickIndex, uint32_t& timestampMs) {
    for (uint32_t i = 0; i < count; ++i) {
        tickIndex++;
        timestampMs += deltaMs;
        SimulationTick tick{tickIndex, timestampMs, deltaMs};
        TEST_ASSERT_TRUE(vt.tick(tick));
    }
}

// 1. Determinism: Identical config + identical ticks + identical inputs produce bit-for-bit identical outputs
void test_determinism() {
    VirtualTreadmill vt1;
    VirtualTreadmill vt2;

    uint64_t tickIndex1 = 0, tickIndex2 = 0;
    uint32_t timestampMs1 = 1000, timestampMs2 = 1000;

    vt1.resetModel(timestampMs1);
    vt2.resetModel(timestampMs2);

    vt1.setTargetSpeedKmh(12.0f);
    vt2.setTargetSpeedKmh(12.0f);
    vt1.setTargetInclinePct(4.0f);
    vt2.setTargetInclinePct(4.0f);

    advanceTicks(vt1, 300, 20, tickIndex1, timestampMs1);
    advanceTicks(vt2, 300, 20, tickIndex2, timestampMs2);

    TEST_ASSERT_EQUAL_FLOAT(vt1.getActualSpeedKmh(), vt2.getActualSpeedKmh());
    TEST_ASSERT_EQUAL_FLOAT(vt1.getActualInclinePct(), vt2.getActualInclinePct());
    TEST_ASSERT_EQUAL_UINT64(vt1.getTachoOutput().totalPulses, vt2.getTachoOutput().totalPulses);
    TEST_ASSERT_EQUAL_UINT64(vt1.getInclineOutput().totalPulses, vt2.getInclineOutput().totalPulses);
    TEST_ASSERT_EQUAL_INT64(vt1.getInclineOutput().totalSignedPulses, vt2.getInclineOutput().totalSignedPulses);

    // Byte-level equivalence of outputs
    TEST_ASSERT_EQUAL_MEMORY(&vt1.getTachoOutput(), &vt2.getTachoOutput(), sizeof(TachoOutput));
    TEST_ASSERT_EQUAL_MEMORY(&vt1.getInclineOutput(), &vt2.getInclineOutput(), sizeof(InclineFeedbackOutput));
}

// 2. Acceleration: 1.0 -> 10.0 km/h follows 1.0 km/h/s without overshoot
void test_acceleration_rate_and_clamp() {
    VirtualTreadmill vt;
    uint64_t tickIndex = 0;
    uint32_t timestampMs = 0;
    vt.resetModel(timestampMs);

    // Initial step to 1.0 km/h
    vt.setTargetSpeedKmh(1.0f);
    advanceTicks(vt, 50, 20, tickIndex, timestampMs); // 1.0 second
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, vt.getActualSpeedKmh());

    // Accelerate from 1.0 to 10.0 km/h (+9.0 km/h takes exactly 9.0 seconds = 450 ticks @ 20ms)
    vt.setTargetSpeedKmh(10.0f);

    // Half-way: 4.5 seconds = 225 ticks -> Speed should be 1.0 + 4.5 = 5.5 km/h
    advanceTicks(vt, 225, 20, tickIndex, timestampMs);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 5.5f, vt.getActualSpeedKmh());

    // Target reached: another 225 ticks -> Speed should be exactly 10.0 km/h
    advanceTicks(vt, 225, 20, tickIndex, timestampMs);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, vt.getActualSpeedKmh());

    // Another 50 ticks: Must not overshoot 10.0 km/h
    advanceTicks(vt, 50, 20, tickIndex, timestampMs);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, vt.getActualSpeedKmh());
}

// 3. Deceleration: Normal stop follows 1.2 km/h/s; clamps exactly at 0.0
void test_normal_deceleration_and_clamp() {
    VirtualTreadmill vt;
    uint64_t tickIndex = 0;
    uint32_t timestampMs = 0;
    vt.resetModel(timestampMs);

    // Accelerate to 6.0 km/h
    vt.setTargetSpeedKmh(6.0f);
    advanceTicks(vt, 300, 20, tickIndex, timestampMs);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, vt.getActualSpeedKmh());

    // Decelerate to 0.0 km/h at 1.2 km/h/s (takes 6.0 / 1.2 = 5.0 seconds = 250 ticks)
    vt.setTargetSpeedKmh(0.0f);

    // Halfway: 2.5 seconds = 125 ticks -> Speed should be 6.0 - 3.0 = 3.0 km/h
    advanceTicks(vt, 125, 20, tickIndex, timestampMs);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 3.0f, vt.getActualSpeedKmh());

    // At 5.0 seconds: 125 more ticks -> Speed must be exactly 0.0 km/h
    advanceTicks(vt, 125, 20, tickIndex, timestampMs);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vt.getActualSpeedKmh());

    // Additional 50 ticks: Must stay clamped at 0.0 km/h (never negative)
    advanceTicks(vt, 50, 20, tickIndex, timestampMs);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vt.getActualSpeedKmh());
    TEST_ASSERT_FALSE(vt.isBeltMoving());
}

// 4. E-Stop Deceleration: Uses 3.5 km/h/s brake rate without instantaneous zeroing
void test_estop_deceleration() {
    VirtualTreadmill vt;
    uint64_t tickIndex = 0;
    uint32_t timestampMs = 0;
    vt.resetModel(timestampMs);

    // Accelerate to 14.0 km/h
    vt.setTargetSpeedKmh(14.0f);
    advanceTicks(vt, 700, 20, tickIndex, timestampMs);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 14.0f, vt.getActualSpeedKmh());

    // Trigger Emergency Stop
    vt.setEmergencyStop(true);
    TEST_ASSERT_TRUE(vt.isEStopActive());

    // Tick 1 (20 ms): Must NOT instantaneously zero!
    // Expected speed: 14.0 - (3.5 * 0.02) = 13.93 km/h
    tickIndex++;
    timestampMs += 20;
    TEST_ASSERT_TRUE(vt.tick({tickIndex, timestampMs, 20}));
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 13.93f, vt.getActualSpeedKmh());

    // Total stop time for 14.0 km/h at 3.5 km/h/s is 4.0 seconds = 200 ticks
    // Advance remaining 199 ticks
    advanceTicks(vt, 199, 20, tickIndex, timestampMs);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, vt.getActualSpeedKmh());
}

// 5. Incline Transit: Moves at 0.35 %/s without overshoot
void test_incline_transit() {
    VirtualTreadmill vt;
    uint64_t tickIndex = 0;
    uint32_t timestampMs = 0;
    vt.resetModel(timestampMs);

    // Incline from 0.0% to 7.0% at 0.35 %/s takes 7.0 / 0.35 = 20.0 seconds = 1000 ticks @ 20ms
    vt.setTargetInclinePct(7.0f);

    // Halfway: 10.0 seconds = 500 ticks -> Incline should be 3.5%
    advanceTicks(vt, 500, 20, tickIndex, timestampMs);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 3.5f, vt.getActualInclinePct());
    TEST_ASSERT_TRUE(vt.getInclineOutput().moving);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(InclineDirection::Up), static_cast<uint8_t>(vt.getInclineOutput().direction));

    // Target reached: 500 more ticks -> Incline should be exactly 7.0%
    advanceTicks(vt, 500, 20, tickIndex, timestampMs);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.0f, vt.getActualInclinePct());

    // Next tick: Incline has settled, moving must be false
    tickIndex++;
    timestampMs += 20;
    TEST_ASSERT_TRUE(vt.tick({tickIndex, timestampMs, 20}));
    TEST_ASSERT_EQUAL_FLOAT(7.0f, vt.getActualInclinePct());
    TEST_ASSERT_FALSE(vt.getInclineOutput().moving);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(InclineDirection::Unknown), static_cast<uint8_t>(vt.getInclineOutput().direction));
}

// 6. Tacho Fractional Math: Whole pulses + fractional remainder match integrated distance within strict numeric tolerance
void test_tacho_fractional_math() {
    VirtualTreadmill vt;
    uint64_t tickIndex = 0;
    uint32_t timestampMs = 0;
    vt.resetModel(timestampMs);

    vt.setTargetSpeedKmh(10.0f);
    advanceTicks(vt, 500, 20, tickIndex, timestampMs); // 10.0 seconds of mixed ramping and steady state

    const double odometerKm = vt.getOdometerKm();
    const uint64_t totalPulses = vt.getTachoOutput().totalPulses;
    const double remainder = vt.getTachoOutput().fractionalRemainder;

    const double reconstructedPulses = static_cast<double>(totalPulses) + remainder;
    const double expectedPulses = odometerKm * vt.getConfig().tachoPulsesPerKm;

    // Strict numerical tolerance (< 1e-6)
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, expectedPulses, reconstructedPulses);
    TEST_ASSERT_TRUE(remainder >= 0.0 && remainder < 1.0);
}

// 7. Incline Pulses: Pulse count matches physical elevation change
void test_incline_pulses() {
    VirtualTreadmill vt;
    uint64_t tickIndex = 0;
    uint32_t timestampMs = 0;
    vt.resetModel(timestampMs);

    // Transit 0% to 5% -> Delta 5.0%
    vt.setTargetInclinePct(5.0f);
    advanceTicks(vt, 800, 20, tickIndex, timestampMs); // Sufficient time to reach 5%

    const double expectedUpPulses = 5.0 * vt.getConfig().inclinePulsesPerPct; // 5 * 312 = 1560
    const double actualUpPulses = static_cast<double>(vt.getInclineOutput().totalPulses) + vt.getInclineOutput().fractionalRemainder;
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, expectedUpPulses, actualUpPulses);
    TEST_ASSERT_EQUAL_INT64(1560, vt.getInclineOutput().totalSignedPulses);

    // Transit 5% down to 2% -> Delta 3.0% down
    vt.setTargetInclinePct(2.0f);
    advanceTicks(vt, 500, 20, tickIndex, timestampMs);

    const double expectedTotalPulses = (5.0 + 3.0) * vt.getConfig().inclinePulsesPerPct; // 8 * 312 = 2496
    const double actualTotalPulses = static_cast<double>(vt.getInclineOutput().totalPulses) + vt.getInclineOutput().fractionalRemainder;
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, expectedTotalPulses, actualTotalPulses);
    TEST_ASSERT_EQUAL_INT64(624, vt.getInclineOutput().totalSignedPulses); // 2 * 312 = 624
}

// 8. resetModel(): Restores two differently exercised instances to identical baseline state
void test_reset_model() {
    VirtualTreadmill vtExercised;
    VirtualTreadmill vtFresh;

    uint64_t tickIndex = 0;
    uint32_t timestampMs = 0;
    vtExercised.resetModel(0);
    vtFresh.resetModel(0);

    // Heavily exercise instance 1
    vtExercised.setTargetSpeedKmh(18.0f);
    vtExercised.setTargetInclinePct(10.0f);
    vtExercised.setFault(VirtualTreadmillFault::TachoLostSignal, true);
    vtExercised.setRunnerLocation(VirtualRunnerLocation::OnBelt);
    vtExercised.setCadenceSpm(180);
    vtExercised.setHeartRateBpm(165);
    advanceTicks(vtExercised, 300, 20, tickIndex, timestampMs);

    // Now reset instance 1 to baseline 0
    vtExercised.resetModel(0);

    TEST_ASSERT_EQUAL_FLOAT(vtFresh.getActualSpeedKmh(), vtExercised.getActualSpeedKmh());
    TEST_ASSERT_EQUAL_FLOAT(vtFresh.getTargetSpeedKmh(), vtExercised.getTargetSpeedKmh());
    TEST_ASSERT_EQUAL_FLOAT(vtFresh.getActualInclinePct(), vtExercised.getActualInclinePct());
    TEST_ASSERT_EQUAL_FLOAT(vtFresh.getTargetInclinePct(), vtExercised.getTargetInclinePct());
    TEST_ASSERT_EQUAL_DOUBLE(vtFresh.getOdometerKm(), vtExercised.getOdometerKm());
    TEST_ASSERT_EQUAL_UINT64(vtFresh.getTachoOutput().totalPulses, vtExercised.getTachoOutput().totalPulses);
    TEST_ASSERT_EQUAL_UINT64(vtFresh.getInclineOutput().totalPulses, vtExercised.getInclineOutput().totalPulses);
    TEST_ASSERT_EQUAL_INT64(vtFresh.getInclineOutput().totalSignedPulses, vtExercised.getInclineOutput().totalSignedPulses);
    TEST_ASSERT_EQUAL_UINT16(vtFresh.getBiometricOutput().cadenceSpm, vtExercised.getBiometricOutput().cadenceSpm);
    TEST_ASSERT_EQUAL_UINT8(vtFresh.getBiometricOutput().heartRateBpm, vtExercised.getBiometricOutput().heartRateBpm);
}

// 9. Rollover safety on timestampMs
void test_timestamp_rollover_safety() {
    VirtualTreadmill vt;
    // Set initial timestamp near uint32_t overflow (e.g. 0xFFFFFFF0 = 4294967280)
    uint32_t ts = 0xFFFFFFF0;
    uint64_t idx = 100;
    vt.resetModel(ts);

    vt.setTargetSpeedKmh(10.0f);

    // Step 1: Normal tick before rollover
    idx++;
    ts += 20; // 0x00000004 (wraps over 0xFFFFFFFF)
    SimulationTick tick1{idx, ts, 20};
    TEST_ASSERT_TRUE(vt.tick(tick1));
    TEST_ASSERT_TRUE(vt.getActualSpeedKmh() > 0.0f);

    // Step 2: Next tick after rollover
    idx++;
    ts += 20; // 0x00000018
    SimulationTick tick2{idx, ts, 20};
    TEST_ASSERT_TRUE(vt.tick(tick2));

    // Reject out-of-order tick (backward timestamp)
    SimulationTick badTick{idx + 1, ts - 50, 20};
    TEST_ASSERT_FALSE(vt.tick(badTick));

    // Reject skipped tickIndex
    SimulationTick skippedTick{idx + 5, ts + 20, 20};
    TEST_ASSERT_FALSE(vt.tick(skippedTick));
}

// 10. Fault isolation: Fault flags alter only their dedicated physical output channel
void test_fault_isolation() {
    VirtualTreadmill vt;
    uint64_t tickIndex = 0;
    uint32_t timestampMs = 0;
    vt.resetModel(timestampMs);

    vt.setTargetSpeedKmh(10.0f);
    vt.setTargetInclinePct(4.0f);
    vt.setHeartRateBpm(140);
    advanceTicks(vt, 100, 20, tickIndex, timestampMs);

    // Inject Tacho fault
    vt.setFault(VirtualTreadmillFault::TachoLostSignal, true);
    tickIndex++;
    timestampMs += 20;
    TEST_ASSERT_TRUE(vt.tick({tickIndex, timestampMs, 20}));

    // Tacho output should be invalid and 0 pulses
    TEST_ASSERT_FALSE(vt.getTachoOutput().signalValid);
    TEST_ASSERT_EQUAL_UINT64(0, vt.getTachoOutput().pulsesThisTick);

    // Physical speed and incline MUST continue unaffected
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f * (100.0f * 0.02f * 1.0f / 10.0f), vt.getActualSpeedKmh());
    TEST_ASSERT_TRUE(vt.getInclineOutput().signalValid);
    TEST_ASSERT_TRUE(vt.getInclineOutput().moving);
    TEST_ASSERT_TRUE(vt.getBiometricOutput().signalValid);
    TEST_ASSERT_EQUAL_UINT8(140, vt.getBiometricOutput().heartRateBpm);

    // Clear tacho fault, inject HeartRate dropout
    vt.setFault(VirtualTreadmillFault::TachoLostSignal, false);
    vt.setFault(VirtualTreadmillFault::HeartRateDropout, true);
    tickIndex++;
    timestampMs += 20;
    TEST_ASSERT_TRUE(vt.tick({tickIndex, timestampMs, 20}));

    TEST_ASSERT_TRUE(vt.getTachoOutput().signalValid);
    TEST_ASSERT_FALSE(vt.getBiometricOutput().signalValid);
    TEST_ASSERT_EQUAL_UINT8(0, vt.getBiometricOutput().heartRateBpm);
}

void run_all_virtual_treadmill_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_determinism);
    RUN_TEST(test_acceleration_rate_and_clamp);
    RUN_TEST(test_normal_deceleration_and_clamp);
    RUN_TEST(test_estop_deceleration);
    RUN_TEST(test_incline_transit);
    RUN_TEST(test_tacho_fractional_math);
    RUN_TEST(test_incline_pulses);
    RUN_TEST(test_reset_model);
    RUN_TEST(test_timestamp_rollover_safety);
    RUN_TEST(test_fault_isolation);
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
    run_all_virtual_treadmill_tests();
}
void loop() {}
#else
int main(int argc, char** argv) {
    run_all_virtual_treadmill_tests();
    return 0;
}
#endif
