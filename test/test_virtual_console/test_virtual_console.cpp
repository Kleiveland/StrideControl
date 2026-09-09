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
#include "VirtualTreadmill.h"
#include "VirtualConsoleAdapter.h"

using namespace stridecontrol;

void test_console_execution_modes() {
    ConsoleInterface consoleSw;
    TEST_ASSERT_TRUE(consoleSw.begin(ConsoleExecutionMode::SoftwareSink));
    TEST_ASSERT_TRUE(consoleSw.isReady());
    TEST_ASSERT_EQUAL(ConsoleExecutionMode::SoftwareSink, consoleSw.getExecutionMode());
    consoleSw.end();
    TEST_ASSERT_FALSE(consoleSw.isReady());
}

void test_virtual_console_adapter_binding() {
    ConsoleInterface console;
    TEST_ASSERT_TRUE(console.begin(ConsoleExecutionMode::SoftwareSink));

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    TEST_ASSERT_EQUAL_UINT32(0, adapter.getTotalCommandsReceived());
    TEST_ASSERT_EQUAL_UINT32(0, adapter.getQuickStartCount());
    TEST_ASSERT_EQUAL_UINT32(0, adapter.getStopCount());
    TEST_ASSERT_EQUAL_UINT32(0, adapter.getEmergencyStopCount());

    console.end();
}

void test_t610_quickstart_contract() {
    ConsoleInterface console;
    console.begin(ConsoleExecutionMode::SoftwareSink);

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    // Initial state: 0 km/h, stopped
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, treadmill.getTargetSpeedKmh());

    // Pre-engage E-stop to test unlatch on QuickStart
    treadmill.setEmergencyStop(true);
    TEST_ASSERT_TRUE(treadmill.isEStopActive());

    // Submit QuickStart
    TreadmillCommand cmd{};
    cmd.requestId = 101;
    cmd.type = CommandType::PressButton;
    cmd.button = ButtonId::QuickStart;
    TEST_ASSERT_TRUE(console.submit(cmd, 0));

    // Assertions: T610 QuickStart starts at 1.0 km/h and clears E-Stop
    TEST_ASSERT_EQUAL_UINT32(1, adapter.getQuickStartCount());
    TEST_ASSERT_FALSE(treadmill.isEStopActive());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, treadmill.getTargetSpeedKmh());

    console.end();
}

void test_t610_delta_stepping_speed() {
    ConsoleInterface console;
    console.begin(ConsoleExecutionMode::SoftwareSink);

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    // Start at 1.0 km/h via QuickStart
    TreadmillCommand qs{};
    qs.requestId = 1;
    qs.type = CommandType::PressButton;
    qs.button = ButtonId::QuickStart;
    console.submit(qs, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, treadmill.getTargetSpeedKmh());

    // Step up: +0.1 km/h -> 1.1 km/h
    TreadmillCommand sp1{};
    sp1.requestId = 2;
    sp1.type = CommandType::PressSpeedPlus;
    TEST_ASSERT_TRUE(console.submit(sp1, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.1f, treadmill.getTargetSpeedKmh());

    // Step up: +0.1 km/h -> 1.2 km/h via ButtonId::SpeedPlus
    TreadmillCommand sp2{};
    sp2.requestId = 3;
    sp2.type = CommandType::PressButton;
    sp2.button = ButtonId::SpeedPlus;
    TEST_ASSERT_TRUE(console.submit(sp2, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.2f, treadmill.getTargetSpeedKmh());

    // Step down: -0.1 km/h -> 1.1 km/h
    TreadmillCommand sm1{};
    sm1.requestId = 4;
    sm1.type = CommandType::PressSpeedMinus;
    TEST_ASSERT_TRUE(console.submit(sm1, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.1f, treadmill.getTargetSpeedKmh());

    // Step down to 1.0, 0.9, 0.8
    sm1.requestId = 5; console.submit(sm1, 0); // 1.0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, treadmill.getTargetSpeedKmh());
    sm1.requestId = 6; console.submit(sm1, 0); // 0.9
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.9f, treadmill.getTargetSpeedKmh());
    sm1.requestId = 7; console.submit(sm1, 0); // 0.8 (T610 manual minimum)
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, treadmill.getTargetSpeedKmh());

    // Clamp at minimum 0.8 km/h: extra SpeedMinus MUST NOT drop below 0.8
    sm1.requestId = 8; console.submit(sm1, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, treadmill.getTargetSpeedKmh());

    // Jump to 21.9 km/h and test ceiling clamping at 22.0 km/h
    TreadmillCommand setSpeed{};
    setSpeed.requestId = 9;
    setSpeed.type = CommandType::SetSpeed;
    setSpeed.value = 21.9f;
    console.submit(setSpeed, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 21.9f, treadmill.getTargetSpeedKmh());

    sp1.requestId = 10; console.submit(sp1, 0); // 22.0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 22.0f, treadmill.getTargetSpeedKmh());

    sp1.requestId = 11; console.submit(sp1, 0); // clamped at 22.0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 22.0f, treadmill.getTargetSpeedKmh());

    console.end();
}

void test_t610_delta_stepping_incline() {
    ConsoleInterface console;
    console.begin(ConsoleExecutionMode::SoftwareSink);

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    // Initial incline: 0.0%
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, treadmill.getTargetInclinePct());

    // Step up: +0.5% -> 0.5%
    TreadmillCommand incPlus{};
    incPlus.requestId = 20;
    incPlus.type = CommandType::PressButton;
    incPlus.button = ButtonId::InclinePlus;
    TEST_ASSERT_TRUE(console.submit(incPlus, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, treadmill.getTargetInclinePct());

    // Step up: +0.5% -> 1.0%
    incPlus.requestId = 21;
    console.submit(incPlus, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, treadmill.getTargetInclinePct());

    // Step down: -0.5% -> 0.5%
    TreadmillCommand incMinus{};
    incMinus.requestId = 22;
    incMinus.type = CommandType::PressButton;
    incMinus.button = ButtonId::InclineMinus;
    TEST_ASSERT_TRUE(console.submit(incMinus, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, treadmill.getTargetInclinePct());

    // Step down to 0.0%
    incMinus.requestId = 23;
    console.submit(incMinus, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, treadmill.getTargetInclinePct());

    // Clamp at floor 0.0%: extra InclineMinus MUST NOT drop below 0.0%
    incMinus.requestId = 24;
    console.submit(incMinus, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, treadmill.getTargetInclinePct());

    // Set to 14.5% and test ceiling clamping at 15.0%
    TreadmillCommand setInc{};
    setInc.requestId = 25;
    setInc.type = CommandType::SetIncline;
    setInc.value = 14.5f;
    console.submit(setInc, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 14.5f, treadmill.getTargetInclinePct());

    incPlus.requestId = 26; console.submit(incPlus, 0); // 15.0%
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, treadmill.getTargetInclinePct());

    incPlus.requestId = 27; console.submit(incPlus, 0); // clamped at 15.0%
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, treadmill.getTargetInclinePct());

    console.end();
}

void test_t610_direct_keypad_targets() {
    ConsoleInterface console;
    console.begin(ConsoleExecutionMode::SoftwareSink);

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    // Direct SetSpeed 8.5 km/h
    TreadmillCommand cmdSpeed{};
    cmdSpeed.requestId = 30;
    cmdSpeed.type = CommandType::SetSpeed;
    cmdSpeed.value = 8.5f;
    TEST_ASSERT_TRUE(console.submit(cmdSpeed, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 8.5f, treadmill.getTargetSpeedKmh());

    // Direct SetIncline 6.0%
    TreadmillCommand cmdInc{};
    cmdInc.requestId = 31;
    cmdInc.type = CommandType::SetIncline;
    cmdInc.value = 6.0f;
    TEST_ASSERT_TRUE(console.submit(cmdInc, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, treadmill.getTargetInclinePct());

    // Direct SetSpeed out-of-range high clamped to 22.0
    cmdSpeed.requestId = 32;
    cmdSpeed.value = 30.0f;
    TEST_ASSERT_TRUE(console.submit(cmdSpeed, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 22.0f, treadmill.getTargetSpeedKmh());

    // Direct SetIncline out-of-range high clamped to 15.0
    cmdInc.requestId = 33;
    cmdInc.value = 25.0f;
    TEST_ASSERT_TRUE(console.submit(cmdInc, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, treadmill.getTargetInclinePct());

    console.end();
}

void test_t610_stop_logic_pause() {
    ConsoleInterface console;
    console.begin(ConsoleExecutionMode::SoftwareSink);

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    // Running at 12.0 km/h, incline 3.5%
    TreadmillCommand s{};
    s.requestId = 40; s.type = CommandType::SetSpeed; s.value = 12.0f;
    console.submit(s, 0);
    TreadmillCommand inc{};
    inc.requestId = 41; inc.type = CommandType::SetIncline; inc.value = 3.5f;
    console.submit(inc, 0);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, treadmill.getTargetSpeedKmh());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.5f, treadmill.getTargetInclinePct());

    // Press Stop (pause behavior)
    TreadmillCommand stopCmd{};
    stopCmd.requestId = 42;
    stopCmd.type = CommandType::PressButton;
    stopCmd.button = ButtonId::Stop;
    TEST_ASSERT_TRUE(console.submit(stopCmd, 0));

    // Stop zeroes speed target (ramps down to pause)
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, treadmill.getTargetSpeedKmh());
    // Incline is preserved
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.5f, treadmill.getTargetInclinePct());
    // E-Stop is NOT latched by normal stop
    TEST_ASSERT_FALSE(treadmill.isEStopActive());
    TEST_ASSERT_EQUAL_UINT32(1, adapter.getStopCount());

    console.end();
}

void test_emergency_stop_contract() {
    ConsoleInterface console;
    console.begin(ConsoleExecutionMode::SoftwareSink);

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    // Running at 10.0 km/h
    TreadmillCommand runCmd{};
    runCmd.requestId = 50; runCmd.type = CommandType::SetSpeed; runCmd.value = 10.0f;
    console.submit(runCmd, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, treadmill.getTargetSpeedKmh());

    // Trigger Emergency Stop
    console.triggerEmergencyStop(true);
    TEST_ASSERT_TRUE(treadmill.isEStopActive());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, treadmill.getTargetSpeedKmh());
    TEST_ASSERT_EQUAL_UINT32(1, adapter.getEmergencyStopCount());

    // Commands while E-Stop active MUST be rejected
    TreadmillCommand rejectedSpeed{};
    rejectedSpeed.requestId = 51;
    rejectedSpeed.type = CommandType::SetSpeed;
    rejectedSpeed.value = 8.0f;
    TEST_ASSERT_FALSE(console.submit(rejectedSpeed, 0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, treadmill.getTargetSpeedKmh());

    // CommandEvent queued was Rejected
    CommandEvent ev{};
    bool foundRejected = false;
    while (console.receiveCommandEvent(ev, 0)) {
        if (ev.requestId == 51 && ev.status == CommandStatus::Rejected) {
            foundRejected = true;
        }
    }
    TEST_ASSERT_TRUE(foundRejected);

    // QuickStart clears E-Stop and resets to 1.0 km/h
    TreadmillCommand qs{};
    qs.requestId = 52;
    qs.type = CommandType::PressButton;
    qs.button = ButtonId::QuickStart;
    TEST_ASSERT_TRUE(console.submit(qs, 0));

    TEST_ASSERT_FALSE(treadmill.isEStopActive());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, treadmill.getTargetSpeedKmh());

    console.end();
}

void test_treadmill_controller_correlation() {
    ConsoleInterface console;
    console.begin(ConsoleExecutionMode::SoftwareSink);

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    SpeedCalibration calibration;
    DiagnosticsService diagnostics;
    TreadmillController controller(console, calibration, diagnostics);
    TEST_ASSERT_TRUE(controller.begin());
    TEST_ASSERT_TRUE(controller.isReady());

    // Submit Speed Target: 7.0 km/h
    TEST_ASSERT_TRUE(controller.submitSpeedTarget(7.0f, 1000));
    TEST_ASSERT_TRUE(controller.isBusy());

    // Update controller to process console ACK
    controller.update(1005);
    TEST_ASSERT_FALSE(controller.isBusy());
    TEST_ASSERT_EQUAL(TreadmillControllerState::Completed, controller.getSnapshot().state);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.0f, treadmill.getTargetSpeedKmh());

    // Submit Incline Target: 4.0%
    TEST_ASSERT_TRUE(controller.submitInclineTarget(4.0f, 1010));
    TEST_ASSERT_TRUE(controller.isBusy());

    // Update controller to process console ACK
    controller.update(1015);
    TEST_ASSERT_FALSE(controller.isBusy());
    TEST_ASSERT_EQUAL(TreadmillControllerState::Completed, controller.getSnapshot().state);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, treadmill.getTargetInclinePct());

    console.end();
}

void test_software_sink_button_press_bridge() {
    ConsoleInterface console;
    console.begin(ConsoleExecutionMode::SoftwareSink);

    VirtualTreadmill treadmill;
    VirtualConsoleAdapter adapter(treadmill);
    console.registerCommandSink(&adapter);

    AckMetrics metrics{};
    ConsoleOutcome outcome = console.pressButton(ButtonId::QuickStart, 82, metrics);
    TEST_ASSERT_EQUAL(ConsoleOutcome::NormalSingle, outcome);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, treadmill.getTargetSpeedKmh());

    outcome = console.pressButton(ButtonId::Stop, 82, metrics);
    TEST_ASSERT_EQUAL(ConsoleOutcome::NormalSingle, outcome);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, treadmill.getTargetSpeedKmh());

    console.end();
}

void run_all_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_console_execution_modes);
    RUN_TEST(test_virtual_console_adapter_binding);
    RUN_TEST(test_t610_quickstart_contract);
    RUN_TEST(test_t610_delta_stepping_speed);
    RUN_TEST(test_t610_delta_stepping_incline);
    RUN_TEST(test_t610_direct_keypad_targets);
    RUN_TEST(test_t610_stop_logic_pause);
    RUN_TEST(test_emergency_stop_contract);
    RUN_TEST(test_treadmill_controller_correlation);
    RUN_TEST(test_software_sink_button_press_bridge);
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
