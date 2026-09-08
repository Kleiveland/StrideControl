#pragma once

#if defined(STRIDECONTROL_TESTBENCH)

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "TreadmillSimulator.h"
#include "WorkoutSession.h"
#include "WorkoutSessionTypes.h"
#include "WorkoutDispatcher.h"
#include "ControlCoordinator.h"
#include "ControlRuntime.h"
#include "WorkoutEngine.h"
#include "ApplicationSnapshot.h"

namespace stridecontrol {

struct TestbenchTelemetry {
    ApplicationSnapshot snapshot{};
    WorkoutSessionSnapshot sessionSnapshot{};
    float simTargetSpeedKmh = 0.0f;
    float simTargetInclinePct = 0.0f;
    bool authoritative = false;
    uint32_t lostAuthorityCount = 0;
    uint32_t minFreeStackBytes = 0;
};

class TestbenchControlRuntime {
public:
    TestbenchControlRuntime();
    ~TestbenchControlRuntime();

    TestbenchControlRuntime(const TestbenchControlRuntime&) = delete;
    TestbenchControlRuntime& operator=(const TestbenchControlRuntime&) = delete;

    bool begin(const WorkoutSessionConfig& sessionConfig = WorkoutSessionConfig{});
    void end();

    bool armWorkout(const WorkoutDefinition& def, uint32_t nowMs);
    bool triggerQuickStart(float speedKmh, uint32_t nowMs);

    bool startControlTask();
    bool stopControlTask(uint32_t timeoutMs = 1000);

    bool isTaskRunning() const;

    ApplicationSnapshot getSnapshot() const;
    WorkoutSessionSnapshot getSessionSnapshot() const;
    TestbenchTelemetry getTelemetry(uint32_t nowMs) const;

    uint32_t getLostAuthorityCount() const;
    uint32_t getMinFreeStackBytes() const;

    static const char* version();

private:
    static void taskEntry(void* param);
    void runTaskLoop();

    mutable portMUX_TYPE snapshotMux_ = portMUX_INITIALIZER_UNLOCKED;

    TreadmillSimulator simulator_;
    WorkoutSession session_;
    WorkoutDispatcher dispatcher_;
    ControlCoordinator coordinator_;
    WorkoutEngine workoutEngine_;

    ApplicationSnapshot publishedSnapshot_{};
    WorkoutSessionSnapshot publishedSessionSnapshot_{};
    float publishedSimTargetSpeedKmh_ = 0.0f;
    float publishedSimTargetInclinePct_ = 0.0f;

    TaskHandle_t taskHandle_ = nullptr;
    SemaphoreHandle_t exitSem_ = nullptr;
    volatile bool taskRunning_ = false;
    volatile bool stopRequested_ = false;

    bool initialized_ = false;
    uint32_t lostAuthorityCount_ = 0;
    uint32_t minFreeStackBytes_ = 8192;

    float pendingQuickStartSpeed_ = -1.0f;
    uint32_t pendingQuickStartMs_ = 0;

    static constexpr uint32_t kPeriodMs = 20; // 50 Hz
    static constexpr uint32_t kTaskStackSize = 8192;
    static constexpr UBaseType_t kTaskPriority = 5;
    static constexpr BaseType_t kTaskCore = 0; // Pinned strictly to Core 0 (PRO_CPU)
};

} // namespace stridecontrol

#endif // STRIDECONTROL_TESTBENCH
