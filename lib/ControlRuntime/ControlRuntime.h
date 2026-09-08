#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "../ConsoleInterface/ConsoleInterface.h"
#include "../SpeedCalibration/SpeedCalibration.h"
#include "../DiagnosticsService/DiagnosticsService.h"
#include "../TreadmillController/TreadmillController.h"
#include "../TreadmillController/TreadmillControllerAdapter.h"
#include "../WorkoutSession/WorkoutSession.h"
#include "../WorkoutDispatcher/WorkoutDispatcher.h"
#include "../ControlCoordinator/ControlCoordinator.h"
#include "../ApplicationSnapshot/ApplicationSnapshot.h"
#include "../ApplicationOrchestrator/ApplicationOrchestrator.h"

namespace stridecontrol {

/**
 * @brief Production runtime coordinating the workout execution and treadmill command chain on Core 0.
 *
 * Encapsulates TreadmillController, TreadmillControllerAdapter, WorkoutSession,
 * WorkoutDispatcher, and ControlCoordinator with strict dependency-safe lifetimes.
 * Enforces snapshot authority gating, lost-telemetry session freezing, and Core 0 serialization.
 */
class ControlRuntime {
public:
    ControlRuntime(
        ConsoleInterface& console,
        SpeedCalibration& calibration,
        DiagnosticsService& diagnostics
    );

    ~ControlRuntime();

    ControlRuntime(const ControlRuntime&) = delete;
    ControlRuntime& operator=(const ControlRuntime&) = delete;

    bool begin(const WorkoutSessionConfig& sessionConfig = WorkoutSessionConfig{});
    void end();

    /**
     * @brief Synchronous single-tick update. Must be called exclusively from Core 0.
     */
    void update(const ApplicationSnapshot& snapshot, uint32_t nowMs);

    /**
     * @brief Snapshot authority evaluation predicate.
     */
    static bool isSnapshotAuthoritative(const ApplicationSnapshot& snapshot, uint32_t nowMs);

    // Workout session lifecycle and rebinding safety guards
    bool armWorkout(const ExpandedWorkout* workout, uint32_t nowMs);
    bool abortWorkout(uint32_t nowMs);
    bool finalizeWorkout(uint32_t nowMs);
    bool canSafelyModifyWorkoutEngine() const;

    // Dedicated Core 0 FreeRTOS task management
    bool startControlTask(ApplicationOrchestrator* orchestrator);
    bool stopControlTask(uint32_t timeoutMs = 1000);
    bool isTaskRunning() const;

    // Read-only snapshots and queries
    WorkoutSessionSnapshot getSessionSnapshot() const;
    TreadmillControllerSnapshot getControllerSnapshot() const;
    StagedTargets getStagedTargets() const;
    bool isSessionActive() const;
    bool isSessionSuspended() const;
    bool isControllerReady() const;
    bool isControllerBusy() const;
    uint32_t getLostAuthorityCount() const;
    uint32_t getLoopCount() const;
    uint32_t getDeadlineMissCount() const;
    uint32_t getOverrunCount() const;

    static const char* version();

    // Timing and task constants
    static constexpr uint32_t kPeriodMs = 20;            // 50 Hz control cadence
    static constexpr uint32_t kMaxSnapshotAgeMs = 200;   // 10 frames @ 50 Hz
    static constexpr uint32_t kTaskStackSize = 4096;
    static constexpr UBaseType_t kTaskPriority = 4;      // Above background BLE (3)
    static constexpr BaseType_t kTaskCore = 0;           // Dedicated Core 0 (PRO_CPU)

private:
    static void taskEntry(void* param);
    void runTaskLoop();

    // STRICT MEMBER DECLARATION ORDER:
    // controller_ MUST precede adapter_ so adapter_ never outlives controller_
    TreadmillController controller_;
    TreadmillControllerAdapter adapter_;
    WorkoutSession session_;
    WorkoutDispatcher dispatcher_;
    ControlCoordinator coordinator_;

    bool initialized_ = false;
    uint32_t lastAuthoritativeTimestampMs_ = 0;
    uint32_t lostAuthorityCount_ = 0;
    bool authorityLostReported_ = false;

    ApplicationOrchestrator* orchestrator_ = nullptr;
    TaskHandle_t taskHandle_ = nullptr;
    SemaphoreHandle_t exitSem_ = nullptr;
    volatile bool stopRequested_ = false;
    volatile bool taskRunning_ = false;

    uint32_t loopCount_ = 0;
    uint32_t deadlineMissCount_ = 0;
    uint32_t overrunCount_ = 0;
};

} // namespace stridecontrol

