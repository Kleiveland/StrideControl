#pragma once

#include <cstdint>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "ControlCommand.h"
#include "../ConsoleInterface/ConsoleInterface.h"
#include "../SpeedCalibration/SpeedCalibration.h"
#include "../DiagnosticsService/DiagnosticsService.h"
#include "../TreadmillController/TreadmillController.h"
#include "../TreadmillController/TreadmillControllerAdapter.h"
#include "../WorkoutSession/WorkoutSession.h"
#include "../WorkoutEngine/WorkoutEngine.h"
#include "../WorkoutDispatcher/WorkoutDispatcher.h"
#include "../ControlCoordinator/ControlCoordinator.h"
#include "../ApplicationSnapshot/ApplicationSnapshot.h"
#include "../ApplicationOrchestrator/ApplicationOrchestrator.h"
#include "../InclineVerifier/InclineVerifierTypes.h"
#include "../RampTestTracker/RampTestTracker.h"

namespace stridecontrol {

/**
 * @brief Production runtime coordinating the workout execution and treadmill command chain on Core 0.
 *
 * Encapsulates TreadmillController, TreadmillControllerAdapter, WorkoutSession,
 * WorkoutDispatcher, and ControlCoordinator with strict dependency-safe lifetimes.
 * Enforces snapshot authority gating, lost-telemetry session freezing, and Core 0 serialization.
 */
class ControlRuntime : public IControlCommandStager {
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
     * @param snapshot The application snapshot to evaluate.
     * @param nowMs Current monotonic millisecond timestamp.
     * @param wasAuthoritative Optional pointer to transition-tracking state.
     */
    static bool isSnapshotAuthoritative(
        const ApplicationSnapshot& snapshot,
        uint32_t nowMs,
        bool* wasAuthoritative = nullptr
    );

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
    InclineVerificationCommandInput getInclineVerificationCommandInput() const;
    CommandExecutionStatus getCommandExecutionStatus() const;
    void publishCommandExecutionStatus();

    bool isRampTestActive() const override { return rampTestTracker_.active(); }
    bool isRampTestComplete() const override { return rampTestTracker_.complete(); }
    bool didRampTestTimeOut() const override { return rampTestTracker_.timedOut(); }
    uint32_t getRampTestDeadTimeMs() const override { return rampTestTracker_.deadTimeMs(); }
    uint32_t getRampTestTotalMs() const override { return rampTestTracker_.totalTimeMs(); }
    RampTestPhase getRampTestPhase() const { return rampTestTracker_.phase(); }

    bool isConnectionWarningActive() const { return connectionWarningActive_; }

    static const char* version();

    // Command Staging Producer Interface
    static constexpr size_t kCommandQueueDepth = 16;
    bool stageCommand(const ControlCommand& cmd) override {
        if (!commandQueue_) return false;
        return (xQueueSend(commandQueue_, &cmd, 0) == pdTRUE);
    }

    // Timing and task constants
    static constexpr uint32_t kPeriodMs = 20;            // 50 Hz control cadence
    static constexpr uint32_t kMaxSnapshotAgeMs = 200;   // 10 frames @ 50 Hz
    static constexpr uint32_t kTaskStackSize = 4096;
    static constexpr UBaseType_t kTaskPriority = 4;      // Above background BLE (3)
    static constexpr BaseType_t kTaskCore = 0;           // Dedicated Core 0 (PRO_CPU)

private:
    static void taskEntry(void* param);
    void runTaskLoop();
    void processQueuedCommands(uint32_t nowMs);
    void publishInclineVerificationCommandInput();
    void publishSnapshots();

    mutable portMUX_TYPE snapshotMux_ = portMUX_INITIALIZER_UNLOCKED;
    WorkoutSessionSnapshot publishedSessionSnapshot_{};
    TreadmillControllerSnapshot publishedControllerSnapshot_{};

    mutable portMUX_TYPE inclineCommandContextMux_ = portMUX_INITIALIZER_UNLOCKED;
    InclineVerificationCommandInput publishedInclineCommandContext_{};

    mutable portMUX_TYPE commandExecutionStatusMux_ = portMUX_INITIALIZER_UNLOCKED;
    CommandExecutionStatus publishedCommandExecutionStatus_{};
    uint32_t lastReportedAbortedRequestId_ = 0;

    QueueHandle_t commandQueue_ = nullptr;

    ConsoleInterface& console_;

    // STRICT MEMBER DECLARATION ORDER:
    // controller_ MUST precede adapter_ so adapter_ never outlives controller_
    TreadmillController controller_;
    TreadmillControllerAdapter adapter_;
    WorkoutSession session_;
    WorkoutEngine workoutEngine_;
    WorkoutDispatcher dispatcher_;
    ControlCoordinator coordinator_;

    CsafeMachineState previousCsafeQualifiedState_ = CsafeMachineState::Unknown;
    bool csafeStateInitialized_ = false;

    RampTestTracker rampTestTracker_;

    bool initialized_ = false;
    uint32_t lastAuthoritativeTimestampMs_ = 0;
    uint32_t authorityLostSinceMs_ = 0; // 0 means "not currently in a lost-authority streak"
    static constexpr uint32_t kAuthorityLossWarningThresholdMs = 5000;
    uint32_t lostAuthorityCount_ = 0;
    bool authorityLostReported_ = false;
    bool connectionWarningActive_ = false;
    bool wasAuthoritative_ = false;

    ApplicationOrchestrator* orchestrator_ = nullptr;
    TaskHandle_t taskHandle_ = nullptr;
    SemaphoreHandle_t exitSem_ = nullptr;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> taskRunning_{false};

    uint32_t loopCount_ = 0;
    uint32_t deadlineMissCount_ = 0;
    uint32_t overrunCount_ = 0;
};

} // namespace stridecontrol

