#pragma once

#if defined(STRIDECONTROL_TESTBENCH)

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "ControlCommand.h"
#include "TreadmillSimulatorComposite.h"
#include "SpeedSensor.h"
#include "InclineSensor.h"
#include "ConsoleInterface.h"
#include "ImuInterface.h"
#include "RunnerDynamics.h"
#include "DiagnosticsService.h"
#include "ApplicationOrchestrator.h"
#include "WorkoutSession.h"
#include "WorkoutSessionTypes.h"
#include "WorkoutDispatcher.h"
#include "ControlCoordinator.h"
#include "ControlRuntime.h"
#include "WorkoutEngine.h"
#include "ApplicationSnapshot.h"
#include "BleManager.h"
#include "HeartRateClient.h"
#include "RscService.h"
#include "FtmsService.h"

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

class TestbenchControlRuntime : public IControlCommandStager {
public:
    TestbenchControlRuntime();
    ~TestbenchControlRuntime();

    TestbenchControlRuntime(const TestbenchControlRuntime&) = delete;
    TestbenchControlRuntime& operator=(const TestbenchControlRuntime&) = delete;

    bool begin(const WorkoutSessionConfig& sessionConfig = WorkoutSessionConfig{},
               const BleConfig& bleConfig = BleConfig{});
    void end();

    bool armWorkout(const WorkoutDefinition& def, uint32_t nowMs);
    
    // Simulator stimulus staging APIs
    bool triggerQuickStart(uint32_t nowMs = 0);
    bool triggerStop(uint32_t nowMs = 0);
    bool triggerEmergencyStop(uint32_t nowMs = 0);
    bool setSimSpeedTarget(float speedKmh, uint32_t nowMs = 0);
    bool setSimInclineTarget(float inclinePct, uint32_t nowMs = 0);
    bool stepSimSpeed(bool positive, uint32_t nowMs = 0);
    bool stepSimIncline(bool positive, uint32_t nowMs = 0);
    void setSimRunner(VirtualRunnerMode mode, uint16_t cadenceSpm = 180, float magnitudeG = 0.35f, bool valid = true);
    void setSimHeartRateFromSpeed(bool enabled);

    bool startControlTask();
    bool stopControlTask(uint32_t timeoutMs = 1000);

    bool isTaskRunning() const;

    ApplicationSnapshot getSnapshot() const;
    WorkoutSessionSnapshot getSessionSnapshot() const;
    StagedTargets getStagedTargets() const { return dispatcher_.getStagedTargets(); }
    TestbenchTelemetry getTelemetry(uint32_t nowMs) const;

    uint32_t getLostAuthorityCount() const;
    uint32_t getMinFreeStackBytes() const;

    bool isRampTestActive() const override { return rampTestActive_; }
    bool isRampTestComplete() const override { return rampTestComplete_; }
    bool didRampTestTimeOut() const override { return rampTestTimedOut_; }
    uint32_t getRampTestDeadTimeMs() const override { return rampTestDeadTimeMs_; }
    uint32_t getRampTestTotalMs() const override { return rampTestTotalMs_; }

    TreadmillSimulatorComposite& getComposite() { return composite_; }
    const TreadmillSimulatorComposite& getComposite() const { return composite_; }

    static const char* version();

    // Command Staging Producer Interface
    static constexpr size_t kCommandQueueDepth = 16;
    bool stageCommand(const ControlCommand& cmd) override {
        if (!commandQueue_) return false;
        return (xQueueSend(commandQueue_, &cmd, 0) == pdTRUE);
    }

private:
    static void taskEntry(void* param);
    void runTaskLoop();
    void processQueuedCommands(uint32_t nowMs);

    QueueHandle_t commandQueue_ = nullptr;
    mutable portMUX_TYPE snapshotMux_ = portMUX_INITIALIZER_UNLOCKED;

    // Production Sensor & Domain Drivers (SoftwareObservation Mode)
    SpeedSensor speedSensor_;
    InclineSensor inclineSensor_;
    ConsoleInterface console_;
    ImuInterface imu_;
    RunnerDynamics runnerDynamics_;
    DiagnosticsService diagService_;
    ApplicationOrchestrator orchestrator_;
    BleManager bleManager_;
    HeartRateClient heartRateClient_;
    RscService rscService_;
    FtmsService ftmsService_;
    volatile bool simHeartRateFromSpeedEnabled_ = false;

    // Passive Composite Simulator
    TreadmillSimulatorComposite composite_;

    // Domain Session & Coordination
    WorkoutSession session_;
    WorkoutDispatcher dispatcher_;
    ControlCoordinator coordinator_;
    WorkoutEngine workoutEngine_;

    volatile bool rampTestActive_ = false;
    volatile bool rampTestComplete_ = false;
    volatile bool rampTestTimedOut_ = false;
    uint32_t rampTestStartMs_ = 0;
    float rampTestStartSpeedKmh_ = 0.0f;
    float rampTestTargetSpeedKmh_ = 0.0f;
    uint32_t rampTestDeadTimeMs_ = 0;
    uint32_t rampTestTotalMs_ = 0;
    static constexpr uint32_t kRampTestTimeoutMs = 30000;
    static constexpr float kRampTestMoveThresholdKmh = 0.1f;
    static constexpr float kRampTestArrivalToleranceKmh = 0.3f;

    ApplicationSnapshot publishedSnapshot_{};
    WorkoutSessionSnapshot publishedSessionSnapshot_{};
    float publishedSimTargetSpeedKmh_ = 0.0f;
    float publishedSimTargetInclinePct_ = 0.0f;
    bool publishedAuthoritative_ = false;

    TaskHandle_t taskHandle_ = nullptr;
    SemaphoreHandle_t exitSem_ = nullptr;
    volatile bool taskRunning_ = false;
    volatile bool stopRequested_ = false;

    bool initialized_ = false;
    CsafeMachineState previousCsafeQualifiedState_ = CsafeMachineState::Unknown;
    bool csafeStateInitialized_ = false;
    uint32_t lostAuthorityCount_ = 0;
    uint32_t authorityLostSinceMs_ = 0; // 0 means "not currently in a lost-authority streak"
    static constexpr uint32_t kAuthorityLossSuspendThresholdMs = 2000;
    uint32_t minFreeStackBytes_ = 8192;

    static constexpr uint32_t kPeriodMs = 20; // 50 Hz
    static constexpr uint32_t kTaskStackSize = 8192;
    static constexpr UBaseType_t kTaskPriority = 5;
    static constexpr BaseType_t kTaskCore = 1; // Moved off Core 0 to reduce WiFi/BT radio task contention
};

} // namespace stridecontrol

#endif // STRIDECONTROL_TESTBENCH
