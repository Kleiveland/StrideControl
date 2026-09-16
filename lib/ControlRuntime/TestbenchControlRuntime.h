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
#include "InclineVerifier.h"
#include "RampTestTracker.h"

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
    static CsafeState provideSimulatedCsafe(void* context);
    static InclineVerificationCommandInput provideSimulatedInclineCommandContext(void* context);
    static CommandExecutionStatus provideSimulatedCommandExecutionStatus(void* context);

    class TargetSinkWrapper : public IWorkoutTargetSink {
    public:
        TargetSinkWrapper(TreadmillSimulatorComposite& composite,
                          InclineVerificationCommandInput& context,
                          portMUX_TYPE& mux)
            : composite_(composite), context_(context), mux_(mux) {}

        bool submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) override {
            return composite_.submitSpeedTarget(targetSpeedKmh, nowMs);
        }

        bool submitInclineTarget(float targetInclinePct, uint32_t nowMs) override {
            const bool ok = composite_.submitInclineTarget(targetInclinePct, nowMs);
            if (ok) {
                portENTER_CRITICAL(&mux_);
                context_.targetInclinePct = targetInclinePct;
                context_.targetInclineValid = true;
                context_.commandSequence++;
                if (context_.commandSequence == 0) {
                    context_.commandSequence++;
                }
                context_.commandTimestampMs = nowMs;
                context_.commandTimestampValid = (nowMs != 0);
                portEXIT_CRITICAL(&mux_);
            }
            return ok;
        }

        bool isBusy() const override {
            return composite_.isBusy();
        }

        bool isReady() const override {
            return composite_.isReady();
        }

    private:
        TreadmillSimulatorComposite& composite_;
        InclineVerificationCommandInput& context_;
        portMUX_TYPE& mux_;
    };

    void runTaskLoop();
    void processQueuedCommands(uint32_t nowMs);

    QueueHandle_t commandQueue_ = nullptr;
    mutable portMUX_TYPE snapshotMux_ = portMUX_INITIALIZER_UNLOCKED;
    mutable portMUX_TYPE inclineCommandContextMux_ = portMUX_INITIALIZER_UNLOCKED;
    InclineVerificationCommandInput publishedInclineCommandContext_{};

    mutable portMUX_TYPE commandExecutionStatusMux_ = portMUX_INITIALIZER_UNLOCKED;
    CommandExecutionStatus publishedCommandExecutionStatus_{};
    uint32_t lastReportedAbortedRequestId_ = 0;

    // Production Sensor & Domain Drivers (SoftwareObservation Mode)
    SpeedSensor speedSensor_;
    InclineSensor inclineSensor_;
    ConsoleInterface console_;
    ImuInterface imu_;
    RunnerDynamics runnerDynamics_;
    InclineVerifier inclineVerifier_;
    DiagnosticsService diagService_;
    ApplicationOrchestrator orchestrator_;
    BleManager bleManager_;
    HeartRateClient heartRateClient_;
    RscService rscService_;
    FtmsService ftmsService_;
    volatile bool simHeartRateFromSpeedEnabled_ = false;

    // Passive Composite Simulator
    TreadmillSimulatorComposite composite_;
    TargetSinkWrapper targetSink_{composite_, publishedInclineCommandContext_, inclineCommandContextMux_};

    // Domain Session & Coordination
    WorkoutSession session_;
    WorkoutDispatcher dispatcher_;
    ControlCoordinator coordinator_;
    WorkoutEngine workoutEngine_;
    RampTestTracker rampTestTracker_;

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
    static constexpr uint32_t kAuthorityLossWarningThresholdMs = 5000;
    bool authorityLostReported_ = false;
    bool connectionWarningActive_ = false;
    bool wasAuthoritative_ = false;
    uint32_t minFreeStackBytes_ = 8192;

    static constexpr uint32_t kPeriodMs = 20; // 50 Hz
    static constexpr uint32_t kTaskStackSize = 8192;
    static constexpr UBaseType_t kTaskPriority = 5;
    static constexpr BaseType_t kTaskCore = 1; // Moved off Core 0 to reduce WiFi/BT radio task contention
};

} // namespace stridecontrol

#endif // STRIDECONTROL_TESTBENCH
