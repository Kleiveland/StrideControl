#pragma once

#if defined(STRIDECONTROL_TESTBENCH)

#include <cstdint>
#include <atomic>
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
#include "HeartRateService.h"
#include "InclineVerifier.h"
#include "RampTestTracker.h"
#include "SpeedCalibration.h"
#include "../SpeedLearningTracker/SpeedLearningTracker.h"

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
    bool triggerEmergencyStop(bool active = true, uint32_t nowMs = 0);
    bool setSimSpeedTarget(float speedKmh, uint32_t nowMs = 0);
    bool setSimInclineTarget(float inclinePct, uint32_t nowMs = 0);
    bool stepSimSpeed(bool positive, uint32_t nowMs = 0);
    bool stepSimIncline(bool positive, uint32_t nowMs = 0);
    void setSimRunner(VirtualRunnerMode mode, uint16_t cadenceSpm = 180, float magnitudeG = 0.35f, bool valid = true);
    void setSimBeltSpeedDirect(float speedKmh) { composite_.setDirectBeltSpeedKmh(speedKmh); }
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
    void requestInclineCommissioningAction(bool confirmHomed, bool zeroImu);

    bool isRampTestActive() const override { return rampTestTracker_.active(); }
    bool isRampTestComplete() const override { return rampTestTracker_.complete(); }
    bool didRampTestTimeOut() const override { return rampTestTracker_.timedOut(); }
    uint32_t getRampTestDeadTimeMs() const override { return rampTestTracker_.deadTimeMs(); }
    uint32_t getRampTestTotalMs() const override { return rampTestTracker_.totalTimeMs(); }
    float getMaxAchievableSpeedKmh() const override { return speedCalibration_.getMaxAchievableSpeedKmh(); }
    bool isMaxAchievableSpeedVerified() const override { return speedCalibration_.isMaxAchievableSpeedVerified(); }
    RampTestPhase getRampTestPhase() const { return rampTestTracker_.phase(); }
    InclineCommissioningPhase getInclineCommissioningPhase() const override { return inclineTracker_.phase(); }
    uint8_t getInclineCommissioningPointCount() const override { return inclineTracker_.pointCount(); }
    bool didInclineCommissioningTimeOut() const override { return inclineTracker_.timedOut(); }
    void onSpeedConfigUpdated(const SpeedConfig& config) override {
        speedCalibration_.setConfiguration(config);
        speedLearningTracker_.setActiveSpeedConfig(config);
    }
    SpeedCalibrationResult calculateSpeedCommand(float physicalSpeedKmh) const override {
        return speedCalibration_.calculateCommand(physicalSpeedKmh);
    }
    uint8_t getSpeedAdaptationLog(SpeedAdaptationLogEntry* outEntries, uint8_t maxEntries) const override {
        return speedLearningTracker_.getAuditLog(outEntries, maxEntries);
    }

    bool isConnectionWarningActive() const { return connectionWarningActive_; }
    bool isEmergencyStopActive() const { return console_.isEmergencyStopActive(); }

    SpeedSensor& getSpeedSensor() { return speedSensor_; }
    const SpeedSensor& getSpeedSensor() const { return speedSensor_; }
    SpeedCalibration& getSpeedCalibration() { return speedCalibration_; }
    const SpeedCalibration& getSpeedCalibration() const { return speedCalibration_; }

    TreadmillSimulatorComposite& getComposite() { return composite_; }
    const TreadmillSimulatorComposite& getComposite() const { return composite_; }
    HeartRateClient& getHeartRateClient() { return heartRateClient_; }
    const HeartRateClient& getHeartRateClient() const { return heartRateClient_; }
    const BleConfig& getBleConfig() const { return bleConfig_; }

    static const char* version();

    // Command Staging Producer Interface
    static constexpr size_t kCommandQueueDepth = 16;
    bool stageCommand(const ControlCommand& cmd) override {
        if (!commandQueue_) return false;
        return (xQueueSend(commandQueue_, &cmd, 0) == pdTRUE);
    }

private:
    static void taskEntry(void* param);
    static void bleTaskEntry(void* param);
    static CsafeState provideSimulatedCsafe(void* context);
    static InclineVerificationCommandInput provideSimulatedInclineCommandContext(void* context);
    static CommandExecutionStatus provideSimulatedCommandExecutionStatus(void* context);
    static SessionTelemetryInput provideSimulatedSessionTelemetry(void* context);

    class TargetSinkWrapper : public IWorkoutTargetSink {
    public:
        TargetSinkWrapper(TreadmillSimulatorComposite& composite,
                          InclineVerificationCommandInput& context,
                          portMUX_TYPE& mux,
                          SpeedCalibration& speedCal)
            : composite_(composite), context_(context), mux_(mux), speedCal_(speedCal) {}

        bool submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) override {
            const float cmdKmh = speedCal_.calculateCommandSpeedKmh(targetSpeedKmh);
            return composite_.submitSpeedTarget(cmdKmh, nowMs);
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
        SpeedCalibration& speedCal_;
    };

    void runTaskLoop();
    void runBleTask();
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
    BleConfig bleConfig_{};
    HeartRateClient heartRateClient_;
    RscService rscService_;
    FtmsService ftmsService_;
    HeartRateService heartRateService_;
    std::atomic<bool> simHeartRateFromSpeedEnabled_{false};

    // Domain Speed Calibration Engine
    SpeedCalibration speedCalibration_;

    // Passive Composite Simulator
    TreadmillSimulatorComposite composite_;
    TargetSinkWrapper targetSink_{composite_, publishedInclineCommandContext_, inclineCommandContextMux_, speedCalibration_};

    // Domain Session & Coordination
    WorkoutSession session_;
    WorkoutDispatcher dispatcher_;
    ControlCoordinator coordinator_;
    WorkoutEngine workoutEngine_;
    RampTestTracker rampTestTracker_;
    InclineCommissioningTracker inclineTracker_;
    SpeedLearningTracker speedLearningTracker_;

    ApplicationSnapshot publishedSnapshot_{};
    WorkoutSessionSnapshot publishedSessionSnapshot_{};
    float publishedSimTargetSpeedKmh_ = 0.0f;
    float publishedSimTargetInclinePct_ = 0.0f;
    bool publishedAuthoritative_ = false;

    TaskHandle_t taskHandle_ = nullptr;
    SemaphoreHandle_t exitSem_ = nullptr;
    std::atomic<bool> taskRunning_{false};
    std::atomic<bool> stopRequested_{false};

    TaskHandle_t bleTaskHandle_ = nullptr;
    SemaphoreHandle_t bleExitSem_ = nullptr;

    bool initialized_ = false;
    CsafeMachineState previousCsafeQualifiedState_ = CsafeMachineState::Unknown;
    bool csafeStateInitialized_ = false;
    bool previousEstopActive_ = false;
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

    static constexpr uint32_t kBleTaskExitTimeoutMs = 3000;
    static constexpr uint32_t kBleTaskStackSize = 8192;
    static constexpr UBaseType_t kBleTaskPriority = 3;
    static constexpr BaseType_t kBleTaskCore = 0; // PRO_CPU_NUM
};

} // namespace stridecontrol

#endif // STRIDECONTROL_TESTBENCH
