#include "TestbenchControlRuntime.h"

#if defined(STRIDECONTROL_TESTBENCH)

#include <cmath>
#include <Arduino.h>
#include "../SettingsService/SettingsService.h"
#include "../DiagnosticsLog/DiagnosticsLog.h"

namespace stridecontrol {

TestbenchControlRuntime::TestbenchControlRuntime()
    : composite_(speedSensor_, inclineSensor_, imu_),
      targetSink_(composite_, publishedInclineCommandContext_, inclineCommandContextMux_) {}

TestbenchControlRuntime::~TestbenchControlRuntime() {
    end();
}

CsafeState TestbenchControlRuntime::provideSimulatedCsafe(
    void* context
) {
    if (context == nullptr) {
        return CsafeState{};
    }

    auto* runtime =
        static_cast<TestbenchControlRuntime*>(context);

    return runtime->composite_.getCsafeState();
}

InclineVerificationCommandInput TestbenchControlRuntime::provideSimulatedInclineCommandContext(
    void* context
) {
    if (context == nullptr) {
        return InclineVerificationCommandInput{};
    }

    auto* runtime =
        static_cast<TestbenchControlRuntime*>(context);

    return runtime->getInclineVerificationCommandInput();
}

InclineVerificationCommandInput TestbenchControlRuntime::getInclineVerificationCommandInput() const {
    InclineVerificationCommandInput input{};
    portENTER_CRITICAL(&inclineCommandContextMux_);
    input = publishedInclineCommandContext_;
    portEXIT_CRITICAL(&inclineCommandContextMux_);
    return input;
}

CommandExecutionStatus TestbenchControlRuntime::provideSimulatedCommandExecutionStatus(
    void* context
) {
    if (context == nullptr) {
        return CommandExecutionStatus{};
    }

    auto* runtime =
        static_cast<TestbenchControlRuntime*>(context);

    return runtime->getCommandExecutionStatus();
}

SessionTelemetryInput TestbenchControlRuntime::provideSimulatedSessionTelemetry(
    void* context
) {
    if (context == nullptr) {
        return SessionTelemetryInput{};
    }

    auto* runtime =
        static_cast<TestbenchControlRuntime*>(context);

    const WorkoutSessionSnapshot sessSnap = runtime->session_.getSnapshot();
    SessionTelemetryInput input{};
    input.sessionActive = sessSnap.active;
    input.sessionState = sessSnap.state;
    input.currentRole = sessSnap.currentRole;
    return input;
}

CommandExecutionStatus TestbenchControlRuntime::getCommandExecutionStatus() const {
    CommandExecutionStatus status{};
    portENTER_CRITICAL(&commandExecutionStatusMux_);
    status = publishedCommandExecutionStatus_;
    portEXIT_CRITICAL(&commandExecutionStatusMux_);
    return status;
}

void TestbenchControlRuntime::publishCommandExecutionStatus() {
    CommandExecutionStatus status{};
    portENTER_CRITICAL(&commandExecutionStatusMux_);
    publishedCommandExecutionStatus_ = status;
    portEXIT_CRITICAL(&commandExecutionStatusMux_);
}

void TestbenchControlRuntime::requestInclineCommissioningAction(bool confirmHomed, bool zeroImu) {
    orchestrator_.requestInclineCommissioningAction(confirmHomed, zeroImu);
}

bool TestbenchControlRuntime::begin(const WorkoutSessionConfig& sessionConfig, const BleConfig& bleConfig) {
    if (initialized_) {
        return true;
    }
    bleConfig_ = bleConfig;

    // 1. Initialize sensor drivers in SoftwareObservation mode
    speedSensor_.begin(SpeedSensorConfig{}, SpeedObservationMode::SoftwareObservation);
    inclineSensor_.begin(InclineSensorConfig{}, InclineCalibration{}, InclineObservationMode::SoftwareObservation);
    console_.begin(ConsoleExecutionMode::SoftwareSink);
    imu_.begin(ImuObservationMode::SoftwareObservation);
    runnerDynamics_.begin();
    inclineVerifier_.begin();

    // 2. Initialize ApplicationOrchestrator in ExternalStep mode
    Serial.printf("[Testbench][BLE] Pre-init heap: free=%u, largest_free_block=%u, min_free=%u\n",
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
    bool bleOk = false;
    bool hrOk = false;
    if (SettingsService::instance().getBleStackEnabled()) {
        bleManager_.attachServices(&rscService_, &ftmsService_, &heartRateService_);
        bleOk = bleManager_.begin(bleConfig);
        hrOk = heartRateClient_.begin(bleConfig, &bleManager_);
    } else {
        Serial.println("[Testbench][BLE] Stack disabled via commissioning setting - skipping init");
    }
    Serial.printf("[Testbench][BLE] BleManager begin: %s\n", bleOk ? "SUCCESS" : "FAILED");
    DiagnosticsLog::instance().addEntryf("[Testbench][BLE] BleManager begin: %s", bleOk ? "SUCCESS" : "FAILED");
    Serial.printf("[Testbench][BLE] HeartRateClient begin: %s\n", hrOk ? "SUCCESS" : "FAILED");
    DiagnosticsLog::instance().addEntryf("[Testbench][BLE] HeartRateClient begin: %s", hrOk ? "SUCCESS" : "FAILED");

    ApplicationOrchestratorDependencies deps{};
    deps.speedSensor = &speedSensor_;
    deps.inclineSensor = &inclineSensor_;
    deps.imuInterface = &imu_;
    deps.csafeStateProvider = &TestbenchControlRuntime::provideSimulatedCsafe;
    deps.csafeStateProviderContext = this;
    deps.inclineCommandContextProvider = &TestbenchControlRuntime::provideSimulatedInclineCommandContext;
    deps.inclineCommandContextProviderContext = this;
    deps.commandExecutionStatusProvider = &TestbenchControlRuntime::provideSimulatedCommandExecutionStatus;
    deps.commandExecutionStatusProviderContext = this;
    deps.sessionTelemetryProvider = &TestbenchControlRuntime::provideSimulatedSessionTelemetry;
    deps.sessionTelemetryProviderContext = this;
    deps.runnerDynamics = &runnerDynamics_;
    deps.inclineVerifier = &inclineVerifier_;
    deps.diagnosticsService = &diagService_;
    deps.bleManager = &bleManager_;
    deps.hrClient = &heartRateClient_;
    orchestrator_.begin(deps, OrchestratorExecutionMode::ExternalStep);

    console_.begin(ConsoleExecutionMode::SoftwareSink);
    composite_.setConsoleInterface(&console_);

    // 3. Initialize Domain engines
    session_.begin(sessionConfig);
    dispatcher_.begin();
    workoutEngine_.reset();

    portENTER_CRITICAL(&inclineCommandContextMux_);
    publishedInclineCommandContext_ = InclineVerificationCommandInput{};
    portEXIT_CRITICAL(&inclineCommandContextMux_);

    portENTER_CRITICAL(&commandExecutionStatusMux_);
    publishedCommandExecutionStatus_ = CommandExecutionStatus{};
    portEXIT_CRITICAL(&commandExecutionStatusMux_);
    lastReportedAbortedRequestId_ = 0;

    composite_.stageRunner(VirtualRunnerMode::RunningOnBelt, 180, 0.35f, true);
    rampTestTracker_.reset();
    inclineTracker_.reset();

    if (commandQueue_ == nullptr) {
        commandQueue_ = xQueueCreate(kCommandQueueDepth, sizeof(ControlCommand));
    }

    initialized_ = true;
    previousCsafeQualifiedState_ = CsafeMachineState::Unknown;
    csafeStateInitialized_ = false;
    lostAuthorityCount_ = 0;
    authorityLostSinceMs_ = 0;
    authorityLostReported_ = false;
    connectionWarningActive_ = false;
    wasAuthoritative_ = false;
    minFreeStackBytes_ = 8192;

    portENTER_CRITICAL(&snapshotMux_);
    publishedSnapshot_ = orchestrator_.getSnapshot();
    publishedSessionSnapshot_ = session_.getSnapshot();
    publishedSimTargetSpeedKmh_ = composite_.getVirtualTreadmill().getTargetSpeedKmh();
    publishedSimTargetInclinePct_ = composite_.getVirtualTreadmill().getTargetInclinePct();
    publishedAuthoritative_ = false;
    portEXIT_CRITICAL(&snapshotMux_);

    return true;
}

void TestbenchControlRuntime::end() {
    stopControlTask(1000);

    if (commandQueue_ != nullptr) {
        vQueueDelete(commandQueue_);
        commandQueue_ = nullptr;
    }

    if (initialized_) {
        composite_.setConsoleInterface(nullptr);
        console_.end();
        orchestrator_.end();
        heartRateClient_.end();
        bleManager_.end();
        dispatcher_.clearAllTargets();
        session_.end();
        workoutEngine_.reset();
        runnerDynamics_.end();
        inclineVerifier_.end();
        imu_.end();
        inclineSensor_.end();
        speedSensor_.end();
        initialized_ = false;
        previousCsafeQualifiedState_ = CsafeMachineState::Unknown;
        csafeStateInitialized_ = false;

        portENTER_CRITICAL(&inclineCommandContextMux_);
        publishedInclineCommandContext_ = InclineVerificationCommandInput{};
        portEXIT_CRITICAL(&inclineCommandContextMux_);

        rampTestTracker_.reset();
        inclineTracker_.reset();
    }
}

bool TestbenchControlRuntime::armWorkout(const WorkoutDefinition& def, uint32_t nowMs) {
    if (!initialized_ || session_.isActive() || rampTestTracker_.active()) {
        return false;
    }
    if (!workoutEngine_.loadWorkout(def)) {
        return false;
    }
    dispatcher_.clearForNewSession();
    const bool armed = session_.armWorkout(&workoutEngine_.getExpandedWorkout(), nowMs);
    if (armed) {
        portENTER_CRITICAL(&snapshotMux_);
        publishedSnapshot_ = orchestrator_.getSnapshot();
        publishedSessionSnapshot_ = session_.getSnapshot();
        publishedSimTargetSpeedKmh_ = composite_.getVirtualTreadmill().getTargetSpeedKmh();
        publishedSimTargetInclinePct_ = composite_.getVirtualTreadmill().getTargetInclinePct();
        publishedAuthoritative_ = false;
        portEXIT_CRITICAL(&snapshotMux_);
    }
    return armed;
}

bool TestbenchControlRuntime::triggerQuickStart(uint32_t nowMs) {
    ControlCommand cmd{};
    cmd.type = ControlCommandType::QuickStart;
    cmd.timestampMs = nowMs != 0 ? nowMs : millis();
    return stageCommand(cmd);
}

bool TestbenchControlRuntime::triggerStop(uint32_t nowMs) {
    ControlCommand cmd{};
    cmd.type = ControlCommandType::Stop;
    cmd.timestampMs = nowMs != 0 ? nowMs : millis();
    return stageCommand(cmd);
}

bool TestbenchControlRuntime::triggerEmergencyStop(uint32_t nowMs) {
    const uint32_t ts = nowMs != 0 ? nowMs : millis();
    return composite_.stageEmergencyStop(ts);
}

bool TestbenchControlRuntime::setSimSpeedTarget(float speedKmh, uint32_t nowMs) {
    ControlCommand cmd{};
    cmd.type = ControlCommandType::SetSpeed;
    cmd.timestampMs = nowMs != 0 ? nowMs : millis();
    cmd.data.target.speedKmh = speedKmh;
    return stageCommand(cmd);
}

bool TestbenchControlRuntime::setSimInclineTarget(float inclinePct, uint32_t nowMs) {
    ControlCommand cmd{};
    cmd.type = ControlCommandType::SetIncline;
    cmd.timestampMs = nowMs != 0 ? nowMs : millis();
    cmd.data.target.inclinePct = inclinePct;
    return stageCommand(cmd);
}

bool TestbenchControlRuntime::stepSimSpeed(bool positive, uint32_t nowMs) {
    ControlCommand cmd{};
    cmd.type = ControlCommandType::StepSpeed;
    cmd.timestampMs = nowMs != 0 ? nowMs : millis();
    cmd.data.stepSpeed.deltaSpeedKmh = positive ? 0.5f : -0.5f;
    return stageCommand(cmd);
}

bool TestbenchControlRuntime::stepSimIncline(bool positive, uint32_t nowMs) {
    ControlCommand cmd{};
    cmd.type = ControlCommandType::StepIncline;
    cmd.timestampMs = nowMs != 0 ? nowMs : millis();
    cmd.data.stepIncline.deltaInclinePct = positive ? 0.5f : -0.5f;
    return stageCommand(cmd);
}

void TestbenchControlRuntime::setSimRunner(VirtualRunnerMode mode, uint16_t cadenceSpm, float magnitudeG, bool valid) {
    composite_.stageRunner(mode, cadenceSpm, magnitudeG, valid);
}

void TestbenchControlRuntime::setSimHeartRateFromSpeed(bool enabled) {
    simHeartRateFromSpeedEnabled_ = enabled;
}

bool TestbenchControlRuntime::startControlTask() {
    if (!initialized_) {
        return false;
    }
    if (taskRunning_ || taskHandle_ != nullptr) {
        return true;
    }

    stopRequested_ = false;
    taskRunning_ = false;

    if (exitSem_ == nullptr) {
        exitSem_ = xSemaphoreCreateBinary();
    } else {
        xSemaphoreTake(exitSem_, 0);
    }

    if (exitSem_ == nullptr) {
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        taskEntry,
        "TbControlTask",
        kTaskStackSize,
        this,
        kTaskPriority,
        &taskHandle_,
        kTaskCore // Pinned strictly to Core 0 (PRO_CPU)
    );

    if (result != pdPASS) {
        vSemaphoreDelete(exitSem_);
        exitSem_ = nullptr;
        taskHandle_ = nullptr;
        return false;
    }

    return true;
}

bool TestbenchControlRuntime::stopControlTask(uint32_t timeoutMs) {
    if (!taskRunning_ && taskHandle_ == nullptr) {
        return true;
    }

    stopRequested_ = true;

    bool cleanExit = false;
    if (exitSem_ != nullptr) {
        cleanExit = (xSemaphoreTake(exitSem_, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
    }

    if (taskHandle_ != nullptr) {
        // Task must be removed either way to avoid a permanently stuck reference, but a
        // forced (non-clean) deletion means the task may still have been mid-execution and
        // could reference exitSem_ - do not delete it in that case.
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }
    taskRunning_ = false;
    stopRequested_ = false;

    if (cleanExit) {
        if (exitSem_ != nullptr) {
            vSemaphoreDelete(exitSem_);
            exitSem_ = nullptr;
        }
    } else {
        // Forced termination: report it, and deliberately leak exitSem_ rather than risk a
        // use-after-free on a task that may not have actually stopped touching it yet.
        diagService_.reportFault(FaultCode::SystemWatchdogWarning, millis());
    }

    return cleanExit;
}

bool TestbenchControlRuntime::isTaskRunning() const {
    return taskRunning_;
}

void TestbenchControlRuntime::taskEntry(void* param) {
    auto* self = static_cast<TestbenchControlRuntime*>(param);
    if (self != nullptr) {
        self->runTaskLoop();
    }
}

void TestbenchControlRuntime::runTaskLoop() {
    taskRunning_ = true;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(kPeriodMs);
    uint32_t lastHeadroomCheckMs = 0;
    uint64_t tickIndex = 0;
    const uint64_t baseTimeUs = static_cast<uint64_t>(millis()) * 1000ULL;

    while (!stopRequested_) {
        tickIndex++;
        const uint64_t scenarioTimeUs = baseTimeUs + (tickIndex * (static_cast<uint64_t>(kPeriodMs) * 1000ULL));
        const uint32_t nowMs = static_cast<uint32_t>(scenarioTimeUs / 1000ULL);

        // 0. Drain staged external commands up to batch limit (8)
        processQueuedCommands(nowMs);

        const SimulationTick simTick(tickIndex, scenarioTimeUs, kPeriodMs * 1000UL);

        // 1. Tick composite simulator (drains staged stimuli, advances physics, forwards to adapters)
        composite_.tick(simTick, 0.0f);

        // 2. Step application orchestrator pipeline in ExternalStep mode
        const ApplicationTickContext ctx(tickIndex, scenarioTimeUs, kPeriodMs);
        orchestrator_.step(ctx);

        // 3. Obtain authoritative ApplicationSnapshot representing resulting state
        ApplicationSnapshot snapshot = orchestrator_.getSnapshot();

        rampTestTracker_.update(
            snapshot.speed.speedKmh,
            snapshot.speed.measurementValid,
            nowMs
        );

        if (rampTestTracker_.targetDispatchRequested()) {
            composite_.submitSpeedTarget(rampTestTracker_.targetSpeedKmh(), nowMs);
            rampTestTracker_.acknowledgeTargetDispatched();
        }

        inclineTracker_.update(
            snapshot.incline.estimatedInclinePct,
            snapshot.imu.relativeDeckAngleDeg,
            snapshot.imu.dataValid,
            nowMs
        );

        if (inclineTracker_.isHomedSettled()) {
            requestInclineCommissioningAction(true, true);
            inclineTracker_.onHomedConfirmed(nowMs);
        }

        if (simHeartRateFromSpeedEnabled_) {
            float bpm = 80.0f + (snapshot.speed.speedKmh - 1.0f) * 6.36f;
            if (bpm < 70.0f) bpm = 70.0f;
            if (bpm > 200.0f) bpm = 200.0f;
            snapshot.heartRate.initialized = true;
            snapshot.heartRate.heartRateValid = true;
            snapshot.heartRate.heartRateBpm = static_cast<uint8_t>(bpm + 0.5f);
            snapshot.heartRate.dataAgeMs = 0;
            snapshot.heartRate.connectionState = HeartRateConnectionState::Connected;
        }

        // 4. Evaluate authority
        const bool authoritative = ControlRuntime::isSnapshotAuthoritative(snapshot, nowMs, &wasAuthoritative_);
        if (authoritative) {
            authorityLostReported_ = false;
            authorityLostSinceMs_ = 0; // Reset the loss streak - authority has recovered
            connectionWarningActive_ = false;

            // CSAFE Stop Hierarchy Detection
            const bool csafeValid = snapshot.csafe.initialized &&
                                    snapshot.csafe.online &&
                                    snapshot.csafe.machineStateFresh &&
                                    snapshot.csafe.qualifiedState != CsafeMachineState::Unknown;
            if (csafeValid) {
                if (!csafeStateInitialized_) {
                    // Re-baseline without firing transitions (initial connect or reconnect)
                    previousCsafeQualifiedState_ = snapshot.csafe.qualifiedState;
                    csafeStateInitialized_ = true;
                } else if (snapshot.csafe.qualifiedState != previousCsafeQualifiedState_) {
                    // Physical Stop 1: InUse -> Paused
                    if (previousCsafeQualifiedState_ == CsafeMachineState::InUse &&
                        snapshot.csafe.qualifiedState == CsafeMachineState::Paused) {
                        session_.registerPhysicalStop(nowMs);
                    }
                    // Physical Stop 2: Paused -> Ready
                    else if (previousCsafeQualifiedState_ == CsafeMachineState::Paused &&
                             snapshot.csafe.qualifiedState == CsafeMachineState::Ready) {
                        session_.registerPhysicalStop(nowMs);
                    }
                    previousCsafeQualifiedState_ = snapshot.csafe.qualifiedState;
                }
            } else {
                // Communication loss or stale telemetry: mark uninitialized so reconnect re-baselines
                csafeStateInitialized_ = false;
            }

            // 3rd Physical Stop Detection: Button press during continuation window when CSAFE is Ready
            PhysicalButtonEvent btnEvent{};
            while (console_.receivePhysicalButtonEvent(btnEvent)) {
                if (btnEvent.button == ButtonId::Stop &&
                    btnEvent.action == PhysicalButtonAction::Pressed &&
                    session_.getSnapshot().continuationWindowActive &&
                    csafeValid &&
                    snapshot.csafe.qualifiedState == CsafeMachineState::Ready) {
                    const uint32_t stopTimestampMs = (btnEvent.timestampMs != 0) ? btnEvent.timestampMs : nowMs;
                    session_.registerPhysicalStop(stopTimestampMs);
                }
            }

            // 5. Tick domain session & target dispatcher
            coordinator_.tick(session_, dispatcher_, targetSink_, snapshot, nowMs);
        } else {
            lostAuthorityCount_++;
            if (authorityLostSinceMs_ == 0) {
                authorityLostSinceMs_ = nowMs;
            }
            connectionWarningActive_ = (nowMs - authorityLostSinceMs_) >= kAuthorityLossWarningThresholdMs;
            if (connectionWarningActive_ && !authorityLostReported_) {
                DiagnosticsLog::instance().addEntryf(
                    "Authority lost for >= %lums, showing reconnect indicator",
                    static_cast<unsigned long>(nowMs - authorityLostSinceMs_));
                authorityLostReported_ = true;
            }

            csafeStateInitialized_ = false;

            // Unconditionally drain physical button queue even when authority is lost, but do not trigger Stop
            PhysicalButtonEvent btnEvent{};
            while (console_.receivePhysicalButtonEvent(btnEvent)) {
                // Drained without action while unauthoritative
            }

            // Telemetry jitter freezes data integration, but must never freeze
            // the athlete's workout session state while the belt moves.
        }

        // 6. Publish snapshot copy and command execution status for external consumers
        publishCommandExecutionStatus();
        const WorkoutSessionSnapshot sessSnap = session_.getSnapshot();
        const float targetSpeed = composite_.getVirtualTreadmill().getTargetSpeedKmh();
        const float targetIncline = composite_.getVirtualTreadmill().getTargetInclinePct();

        portENTER_CRITICAL(&snapshotMux_);
        publishedSnapshot_ = snapshot;
        publishedSessionSnapshot_ = sessSnap;
        publishedSimTargetSpeedKmh_ = targetSpeed;
        publishedSimTargetInclinePct_ = targetIncline;
        publishedAuthoritative_ = authoritative;
        portEXIT_CRITICAL(&snapshotMux_);

        // 7. Dispatch Periodic BLE Stack, Client, and Service Telemetry Updates
        bleManager_.update(nowMs);
        heartRateClient_.update(nowMs);
        bleManager_.updateServices(nowMs, snapshot);

        // 8. Periodically check stack high-water mark (every 1000 ms)
        if (nowMs - lastHeadroomCheckMs >= 1000) {
            lastHeadroomCheckMs = nowMs;
            UBaseType_t highWater = uxTaskGetStackHighWaterMark(NULL);
            portENTER_CRITICAL(&snapshotMux_);
            minFreeStackBytes_ = static_cast<uint32_t>(highWater);
            portEXIT_CRITICAL(&snapshotMux_);
        }

        xTaskDelayUntil(&lastWakeTime, periodTicks);
    }

    taskRunning_ = false;
    if (exitSem_ != nullptr) {
        xSemaphoreGive(exitSem_);
    }
    vTaskSuspend(NULL);
}

void TestbenchControlRuntime::processQueuedCommands(uint32_t nowMs) {
    ControlCommand cmd{};
    size_t processed = 0;
    while (commandQueue_ != nullptr && xQueueReceive(commandQueue_, &cmd, 0) == pdTRUE && processed < 8) {
        const uint32_t cmdNowMs = (cmd.timestampMs != 0) ? cmd.timestampMs : nowMs;
        switch (cmd.type) {
            case ControlCommandType::QuickStart:
                if (!rampTestTracker_.active()) {
                    composite_.stageQuickStart(cmdNowMs);
                    composite_.stageRunner(VirtualRunnerMode::RunningOnBelt, 180, 0.35f, true);
                }
                break;
            case ControlCommandType::Stop:
                rampTestTracker_.abort(cmdNowMs);
                inclineTracker_.abort(cmdNowMs);
                composite_.stageStop(cmdNowMs);
                break;
            case ControlCommandType::Pause:
                session_.suspend(cmdNowMs);
                break;
            case ControlCommandType::Resume:
                if (!session_.isActive() && !rampTestTracker_.active()) {
                    composite_.stageQuickStart(cmdNowMs);
                    composite_.stageRunner(VirtualRunnerMode::RunningOnBelt, 180, 0.35f, true);
                }
                break;
            case ControlCommandType::SetSpeed: {
                if (rampTestTracker_.active()) {
                    break;
                }
                TargetContext ctx;
                if (session_.isActive()) {
                    ctx.origin = TargetOrigin::SessionManualAdjustment;
                    ctx.sessionGeneration = session_.getSessionGeneration();
                    ctx.stepIndex = session_.getCurrentStepIndex();
                } else {
                    ctx.origin = TargetOrigin::StandaloneManual;
                }
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stageSpeedTarget(cmd.data.target.speedKmh, ctx);
                session_.reportWorkSpeedAdjustment(cmd.data.target.speedKmh);
                break;
            }
            case ControlCommandType::SetIncline: {
                TargetContext ctx;
                if (session_.isActive()) {
                    ctx.origin = TargetOrigin::SessionManualAdjustment;
                    ctx.sessionGeneration = session_.getSessionGeneration();
                    ctx.stepIndex = session_.getCurrentStepIndex();
                } else {
                    ctx.origin = TargetOrigin::StandaloneManual;
                }
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stageInclineTarget(cmd.data.target.inclinePct, ctx);
                break;
            }
            case ControlCommandType::StepSpeed: {
                if (rampTestTracker_.active()) {
                    break;
                }
                const float currentSimSpd = composite_.getVirtualTreadmill().getTargetSpeedKmh();
                TargetContext ctx;
                if (session_.isActive()) {
                    ctx.origin = TargetOrigin::SessionManualAdjustment;
                    ctx.sessionGeneration = session_.getSessionGeneration();
                    ctx.stepIndex = session_.getCurrentStepIndex();
                } else {
                    ctx.origin = TargetOrigin::StandaloneManual;
                }
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stepSpeedTarget(cmd.data.stepSpeed.deltaSpeedKmh, currentSimSpd, ctx);
                session_.reportWorkSpeedAdjustment(currentSimSpd + cmd.data.stepSpeed.deltaSpeedKmh);
                break;
            }
            case ControlCommandType::StepIncline: {
                const float currentSimInc = composite_.getVirtualTreadmill().getTargetInclinePct();
                TargetContext ctx;
                if (session_.isActive()) {
                    ctx.origin = TargetOrigin::SessionManualAdjustment;
                    ctx.sessionGeneration = session_.getSessionGeneration();
                    ctx.stepIndex = session_.getCurrentStepIndex();
                } else {
                    ctx.origin = TargetOrigin::StandaloneManual;
                }
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stepInclineTarget(cmd.data.stepIncline.deltaInclinePct, currentSimInc, ctx);
                break;
            }
            case ControlCommandType::ArmWorkout: {
                if (session_.isActive() || rampTestTracker_.active()) {
                    Serial.println("[TestbenchControlRuntime] Cannot arm workout: session or ramp test already active");
                    break;
                }
                const WorkoutDefinition* def = SettingsService::instance().findWorkout(
                    cmd.data.arm.userId, cmd.data.arm.workoutId);
                if (def != nullptr && workoutEngine_.loadWorkout(*def)) {
                    dispatcher_.clearForNewSession();
                    session_.armWorkout(&workoutEngine_.getExpandedWorkout(), cmdNowMs, cmd.data.arm.userId);
                    Serial.printf("[TestbenchControlRuntime] Armed workout id=%u for user=%u\n", cmd.data.arm.workoutId, cmd.data.arm.userId);
                } else {
                    Serial.printf("[TestbenchControlRuntime] FAILED to arm workout id=%u for user=%u (def=%p)\n", cmd.data.arm.workoutId, cmd.data.arm.userId, def);
                }
                break;
            }
            case ControlCommandType::CancelWorkout:
                dispatcher_.clearWorkoutTargets();
                session_.abortSession(cmdNowMs);
                break;
            case ControlCommandType::FinalizeWorkout:
                dispatcher_.clearWorkoutTargets();
                session_.finalizeSession(cmdNowMs);
                break;
            case ControlCommandType::CutDrag:
                session_.cutDrag(cmdNowMs);
                break;
            case ControlCommandType::SkipToNextDrag:
                session_.skipToNextDrag(cmdNowMs);
                break;
            case ControlCommandType::ExtendRest:
                session_.extendRest();
                break;
            case ControlCommandType::AcceptSpeedShift:
                session_.acceptSpeedAdjustmentShift();
                break;
            case ControlCommandType::RejectSpeedShift:
                session_.rejectSpeedAdjustmentShift();
                break;
            case ControlCommandType::StartRampCalibrationTest:
                if (session_.isActive()) {
                    Serial.println("[TestbenchControlRuntime] Cannot start ramp test: session already active");
                    break;
                }
                if (rampTestTracker_.active()) {
                    Serial.println("[TestbenchControlRuntime] Cannot start ramp test: ramp test already active");
                    break;
                }
                rampTestTracker_.beginTest(
                    cmd.data.rampTest.startSpeedKmh,
                    cmd.data.rampTest.targetSpeedKmh,
                    cmdNowMs
                );
                break;
            case ControlCommandType::SetGuiMode:
                session_.setDesiredGuiMode(cmd.data.guiMode.userId, cmd.data.guiMode.isManual, cmdNowMs);
                break;
            case ControlCommandType::SelectUser: {
                const uint8_t selectedId = cmd.data.selectUser.userId;
                session_.selectUser(selectedId);
                const auto* settings = SettingsService::instance().getActiveSettings();
                if (settings != nullptr) {
                    for (size_t i = 0; i < MAX_USERS; ++i) {
                        if (settings->users[i].id == selectedId) {
                            const char* mac = settings->users[i].preferredHrMac;
                            strncpy(bleConfig_.preferredHrMac, mac, sizeof(bleConfig_.preferredHrMac) - 1);
                            bleConfig_.preferredHrMac[sizeof(bleConfig_.preferredHrMac) - 1] = '\0';
                            bleConfig_.autoConnectHr = (mac[0] != '\0');
                            heartRateClient_.updateConfig(bleConfig_);
                            break;
                        }
                    }
                }
                break;
            }
            case ControlCommandType::StartInclineHoming: {
                if (session_.isActive()) {
                    Serial.println("[TestbenchControlRuntime] Cannot start incline homing: session already active");
                    break;
                }
                if (rampTestTracker_.active()) {
                    Serial.println("[TestbenchControlRuntime] Cannot start incline homing: ramp test already active");
                    break;
                }
                const auto phase = inclineTracker_.phase();
                if (phase != InclineCommissioningPhase::Idle &&
                    phase != InclineCommissioningPhase::Aborted &&
                    phase != InclineCommissioningPhase::TimedOut &&
                    phase != InclineCommissioningPhase::Failed &&
                    phase != InclineCommissioningPhase::Complete) {
                    Serial.println("[TestbenchControlRuntime] Cannot start incline homing: incline commissioning already active");
                    break;
                }
                inclineTracker_.beginHoming(cmdNowMs);
                composite_.submitInclineTarget(0.0f, cmdNowMs);
                break;
            }
            case ControlCommandType::StartInclineMeasurePoint: {
                if (session_.isActive()) {
                    Serial.println("[TestbenchControlRuntime] Cannot start incline measurement: session already active");
                    break;
                }
                if (rampTestTracker_.active()) {
                    Serial.println("[TestbenchControlRuntime] Cannot start incline measurement: ramp test already active");
                    break;
                }
                if (inclineTracker_.pointCount() == 0) {
                    Serial.println("[TestbenchControlRuntime] Cannot start incline measurement: homing not confirmed");
                    break;
                }
                const auto phase = inclineTracker_.phase();
                if (phase != InclineCommissioningPhase::Idle &&
                    phase != InclineCommissioningPhase::Complete) {
                    Serial.println("[TestbenchControlRuntime] Cannot start incline measurement: incline commissioning already active");
                    break;
                }
                const float cmdPct = cmd.data.inclineMeasurePoint.commandedPct;
                const auto dir = static_cast<InclineDirection>(cmd.data.inclineMeasurePoint.expectedDirection);
                inclineTracker_.beginMeasurePoint(cmdPct, dir, cmdNowMs);
                composite_.submitInclineTarget(cmdPct, cmdNowMs);
                break;
            }
            case ControlCommandType::SaveInclineCalibration: {
                InclineConfig candidate = inclineTracker_.buildCandidateConfig();
                if (candidate.commandMapValid) {
                    SettingsService::instance().saveInclineConfig(candidate);
                }
                break;
            }
            case ControlCommandType::AbortInclineCommissioning: {
                inclineTracker_.abort(cmdNowMs);
                break;
            }
            case ControlCommandType::None:
            default:
                break;
        }
        processed++;
    }
}

ApplicationSnapshot TestbenchControlRuntime::getSnapshot() const {
    ApplicationSnapshot snap;
    portENTER_CRITICAL(&snapshotMux_);
    snap = publishedSnapshot_;
    portEXIT_CRITICAL(&snapshotMux_);

    if (simHeartRateFromSpeedEnabled_) {
        float bpm = 80.0f + (snap.speed.speedKmh - 1.0f) * 6.36f;
        if (bpm < 70.0f) bpm = 70.0f;
        if (bpm > 200.0f) bpm = 200.0f;
        snap.heartRate.initialized = true;
        snap.heartRate.heartRateValid = true;
        snap.heartRate.heartRateBpm = static_cast<uint8_t>(bpm + 0.5f);
        snap.heartRate.dataAgeMs = 0;
        snap.heartRate.connectionState = HeartRateConnectionState::Connected;
    }

    return snap;
}

WorkoutSessionSnapshot TestbenchControlRuntime::getSessionSnapshot() const {
    WorkoutSessionSnapshot sessSnap;
    portENTER_CRITICAL(&snapshotMux_);
    sessSnap = publishedSessionSnapshot_;
    portEXIT_CRITICAL(&snapshotMux_);
    return sessSnap;
}

TestbenchTelemetry TestbenchControlRuntime::getTelemetry(uint32_t nowMs) const {
    (void)nowMs;
    TestbenchTelemetry telem;
    portENTER_CRITICAL(&snapshotMux_);
    telem.snapshot = publishedSnapshot_;
    telem.sessionSnapshot = publishedSessionSnapshot_;
    telem.simTargetSpeedKmh = publishedSimTargetSpeedKmh_;
    telem.simTargetInclinePct = publishedSimTargetInclinePct_;
    telem.authoritative = publishedAuthoritative_;
    telem.lostAuthorityCount = lostAuthorityCount_;
    telem.minFreeStackBytes = minFreeStackBytes_;
    portEXIT_CRITICAL(&snapshotMux_);

    if (simHeartRateFromSpeedEnabled_) {
        float bpm = 80.0f + (telem.snapshot.speed.speedKmh - 1.0f) * 6.36f;
        if (bpm < 70.0f) bpm = 70.0f;
        if (bpm > 200.0f) bpm = 200.0f;
        telem.snapshot.heartRate.initialized = true;
        telem.snapshot.heartRate.heartRateValid = true;
        telem.snapshot.heartRate.heartRateBpm = static_cast<uint8_t>(bpm + 0.5f);
        telem.snapshot.heartRate.dataAgeMs = 0;
        telem.snapshot.heartRate.connectionState = HeartRateConnectionState::Connected;
    }

    return telem;
}

uint32_t TestbenchControlRuntime::getLostAuthorityCount() const {
    return lostAuthorityCount_;
}

uint32_t TestbenchControlRuntime::getMinFreeStackBytes() const {
    portENTER_CRITICAL(&snapshotMux_);
    const uint32_t val = minFreeStackBytes_;
    portEXIT_CRITICAL(&snapshotMux_);
    return val;
}

const char* TestbenchControlRuntime::version() {
    return "TestbenchControlRuntime/2.0.0";
}

} // namespace stridecontrol

#endif // STRIDECONTROL_TESTBENCH
