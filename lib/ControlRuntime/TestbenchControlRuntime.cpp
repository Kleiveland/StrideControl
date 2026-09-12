#include "TestbenchControlRuntime.h"

#if defined(STRIDECONTROL_TESTBENCH)

#include <Arduino.h>
#include "../SettingsService/SettingsService.h"

namespace stridecontrol {

TestbenchControlRuntime::TestbenchControlRuntime()
    : composite_(speedSensor_, inclineSensor_, imu_) {}

TestbenchControlRuntime::~TestbenchControlRuntime() {
    end();
}

bool TestbenchControlRuntime::begin(const WorkoutSessionConfig& sessionConfig, const BleConfig& bleConfig) {
    if (initialized_) {
        return true;
    }

    // 1. Initialize sensor drivers in SoftwareObservation mode
    speedSensor_.begin(SpeedSensorConfig{}, SpeedObservationMode::SoftwareObservation);
    inclineSensor_.begin(InclineSensorConfig{}, InclineCalibration{}, InclineObservationMode::SoftwareObservation);
    console_.begin(ConsoleExecutionMode::SoftwareSink);
    imu_.begin(ImuObservationMode::SoftwareObservation);
    runnerDynamics_.begin();

    // 2. Initialize ApplicationOrchestrator in ExternalStep mode
    Serial.printf("[Testbench][BLE] Pre-init heap: free=%u, largest_free_block=%u, min_free=%u\n",
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
    bleManager_.attachServices(&rscService_, &ftmsService_);
    const bool bleOk = bleManager_.begin(bleConfig);
    const bool hrOk = heartRateClient_.begin(bleConfig, &bleManager_);
    Serial.printf("[Testbench][BLE] BleManager begin: %s\n", bleOk ? "SUCCESS" : "FAILED");
    Serial.printf("[Testbench][BLE] HeartRateClient begin: %s\n", hrOk ? "SUCCESS" : "FAILED");

    ApplicationOrchestratorDependencies deps{};
    deps.speedSensor = &speedSensor_;
    deps.inclineSensor = &inclineSensor_;
    deps.imuInterface = &imu_;
    deps.runnerDynamics = &runnerDynamics_;
    deps.diagnosticsService = &diagService_;
    deps.bleManager = &bleManager_;
    deps.hrClient = &heartRateClient_;
    orchestrator_.begin(deps, OrchestratorExecutionMode::ExternalStep);

    // 3. Initialize Domain engines
    session_.begin(sessionConfig);
    dispatcher_.begin();
    workoutEngine_.reset();

    composite_.stageRunner(VirtualRunnerMode::RunningOnBelt, 0, 0.0f, true);

    if (commandQueue_ == nullptr) {
        commandQueue_ = xQueueCreate(kCommandQueueDepth, sizeof(ControlCommand));
    }

    initialized_ = true;
    lostAuthorityCount_ = 0;
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
        orchestrator_.end();
        heartRateClient_.end();
        bleManager_.end();
        session_.end();
        workoutEngine_.reset();
        runnerDynamics_.end();
        imu_.end();
        inclineSensor_.end();
        speedSensor_.end();
        initialized_ = false;
    }
}

bool TestbenchControlRuntime::armWorkout(const WorkoutDefinition& def, uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    if (!workoutEngine_.loadWorkout(def)) {
        return false;
    }
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
    ControlCommand cmd{};
    cmd.type = ControlCommandType::Stop;
    cmd.timestampMs = nowMs != 0 ? nowMs : millis();
    return stageCommand(cmd);
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
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }

    taskRunning_ = false;
    stopRequested_ = false;

    if (exitSem_ != nullptr) {
        vSemaphoreDelete(exitSem_);
        exitSem_ = nullptr;
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
        const bool authoritative = ControlRuntime::isSnapshotAuthoritative(snapshot, nowMs);
        if (authoritative) {
            // 5. Tick domain session & target dispatcher
            coordinator_.tick(session_, dispatcher_, composite_, snapshot, nowMs);
        } else {
            lostAuthorityCount_++;
            if (session_.getSnapshot().state == WorkoutSessionState::Running) {
                session_.suspend(nowMs);
            }
        }

        // 6. Publish snapshot copy for external consumers (cross-core spinlock protected)
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

        // 7. Periodically check stack high-water mark (every 1000 ms)
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
                composite_.stageQuickStart(cmdNowMs);
                composite_.stageRunner(VirtualRunnerMode::RunningOnBelt, 180, 0.35f, true);
                break;
            case ControlCommandType::Stop:
                composite_.stageStop(cmdNowMs);
                session_.registerPhysicalStop(cmdNowMs);
                break;
            case ControlCommandType::Pause:
                session_.suspend(cmdNowMs);
                break;
            case ControlCommandType::Resume:
                session_.resume(cmdNowMs);
                break;
            case ControlCommandType::SetSpeed:
                composite_.stageSpeedTarget(cmd.data.target.speedKmh, cmdNowMs);
                dispatcher_.stageSpeedTarget(cmd.data.target.speedKmh);
                break;
            case ControlCommandType::SetIncline:
                composite_.stageInclineTarget(cmd.data.target.inclinePct, cmdNowMs);
                dispatcher_.stageInclineTarget(cmd.data.target.inclinePct);
                break;
            case ControlCommandType::StepSpeed: {
                const bool positive = (cmd.data.stepSpeed.deltaSpeedKmh > 0.0f);
                composite_.stageSpeedStep(positive, cmdNowMs);
                const float currentSimSpd = composite_.getVirtualTreadmill().getTargetSpeedKmh();
                dispatcher_.stepSpeedTarget(cmd.data.stepSpeed.deltaSpeedKmh, currentSimSpd);
                break;
            }
            case ControlCommandType::StepIncline: {
                const bool positive = (cmd.data.stepIncline.deltaInclinePct > 0.0f);
                composite_.stageInclineStep(positive, cmdNowMs);
                const float currentSimInc = composite_.getVirtualTreadmill().getTargetInclinePct();
                dispatcher_.stepInclineTarget(cmd.data.stepIncline.deltaInclinePct, currentSimInc);
                break;
            }
            case ControlCommandType::ArmWorkout: {
                const WorkoutDefinition* def = SettingsService::instance().findWorkout(
                    cmd.data.arm.userId, cmd.data.arm.workoutId);
                if (def != nullptr && workoutEngine_.loadWorkout(*def)) {
                    session_.armWorkout(&workoutEngine_.getExpandedWorkout(), cmdNowMs, cmd.data.arm.userId);
                    Serial.printf("[TestbenchControlRuntime] Armed workout id=%u for user=%u\n", cmd.data.arm.workoutId, cmd.data.arm.userId);
                } else {
                    Serial.printf("[TestbenchControlRuntime] FAILED to arm workout id=%u for user=%u (def=%p)\n", cmd.data.arm.workoutId, cmd.data.arm.userId, def);
                }
                break;
            }
            case ControlCommandType::CancelWorkout:
                session_.abortSession(cmdNowMs);
                break;
            case ControlCommandType::FinalizeWorkout:
                session_.finalizeSession(cmdNowMs);
                break;
            case ControlCommandType::SetGuiMode:
                session_.setDesiredGuiMode(cmd.data.guiMode.userId, cmd.data.guiMode.isManual);
                break;
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
