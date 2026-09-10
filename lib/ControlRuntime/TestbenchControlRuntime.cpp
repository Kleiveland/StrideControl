#include "TestbenchControlRuntime.h"

#if defined(STRIDECONTROL_TESTBENCH)

#include <Arduino.h>

namespace stridecontrol {

TestbenchControlRuntime::TestbenchControlRuntime()
    : composite_(speedSensor_, inclineSensor_, imu_) {}

TestbenchControlRuntime::~TestbenchControlRuntime() {
    end();
}

bool TestbenchControlRuntime::begin(const WorkoutSessionConfig& sessionConfig) {
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
    ApplicationOrchestratorDependencies deps{};
    deps.speedSensor = &speedSensor_;
    deps.inclineSensor = &inclineSensor_;
    deps.imuInterface = &imu_;
    deps.runnerDynamics = &runnerDynamics_;
    deps.diagnosticsService = &diagService_;
    orchestrator_.begin(deps, OrchestratorExecutionMode::ExternalStep);

    // 3. Initialize Domain engines
    session_.begin(sessionConfig);
    dispatcher_.begin();
    workoutEngine_.reset();

    composite_.stageRunner(VirtualRunnerMode::RunningOnBelt, 0, 0.0f, true);

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

    if (initialized_) {
        orchestrator_.end();
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
    if (!initialized_) {
        return false;
    }
    return composite_.stageQuickStart(nowMs != 0 ? nowMs : millis());
}

bool TestbenchControlRuntime::triggerStop(uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    return composite_.stageStop(nowMs != 0 ? nowMs : millis());
}

bool TestbenchControlRuntime::triggerEmergencyStop(uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    return composite_.stageEmergencyStop(nowMs != 0 ? nowMs : millis());
}

bool TestbenchControlRuntime::setSimSpeedTarget(float speedKmh, uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    return composite_.stageSpeedTarget(speedKmh, nowMs != 0 ? nowMs : millis());
}

bool TestbenchControlRuntime::setSimInclineTarget(float inclinePct, uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    return composite_.stageInclineTarget(inclinePct, nowMs != 0 ? nowMs : millis());
}

bool TestbenchControlRuntime::stepSimSpeed(bool positive, uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    return composite_.stageSpeedStep(positive, nowMs != 0 ? nowMs : millis());
}

bool TestbenchControlRuntime::stepSimIncline(bool positive, uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    return composite_.stageInclineStep(positive, nowMs != 0 ? nowMs : millis());
}

void TestbenchControlRuntime::setSimRunner(VirtualRunnerMode mode, uint16_t cadenceSpm, float magnitudeG, bool valid) {
    composite_.stageRunner(mode, cadenceSpm, magnitudeG, valid);
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
        const SimulationTick simTick(tickIndex, scenarioTimeUs, kPeriodMs * 1000UL);

        // 1. Tick composite simulator (drains staged stimuli, advances physics, forwards to adapters)
        composite_.tick(simTick, 0.0f);

        // 2. Step application orchestrator pipeline in ExternalStep mode
        const ApplicationTickContext ctx(tickIndex, scenarioTimeUs, kPeriodMs);
        orchestrator_.step(ctx);

        // 3. Obtain authoritative ApplicationSnapshot representing resulting state
        const ApplicationSnapshot snapshot = orchestrator_.getSnapshot();

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

ApplicationSnapshot TestbenchControlRuntime::getSnapshot() const {
    ApplicationSnapshot snap;
    portENTER_CRITICAL(&snapshotMux_);
    snap = publishedSnapshot_;
    portEXIT_CRITICAL(&snapshotMux_);
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
