#include "TestbenchControlRuntime.h"

#if defined(STRIDECONTROL_TESTBENCH)

#include <Arduino.h>

namespace stridecontrol {

TestbenchControlRuntime::TestbenchControlRuntime() = default;

TestbenchControlRuntime::~TestbenchControlRuntime() {
    end();
}

bool TestbenchControlRuntime::begin(const WorkoutSessionConfig& sessionConfig) {
    if (initialized_) {
        return true;
    }

    simulator_.begin(millis());
    session_.begin(sessionConfig);
    dispatcher_.begin();
    workoutEngine_.reset();

    initialized_ = true;
    lostAuthorityCount_ = 0;
    minFreeStackBytes_ = 8192;
    pendingQuickStartSpeed_ = -1.0f;
    pendingQuickStartMs_ = 0;

    portENTER_CRITICAL(&snapshotMux_);
    publishedSnapshot_ = simulator_.getSnapshot();
    publishedSessionSnapshot_ = session_.getSnapshot();
    publishedSimTargetSpeedKmh_ = simulator_.getTargetSpeedKmh();
    publishedSimTargetInclinePct_ = simulator_.getTargetInclinePct();
    portEXIT_CRITICAL(&snapshotMux_);

    return true;
}

void TestbenchControlRuntime::end() {
    stopControlTask(1000);

    if (initialized_) {
        session_.end();
        workoutEngine_.reset();
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
        publishedSnapshot_ = simulator_.getSnapshot();
        publishedSessionSnapshot_ = session_.getSnapshot();
        publishedSimTargetSpeedKmh_ = simulator_.getTargetSpeedKmh();
        publishedSimTargetInclinePct_ = simulator_.getTargetInclinePct();
        portEXIT_CRITICAL(&snapshotMux_);
    }
    return armed;
}

bool TestbenchControlRuntime::triggerQuickStart(float speedKmh, uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    portENTER_CRITICAL(&snapshotMux_);
    pendingQuickStartSpeed_ = speedKmh;
    pendingQuickStartMs_ = nowMs;
    portEXIT_CRITICAL(&snapshotMux_);
    return true;
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

    while (!stopRequested_) {
        const uint32_t nowMs = millis();

        // 0. Process external startup stimuli (e.g. Quick Start) if staged
        float quickStartSpeed = -1.0f;
        uint32_t quickStartMs = 0;
        portENTER_CRITICAL(&snapshotMux_);
        if (pendingQuickStartSpeed_ >= 0.0f) {
            quickStartSpeed = pendingQuickStartSpeed_;
            quickStartMs = pendingQuickStartMs_;
            pendingQuickStartSpeed_ = -1.0f;
        }
        portEXIT_CRITICAL(&snapshotMux_);

        if (quickStartSpeed >= 0.0f) {
            simulator_.submitSpeedTarget(quickStartSpeed, quickStartMs);
        }

        // 1. Advance simulator state using target accepted prior to this tick
        simulator_.update(nowMs);

        // 2. Obtain authoritative ApplicationSnapshot representing resulting simulated state
        const ApplicationSnapshot snapshot = simulator_.getSnapshot();

        // 3. Evaluate authority
        const bool authoritative = ControlRuntime::isSnapshotAuthoritative(snapshot, nowMs);
        if (!authoritative) {
            lostAuthorityCount_++;
        }

        // 4. Tick domain session & target dispatcher
        coordinator_.tick(session_, dispatcher_, simulator_, snapshot, nowMs);

        // 5. Publish snapshot copy for external consumers (cross-core spinlock protected)
        const WorkoutSessionSnapshot sessSnap = session_.getSnapshot();
        const float targetSpeed = simulator_.getTargetSpeedKmh();
        const float targetIncline = simulator_.getTargetInclinePct();

        portENTER_CRITICAL(&snapshotMux_);
        publishedSnapshot_ = snapshot;
        publishedSessionSnapshot_ = sessSnap;
        publishedSimTargetSpeedKmh_ = targetSpeed;
        publishedSimTargetInclinePct_ = targetIncline;
        portEXIT_CRITICAL(&snapshotMux_);

        // 6. Periodically check stack high-water mark (every 1000 ms)
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
    TestbenchTelemetry telem;
    portENTER_CRITICAL(&snapshotMux_);
    telem.snapshot = publishedSnapshot_;
    telem.sessionSnapshot = publishedSessionSnapshot_;
    telem.simTargetSpeedKmh = publishedSimTargetSpeedKmh_;
    telem.simTargetInclinePct = publishedSimTargetInclinePct_;
    telem.lostAuthorityCount = lostAuthorityCount_;
    telem.minFreeStackBytes = minFreeStackBytes_;
    portEXIT_CRITICAL(&snapshotMux_);

    telem.authoritative = ControlRuntime::isSnapshotAuthoritative(telem.snapshot, nowMs);
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
    return "TestbenchControlRuntime/1.0.0";
}

} // namespace stridecontrol

#endif // STRIDECONTROL_TESTBENCH
