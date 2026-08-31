#include "ApplicationOrchestrator.h"
#include <Arduino.h>

namespace stridecontrol {

ApplicationOrchestrator::ApplicationOrchestrator() = default;

ApplicationOrchestrator::~ApplicationOrchestrator() {
    end(1000);
    if (exitSem_ != nullptr) {
        vSemaphoreDelete(exitSem_);
        exitSem_ = nullptr;
    }
}

bool ApplicationOrchestrator::begin(const ApplicationOrchestratorDependencies& deps) {
    portENTER_CRITICAL(&metricsMux_);
    if (running_ || taskHandle_ != nullptr) {
        portEXIT_CRITICAL(&metricsMux_);
        return false;
    }
    deps_ = deps;
    stopRequested_ = false;
    running_ = true;
    loopCount_ = 0;
    deadlineMissCount_ = 0;
    overrunCount_ = 0;
    sequenceNumber_ = 0;
    portEXIT_CRITICAL(&metricsMux_);

    if (exitSem_ == nullptr) {
        exitSem_ = xSemaphoreCreateBinary();
    } else {
        xSemaphoreTake(exitSem_, 0);
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        taskEntry,
        "AppOrchestrator",
        kTaskStackSize,
        this,
        kTaskPriority,
        &taskHandle_,
        kTaskCore
    );

    if (result != pdPASS) {
        portENTER_CRITICAL(&metricsMux_);
        running_ = false;
        taskHandle_ = nullptr;
        portEXIT_CRITICAL(&metricsMux_);
        return false;
    }

    return true;
}

bool ApplicationOrchestrator::end(uint32_t timeoutMs) {
    portENTER_CRITICAL(&metricsMux_);
    if (!running_ && taskHandle_ == nullptr) {
        portEXIT_CRITICAL(&metricsMux_);
        return true;
    }
    stopRequested_ = true;
    portEXIT_CRITICAL(&metricsMux_);

    bool cleanExit = false;
    if (exitSem_ != nullptr) {
        cleanExit = (xSemaphoreTake(exitSem_, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
    } else {
        const uint32_t startMs = millis();
        while (millis() - startMs < timeoutMs) {
            portENTER_CRITICAL(&metricsMux_);
            const bool stillRunning = running_;
            portEXIT_CRITICAL(&metricsMux_);
            if (!stillRunning) {
                cleanExit = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    portENTER_CRITICAL(&metricsMux_);
    if (!cleanExit && taskHandle_ != nullptr) {
        // Cooperative shutdown timed out. Forcefully delete task to guarantee zero use-after-free.
        vTaskDelete(taskHandle_);
        if (deps_.diagnosticsService != nullptr) {
            deps_.diagnosticsService->reportFault(FaultCode::SystemWatchdogWarning, millis());
        }
    }
    running_ = false;
    taskHandle_ = nullptr;
    portEXIT_CRITICAL(&metricsMux_);

    // Allow scheduler brief tick to cleanup task control block
    vTaskDelay(pdMS_TO_TICKS(10));

    return cleanExit;
}

bool ApplicationOrchestrator::isRunning() const {
    portENTER_CRITICAL(&metricsMux_);
    const bool r = running_;
    portEXIT_CRITICAL(&metricsMux_);
    return r;
}

ApplicationSnapshot ApplicationOrchestrator::getSnapshot() const {
    ApplicationSnapshot snap;
    portENTER_CRITICAL(&snapshotMux_);
    snap = publishedSnapshot_;
    portEXIT_CRITICAL(&snapshotMux_);
    return snap;
}

uint32_t ApplicationOrchestrator::getLoopCount() const {
    portENTER_CRITICAL(&metricsMux_);
    const uint32_t val = loopCount_;
    portEXIT_CRITICAL(&metricsMux_);
    return val;
}

uint32_t ApplicationOrchestrator::getDeadlineMissCount() const {
    portENTER_CRITICAL(&metricsMux_);
    const uint32_t val = deadlineMissCount_;
    portEXIT_CRITICAL(&metricsMux_);
    return val;
}

uint32_t ApplicationOrchestrator::getOverrunCount() const {
    portENTER_CRITICAL(&metricsMux_);
    const uint32_t val = overrunCount_;
    portEXIT_CRITICAL(&metricsMux_);
    return val;
}

void ApplicationOrchestrator::taskEntry(void* param) {
    auto* self = static_cast<ApplicationOrchestrator*>(param);
    if (self != nullptr) {
        self->runLoop();
    }
}

void ApplicationOrchestrator::runLoop() {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(kPeriodMs);
    uint32_t lastLoopTimestampMs = millis();

    while (!stopRequested_) {
        const uint32_t nowMs = millis();
        const uint32_t loopDeltaMs = nowMs - lastLoopTimestampMs;
        lastLoopTimestampMs = nowMs;

        // 1. Update Hardware & Bus Interfaces
        if (deps_.speedSensor != nullptr) {
            deps_.speedSensor->update();
        }
        if (deps_.inclineSensor != nullptr) {
            deps_.inclineSensor->update();
        }
        if (deps_.csafeInterface != nullptr) {
            deps_.csafeInterface->update();
        }
        if (deps_.imuInterface != nullptr) {
            deps_.imuInterface->update();
        }

        // 2. Obtain Immutable Interface States
        SpeedSensorState speedState = deps_.speedSensor ? deps_.speedSensor->getState() : SpeedSensorState{};
        InclineState inclineState = deps_.inclineSensor ? deps_.inclineSensor->getState() : InclineState{};
        CsafeState csafeState = deps_.csafeInterface ? deps_.csafeInterface->getState() : CsafeState{};
        ImuState imuState = deps_.imuInterface ? deps_.imuInterface->getState() : ImuState{};

        // 3. Drain and Buffer IMU Samples
        ImuSample imuSamples[kMaxImuBatchSize];
        size_t sampleCount = 0;
        if (deps_.imuInterface != nullptr) {
            sampleCount = deps_.imuInterface->readSamples(imuSamples, kMaxImuBatchSize);
        }

        // 4. Update Signal Processing & Analysis Engines
        RunnerDynamicsState runnerState{};
        if (deps_.runnerDynamics != nullptr) {
            deps_.runnerDynamics->update(imuSamples, sampleCount, speedState, csafeState, nowMs);
            runnerState = deps_.runnerDynamics->getState();
        }

        InclineVerifierState verifierState{};
        if (deps_.inclineVerifier != nullptr) {
            InclineVerificationCommandInput cmdInput{};
            deps_.inclineVerifier->update(cmdInput, inclineState, imuState, nowMs);
            verifierState = deps_.inclineVerifier->getState();
        }

        // 5. Update Maintenance Odometer (Wear-Leveled RAM Accumulation)
        if (deps_.maintenanceService != nullptr) {
            deps_.maintenanceService->update(speedState.speedKmh, loopDeltaMs > 0 ? loopDeltaMs : kPeriodMs);
        }

        // 6. Obtain Current Health Snapshot
        SystemHealthSnapshot healthSnap{};
        if (deps_.diagnosticsService != nullptr) {
            healthSnap = deps_.diagnosticsService->getSnapshot(nowMs);
        }

        // 7. Assemble Complete ApplicationSnapshot DTO
        ApplicationSnapshot snap{};
        snap.timestampMs = nowMs;
        snap.sequenceNumber = sequenceNumber_++;
        snap.speed = speedState;
        snap.incline = inclineState;
        snap.imu = imuState;
        snap.runner = runnerState;
        snap.inclineVerifier = verifierState;
        snap.health = healthSnap;

        // 8. Publish Snapshot via Spinlock-Protected By-Value Copy
        portENTER_CRITICAL(&snapshotMux_);
        publishedSnapshot_ = snap;
        portEXIT_CRITICAL(&snapshotMux_);

        // 9. Update Execution Metrics
        const uint32_t executionDurationMs = millis() - nowMs;
        portENTER_CRITICAL(&metricsMux_);
        loopCount_++;
        if (executionDurationMs > kPeriodMs) {
            overrunCount_++;
        }
        portEXIT_CRITICAL(&metricsMux_);

        // 10. Drift-Resistant Periodic Delay
        const BaseType_t delaySuccess = xTaskDelayUntil(&lastWakeTime, periodTicks);
        if (delaySuccess != pdTRUE) {
            portENTER_CRITICAL(&metricsMux_);
            deadlineMissCount_++;
            portEXIT_CRITICAL(&metricsMux_);
        }
    }

    portENTER_CRITICAL(&metricsMux_);
    running_ = false;
    portEXIT_CRITICAL(&metricsMux_);

    if (exitSem_ != nullptr) {
        xSemaphoreGive(exitSem_);
    }

    vTaskDelete(nullptr);
}

const char* ApplicationOrchestrator::version() {
    return "1.1.0";
}

} // namespace stridecontrol
