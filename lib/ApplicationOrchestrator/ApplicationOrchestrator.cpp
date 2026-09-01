#include "ApplicationOrchestrator.h"
#include <Arduino.h>
#include "../BleManager/BleManager.h"
#include "../HeartRateClient/HeartRateClient.h"

namespace stridecontrol {

ApplicationOrchestrator::ApplicationOrchestrator() = default;

ApplicationOrchestrator::~ApplicationOrchestrator() {
    end(1000);
    if (exitSem_ != nullptr) {
        vSemaphoreDelete(exitSem_);
        exitSem_ = nullptr;
    }
    if (bleExitSem_ != nullptr) {
        vSemaphoreDelete(bleExitSem_);
        bleExitSem_ = nullptr;
    }
}

bool ApplicationOrchestrator::begin(const ApplicationOrchestratorDependencies& deps) {
    portENTER_CRITICAL(&metricsMux_);
    if (running_ || taskHandle_ != nullptr || bleTaskHandle_ != nullptr) {
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

    if (exitSem_ == nullptr) {
        portENTER_CRITICAL(&metricsMux_);
        running_ = false;
        portEXIT_CRITICAL(&metricsMux_);
        return false;
    }

    // Allocate BLE exit semaphore if BLE manager is provided
    if (deps_.bleManager != nullptr) {
        if (bleExitSem_ == nullptr) {
            bleExitSem_ = xSemaphoreCreateBinary();
        } else {
            xSemaphoreTake(bleExitSem_, 0);
        }
        if (bleExitSem_ == nullptr) {
            portENTER_CRITICAL(&metricsMux_);
            running_ = false;
            portEXIT_CRITICAL(&metricsMux_);
            return false;
        }
    }

    // 1. Spawn Core 1 Real-Time Sensor / Dynamics Task
    BaseType_t rtResult = xTaskCreatePinnedToCore(
        taskEntry,
        "AppOrchestrator",
        kTaskStackSize,
        this,
        kTaskPriority,
        &taskHandle_,
        kTaskCore
    );

    if (rtResult != pdPASS) {
        portENTER_CRITICAL(&metricsMux_);
        running_ = false;
        taskHandle_ = nullptr;
        portEXIT_CRITICAL(&metricsMux_);
        if (bleExitSem_ != nullptr) {
            vSemaphoreDelete(bleExitSem_);
            bleExitSem_ = nullptr;
        }
        return false;
    }

    // 2. Spawn Core 0 Dedicated BLE Lifecycle Task
    if (deps_.bleManager != nullptr) {
        BaseType_t bleResult = xTaskCreatePinnedToCore(
            bleTaskEntry,
            "BleLifecycle",
            kBleTaskStackSize,
            this,
            kBleTaskPriority,
            &bleTaskHandle_,
            kBleTaskCore
        );

        if (bleResult != pdPASS) {
            // Rollback Core 1 real-time task
            end(1000);
            if (bleExitSem_ != nullptr) {
                vSemaphoreDelete(bleExitSem_);
                bleExitSem_ = nullptr;
            }
            return false;
        }
    }

    return true;
}

bool ApplicationOrchestrator::end(uint32_t timeoutMs) {
    portENTER_CRITICAL(&metricsMux_);
    if (!running_ && taskHandle_ == nullptr && bleTaskHandle_ == nullptr) {
        portEXIT_CRITICAL(&metricsMux_);
        return true;
    }
    stopRequested_ = true;
    portEXIT_CRITICAL(&metricsMux_);

    // Step 1 & 2: Wait for confirmed Core 1 task exit
    bool cleanRtExit = false;
    if (exitSem_ != nullptr) {
        cleanRtExit = (xSemaphoreTake(exitSem_, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
    } else {
        const uint32_t startMs = millis();
        while (millis() - startMs < timeoutMs) {
            portENTER_CRITICAL(&metricsMux_);
            const bool stillRunning = running_;
            portEXIT_CRITICAL(&metricsMux_);
            if (!stillRunning) {
                cleanRtExit = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Step 3: If Core 1 exit times out, do NOT signal BLE task, delete semaphores, or reset storage
    if (!cleanRtExit) {
        portENTER_CRITICAL(&metricsMux_);
        if (taskHandle_ != nullptr) {
            // Forcefully delete timed out Core 1 task to avoid dangling memory access
            vTaskDelete(taskHandle_);
            taskHandle_ = nullptr;
            if (deps_.diagnosticsService != nullptr) {
                deps_.diagnosticsService->reportFault(FaultCode::SystemWatchdogWarning, millis());
            }
        }
        running_ = false;
        portEXIT_CRITICAL(&metricsMux_);
        return false;
    }

    portENTER_CRITICAL(&metricsMux_);
    if (taskHandle_ != nullptr) {
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }
    running_ = false;
    portEXIT_CRITICAL(&metricsMux_);

    // Step 4: Signal Core 0 BLE Lifecycle Task to terminate cleanly
    bool cleanBleExit = true;
    if (bleTaskHandle_ != nullptr) {
        xTaskNotifyGive(bleTaskHandle_);

        // Step 5: Wait for bleExitSem_ with bounded timeout
        cleanBleExit = (bleExitSem_ != nullptr &&
                        xSemaphoreTake(bleExitSem_, pdMS_TO_TICKS(kBleTaskExitTimeoutMs)) == pdTRUE);

        // Step 6 & 7: Handshake resolution
        if (cleanBleExit) {
            vTaskDelete(bleTaskHandle_);
            bleTaskHandle_ = nullptr;
            vSemaphoreDelete(bleExitSem_);
            bleExitSem_ = nullptr;
        } else {
            // BLE exit timed out: Preserve bleTaskHandle_ and bleExitSem_ to prevent use-after-free
            return false;
        }
    }

    // Allow scheduler brief tick to cleanup task control block
    vTaskDelay(pdMS_TO_TICKS(10));

    return true;
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

const char* ApplicationOrchestrator::version() {
    return "1.3.0";
}

void ApplicationOrchestrator::taskEntry(void* param) {
    auto* self = static_cast<ApplicationOrchestrator*>(param);
    if (self != nullptr) {
        self->runLoop();
    }
}

void ApplicationOrchestrator::bleTaskEntry(void* param) {
    auto* self = static_cast<ApplicationOrchestrator*>(param);
    if (self != nullptr) {
        self->runBleTask();
    }
}

void ApplicationOrchestrator::runBleTask() {
    BleManager* bleMgr = deps_.bleManager;
    HeartRateClient* hrCli = deps_.hrClient;
    const BleConfig& bleCfg = deps_.bleConfig;

    bool bleOk = false;
    bool hrOk = false;

    // Initialization Phase (Core 0 context)
    if (bleMgr != nullptr) {
        bleOk = bleMgr->begin(bleCfg);
        if (bleOk && hrCli != nullptr) {
            hrOk = hrCli->begin(bleCfg, bleMgr);
        }
    }

    // Periodic Execution Loop (Cadence ~20ms, non-deterministic)
    while (ulTaskNotifyTake(pdTRUE, 0) == 0) {
        const uint32_t nowMs = millis();
        if (bleOk) {
            bleMgr->update(nowMs);
        }
        if (hrOk) {
            hrCli->update(nowMs);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Teardown Phase (Strict ordering: HeartRateClient -> BleManager)
    if (hrOk && hrCli != nullptr) {
        hrCli->end();
    }
    if (bleOk && bleMgr != nullptr) {
        bleMgr->end();
    }

    // Signal confirmation semaphore and self-suspend
    SemaphoreHandle_t sem = bleExitSem_;
    if (sem != nullptr) {
        xSemaphoreGive(sem);
    }

    // STRICT RULE: After xSemaphoreGive(), perform NO further member access
    vTaskSuspend(NULL);
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

        // 7. Assemble Complete ApplicationSnapshot DTO (Fast Critical-Section Telemetry Sample)
        ApplicationSnapshot snap{};
        snap.timestampMs = nowMs;
        snap.sequenceNumber = sequenceNumber_++;
        snap.speed = speedState;
        snap.incline = inclineState;
        snap.imu = imuState;
        snap.runner = runnerState;
        snap.inclineVerifier = verifierState;
        snap.health = healthSnap;
        if (deps_.hrClient != nullptr) {
            snap.heartRate = deps_.hrClient->getState();
        }
        if (deps_.bleManager != nullptr) {
            snap.ble = deps_.bleManager->getState();
        }

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

} // namespace stridecontrol
