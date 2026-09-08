#pragma once

#include <cstdint>
#include <cstddef>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "../ApplicationSnapshot/ApplicationSnapshot.h"
#include "../SpeedSensor/SpeedSensor.h"
#include "../InclineSensor/InclineSensor.h"
#include "../ImuInterface/ImuInterface.h"
#include "../CsafeInterface/CsafeInterface.h"
#include "../RunnerDynamics/RunnerDynamics.h"
#include "../InclineVerifier/InclineVerifier.h"
#include "../DiagnosticsService/DiagnosticsService.h"
#include "../MaintenanceService/MaintenanceService.h"
#include "../SettingsService/SettingsServiceTypes.h"

namespace stridecontrol {

class BleManager;
class HeartRateClient;

/**
 * @brief Dependencies injected into ApplicationOrchestrator.
 */
struct ApplicationOrchestratorDependencies {
    SpeedSensor* speedSensor = nullptr;
    InclineSensor* inclineSensor = nullptr;
    ImuInterface* imuInterface = nullptr;
    CsafeInterface* csafeInterface = nullptr;
    RunnerDynamics* runnerDynamics = nullptr;
    InclineVerifier* inclineVerifier = nullptr;
    DiagnosticsService* diagnosticsService = nullptr;
    MaintenanceService* maintenanceService = nullptr;
    BleManager* bleManager = nullptr;
    HeartRateClient* hrClient = nullptr;
    BleConfig bleConfig{};
};

/**
 * @brief Active Object orchestrating real-time 50Hz sensor polling on Core 1
 *        and dedicated BLE lifecycle coordination on Core 0.
 */
class ApplicationOrchestrator {
public:
    ApplicationOrchestrator();
    virtual ~ApplicationOrchestrator();

    ApplicationOrchestrator(const ApplicationOrchestrator&) = delete;
    ApplicationOrchestrator& operator=(const ApplicationOrchestrator&) = delete;

    bool begin(const ApplicationOrchestratorDependencies& deps);
    bool end(uint32_t timeoutMs = 1000);

    bool isRunning() const;

    virtual ApplicationSnapshot getSnapshot() const;

    uint32_t getLoopCount() const;
    uint32_t getDeadlineMissCount() const;
    uint32_t getOverrunCount() const;
    uint32_t getMinFreeStackBytes() const;

    static const char* version();

private:
    static void taskEntry(void* param);
    void runLoop();

    static void bleTaskEntry(void* param);
    void runBleTask();

    mutable portMUX_TYPE snapshotMux_ = portMUX_INITIALIZER_UNLOCKED;
    mutable portMUX_TYPE metricsMux_ = portMUX_INITIALIZER_UNLOCKED;

    ApplicationOrchestratorDependencies deps_{};
    ApplicationSnapshot publishedSnapshot_{};
    ApplicationSnapshot stagingSnapshot_{};

    TaskHandle_t taskHandle_ = nullptr;
    SemaphoreHandle_t exitSem_ = nullptr;
    volatile bool running_ = false;
    volatile bool stopRequested_ = false;

    TaskHandle_t bleTaskHandle_ = nullptr;
    SemaphoreHandle_t bleExitSem_ = nullptr;

    uint32_t sequenceNumber_ = 0;
    uint32_t loopCount_ = 0;
    uint32_t deadlineMissCount_ = 0;
    uint32_t overrunCount_ = 0;
    uint32_t minFreeStackBytes_ = 8192;

    static constexpr uint32_t kPeriodMs = 20; // 50 Hz
    static constexpr size_t kMaxImuBatchSize = 32;
    ImuSample imuSamples_[kMaxImuBatchSize]{};
    static constexpr uint32_t kTaskStackSize = 8192;
    static constexpr UBaseType_t kTaskPriority = 5;
    static constexpr BaseType_t kTaskCore = 1; // APP_CPU_NUM

    static constexpr uint32_t kBleTaskExitTimeoutMs = 3000;
    static constexpr uint32_t kBleTaskStackSize = 4096;
    static constexpr UBaseType_t kBleTaskPriority = 3;
    static constexpr BaseType_t kBleTaskCore = 0; // PRO_CPU_NUM
};

} // namespace stridecontrol
