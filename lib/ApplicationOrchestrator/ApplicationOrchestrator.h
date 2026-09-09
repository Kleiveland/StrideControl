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
 * @brief Context passed to ApplicationOrchestrator when stepped externally.
 */
struct ApplicationTickContext {
    uint64_t tickIndex = 0;
    uint64_t scenarioTimeUs = 0;
    uint32_t nowUs32 = 0;
    uint32_t nowMs = 0;
    uint32_t loopDeltaMs = 0;

    ApplicationTickContext() = default;
    ApplicationTickContext(uint64_t idx, uint64_t scUs, uint32_t dtMs)
        : tickIndex(idx),
          scenarioTimeUs(scUs),
          nowUs32(static_cast<uint32_t>(scUs)),
          nowMs(static_cast<uint32_t>(scUs / 1000ULL)),
          loopDeltaMs(dtMs) {}
};

/**
 * @brief Execution mode governing ApplicationOrchestrator thread lifecycle.
 */
enum class OrchestratorExecutionMode : uint8_t {
    AutonomousTask,  ///< Production: Spawns Core 1 RT FreeRTOS task and Core 0 BLE task
    ExternalStep     ///< Firmware-HIL: Stepped synchronously by ScenarioRunner/test runner
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

    bool begin(const ApplicationOrchestratorDependencies& deps,
               OrchestratorExecutionMode mode = OrchestratorExecutionMode::AutonomousTask);
    bool end(uint32_t timeoutMs = 1000);

    bool isRunning() const;
    bool isInitialized() const;
    bool isWorkerTaskRunning() const;
    bool acceptsExternalSteps() const;
    OrchestratorExecutionMode executionMode() const;

    bool step(const ApplicationTickContext& context);

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

    bool executePipelineStep(const ApplicationTickContext& context);

    using SpeedUpdateStrategy = void (*)(SpeedSensor& sensor, const ApplicationTickContext& context);
    static void hardwareSpeedStrategy(SpeedSensor& sensor, const ApplicationTickContext& context);
    static void softwareSpeedStrategy(SpeedSensor& sensor, const ApplicationTickContext& context);

    using InclineUpdateStrategy = bool (*)(InclineSensor& sensor, const ApplicationTickContext& context);
    static bool hardwareInclineStrategy(InclineSensor& sensor, const ApplicationTickContext& context);
    static bool softwareInclineStrategy(InclineSensor& sensor, const ApplicationTickContext& context);

    SpeedUpdateStrategy speedUpdateStrategy_ = nullptr;
    InclineUpdateStrategy inclineUpdateStrategy_ = nullptr;

    mutable portMUX_TYPE snapshotMux_ = portMUX_INITIALIZER_UNLOCKED;
    mutable portMUX_TYPE metricsMux_ = portMUX_INITIALIZER_UNLOCKED;

    ApplicationOrchestratorDependencies deps_{};
    ApplicationSnapshot publishedSnapshot_{};
    ApplicationSnapshot stagingSnapshot_{};

    OrchestratorExecutionMode executionMode_ = OrchestratorExecutionMode::AutonomousTask;
    bool initialized_ = false;

    TaskHandle_t taskHandle_ = nullptr;
    SemaphoreHandle_t exitSem_ = nullptr;
    volatile bool running_ = false;
    volatile bool stopRequested_ = false;

    TaskHandle_t bleTaskHandle_ = nullptr;
    SemaphoreHandle_t bleExitSem_ = nullptr;

    bool hasAcceptedFirstTick_ = false;
    uint64_t lastAcceptedTickIndex_ = 0;
    uint64_t lastAcceptedScenarioTimeUs_ = 0;

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
