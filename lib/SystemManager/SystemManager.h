#pragma once

#include <cstdint>
#include "SystemManagerTypes.h"
#include "../ApplicationOrchestrator/ApplicationOrchestrator.h"
#include "../ControlRuntime/ControlRuntime.h"

namespace stridecontrol {

/**
 * @brief Core 0 domain logic and control runtime owner for StrideControl.
 *
 * Encapsulates ControlRuntime (managing TreadmillController, WorkoutSession,
 * WorkoutDispatcher, and ControlCoordinator) on Core 0, and passively evaluates
 * the latest ApplicationSnapshot from ApplicationOrchestrator to track high-level
 * system state.
 */
class SystemManager {
public:
    SystemManager(
        ConsoleInterface& console,
        SpeedCalibration& calibration,
        DiagnosticsService& diagnostics
    );
    ~SystemManager() = default;

    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    bool begin(uint32_t nowMs = 0);
    bool begin(ApplicationOrchestrator* orchestrator, uint32_t nowMs = 0);
    void end();

    void update();

    bool startControlTask(ApplicationOrchestrator* orchestrator);
    bool stopControlTask(uint32_t timeoutMs = 1000);

    const ControlRuntime& getControlRuntime() const { return controlRuntime_; }
    ControlRuntime& getControlRuntime() { return controlRuntime_; }

    SystemState getState() const;
    bool isFaulted() const;
    bool isSafetyStop() const;

    static const char* version();

private:
    ControlRuntime controlRuntime_;
    ApplicationOrchestrator* orchestrator_ = nullptr;
    SystemState state_ = SystemState::Initializing;
    bool initialized_ = false;
    float previousSpeedKmh_ = 0.0f;
};

} // namespace stridecontrol

