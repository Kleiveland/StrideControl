#pragma once

#include <cstdint>
#include "SystemManagerTypes.h"
#include "../ApplicationOrchestrator/ApplicationOrchestrator.h"

namespace stridecontrol {

/**
 * @brief Core 0 domain logic state machine for StrideControl.
 *
 * Passively evaluates the latest ApplicationSnapshot from the orchestrator
 * and transitions between high-level operational states without direct hardware access.
 */
class SystemManager {
public:
    SystemManager();
    ~SystemManager() = default;

    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    void begin(ApplicationOrchestrator* orchestrator);
    void update();

    SystemState getState() const;
    bool isFaulted() const;
    bool isSafetyStop() const;

    static const char* version();

private:
    ApplicationOrchestrator* orchestrator_ = nullptr;
    SystemState state_ = SystemState::Initializing;
    bool initialized_ = false;
    float previousSpeedKmh_ = 0.0f;
};

} // namespace stridecontrol

