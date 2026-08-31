#pragma once

#include <cstdint>

namespace stridecontrol {

/**
 * @brief High-level domain logic states for StrideControl.
 */
enum class SystemState : uint8_t {
    Initializing = 0,
    Ready,
    Starting,
    Running,
    Stopping,
    Faulted,
    SafetyStop
};

/**
 * @brief Convert SystemState enum to human-readable string (C++11 constexpr compliant).
 */
constexpr const char* systemStateToString(SystemState state) {
    return (state == SystemState::Initializing) ? "INITIALIZING" :
           (state == SystemState::Ready)        ? "READY" :
           (state == SystemState::Starting)     ? "STARTING" :
           (state == SystemState::Running)      ? "RUNNING" :
           (state == SystemState::Stopping)     ? "STOPPING" :
           (state == SystemState::Faulted)      ? "FAULTED" :
           (state == SystemState::SafetyStop)   ? "SAFETY_STOP" :
           "UNKNOWN";
}

} // namespace stridecontrol

