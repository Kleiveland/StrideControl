#pragma once

#include <cstdint>

namespace stridecontrol {

/**
 * @brief High-level domain logic states for StrideControl.
 */
enum class SystemState : uint8_t {
    Initializing = 0,
    Operational,
    Degraded,
    Faulted,
    ShuttingDown
};

/**
 * @brief Convert SystemState enum to human-readable string (C++11 constexpr compliant).
 */
constexpr const char* systemStateToString(SystemState state) {
    return (state == SystemState::Initializing) ? "INITIALIZING" :
           (state == SystemState::Operational)  ? "OPERATIONAL" :
           (state == SystemState::Degraded)     ? "DEGRADED" :
           (state == SystemState::Faulted)      ? "FAULTED" :
           (state == SystemState::ShuttingDown) ? "SHUTTING_DOWN" :
           "UNKNOWN";
}

} // namespace stridecontrol

