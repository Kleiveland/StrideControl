#pragma once

#include <cstdint>

namespace stridecontrol {

/**
 * @brief Thread-safe, copy-based telemetry payload consumed by WebServerManager.
 */
struct TelemetryReport {
    uint32_t timestampMs = 0;
    bool authority = false;
    float actualSpeedKmh = 0.0f;
    float actualInclinePct = 0.0f;
    double runnerDistanceKm = 0.0;
    const char* sessionState = "Idle";
    uint8_t stepIndex = 0;
    uint32_t stepRemainingMs = 0;
    float targetSpeedKmh = 0.0f;
    float targetInclinePct = 0.0f;
};

/**
 * @brief Agnostic provider interface for real-time telemetry feeding REST & WebSockets.
 */
class ITelemetryProvider {
public:
    virtual ~ITelemetryProvider() = default;
    virtual bool getTelemetry(TelemetryReport& report) const = 0;
};

} // namespace stridecontrol
