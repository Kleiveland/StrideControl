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
    float runnerSpeedKmh = 0.0f;
    double beltDistanceKm = 0.0;
    const char* runnerPresence = "UNKNOWN";
    uint32_t droppedEventsCount = 0;
    uint8_t heartRateBpm = 0;
    bool heartRateValid = false;
    uint32_t totalElapsedTimeMs = 0;
    double totalElevationMeters = 0.0;
    uint32_t avgHeartRateBpm = 0;
    uint8_t maxHeartRateBpm = 0;
    bool heartRateEverValid = false;
    uint16_t workoutId = 0;
    uint8_t totalStepCount = 0;
    float stepProgressFraction = 0.0f;
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
