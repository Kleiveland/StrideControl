#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace stridecontrol {

constexpr size_t kMaxSpeedCalibrationPoints = 10;

/**
 * @brief Incline subsystem persistent calibration and configuration.
 */
struct InclineConfig {
    int32_t homingOffset = 0;
    bool isCalibrated = false;
};

/**
 * @brief Individual speed mapping calibration point.
 */
struct SpeedCalibrationPoint {
    float targetSpeedKmh = 0.0f;
    float consoleSpeedKmh = 0.0f;
};

/**
 * @brief Speed subsystem persistent calibration and configuration.
 */
struct SpeedConfig {
    std::array<SpeedCalibrationPoint, kMaxSpeedCalibrationPoints> points{};
    uint8_t pointCount = 0;
};

/**
 * @brief Hardware utilization and odometer maintenance counters.
 */
struct MaintenanceConfig {
    uint64_t totalDistanceMeters = 0;
    uint64_t totalTimeSeconds = 0;
};

/**
 * @brief Aggregate system configuration holding all subsystem settings.
 */
struct SystemConfig {
    InclineConfig incline{};
    SpeedConfig speed{};
    MaintenanceConfig maintenance{};
};

} // namespace stridecontrol
