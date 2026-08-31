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
 * @brief Individual physical-to-command speed calibration point.
 *
 * Represents an empirical observation: When commanding 'treadmillCommandKmh'
 * to the console, the physical belt tachometer measured 'measuredPhysicalSpeedKmh'.
 */
struct SpeedCalibrationPoint {
    float measuredPhysicalSpeedKmh = 0.0f; // Physical belt speed measured by SpeedSensor (X-axis)
    float treadmillCommandKmh = 0.0f;      // Treadmill console command injected to O2 (Y-axis)
};

/**
 * @brief Speed subsystem persistent calibration and configuration.
 */
struct SpeedConfig {
    std::array<SpeedCalibrationPoint, kMaxSpeedCalibrationPoints> points{};
    uint8_t pointCount = 0;
    bool commandMapValid = false;
    float maxAchievableSpeedKmh = 25.0f;
    bool maxAchievableSpeedVerified = false;
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
