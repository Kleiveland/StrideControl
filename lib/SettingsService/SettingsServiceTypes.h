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
 * @brief Persistent Bluetooth subsystem configuration.
 */
struct BleConfig {
    char preferredHrMac[18] = "";                          // Target HR sensor MAC ("AA:BB:CC:DD:EE:FF" or empty)
    bool autoConnectHr = true;                             // Auto-connect to preferred HR sensor
    char advertisedDeviceName[32] = "Sportsmaster T610";   // Advertised peripheral name
    bool enableFtms = true;                                // Enable FTMS Treadmill Service (0x1826)
    bool enableRscFootpod = false;                         // Enable RSC Footpod Service (0x1814, default disabled)
    uint8_t ftmsNotifyRateHz = 2;                          // FTMS notification frequency (1-4 Hz)
    uint8_t rscNotifyRateHz = 2;                           // RSC notification frequency (1-4 Hz)
};

/**
 * @brief Aggregate system configuration holding all subsystem settings.
 */
struct SystemConfig {
    InclineConfig incline{};
    SpeedConfig speed{};
    MaintenanceConfig maintenance{};
    BleConfig ble{};
};

} // namespace stridecontrol
