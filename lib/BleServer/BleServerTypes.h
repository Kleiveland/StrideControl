#pragma once

#include <cstdint>
#include <cstddef>

namespace stridecontrol {

// Maximum number of simultaneous subscriber connections tracked per characteristic
static constexpr size_t kMaxTrackedSubscribers = 4;

// Proposed StrideControl V1 Telemetry Cadences (250 ms = 4 Hz)
static constexpr uint32_t kFtmsNotificationIntervalMs = 250;
static constexpr uint32_t kRscNotificationIntervalMs  = 250;

// RSC Constants (0x1814)
// Feature: Total Distance Measurement Supported (Bit 1) + Walking or Running Status Supported (Bit 2) = 0x0006
static constexpr uint16_t kRscFeatureTotalDistanceAndStatus = 0x0006;
// Sensor Location: 0x00 = Other (Virtual treadmill bridge presentation per Bluetooth SIG Assigned Numbers)
static constexpr uint8_t kRscSensorLocationOther = 0x00;

// FTMS Constants (0x1826 - Telemetry Scope)
// Machine Features (Byte 0..3): Bit 2 (Total Distance), Bit 3 (Inclination & Ramp Angle), Bit 10 (Heart Rate)
static constexpr uint32_t kFtmsMachineFeatures =
    (1UL << 2) |   // Total Distance Supported
    (1UL << 3) |   // Inclination Supported (includes Ramp Angle)
    (1UL << 10);   // Heart Rate Measurement Supported (static capability)

// Target Setting Features (Byte 4..7): Telemetry-Only V1 (0 = Control Point / Target Setting Disabled)
static constexpr uint32_t kFtmsTargetSettingFeatures = 0x00000000;

/**
 * @brief Subscription tracking entry for fixed-capacity subscriber tables.
 */
struct BleSubscriptionEntry {
    uint16_t connectionHandle{0xFFFF};
    bool active{false};
};

/**
 * @brief Read-only operational state DTO for RSC Service.
 */
struct RscServerStateDto {
    bool initialized{false};
    bool hasMeasurementSubscribers{false};
    uint32_t lastNotificationMs{0};
};

/**
 * @brief Read-only operational state DTO for FTMS Service.
 */
struct FtmsServerStateDto {
    bool initialized{false};
    bool hasTreadmillDataSubscribers{false};
    uint32_t lastNotificationMs{0};
};

} // namespace stridecontrol

