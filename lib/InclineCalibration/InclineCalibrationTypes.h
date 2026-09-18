#pragma once
#ifndef STRIDECONTROL_INCLINECALIBRATIONTYPES_H
#define STRIDECONTROL_INCLINECALIBRATIONTYPES_H

#include <cstdint>
#include <cstddef>
#include <array>

namespace stridecontrol {

enum class CalibrationSource : uint8_t {
    Unknown = 0,
    FactoryDefault = 1,
    Simulated = 2,
    PhysicalCommissioning = 3
};

/**
 * @brief Individual empirical actual-to-command incline calibration point.
 *
 * Represents an empirical observation: When commanding 'treadmillCommandPct'
 * to the console, the IMU measured 'measuredActualInclinePct'.
 */
struct InclineCalibrationPoint {
    float measuredActualInclinePct = 0.0f; // Actual incline measured via IMU during commissioning (X-axis)
    float treadmillCommandPct = 0.0f;      // Treadmill console command that produced it (Y-axis)
};

static constexpr uint8_t kMaxInclineCalibrationPoints = 10; // Mirrors kMaxSpeedCalibrationPoints

/**
 * @brief Incline subsystem persistent calibration and configuration.
 */
struct InclineConfig {
    std::array<InclineCalibrationPoint, kMaxInclineCalibrationPoints> points{};
    uint8_t pointCount = 0;
    bool commandMapValid = false;
    float maxAchievableInclinePct = 15.0f; // Confirmed real user-visible range is 0-15%
    bool maxAchievableInclineVerified = false;
    CalibrationSource source = CalibrationSource::Unknown;
    uint32_t calibratedAtMs = 0;
};

/**
 * @brief Evaluation status classification for incline calibration lookups.
 */
enum class InclineCalibrationResultStatus : uint8_t {
    Flat,                 // Desired incline == 0%
    Calibrated,           // Valid piecewise linear interpolation or exact point match
    IdentityFallback,     // Uncalibrated table or default fallback
    BelowCalibratedRange, // Desired physical incline < X_0 (handled via identity policy)
    AboveCalibratedRange, // Desired physical incline > X_{N-1} (held at highest calibrated command)
    ClampedLow,           // Clamped to 0% console minimum
    ClampedHigh,          // Clamped to 15% console maximum
    InvalidInput          // Non-finite input (NaN, Inf)
};

/**
 * @brief Result structure capturing the input desired incline, resulting command, and status.
 */
struct InclineCalibrationResult {
    float desiredActualInclinePct = 0.0f;
    float treadmillCommandPct = 0.0f;
    InclineCalibrationResultStatus status = InclineCalibrationResultStatus::IdentityFallback;

    InclineCalibrationResult() = default;
    InclineCalibrationResult(float desired, float cmd, InclineCalibrationResultStatus st)
        : desiredActualInclinePct(desired), treadmillCommandPct(cmd), status(st) {}
};

} // namespace stridecontrol

#endif // STRIDECONTROL_INCLINECALIBRATIONTYPES_H

