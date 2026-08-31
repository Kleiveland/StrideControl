#pragma once

#include <cstdint>

namespace stridecontrol {

/**
 * @brief Evaluation status classification for speed calibration lookups.
 */
enum class SpeedCalibrationResultStatus : uint8_t {
    Stopped,              // Speed <= 0.0 km/h
    Calibrated,           // Valid piecewise linear interpolation within [X_0, X_{N-1}] or exact point match
    IdentityFallback,     // Uncalibrated table or default fallback
    BelowCalibratedRange, // Desired physical speed < X_0 (handled via identity policy)
    AboveCalibratedRange, // Desired physical speed > X_{N-1} (held at highest calibrated command)
    ClampedLow,           // Command clamped to 0.8 km/h console minimum
    ClampedHigh,          // Command clamped to 25.0 km/h console maximum
    InvalidInput          // Non-finite input (NaN, Inf)
};

/**
 * @brief Result structure capturing the input desired speed, resulting command, and status.
 */
struct SpeedCalibrationResult {
    float desiredPhysicalSpeedKmh = 0.0f;
    float treadmillCommandKmh = 0.0f;
    SpeedCalibrationResultStatus status = SpeedCalibrationResultStatus::InvalidInput;

    SpeedCalibrationResult() = default;
    SpeedCalibrationResult(float desired, float cmd, SpeedCalibrationResultStatus st)
        : desiredPhysicalSpeedKmh(desired), treadmillCommandKmh(cmd), status(st) {}
};

} // namespace stridecontrol

