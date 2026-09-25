#include "SpeedCalibration.h"
#include <cmath>
#include <algorithm>

namespace stridecontrol {

bool SpeedCalibration::setConfiguration(const SpeedConfig& config) {
    if (!validateCandidate(config)) {
        return false;
    }
    config_ = config;
    return true;
}

SpeedCalibrationResult SpeedCalibration::calculateCommand(float desiredPhysicalSpeedKmh) const {
    // 1. Non-finite check
    if (!std::isfinite(desiredPhysicalSpeedKmh)) {
        return {desiredPhysicalSpeedKmh, 0.0f, SpeedCalibrationResultStatus::InvalidInput};
    }

    // 2. Stopped check
    if (desiredPhysicalSpeedKmh <= 0.0f) {
        return {desiredPhysicalSpeedKmh, 0.0f, SpeedCalibrationResultStatus::Stopped};
    }

    // 3. Sub-minimum physical speed input
    if (desiredPhysicalSpeedKmh < 0.8f) {
        return {desiredPhysicalSpeedKmh, 0.8f, SpeedCalibrationResultStatus::ClampedLow};
    }

    // 4. Uncalibrated Mode: Identity Fallback
    if (!config_.commandMapValid || config_.pointCount < 2) {
        if (desiredPhysicalSpeedKmh > 25.0f) {
            return {desiredPhysicalSpeedKmh, 25.0f, SpeedCalibrationResultStatus::ClampedHigh};
        }
        return {desiredPhysicalSpeedKmh, desiredPhysicalSpeedKmh, SpeedCalibrationResultStatus::IdentityFallback};
    }

    // 5. Calibrated Mode
    const size_t count = config_.pointCount;
    const float x0 = config_.points[0].measuredPhysicalSpeedKmh;
    const float xLast = config_.points[count - 1].measuredPhysicalSpeedKmh;

    // Case A: Below lowest calibrated point (desired < X_0) -> Identity clamped to [0.8, 25.0]
    if (desiredPhysicalSpeedKmh < x0) {
        const float cmd = clamp(desiredPhysicalSpeedKmh, 0.8f, 25.0f);
        return {desiredPhysicalSpeedKmh, cmd, SpeedCalibrationResultStatus::BelowCalibratedRange};
    }

    // Case B: Above highest calibrated point (desired > X_{N-1}) -> Held at highest command
    if (desiredPhysicalSpeedKmh > xLast) {
        const float cmd = clamp(config_.points[count - 1].treadmillCommandKmh, 0.8f, 25.0f);
        return {desiredPhysicalSpeedKmh, cmd, SpeedCalibrationResultStatus::AboveCalibratedRange};
    }

    // Case C: Piecewise linear interpolation within [X_0, X_{N-1}]
    for (size_t i = 0; i < count - 1; ++i) {
        const float segX0 = config_.points[i].measuredPhysicalSpeedKmh;
        const float segX1 = config_.points[i + 1].measuredPhysicalSpeedKmh;

        if (desiredPhysicalSpeedKmh >= segX0 && desiredPhysicalSpeedKmh <= segX1) {
            const float segY0 = config_.points[i].treadmillCommandKmh;
            const float segY1 = config_.points[i + 1].treadmillCommandKmh;

            const float y = segY0 + ((segY1 - segY0) / (segX1 - segX0)) * (desiredPhysicalSpeedKmh - segX0);

            if (y > 25.0f) {
                return {desiredPhysicalSpeedKmh, 25.0f, SpeedCalibrationResultStatus::ClampedHigh};
            }
            if (y < 0.8f) {
                return {desiredPhysicalSpeedKmh, 0.8f, SpeedCalibrationResultStatus::ClampedLow};
            }
            return {desiredPhysicalSpeedKmh, y, SpeedCalibrationResultStatus::Calibrated};
        }
    }

    // Safety fallback (should never be reached with monotonic points)
    const float fallbackCmd = clamp(config_.points[count - 1].treadmillCommandKmh, 0.8f, 25.0f);
    return {desiredPhysicalSpeedKmh, fallbackCmd, SpeedCalibrationResultStatus::AboveCalibratedRange};
}

float SpeedCalibration::calculateCommandSpeedKmh(float desiredPhysicalSpeedKmh) const {
    return calculateCommand(desiredPhysicalSpeedKmh).treadmillCommandKmh;
}

float SpeedCalibration::getMaxAchievableSpeedKmh() const {
    return config_.maxAchievableSpeedKmh;
}

bool SpeedCalibration::isCommandMapValid() const {
    return config_.commandMapValid;
}

bool SpeedCalibration::isMaxAchievableSpeedVerified() const {
    return config_.maxAchievableSpeedVerified;
}

uint8_t SpeedCalibration::getPointCount() const {
    return config_.pointCount;
}

static void setValidationError(char* errBuf, size_t errBufLen, const char* msg) {
    if (errBuf != nullptr && errBufLen > 0) {
        snprintf(errBuf, errBufLen, "%s", msg);
    }
}

bool SpeedCalibration::validateCandidate(const SpeedConfig& config, char* errBuf, size_t errBufLen) {
    if (config.pointCount > kMaxSpeedCalibrationPoints) {
        setValidationError(errBuf, errBufLen, "Point count exceeds maximum of 10");
        return false;
    }
    if (!std::isfinite(config.maxAchievableSpeedKmh) ||
        config.maxAchievableSpeedKmh <= 0.0f ||
        config.maxAchievableSpeedKmh > 25.0f) {
        setValidationError(errBuf, errBufLen, "Max achievable speed must be between 0.8 and 25.0 km/h");
        return false;
    }
    if (config.commandMapValid && config.pointCount < 2) {
        setValidationError(errBuf, errBufLen, "At least 2 points required for a valid command map");
        return false;
    }
    if (config.maxAchievableSpeedVerified && !config.commandMapValid) {
        setValidationError(errBuf, errBufLen, "Verified max speed requires an active command map");
        return false;
    }

    for (size_t i = 0; i < config.pointCount; ++i) {
        const auto& pt = config.points[i];
        if (!std::isfinite(pt.measuredPhysicalSpeedKmh) || !std::isfinite(pt.treadmillCommandKmh)) {
            setValidationError(errBuf, errBufLen, "Point contains non-finite speed values");
            return false;
        }
        if (pt.measuredPhysicalSpeedKmh <= 0.0f) {
            setValidationError(errBuf, errBufLen, "Measured physical speed must be positive");
            return false;
        }
        if (pt.treadmillCommandKmh < 0.8f || pt.treadmillCommandKmh > 25.0f) {
            setValidationError(errBuf, errBufLen, "Treadmill command must be between 0.8 and 25.0 km/h");
            return false;
        }
        if (i > 0) {
            const auto& prev = config.points[i - 1];
            if (pt.measuredPhysicalSpeedKmh <= prev.measuredPhysicalSpeedKmh) {
                setValidationError(errBuf, errBufLen, "Measured speed points must be in strictly ascending order");
                return false;
            }
            if (pt.treadmillCommandKmh <= prev.treadmillCommandKmh) {
                setValidationError(errBuf, errBufLen, "Treadmill command points must be in strictly ascending order");
                return false;
            }
        }
    }

    if (config.maxAchievableSpeedVerified && config.commandMapValid && config.pointCount >= 2) {
        const float highestMeasured = config.points[config.pointCount - 1].measuredPhysicalSpeedKmh;
        if (config.maxAchievableSpeedKmh > highestMeasured) {
            setValidationError(errBuf, errBufLen, "Verified max speed cannot exceed highest measured point in table");
            return false;
        }
    }

    return true;
}

float SpeedCalibration::clamp(float v, float minVal, float maxVal) {
    if (v < minVal) return minVal;
    if (v > maxVal) return maxVal;
    return v;
}

const char* SpeedCalibration::version() {
    return "1.0.0";
}

} // namespace stridecontrol

