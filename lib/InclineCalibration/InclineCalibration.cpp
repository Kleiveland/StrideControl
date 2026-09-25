#include "InclineCalibration.h"
#include <cmath>
#include <algorithm>
#include <cstring>

namespace stridecontrol {

bool InclineCalibration::setConfiguration(const InclineConfig& config) {
    if (!validateCandidate(config)) {
        return false;
    }
    config_ = config;
    return true;
}

InclineCalibrationResult InclineCalibration::calculateCommand(float desiredActualInclinePct) const {
    // 1. Non-finite check
    if (!std::isfinite(desiredActualInclinePct)) {
        return {desiredActualInclinePct, 0.0f, InclineCalibrationResultStatus::InvalidInput};
    }

    // 2. Flat incline check (0%)
    if (desiredActualInclinePct == 0.0f) {
        return {desiredActualInclinePct, 0.0f, InclineCalibrationResultStatus::Flat};
    }

    // 3. Sub-minimum actual incline input (< 0%)
    if (desiredActualInclinePct < 0.0f) {
        return {desiredActualInclinePct, 0.0f, InclineCalibrationResultStatus::ClampedLow};
    }

    // 4. Uncalibrated Mode: Identity Fallback
    if (!config_.commandMapValid || config_.pointCount < 2) {
        if (desiredActualInclinePct > 15.0f) {
            return {desiredActualInclinePct, 15.0f, InclineCalibrationResultStatus::ClampedHigh};
        }
        return {desiredActualInclinePct, desiredActualInclinePct, InclineCalibrationResultStatus::IdentityFallback};
    }

    // 5. Calibrated Mode
    const size_t count = config_.pointCount;
    const float x0 = config_.points[0].measuredActualInclinePct;
    const float xLast = config_.points[count - 1].measuredActualInclinePct;

    // Case A: Below lowest calibrated point (desired < X_0) -> Identity clamped to [0.0, 15.0]
    if (desiredActualInclinePct < x0) {
        const float cmd = clamp(desiredActualInclinePct, 0.0f, 15.0f);
        return {desiredActualInclinePct, cmd, InclineCalibrationResultStatus::BelowCalibratedRange};
    }

    // Case B: Above highest calibrated point (desired > X_{N-1}) -> Held at highest command
    if (desiredActualInclinePct > xLast) {
        const float cmd = clamp(config_.points[count - 1].treadmillCommandPct, 0.0f, 15.0f);
        return {desiredActualInclinePct, cmd, InclineCalibrationResultStatus::AboveCalibratedRange};
    }

    // Case C: Piecewise linear interpolation within [X_0, X_{N-1}]
    for (size_t i = 0; i < count - 1; ++i) {
        const float segX0 = config_.points[i].measuredActualInclinePct;
        const float segX1 = config_.points[i + 1].measuredActualInclinePct;

        if (desiredActualInclinePct >= segX0 && desiredActualInclinePct <= segX1) {
            const float segY0 = config_.points[i].treadmillCommandPct;
            const float segY1 = config_.points[i + 1].treadmillCommandPct;

            const float y = segY0 + ((segY1 - segY0) / (segX1 - segX0)) * (desiredActualInclinePct - segX0);

            if (y > 15.0f) {
                return {desiredActualInclinePct, 15.0f, InclineCalibrationResultStatus::ClampedHigh};
            }
            if (y < 0.0f) {
                return {desiredActualInclinePct, 0.0f, InclineCalibrationResultStatus::ClampedLow};
            }
            return {desiredActualInclinePct, y, InclineCalibrationResultStatus::Calibrated};
        }
    }

    // Safety fallback (should never be reached with monotonic points)
    const float fallbackCmd = clamp(config_.points[count - 1].treadmillCommandPct, 0.0f, 15.0f);
    return {desiredActualInclinePct, fallbackCmd, InclineCalibrationResultStatus::AboveCalibratedRange};
}

float InclineCalibration::calculateCommandInclinePct(float desiredActualInclinePct) const {
    return calculateCommand(desiredActualInclinePct).treadmillCommandPct;
}

float InclineCalibration::getMaxAchievableInclinePct() const {
    return config_.maxAchievableInclinePct;
}

bool InclineCalibration::isCommandMapValid() const {
    return config_.commandMapValid;
}

bool InclineCalibration::isMaxAchievableInclineVerified() const {
    return config_.maxAchievableInclineVerified;
}

uint8_t InclineCalibration::getPointCount() const {
    return config_.pointCount;
}

bool InclineCalibration::validateCandidate(const InclineConfig& config, char* errBuf, size_t errBufLen) {
    auto setErr = [&](const char* msg) {
        if (errBuf && errBufLen > 0) {
            strncpy(errBuf, msg, errBufLen - 1);
            errBuf[errBufLen - 1] = '\0';
        }
    };

    if (config.pointCount > kMaxInclineCalibrationPoints) {
        setErr("Point count exceeds kMaxInclineCalibrationPoints (10)");
        return false;
    }
    if (!std::isfinite(config.maxAchievableInclinePct) ||
        config.maxAchievableInclinePct <= 0.0f ||
        config.maxAchievableInclinePct > 15.0f) {
        setErr("maxAchievableInclinePct must be finite and in (0.0, 15.0]");
        return false;
    }
    if (config.commandMapValid && config.pointCount < 2) {
        setErr("commandMapValid requires at least 2 points");
        return false;
    }
    if (config.maxAchievableInclineVerified && !config.commandMapValid) {
        setErr("maxAchievableInclineVerified requires valid command map");
        return false;
    }

    for (size_t i = 0; i < config.pointCount; ++i) {
        const auto& pt = config.points[i];
        if (!std::isfinite(pt.measuredActualInclinePct) || !std::isfinite(pt.treadmillCommandPct)) {
            setErr("Calibration point values must be finite numbers");
            return false;
        }
        if (pt.measuredActualInclinePct < 0.0f || pt.measuredActualInclinePct > 15.0f) {
            setErr("measuredActualInclinePct out of range [0.0, 15.0]");
            return false;
        }
        if (pt.treadmillCommandPct < 0.0f || pt.treadmillCommandPct > 15.0f) {
            setErr("treadmillCommandPct out of range [0.0, 15.0]");
            return false;
        }
        if (i > 0) {
            const auto& prev = config.points[i - 1];
            if (pt.measuredActualInclinePct <= prev.measuredActualInclinePct) {
                setErr("Points must be strictly monotonically increasing in measured incline");
                return false;
            }
            if (pt.treadmillCommandPct <= prev.treadmillCommandPct) {
                setErr("Points must be strictly monotonically increasing in command incline");
                return false;
            }
        }
    }

    if (config.maxAchievableInclineVerified && config.commandMapValid && config.pointCount >= 2) {
        const float highestMeasured = config.points[config.pointCount - 1].measuredActualInclinePct;
        if (config.maxAchievableInclinePct > highestMeasured) {
            setErr("Verified maxAchievableInclinePct exceeds highest calibrated measured incline");
            return false;
        }
    }

    return true;
}

float InclineCalibration::clamp(float v, float minVal, float maxVal) {
    if (v < minVal) return minVal;
    if (v > maxVal) return maxVal;
    return v;
}

const char* InclineCalibration::version() {
    return "1.0.0";
}

} // namespace stridecontrol

