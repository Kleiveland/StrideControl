#pragma once

#include <cstdint>
#include <cstddef>
#include "InclineCalibrationTypes.h"

namespace stridecontrol {

/**
 * @brief Pure-logic command-incline compensation and piecewise linear interpolation engine.
 *
 * Implements deterministic mapping from desired actual incline % to corrected
 * treadmill console commands based on stored empirical calibration points.
 */
class InclineCalibration {
public:
    InclineCalibration() = default;

    // Ingests InclineConfig. Validates candidate against rules.
    // On valid: updates active config_ and returns true.
    // On invalid: leaves active config_ byte-for-byte unchanged and returns false.
    bool setConfiguration(const InclineConfig& config);

    // Calculates the command result for a desired actual incline (%).
    InclineCalibrationResult calculateCommand(float desiredActualInclinePct) const;

    // Direct float convenience wrapper (returns calculateCommand().treadmillCommandPct)
    float calculateCommandInclinePct(float desiredActualInclinePct) const;

    // Metadata & queries
    float getMaxAchievableInclinePct() const;
    bool isCommandMapValid() const;
    bool isMaxAchievableInclineVerified() const;
    uint8_t getPointCount() const;

    // Configuration & validation
    const InclineConfig& getConfiguration() const { return config_; }
    static bool validateCandidate(const InclineConfig& config, char* errBuf = nullptr, size_t errBufLen = 0);

    static const char* version();

private:
    static float clamp(float v, float minVal, float maxVal);

    InclineConfig config_{};
};

} // namespace stridecontrol

