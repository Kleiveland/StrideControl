#pragma once

#include <cstdint>
#include <cstddef>
#include "../SettingsService/SettingsServiceTypes.h"
#include "SpeedCalibrationTypes.h"

namespace stridecontrol {

/**
 * @brief Pure-logic command-speed compensation and piecewise linear interpolation engine.
 *
 * Implements deterministic mapping from desired physical belt speed to corrected
 * treadmill console commands based on stored empirical calibration points.
 */
class SpeedCalibration {
public:
    SpeedCalibration() = default;

    // Ingests SpeedConfig. Validates candidate against SettingsService rules.
    // On valid: updates active config_ and returns true.
    // On invalid: leaves active config_ byte-for-byte unchanged and returns false.
    bool setConfiguration(const SpeedConfig& config);

    // Calculates the command result for a desired physical speed (km/h).
    SpeedCalibrationResult calculateCommand(float desiredPhysicalSpeedKmh) const;

    // Direct float convenience wrapper (returns calculateCommand().treadmillCommandKmh)
    float calculateCommandSpeedKmh(float desiredPhysicalSpeedKmh) const;

    // Metadata & queries
    float getMaxAchievableSpeedKmh() const;
    bool isCommandMapValid() const;
    bool isMaxAchievableSpeedVerified() const;
    uint8_t getPointCount() const;

    static const char* version();

private:
    bool validateCandidate(const SpeedConfig& config) const;
    static float clamp(float v, float minVal, float maxVal);

    SpeedConfig config_{};
};

} // namespace stridecontrol

