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
 *
 * NOT to be confused with two other, unrelated calibration concepts in this codebase:
 * - Speed Sensor Calibration (SpeedSensorConfig::kmhPerHz) - converts raw tachometer
 *   frequency into a physical speed reading. Has nothing to do with console commands.
 * - Ramp Timing Calibration (RampCalibrationConfig) - measures how long the belt takes
 *   to physically reach a commanded speed. Has nothing to do with command accuracy.
 * This class solely corrects for the console's own command-to-actual-speed inaccuracy.
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

    // Configuration & validation
    const SpeedConfig& getConfiguration() const { return config_; }
    static bool validateCandidate(const SpeedConfig& config, char* errBuf = nullptr, size_t errBufLen = 0);

    // Metadata & queries
    float getMaxAchievableSpeedKmh() const;
    bool isCommandMapValid() const;
    bool isMaxAchievableSpeedVerified() const;
    uint8_t getPointCount() const;

    static const char* version();

private:
    static float clamp(float v, float minVal, float maxVal);

    SpeedConfig config_{};
};

} // namespace stridecontrol

