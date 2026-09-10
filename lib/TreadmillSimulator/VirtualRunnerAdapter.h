#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include "../ImuInterface/ImuInterface.h"
#include "../ImuInterface/ImuTypes.h"
#include "VirtualTreadmill.h"

namespace stridecontrol {

enum class VirtualRunnerMode : uint8_t {
    RunningOnBelt = 0,
    OnSideRails   = 1,
    NotPresent    = 2
};

struct VirtualRunnerConfig {
    VirtualRunnerMode mode = VirtualRunnerMode::RunningOnBelt;
    uint16_t cadenceSpm = 0;
    float impactMagnitudeG = 0.0f;
    uint32_t impactDurationUs = 30000;
    bool imuSignalValid = true;
};

/**
 * @brief Pragmatic, deterministic runner dynamics adapter translating high-level
 *        runner presence and cadence into synthetic IMU sample streams.
 */
class VirtualRunnerAdapter {
public:
    explicit VirtualRunnerAdapter(ImuInterface& imu)
        : imu_(imu) {}

    void setMode(VirtualRunnerMode mode) { config_.mode = mode; }
    void setCadenceSpm(uint16_t spm) {
        if (spm == 0) {
            config_.cadenceSpm = 0;
        } else {
            config_.cadenceSpm = std::max<uint16_t>(40, std::min<uint16_t>(240, spm));
        }
    }
    void setSignalValid(bool valid) { config_.imuSignalValid = valid; }
    void setImpactMagnitudeG(float g) { config_.impactMagnitudeG = g; }

    VirtualRunnerMode getMode() const { return config_.mode; }
    uint16_t getCadenceSpm() const { return config_.cadenceSpm; }
    bool isSignalValid() const { return config_.imuSignalValid; }
    const VirtualRunnerConfig& getConfig() const { return config_; }

    /**
     * @brief Synthesizes deterministic IMU samples for the current simulation tick.
     * @param tick Current simulation tick context
     * @param simulatedDeckAngleDeg Deck angle feedback for ImuState
     * @return Number of samples generated and observed
     */
    size_t processTick(const SimulationTick& tick, float simulatedDeckAngleDeg = 0.0f) {
        if (config_.mode == VirtualRunnerMode::NotPresent) {
            // Emits zero samples; triggers imuDataMaxAgeMs timeout in RunnerDynamics
            return 0;
        }

        static constexpr size_t kSamplesPerTick = 4;
        static constexpr uint32_t kSampleIntervalUs = 5000; // 200 Hz equivalent (matches LSM6DSOX 208Hz ODR)
        ImuSample batch[kSamplesPerTick];

        const uint32_t stepPeriodUs = (config_.cadenceSpm > 0) ? (60000000UL / config_.cadenceSpm) : 500000UL;

        for (size_t i = 0; i < kSamplesPerTick; ++i) {
            uint64_t sampleTimeUs = tick.scenarioTimeUs + (i * kSampleIntervalUs);
            batch[i].timestampUs = static_cast<uint32_t>(sampleTimeUs);
            batch[i].sequence = monotonicSequence_++;
            batch[i].accelValid = config_.imuSignalValid;
            batch[i].gyroValid = config_.imuSignalValid;
            batch[i].timestampEstimated = false;

            float verticalG = 1.0f; // Gravity baseline

            if (config_.mode == VirtualRunnerMode::RunningOnBelt && config_.imuSignalValid &&
                config_.cadenceSpm > 0 && config_.impactMagnitudeG > 0.0f) {
                uint32_t cycleUs = static_cast<uint32_t>(sampleTimeUs % stepPeriodUs);
                if (cycleUs < config_.impactDurationUs) {
                    // Symmetric triangular footstrike impact
                    float norm = static_cast<float>(cycleUs) / static_cast<float>(config_.impactDurationUs);
                    float triangle = 1.0f - std::abs(2.0f * norm - 1.0f);
                    verticalG += (config_.impactMagnitudeG * triangle);
                }
            }

            batch[i].accelVerticalG = verticalG;
            batch[i].accelLongitudinalG = 0.0f;
            batch[i].accelLateralG = 0.0f;
            batch[i].gyroLongitudinalDps = 0.0f;
            batch[i].gyroLateralDps = 0.0f;
            batch[i].gyroVerticalDps = 0.0f;
        }

        ImuState simulatedState{};
        simulatedState.initialized = true;
        simulatedState.connected = config_.imuSignalValid;
        simulatedState.dataValid = config_.imuSignalValid;
        simulatedState.status = config_.imuSignalValid ? ImuStatus::Ready : ImuStatus::InvalidData;
        simulatedState.rawDeckAngleDeg = simulatedDeckAngleDeg;
        simulatedState.filteredDeckAngleDeg = simulatedDeckAngleDeg;
        simulatedState.deckAngleValid = config_.imuSignalValid;
        simulatedState.stability = ImuStability::Stable;
        simulatedState.sampleTimestampUs = static_cast<uint32_t>(tick.scenarioTimeUs + tick.deltaUs);

        imu_.observeSamples(batch, kSamplesPerTick, &simulatedState);
        return kSamplesPerTick;
    }

private:
    ImuInterface& imu_;
    VirtualRunnerConfig config_{};
    uint32_t monotonicSequence_ = 1;
};

} // namespace stridecontrol
