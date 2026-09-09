#pragma once

#include <cstdint>
#include "../InclineSensor/InclineSensor.h"
#include "VirtualTreadmill.h"

namespace stridecontrol {

/**
 * @brief Zero-overhead adapter forwarding VirtualTreadmill incline feedback
 *        directly into InclineSensor in SoftwareObservation mode.
 */
class VirtualInclineAdapter {
public:
    explicit VirtualInclineAdapter(InclineSensor& sensor)
        : sensor_(sensor) {}

    /**
     * @brief Process incline feedback from VirtualTreadmill and forward to InclineSensor.
     * @param feedback InclineFeedbackOutput from current simulation tick
     * @param nowMs Current simulation time in milliseconds
     * @return True if observation was successfully ingested
     */
    bool processFeedback(const InclineFeedbackOutput& feedback, uint32_t nowMs) {
        InclinePulseObservation obs{};
        obs.pulseCount = static_cast<uint32_t>(feedback.pulsesThisTick);
        obs.direction = feedback.direction;
        obs.signalValid = feedback.signalValid;
        lastObservedMs_ = nowMs;
        return sensor_.observePulses(obs, nowMs);
    }

    /**
     * @brief Directly inject custom pulse observation.
     */
    bool injectObservation(const InclinePulseObservation& obs, uint32_t nowMs) {
        lastObservedMs_ = nowMs;
        return sensor_.observePulses(obs, nowMs);
    }

    uint32_t getLastObservedMs() const { return lastObservedMs_; }

private:
    InclineSensor& sensor_;
    uint32_t lastObservedMs_ = 0;
};

} // namespace stridecontrol

