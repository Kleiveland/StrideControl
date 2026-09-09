#pragma once

#include <cstdint>
#include "../SpeedSensor/SpeedSensor.h"
#include "VirtualTreadmill.h"

namespace stridecontrol {

/**
 * @brief Zero-overhead adapter forwarding VirtualTreadmill tacho observations
 *        directly into SpeedSensor in SoftwareObservation mode.
 */
class VirtualSpeedSensorAdapter {
public:
    explicit VirtualSpeedSensorAdapter(SpeedSensor& sensor)
        : sensor_(sensor) {}

    /**
     * @brief Process tacho output from VirtualTreadmill and forward edge timestamps.
     * @param tacho TachoOutput from current simulation tick
     * @return Number of edges forwarded
     */
    uint8_t processTacho(const TachoOutput& tacho) {
        uint8_t forwarded = 0;
        for (uint8_t i = 0; i < tacho.edgeCount && i < kMaxTachoEdgesPerTick; ++i) {
            lastObservedEdgeUs_ = tacho.edgeTimestampUs[i];
            if (sensor_.observeEdge(tacho.edgeTimestampUs[i])) {
                forwarded++;
            }
        }
        return forwarded;
    }

    /**
     * @brief Inject a glitch edge relative to the last observed edge timestamp.
     * @param relativeOffsetUs Default 150us (below typical 300us glitch rejection threshold)
     * @return True if observeEdge was invoked
     */
    bool injectGlitchEdge(uint32_t relativeOffsetUs = 150) {
        return sensor_.observeEdge(lastObservedEdgeUs_ + relativeOffsetUs);
    }

    /**
     * @brief Inject a pulse lockout edge relative to the last observed edge timestamp.
     * @param relativeOffsetUs Default 1000us (above glitch threshold but below 2000us lockout)
     * @return True if observeEdge was invoked
     */
    bool injectLockoutEdge(uint32_t relativeOffsetUs = 1000) {
        return sensor_.observeEdge(lastObservedEdgeUs_ + relativeOffsetUs);
    }

    /**
     * @brief Explicitly inject an edge at an absolute timestamp.
     */
    bool injectEdge(uint32_t timestampUs) {
        lastObservedEdgeUs_ = timestampUs;
        return sensor_.observeEdge(timestampUs);
    }

    uint32_t getLastObservedEdgeUs() const { return lastObservedEdgeUs_; }

private:
    SpeedSensor& sensor_;
    uint32_t lastObservedEdgeUs_ = 0;
};

} // namespace stridecontrol

