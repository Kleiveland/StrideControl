#pragma once

#include <cstdint>

namespace stridecontrol {

/**
 * @brief Narrow target-sink interface for workout command dispatching.
 *
 * Defines the contract required by WorkoutDispatcher to submit physical speed
 * and incline targets to an underlying treadmill controller or test simulator.
 *
 * @note Physical stop authority is deliberately excluded; WorkoutCommandIntent
 *       has no stop authority.
 */
class IWorkoutTargetSink {
public:
    virtual ~IWorkoutTargetSink() = default;

    virtual bool submitSpeedTarget(
        float targetSpeedKmh,
        uint32_t nowMs) = 0;

    virtual bool submitInclineTarget(
        float targetInclinePct,
        uint32_t nowMs) = 0;

    virtual bool isBusy() const = 0;
    virtual bool isReady() const = 0;
};

} // namespace stridecontrol

