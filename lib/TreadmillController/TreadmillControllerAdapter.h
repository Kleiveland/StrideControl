#pragma once

#include <cstdint>
#include "../WorkoutDispatcher/IWorkoutTargetSink.h"
#include "TreadmillController.h"

namespace stridecontrol {

/**
 * @brief Zero-allocation adapter wrapping TreadmillController as an IWorkoutTargetSink.
 *
 * Exposes target submission and status polling to WorkoutDispatcher without
 * exposing physical stop authority or controller-internal lifecycle methods.
 */
class TreadmillControllerAdapter : public IWorkoutTargetSink {
public:
    explicit TreadmillControllerAdapter(TreadmillController& controller);
    virtual ~TreadmillControllerAdapter() = default;

    TreadmillControllerAdapter(const TreadmillControllerAdapter&) = delete;
    TreadmillControllerAdapter& operator=(const TreadmillControllerAdapter&) = delete;

    bool submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) override;
    bool submitInclineTarget(float targetInclinePct, uint32_t nowMs) override;

    bool isBusy() const override;
    bool isReady() const override;

    static const char* version();

private:
    TreadmillController& controller_;
};

} // namespace stridecontrol

