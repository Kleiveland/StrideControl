#pragma once

#include <cstdint>
#include "../WorkoutDispatcher/IWorkoutTargetSink.h"
#include "../ApplicationSnapshot/ApplicationSnapshot.h"

namespace stridecontrol {

/**
 * @brief Deterministic, non-allocating simulated treadmill target sink.
 *
 * Implements IWorkoutTargetSink for testing and commissioning. Simulates 500 ms
 * busy periods upon command acceptance, ramps speed at 0.5 km/h per second without
 * overshoot, integrates trapezoidal distance, and provides an authoritative
 * ApplicationSnapshot.
 */
class TreadmillSimulator : public IWorkoutTargetSink {
public:
    TreadmillSimulator() = default;
    virtual ~TreadmillSimulator() = default;

    TreadmillSimulator(const TreadmillSimulator&) = delete;
    TreadmillSimulator& operator=(const TreadmillSimulator&) = delete;

    void begin(uint32_t nowMs);
    void update(uint32_t nowMs);

    bool submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) override;
    bool submitInclineTarget(float targetInclinePct, uint32_t nowMs) override;
    bool submitStop(uint32_t nowMs);

    bool isBusy() const override;
    bool isReady() const override;

    ApplicationSnapshot getSnapshot() const;

    float getCurrentSpeedKmh() const { return currentSpeedKmh_; }
    float getTargetSpeedKmh() const { return targetSpeedKmh_; }
    float getCurrentInclinePct() const { return currentInclinePct_; }
    float getTargetInclinePct() const { return targetInclinePct_; }
    double getCumulativeDistanceKm() const { return cumulativeDistanceKm_; }

    static const char* version();

private:
    static constexpr uint32_t kBusyDurationMs = 500;
    static constexpr float kSpeedRampRatePerSec = 0.5f;

    bool ready_ = false;
    bool busy_ = false;
    uint32_t busyStartMs_ = 0;
    uint32_t lastUpdateMs_ = 0;
    uint32_t snapshotSequence_ = 0;

    float currentSpeedKmh_ = 0.0f;
    float targetSpeedKmh_ = 0.0f;
    float currentInclinePct_ = 0.0f;
    float targetInclinePct_ = 0.0f;
    double cumulativeDistanceKm_ = 0.0;
};

} // namespace stridecontrol

