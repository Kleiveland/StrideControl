#pragma once

#include <cstdint>
#include <cmath>

namespace stridecontrol {

enum class RampTestPhase : uint8_t {
    Idle = 0,
    WaitingForStableStart,
    MeasuringDeadTime,
    MeasuringRamp,
    Complete,
    TimedOut,
    Aborted
};

struct RampTestConfig {
    float startToleranceKmh = 0.25f;
    uint32_t stableDurationMs = 1000;
    uint32_t stabilityTimeoutMs = 20000;
    float movementThresholdKmh = 0.10f;
    uint32_t movementConfirmMs = 200;
    float arrivalToleranceKmh = 0.30f;
    uint32_t arrivalConfirmMs = 300;
    uint32_t measurementTimeoutMs = 30000;
};

class RampTestTracker {
public:
    explicit RampTestTracker(const RampTestConfig& config = RampTestConfig());

    bool beginTest(
        float startSpeedKmh,
        float targetSpeedKmh,
        uint32_t nowMs
    );

    void update(
        float measuredSpeedKmh,
        bool speedValid,
        uint32_t nowMs
    );

    void abort(uint32_t nowMs);
    void reset();

    RampTestPhase phase() const;
    bool targetDispatchRequested() const;
    void acknowledgeTargetDispatched();

    bool active() const;
    bool complete() const;
    bool timedOut() const;
    bool aborted() const;

    float startSpeedKmh() const;
    float targetSpeedKmh() const;
    uint32_t stableDurationMs(uint32_t nowMs) const;
    uint32_t deadTimeMs() const;
    uint32_t totalTimeMs() const;

private:
    RampTestConfig config_;
    RampTestPhase phase_ = RampTestPhase::Idle;

    float startSpeedKmh_ = 0.0f;
    float targetSpeedKmh_ = 0.0f;

    bool targetDispatchRequested_ = false;

    uint32_t testBeginMs_ = 0;
    uint32_t stableSinceMs_ = 0;
    bool stableActive_ = false;

    uint32_t measurementStartMs_ = 0;

    uint32_t movementSinceMs_ = 0;
    bool movementActive_ = false;

    uint32_t arrivalSinceMs_ = 0;
    bool arrivalActive_ = false;

    uint32_t deadTimeMs_ = 0;
    uint32_t totalTimeMs_ = 0;
};

} // namespace stridecontrol

