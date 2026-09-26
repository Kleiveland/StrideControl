#pragma once

#include <cstdint>
#include "../CsafeInterface/CsafeTypes.h"

namespace stridecontrol {

enum class DeadTimePhase : uint8_t {
    Idle = 0,
    Armed,               // Armed, waiting for CSAFE to leave Ready/Idle and enter Starting (or InUse)
    MeasuringStarting,   // CSAFE is Starting (countdown underway), waiting for movement or InUse
    MeasuringInUse,      // CSAFE is InUse (countdown finished), waiting for belt movement
    Complete,            // Belt movement detected, measurement valid
    TimedOut,            // Exceeded armed timeout (60s) or measuring timeout (15s)
    Aborted              // Canceled by user, E-Stop, or unexpected CSAFE state change
};

const char* deadTimePhaseToString(DeadTimePhase phase);

struct DeadTimeTrackerConfig {
    float movementThresholdKmh = 0.05f;
    uint32_t armTimeoutMs = 60000;          // 60s to press QuickStart
    uint32_t measurementTimeoutMs = 15000;  // 15s max from QuickStart to movement
};

class DeadTimeTracker {
public:
    explicit DeadTimeTracker(const DeadTimeTrackerConfig& config = DeadTimeTrackerConfig());

    bool arm(uint32_t nowMs, float currentSpeedKmh, CsafeMachineState csafe);
    void update(float speedKmh, bool speedValid, CsafeMachineState csafe, bool eStop, uint32_t nowMs);
    void abort();
    void reset();

    DeadTimePhase phase() const { return phase_; }
    bool active() const {
        return phase_ == DeadTimePhase::Armed ||
               phase_ == DeadTimePhase::MeasuringStarting ||
               phase_ == DeadTimePhase::MeasuringInUse;
    }
    bool complete() const { return phase_ == DeadTimePhase::Complete; }
    bool timedOut() const { return phase_ == DeadTimePhase::TimedOut; }
    bool aborted() const { return phase_ == DeadTimePhase::Aborted; }

    uint32_t measuredDeadTimeMs() const { return measuredDeadTimeMs_; }
    uint32_t countdownDurationMs() const { return countdownDurationMs_; }
    uint32_t motorDeadTimeMs() const { return motorDeadTimeMs_; }

private:
    DeadTimeTrackerConfig config_;
    DeadTimePhase phase_ = DeadTimePhase::Idle;

    uint32_t armStartMs_ = 0;
    uint32_t quickStartTsMs_ = 0;
    uint32_t inUseTsMs_ = 0;
    uint32_t movementTsMs_ = 0;

    uint32_t measuredDeadTimeMs_ = 0;
    uint32_t countdownDurationMs_ = 0;
    uint32_t motorDeadTimeMs_ = 0;
};

} // namespace stridecontrol

