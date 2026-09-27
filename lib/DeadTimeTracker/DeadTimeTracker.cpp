#include "DeadTimeTracker.h"
#include <cmath>

namespace stridecontrol {

const char* deadTimePhaseToString(DeadTimePhase phase) {
    switch (phase) {
        case DeadTimePhase::Idle:              return "Idle";
        case DeadTimePhase::Armed:             return "Armed";
        case DeadTimePhase::MeasuringStarting: return "MeasuringStarting";
        case DeadTimePhase::MeasuringInUse:    return "MeasuringInUse";
        case DeadTimePhase::Complete:          return "Complete";
        case DeadTimePhase::TimedOut:          return "TimedOut";
        case DeadTimePhase::Aborted:           return "Aborted";
        default:                               return "Unknown";
    }
}

DeadTimeTracker::DeadTimeTracker(const DeadTimeTrackerConfig& config)
    : config_(config) {
    reset();
}

bool DeadTimeTracker::arm(uint32_t nowMs, float currentSpeedKmh, CsafeMachineState csafe) {
    if (std::isfinite(currentSpeedKmh) && currentSpeedKmh > config_.movementThresholdKmh) {
        return false;
    }
    if (csafe == CsafeMachineState::InUse || csafe == CsafeMachineState::Starting) {
        return false;
    }

    reset();
    phase_ = DeadTimePhase::Armed;
    armStartMs_ = nowMs;
    return true;
}

void DeadTimeTracker::update(float speedKmh, bool speedValid, CsafeMachineState csafe, bool eStop, uint32_t nowMs) {
    if (phase_ == DeadTimePhase::Idle ||
        phase_ == DeadTimePhase::Complete ||
        phase_ == DeadTimePhase::TimedOut ||
        phase_ == DeadTimePhase::Aborted) {
        return;
    }

    if (eStop) {
        abort();
        return;
    }

    const bool moving = speedValid && std::isfinite(speedKmh) && (speedKmh > config_.movementThresholdKmh);

    switch (phase_) {
        case DeadTimePhase::Armed: {
            if (nowMs - armStartMs_ >= config_.armTimeoutMs) {
                phase_ = DeadTimePhase::TimedOut;
                return;
            }

            if (csafe == CsafeMachineState::Starting) {
                quickStartTsMs_ = nowMs;
                phase_ = DeadTimePhase::MeasuringStarting;
            } else if (csafe == CsafeMachineState::InUse) {
                // Treadmill jumped directly to InUse (e.g. no audible countdown)
                quickStartTsMs_ = nowMs;
                inUseTsMs_ = nowMs;
                countdownDurationMs_ = 0;
                phase_ = DeadTimePhase::MeasuringInUse;
            } else if (csafe == CsafeMachineState::Error || csafe == CsafeMachineState::Offline) {
                abort();
            }
            break;
        }

        case DeadTimePhase::MeasuringStarting: {
            if (nowMs - quickStartTsMs_ >= config_.startingTimeoutMs) {
                phase_ = DeadTimePhase::TimedOut;
                return;
            }

            if (moving) {
                // Belt began moving even during Starting phase
                movementTsMs_ = nowMs;
                measuredDeadTimeMs_ = movementTsMs_ - quickStartTsMs_;
                countdownDurationMs_ = (inUseTsMs_ > quickStartTsMs_) ? (inUseTsMs_ - quickStartTsMs_) : 0;
                motorDeadTimeMs_ = measuredDeadTimeMs_;
                phase_ = DeadTimePhase::Complete;
                return;
            }

            if (csafe == CsafeMachineState::InUse) {
                inUseTsMs_ = nowMs;
                countdownDurationMs_ = inUseTsMs_ - quickStartTsMs_;
                phase_ = DeadTimePhase::MeasuringInUse;
            } else if (csafe != CsafeMachineState::Starting) {
                // Console stopped or canceled countdown
                abort();
            }
            break;
        }

        case DeadTimePhase::MeasuringInUse: {
            if (nowMs - inUseTsMs_ >= config_.inUseTimeoutMs) {
                phase_ = DeadTimePhase::TimedOut;
                return;
            }

            if (csafe != CsafeMachineState::InUse) {
                // Console stopped or faulted before belt moved
                abort();
                return;
            }

            if (moving) {
                movementTsMs_ = nowMs;
                measuredDeadTimeMs_ = movementTsMs_ - quickStartTsMs_;
                if (inUseTsMs_ >= quickStartTsMs_ && movementTsMs_ >= inUseTsMs_) {
                    motorDeadTimeMs_ = movementTsMs_ - inUseTsMs_;
                } else {
                    motorDeadTimeMs_ = measuredDeadTimeMs_;
                }
                phase_ = DeadTimePhase::Complete;
            }
            break;
        }

        default:
            break;
    }
}

void DeadTimeTracker::abort() {
    phase_ = DeadTimePhase::Aborted;
}

void DeadTimeTracker::reset() {
    phase_ = DeadTimePhase::Idle;
    armStartMs_ = 0;
    quickStartTsMs_ = 0;
    inUseTsMs_ = 0;
    movementTsMs_ = 0;
    measuredDeadTimeMs_ = 0;
    countdownDurationMs_ = 0;
    motorDeadTimeMs_ = 0;
}

} // namespace stridecontrol

