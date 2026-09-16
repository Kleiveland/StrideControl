#include "RampTestTracker.h"

namespace stridecontrol {

RampTestTracker::RampTestTracker(const RampTestConfig& config)
    : config_(config) {
    reset();
}

bool RampTestTracker::beginTest(float startSpeedKmh, float targetSpeedKmh, uint32_t nowMs) {
    if (!std::isfinite(startSpeedKmh) || !std::isfinite(targetSpeedKmh)) {
        return false;
    }
    if (std::abs(startSpeedKmh - targetSpeedKmh) < 0.001f) {
        return false;
    }
    if (active()) {
        return false;
    }

    startSpeedKmh_ = startSpeedKmh;
    targetSpeedKmh_ = targetSpeedKmh;
    testBeginMs_ = nowMs;

    phase_ = RampTestPhase::WaitingForStableStart;
    targetDispatchRequested_ = false;

    stableSinceMs_ = 0;
    stableActive_ = false;

    measurementStartMs_ = 0;

    movementSinceMs_ = 0;
    movementActive_ = false;

    arrivalSinceMs_ = 0;
    arrivalActive_ = false;

    deadTimeMs_ = 0;
    totalTimeMs_ = 0;

    return true;
}

void RampTestTracker::update(float measuredSpeedKmh, bool speedValid, uint32_t nowMs) {
    switch (phase_) {
        case RampTestPhase::WaitingForStableStart: {
            if (nowMs - testBeginMs_ >= config_.stabilityTimeoutMs) {
                phase_ = RampTestPhase::TimedOut;
                targetDispatchRequested_ = false;
                stableActive_ = false;
                return;
            }

            const bool inStartBand = speedValid &&
                std::isfinite(measuredSpeedKmh) &&
                (std::abs(measuredSpeedKmh - startSpeedKmh_) <= config_.startToleranceKmh);

            if (inStartBand) {
                if (!stableActive_) {
                    stableActive_ = true;
                    stableSinceMs_ = nowMs;
                }

                if (nowMs - stableSinceMs_ >= config_.stableDurationMs) {
                    phase_ = RampTestPhase::MeasuringDeadTime;
                    measurementStartMs_ = nowMs;
                    targetDispatchRequested_ = true;
                    movementActive_ = false;
                    movementSinceMs_ = 0;
                }
            } else {
                stableActive_ = false;
                stableSinceMs_ = 0;
            }
            break;
        }

        case RampTestPhase::MeasuringDeadTime: {
            const uint32_t elapsedMs = nowMs - measurementStartMs_;
            if (elapsedMs >= config_.measurementTimeoutMs) {
                phase_ = RampTestPhase::TimedOut;
                targetDispatchRequested_ = false;
                return;
            }

            bool correctDirection = false;
            if (speedValid && std::isfinite(measuredSpeedKmh)) {
                const float signedDelta = measuredSpeedKmh - startSpeedKmh_;
                if (targetSpeedKmh_ > startSpeedKmh_) {
                    correctDirection = (signedDelta >= config_.movementThresholdKmh);
                } else {
                    correctDirection = (signedDelta <= -config_.movementThresholdKmh);
                }
            }

            if (correctDirection) {
                if (!movementActive_) {
                    movementActive_ = true;
                    movementSinceMs_ = nowMs;
                }

                if (nowMs - movementSinceMs_ >= config_.movementConfirmMs) {
                    deadTimeMs_ = movementSinceMs_ - measurementStartMs_;
                    phase_ = RampTestPhase::MeasuringRamp;
                    arrivalActive_ = false;
                    arrivalSinceMs_ = 0;
                }
            } else {
                movementActive_ = false;
                movementSinceMs_ = 0;
            }
            break;
        }

        case RampTestPhase::MeasuringRamp: {
            const uint32_t elapsedMs = nowMs - measurementStartMs_;
            if (elapsedMs >= config_.measurementTimeoutMs) {
                phase_ = RampTestPhase::TimedOut;
                targetDispatchRequested_ = false;
                return;
            }

            const bool inArrivalBand = speedValid &&
                std::isfinite(measuredSpeedKmh) &&
                (std::abs(measuredSpeedKmh - targetSpeedKmh_) <= config_.arrivalToleranceKmh);

            if (inArrivalBand) {
                if (!arrivalActive_) {
                    arrivalActive_ = true;
                    arrivalSinceMs_ = nowMs;
                }

                if (nowMs - arrivalSinceMs_ >= config_.arrivalConfirmMs) {
                    totalTimeMs_ = arrivalSinceMs_ - measurementStartMs_;
                    phase_ = RampTestPhase::Complete;
                }
            } else {
                arrivalActive_ = false;
                arrivalSinceMs_ = 0;
            }
            break;
        }

        case RampTestPhase::Idle:
        case RampTestPhase::Complete:
        case RampTestPhase::TimedOut:
        case RampTestPhase::Aborted:
        default:
            break;
    }
}

void RampTestTracker::abort(uint32_t nowMs) {
    (void)nowMs;
    phase_ = RampTestPhase::Aborted;
    targetDispatchRequested_ = false;
    stableActive_ = false;
    movementActive_ = false;
    arrivalActive_ = false;
}

void RampTestTracker::reset() {
    phase_ = RampTestPhase::Idle;
    startSpeedKmh_ = 0.0f;
    targetSpeedKmh_ = 0.0f;
    targetDispatchRequested_ = false;

    testBeginMs_ = 0;
    stableSinceMs_ = 0;
    stableActive_ = false;

    measurementStartMs_ = 0;

    movementSinceMs_ = 0;
    movementActive_ = false;

    arrivalSinceMs_ = 0;
    arrivalActive_ = false;

    deadTimeMs_ = 0;
    totalTimeMs_ = 0;
}

RampTestPhase RampTestTracker::phase() const {
    return phase_;
}

bool RampTestTracker::targetDispatchRequested() const {
    return targetDispatchRequested_;
}

void RampTestTracker::acknowledgeTargetDispatched() {
    targetDispatchRequested_ = false;
}

bool RampTestTracker::active() const {
    return phase_ == RampTestPhase::WaitingForStableStart ||
           phase_ == RampTestPhase::MeasuringDeadTime ||
           phase_ == RampTestPhase::MeasuringRamp;
}

bool RampTestTracker::complete() const {
    return phase_ == RampTestPhase::Complete;
}

bool RampTestTracker::timedOut() const {
    return phase_ == RampTestPhase::TimedOut;
}

bool RampTestTracker::aborted() const {
    return phase_ == RampTestPhase::Aborted;
}

float RampTestTracker::startSpeedKmh() const {
    return startSpeedKmh_;
}

float RampTestTracker::targetSpeedKmh() const {
    return targetSpeedKmh_;
}

uint32_t RampTestTracker::stableDurationMs(uint32_t nowMs) const {
    if (!stableActive_ || nowMs < stableSinceMs_) {
        return 0;
    }
    return nowMs - stableSinceMs_;
}

uint32_t RampTestTracker::deadTimeMs() const {
    return deadTimeMs_;
}

uint32_t RampTestTracker::totalTimeMs() const {
    return totalTimeMs_;
}

} // namespace stridecontrol

