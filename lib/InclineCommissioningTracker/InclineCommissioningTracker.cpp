#include "InclineCommissioningTracker.h"
#include <algorithm>

namespace stridecontrol {

InclineCommissioningTracker::InclineCommissioningTracker(const InclineCommissioningConfig& config)
    : config_(config) {
    reset();
}

void InclineCommissioningTracker::beginHoming(uint32_t nowMs) {
    reset();
    homingStartMs_ = nowMs;
    phase_ = InclineCommissioningPhase::Homing;
}

void InclineCommissioningTracker::onHomedConfirmed(uint32_t nowMs) {
    if (phase_ == InclineCommissioningPhase::Homing || phase_ == InclineCommissioningPhase::Idle) {
        phase_ = InclineCommissioningPhase::Idle;
        points_[0] = InclineCalibrationPoint{0.0f, 0.0f};
        pointCount_ = 1;
        lastCompleteMs_ = nowMs;
    }
}

void InclineCommissioningTracker::beginMeasurePoint(float commandedPct, InclineDirection expectedDirection, uint32_t nowMs) {
    currentCommandedPct_ = commandedPct;
    currentExpectedDirection_ = expectedDirection;
    pointStartMs_ = nowMs;
    phase_ = InclineCommissioningPhase::MeasuringPoint;

    startAngleRecorded_ = false;
    startAngleDeg_ = 0.0f;

    directionSinceMs_ = 0;
    directionActive_ = false;
    directionConfirmed_ = false;

    stableSinceMs_ = 0;
    stableActive_ = false;
    lastSampleAngleDeg_ = 0.0f;
}

void InclineCommissioningTracker::update(float pulseSensorReportedPct, float imuRelativeDeckAngleDeg, bool imuDataValid, uint32_t nowMs) {
    if (phase_ == InclineCommissioningPhase::Homing) {
        if (nowMs - homingStartMs_ >= config_.homingTimeoutMs) {
            phase_ = InclineCommissioningPhase::TimedOut;
        }
        return;
    }

    if (phase_ != InclineCommissioningPhase::MeasuringPoint) {
        return;
    }

    // Check point timeout
    if (nowMs - pointStartMs_ >= config_.pointTimeoutMs) {
        phase_ = InclineCommissioningPhase::TimedOut;
        return;
    }

    if (!imuDataValid || !std::isfinite(imuRelativeDeckAngleDeg)) {
        directionActive_ = false;
        directionSinceMs_ = 0;
        stableActive_ = false;
        stableSinceMs_ = 0;
        return;
    }

    if (!startAngleRecorded_) {
        startAngleDeg_ = imuRelativeDeckAngleDeg;
        startAngleRecorded_ = true;
        return;
    }

    // 1. Safety Direction Qualification
    if (!directionConfirmed_) {
        const float deltaAngle = imuRelativeDeckAngleDeg - startAngleDeg_;
        bool correctDirection = false;
        if (currentExpectedDirection_ == InclineDirection::Up) {
            correctDirection = (deltaAngle >= config_.directionConfirmAngleDeg);
        } else if (currentExpectedDirection_ == InclineDirection::Down) {
            correctDirection = (deltaAngle <= -config_.directionConfirmAngleDeg);
        } else {
            correctDirection = true; // Unknown direction skips directional lock
        }

        if (correctDirection) {
            if (!directionActive_) {
                directionActive_ = true;
                directionSinceMs_ = nowMs;
            }
            if (nowMs - directionSinceMs_ >= config_.directionConfirmMs) {
                directionConfirmed_ = true;
            }
        } else {
            directionActive_ = false;
            directionSinceMs_ = 0;
        }
    }

    // 2. Arrival & Stabilization Qualification
    if (directionConfirmed_) {
        const bool pulseArrived = std::isfinite(pulseSensorReportedPct) &&
            (std::abs(pulseSensorReportedPct - currentCommandedPct_) <= config_.arrivalTolerancePct);

        if (pulseArrived) {
            if (!stableActive_) {
                stableActive_ = true;
                stableSinceMs_ = nowMs;
                lastSampleAngleDeg_ = imuRelativeDeckAngleDeg;
            } else {
                if (std::abs(imuRelativeDeckAngleDeg - lastSampleAngleDeg_) > config_.stabilityAngleToleranceDeg) {
                    stableSinceMs_ = nowMs;
                    lastSampleAngleDeg_ = imuRelativeDeckAngleDeg;
                } else if (nowMs - stableSinceMs_ >= config_.stableDurationMs) {
                    // Confirmed stable! Calculate grade from angle
                    // Inverse of FtmsService::packTreadmillData:
                    //   angleDeg = atanf(grade / 100.0f) * (180.0f / pi)
                    //   grade = tanf(angleDeg * (pi / 180.0f)) * 100.0f
                    const float kPi = 3.14159265358979323846f;
                    float measuredPct = tanf(imuRelativeDeckAngleDeg * (kPi / 180.0f)) * 100.0f;
                    if (measuredPct < 0.0f) measuredPct = 0.0f;
                    if (measuredPct > config_.maxAchievableInclinePct) measuredPct = config_.maxAchievableInclinePct;

                    InclineCalibrationPoint pt{};
                    pt.measuredActualInclinePct = measuredPct;
                    pt.treadmillCommandPct = currentCommandedPct_;

                    if (pointCount_ < kMaxInclineCalibrationPoints) {
                        points_[pointCount_++] = pt;
                    }
                    lastCompleteMs_ = nowMs;
                    phase_ = InclineCommissioningPhase::Complete;
                }
            }
        } else {
            stableActive_ = false;
            stableSinceMs_ = 0;
        }
    }
}

void InclineCommissioningTracker::abort(uint32_t /*nowMs*/) {
    phase_ = InclineCommissioningPhase::Aborted;
    directionActive_ = false;
    stableActive_ = false;
}

void InclineCommissioningTracker::reset() {
    phase_ = InclineCommissioningPhase::Idle;
    currentCommandedPct_ = 0.0f;
    currentExpectedDirection_ = InclineDirection::Unknown;

    homingStartMs_ = 0;
    pointStartMs_ = 0;

    startAngleRecorded_ = false;
    startAngleDeg_ = 0.0f;

    directionSinceMs_ = 0;
    directionActive_ = false;
    directionConfirmed_ = false;

    stableSinceMs_ = 0;
    stableActive_ = false;
    lastSampleAngleDeg_ = 0.0f;

    for (auto& pt : points_) {
        pt = InclineCalibrationPoint{};
    }
    pointCount_ = 0;
    lastCompleteMs_ = 0;
}

InclineCommissioningPhase InclineCommissioningTracker::phase() const {
    return phase_;
}

bool InclineCommissioningTracker::currentPointDirectionConfirmed() const {
    return directionConfirmed_;
}

InclineConfig InclineCommissioningTracker::buildCandidateConfig() const {
    InclineConfig cfg{};
    cfg.pointCount = pointCount_;
    cfg.points = points_;
    cfg.maxAchievableInclinePct = config_.maxAchievableInclinePct;
    cfg.source = CalibrationSource::PhysicalCommissioning;
    cfg.calibratedAtMs = lastCompleteMs_;

    if (cfg.pointCount >= 2) {
        // Sort points ascending by measuredActualInclinePct
        std::sort(cfg.points.begin(), cfg.points.begin() + cfg.pointCount,
            [](const InclineCalibrationPoint& a, const InclineCalibrationPoint& b) {
                return a.measuredActualInclinePct < b.measuredActualInclinePct;
            });

        // Validate strictly monotonic
        bool valid = true;
        for (size_t i = 1; i < cfg.pointCount; ++i) {
            if (cfg.points[i].measuredActualInclinePct <= cfg.points[i - 1].measuredActualInclinePct ||
                cfg.points[i].treadmillCommandPct <= cfg.points[i - 1].treadmillCommandPct) {
                valid = false;
                break;
            }
        }
        cfg.commandMapValid = valid;
        cfg.maxAchievableInclineVerified = valid;
    } else {
        cfg.commandMapValid = false;
        cfg.maxAchievableInclineVerified = false;
    }
    return cfg;
}

bool InclineCommissioningTracker::timedOut() const {
    return phase_ == InclineCommissioningPhase::TimedOut;
}

float InclineCommissioningTracker::currentCommandedPct() const {
    return currentCommandedPct_;
}

InclineDirection InclineCommissioningTracker::currentExpectedDirection() const {
    return currentExpectedDirection_;
}

uint8_t InclineCommissioningTracker::pointCount() const {
    return pointCount_;
}

} // namespace stridecontrol

