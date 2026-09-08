#include "TreadmillSimulator.h"
#include <cmath>

namespace stridecontrol {

void TreadmillSimulator::begin(uint32_t nowMs) {
    ready_ = true;
    busy_ = false;
    busyStartMs_ = 0;
    lastUpdateMs_ = nowMs;
    snapshotSequence_ = 0;
    currentSpeedKmh_ = 0.0f;
    targetSpeedKmh_ = 0.0f;
    currentInclinePct_ = 0.0f;
    targetInclinePct_ = 0.0f;
    cumulativeDistanceKm_ = 0.0;
}

void TreadmillSimulator::update(uint32_t nowMs) {
    if (!ready_) {
        return;
    }

    // 1. Rollover-safe busy evaluation
    if (busy_) {
        if (static_cast<uint32_t>(nowMs - busyStartMs_) >= kBusyDurationMs) {
            busy_ = false;
        }
    }

    // 2. Rollover-safe elapsed time calculation
    const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - lastUpdateMs_);
    lastUpdateMs_ = nowMs;
    snapshotSequence_++;

    if (elapsedMs == 0) {
        return;
    }

    // 3. Advance tracked incline state
    currentInclinePct_ = targetInclinePct_;

    // 4. Advance physical speed with 0.5 km/h per second ramp rate and overshoot clamp
    const float previousSpeed = currentSpeedKmh_;
    if (currentSpeedKmh_ < targetSpeedKmh_) {
        const float maxDelta = kSpeedRampRatePerSec * (static_cast<float>(elapsedMs) / 1000.0f);
        currentSpeedKmh_ += maxDelta;
        if (currentSpeedKmh_ > targetSpeedKmh_) {
            currentSpeedKmh_ = targetSpeedKmh_;
        }
    } else if (currentSpeedKmh_ > targetSpeedKmh_) {
        const float maxDelta = kSpeedRampRatePerSec * (static_cast<float>(elapsedMs) / 1000.0f);
        currentSpeedKmh_ -= maxDelta;
        if (currentSpeedKmh_ < targetSpeedKmh_) {
            currentSpeedKmh_ = targetSpeedKmh_;
        }
    }

    // 5. Trapezoidal distance integration (Rule 9: Do not synthesize distance while speed is zero)
    if (previousSpeed > 0.0f || currentSpeedKmh_ > 0.0f) {
        const float averageSpeedKmh = (previousSpeed + currentSpeedKmh_) * 0.5f;
        if (averageSpeedKmh > 0.0f) {
            const double deltaKm = (static_cast<double>(averageSpeedKmh) * static_cast<double>(elapsedMs)) / 3600000.0;
            cumulativeDistanceKm_ += deltaKm;
        }
    }
}

bool TreadmillSimulator::submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) {
    if (!ready_ || busy_) {
        return false;
    }
    targetSpeedKmh_ = targetSpeedKmh;
    busy_ = true;
    busyStartMs_ = nowMs;
    return true;
}

bool TreadmillSimulator::submitInclineTarget(float targetInclinePct, uint32_t nowMs) {
    if (!ready_ || busy_) {
        return false;
    }
    targetInclinePct_ = targetInclinePct;
    busy_ = true;
    busyStartMs_ = nowMs;
    return true;
}

bool TreadmillSimulator::submitStop(uint32_t nowMs) {
    if (!ready_) {
        return false;
    }
    (void)nowMs;
    targetSpeedKmh_ = 0.0f;
    currentSpeedKmh_ = 0.0f;
    busy_ = false;
    return true;
}

bool TreadmillSimulator::isBusy() const {
    return busy_;
}

bool TreadmillSimulator::isReady() const {
    return ready_;
}

ApplicationSnapshot TreadmillSimulator::getSnapshot() const {
    ApplicationSnapshot snap{};
    snap.timestampMs = lastUpdateMs_;
    snap.sequenceNumber = snapshotSequence_;

    // Measured physical belt speed
    snap.speed.initialized = true;
    snap.speed.signalPresent = (currentSpeedKmh_ > 0.0f);
    snap.speed.measurementValid = true;
    snap.speed.speedKmh = currentSpeedKmh_;
    snap.speed.status = (currentSpeedKmh_ > 0.0f) ? SpeedSensorStatus::Measuring : SpeedSensorStatus::Ready;

    // Tracked incline
    snap.incline.initialized = true;
    snap.incline.homed = true;
    snap.incline.positionTrusted = true;
    snap.incline.signalPresent = true;
    snap.incline.estimatedInclinePct = currentInclinePct_;
    snap.incline.status = InclineStatus::Ready;

    // Authoritative runner dynamics & distance consumed by WorkoutSession
    snap.runner.initialized = true;
    snap.runner.status = (currentSpeedKmh_ > 0.0f) ? RunnerDynamicsStatus::Tracking : RunnerDynamicsStatus::Ready;
    snap.runner.presence = (currentSpeedKmh_ > 0.0f) ? RunnerPresence::Active : RunnerPresence::Inactive;
    snap.runner.activity = (currentSpeedKmh_ > 8.0f) ? RunnerActivity::Running : 
                           (currentSpeedKmh_ > 0.0f) ? RunnerActivity::Walking : RunnerActivity::Inactive;
    snap.runner.beltSpeedKmh = currentSpeedKmh_;
    snap.runner.runnerSpeedKmh = currentSpeedKmh_;
    snap.runner.runnerSpeedValid = true;
    snap.runner.speedCreditEnabled = true;

    snap.runner.validatedDistanceKm = cumulativeDistanceKm_;
    snap.runner.distanceAccumulationEnabled = (currentSpeedKmh_ > 0.0f);
    snap.runner.distanceValid = true;
    snap.runner.distancePauseReason = (currentSpeedKmh_ > 0.0f) ? RunnerDistancePauseReason::None : RunnerDistancePauseReason::BeltStopped;
    snap.runner.inputDataValid = true;
    snap.runner.speedInputValid = true;

    // System health
    snap.health.isHealthy = true;
    snap.health.highestSeverity = FaultSeverity::None;

    return snap;
}

const char* TreadmillSimulator::version() {
    return "TreadmillSimulator/1.0.0";
}

} // namespace stridecontrol

