#include "RunnerDynamics.h"
#include <cmath>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace stridecontrol {

static constexpr float kZeroSpeedEpsilonKmh = 0.01f;

const char* runnerDynamicsStatusName(RunnerDynamicsStatus status) {
    switch (status) {
        case RunnerDynamicsStatus::Uninitialized: return "UNINITIALIZED";
        case RunnerDynamicsStatus::Ready: return "READY";
        case RunnerDynamicsStatus::Tracking: return "TRACKING";
        case RunnerDynamicsStatus::DegradedInput: return "DEGRADED_INPUT";
        default: return "UNINITIALIZED";
    }
}

const char* runnerPresenceName(RunnerPresence presence) {
    switch (presence) {
        case RunnerPresence::Unknown: return "UNKNOWN";
        case RunnerPresence::Inactive: return "INACTIVE";
        case RunnerPresence::Candidate: return "CANDIDATE";
        case RunnerPresence::Active: return "ACTIVE";
        case RunnerPresence::SideRailsCandidate: return "SIDE_RAILS_CANDIDATE";
        case RunnerPresence::SideRails: return "SIDE_RAILS";
        default: return "UNKNOWN";
    }
}

const char* runnerActivityName(RunnerActivity activity) {
    switch (activity) {
        case RunnerActivity::Unknown: return "UNKNOWN";
        case RunnerActivity::Inactive: return "INACTIVE";
        case RunnerActivity::Walking: return "WALKING";
        case RunnerActivity::Running: return "RUNNING";
        case RunnerActivity::SideRails: return "SIDE_RAILS";
        default: return "UNKNOWN";
    }
}

const char* runnerConfidenceName(RunnerDynamicsConfidence conf) {
    switch (conf) {
        case RunnerDynamicsConfidence::Unavailable: return "UNAVAILABLE";
        case RunnerDynamicsConfidence::Low: return "LOW";
        case RunnerDynamicsConfidence::Medium: return "MEDIUM";
        case RunnerDynamicsConfidence::High: return "HIGH";
        default: return "UNAVAILABLE";
    }
}

const char* runnerDistancePauseReasonName(RunnerDistancePauseReason reason) {
    switch (reason) {
        case RunnerDistancePauseReason::None: return "NONE";
        case RunnerDistancePauseReason::BeltStopped: return "BELT_STOPPED";
        case RunnerDistancePauseReason::SpeedInvalid: return "SPEED_INVALID";
        case RunnerDistancePauseReason::ImuInvalid: return "IMU_INVALID";
        case RunnerDistancePauseReason::Inactive: return "INACTIVE";
        case RunnerDistancePauseReason::Candidate: return "CANDIDATE";
        case RunnerDistancePauseReason::SideRailsCandidate: return "SIDE_RAILS_CANDIDATE";
        case RunnerDistancePauseReason::SideRails: return "SIDE_RAILS";
        case RunnerDistancePauseReason::UnknownState: return "UNKNOWN_STATE";
        default: return "UNKNOWN_STATE";
    }
}

const char* runnerPeakDetectorStateName(RunnerPeakDetectorState state) {
    switch (state) {
        case RunnerPeakDetectorState::Idle: return "IDLE";
        case RunnerPeakDetectorState::Rising: return "RISING";
        case RunnerPeakDetectorState::Falling: return "FALLING";
        case RunnerPeakDetectorState::Refractory: return "REFRACTORY";
        default: return "IDLE";
    }
}

struct RunnerDynamics::Impl {
    SemaphoreHandle_t opMutex = nullptr;
    mutable portMUX_TYPE stateMux;

    RunnerDynamicsConfig config_;
    RunnerDynamicsState publicState_;
    RunnerDynamicsConfig publicConfig_;

    // Lifecycle
    bool initialized_ = false;
    bool active_ = false;
    uint32_t lastUpdateMs_ = 0;

    // Stream continuity & Baseline Initialization
    bool hasPreviousSample_ = false;
    uint32_t lastSampleTimestampUs_ = 0;
    uint32_t lastSampleSequence_ = 0;
    bool baselineInitialized_ = false;

    // IMU Freshness Tracking
    uint32_t lastValidImuDataMs_ = 0;
    bool hasValidImuData_ = false;

    // Signals & Filters
    float rawSignalG_ = 0.0f;
    float baselineG_ = 0.0f;
    float dynamicSignalG_ = 0.0f;
    float noiseFloorG_ = 0.01f;
    float effectiveThresholdG_ = 0.03f;
    bool thresholdSaturated_ = false;
    uint32_t thresholdSaturationCount_ = 0;

    // Peak detector state machine
    RunnerPeakDetectorState peakState_ = RunnerPeakDetectorState::Idle;
    float peakValueG_ = 0.0f;
    uint32_t peakTimestampUs_ = 0;
    uint32_t lastEmittedPeakTimestampUs_ = 0;
    float lastPeakMagnitudeG_ = 0.0f;

    // Step & Interval history
    uint32_t intervalHistory_[4] = {0};
    uint8_t intervalIndex_ = 0;
    uint8_t validIntervalCount_ = 0;
    uint32_t lastStepIntervalMs_ = 0;
    uint32_t meanStepIntervalMs_ = 0;
    float stepIntervalVariationPct_ = 0.0f;
    bool stepRegularityValid_ = false;

    // Step counters & Cadence
    uint32_t sessionStepCount_ = 0;
    uint32_t lifetimeQualifiedStepCount_ = 0;
    uint32_t detectedImpactCount_ = 0;
    uint8_t qualifiedActiveSteps_ = 0;
    float instantaneousCadenceSpm_ = 0.0f;
    float smoothedCadenceSpm_ = 0.0f;
    bool cadenceValid_ = false;
    uint32_t lastStepTimestampMs_ = 0;
    bool hasValidStepApplicationTimestamp_ = false;
    uint32_t lastStepAgeMs_ = 0;

    // Impact Anchor
    uint32_t lastImpactTimestampUs_ = 0;
    bool hasImpactAnchor_ = false;

    // Candidate Tracking
    uint32_t candidateStartTimestampMs_ = 0;
    bool candidateStartTimeValid_ = false;

    // Presence & Resume state machine
    RunnerPresence presence_ = RunnerPresence::Unknown;
    RunnerActivity activity_ = RunnerActivity::Unknown;
    RunnerDynamicsConfidence confidence_ = RunnerDynamicsConfidence::Unavailable;
    bool inResumeQualification_ = false;
    uint32_t resumeAnchorTimestampUs_ = 0;
    uint32_t resumeAnchorApplicationMs_ = 0;
    bool resumeApplicationTimeValid_ = false;
    uint32_t resumeLastImpactTimestampUs_ = 0;
    uint8_t provisionalResumeSteps_ = 0;
    uint32_t provisionalIntervals_[2] = {0};
    uint8_t provisionalIntervalCount_ = 0;

    // Activity classification & Evidence
    RunnerActivity pendingActivity_ = RunnerActivity::Unknown;
    uint8_t pendingActivitySteps_ = 0;
    uint32_t lastActivityTransitionMs_ = 0;
    uint32_t lastValidActivityEvidenceMs_ = 0;
    bool hasValidActivityEvidence_ = false;

    // Speed & Distance
    float beltSpeedKmh_ = 0.0f;
    float runnerSpeedKmh_ = 0.0f;
    bool runnerSpeedValid_ = false;
    bool speedCreditEnabled_ = false;
    double validatedDistanceKm_ = 0.0;
    double lastDistanceDeltaKm_ = 0.0;
    bool distanceAccumulationEnabled_ = false;
    bool distanceValid_ = false;
    RunnerDistancePauseReason distancePauseReason = RunnerDistancePauseReason::UnknownState;

    uint32_t lastDistanceIntegrationMs_ = 0;
    bool distanceIntegrationInitialized_ = false;
    float previousRunnerSpeedKmh_ = 0.0f;
    bool previousEndpointValid_ = false;

    // Diagnostics & Validity
    bool imuInputValid_ = false;
    bool speedInputValid_ = false;
    bool csafeInputValid_ = false;
    bool inputDataValid_ = false;
    uint32_t presenceTransitionCount_ = 0;
    uint32_t activityTransitionCount_ = 0;
    uint32_t invalidSampleCount_ = 0;
    uint32_t invalidSampleBlockCount_ = 0;
    uint32_t oversizedInputBlockCount_ = 0;
    uint32_t timestampDiscontinuityCount_ = 0;
    uint32_t sequenceDiscontinuityCount_ = 0;
    uint32_t streamDiscontinuityCount_ = 0;
    uint32_t applicationTimestampRejectionCount_ = 0;
    uint32_t rejectedIntervalCount_ = 0;
    uint32_t processedSampleCount_ = 0;

    Impl() {
        portMUX_INITIALIZE(&stateMux);
        opMutex = xSemaphoreCreateMutex();
    }

    ~Impl() {
        if (opMutex != nullptr) {
            vSemaphoreDelete(opMutex);
            opMutex = nullptr;
        }
    }

    void setPresence(RunnerPresence next) {
        if (presence_ != next) {
            presence_ = next;
            presenceTransitionCount_++;
        }
    }

    void setActivity(RunnerActivity next, uint32_t nowMs) {
        if (activity_ != next) {
            activity_ = next;
            activityTransitionCount_++;
            lastActivityTransitionMs_ = nowMs;
        }
    }

    bool validateConfiguration(const RunnerDynamicsConfig& cfg) const {
        if (!std::isfinite(cfg.baselineTimeConstantMs) ||
            !std::isfinite(cfg.noiseFloorTimeConstantMs) ||
            !std::isfinite(cfg.initialNoiseFloorG) ||
            !std::isfinite(cfg.minimumImpactThresholdG) ||
            !std::isfinite(cfg.noiseMultiplier) ||
            !std::isfinite(cfg.maximumNoiseFloorG) ||
            !std::isfinite(cfg.maximumEffectiveThresholdG) ||
            !std::isfinite(cfg.cadenceSmoothingAlpha) ||
            !std::isfinite(cfg.maximumActiveStepVariationPct) ||
            !std::isfinite(cfg.walkingCadenceMinimumSpm) ||
            !std::isfinite(cfg.walkingCadenceMaximumSpm) ||
            !std::isfinite(cfg.runningCadenceMinimumSpm) ||
            !std::isfinite(cfg.runningCadenceMaximumSpm) ||
            !std::isfinite(cfg.walkingSpeedSupportMaximumKmh) ||
            !std::isfinite(cfg.runningSpeedSupportMinimumKmh) ||
            !std::isfinite(cfg.beltMovingThresholdKmh)) {
            return false;
        }

        if (cfg.initialNoiseFloorG <= 0.0f) return false;
        if (cfg.minimumImpactThresholdG <= 0.0f) return false;
        if (cfg.maximumNoiseFloorG < cfg.initialNoiseFloorG) return false;
        if (cfg.maximumEffectiveThresholdG <= cfg.minimumImpactThresholdG) return false;
        if (cfg.noiseMultiplier < 1.0f) return false;

        if (cfg.refractoryIntervalMs == 0) return false;
        if (cfg.minimumStepIntervalMs <= cfg.refractoryIntervalMs) return false;
        if (cfg.maximumStepIntervalMs <= cfg.minimumStepIntervalMs) return false;
        if (cfg.maxSampleTimestampGapUs == 0) return false;

        if (cfg.minQualifiedStepsForCadence < 2) return false;
        if (cfg.minQualifiedStepsForActive < cfg.minQualifiedStepsForCadence) return false;
        if (cfg.sideRailCreditGraceMs >= cfg.sideRailTimeoutMs) return false;
        if (cfg.sideRailResumeQualifiedSteps < 2) return false;
        if (cfg.sideRailResumeTimeoutMs == 0) return false;
        if (cfg.candidateTimeoutMs == 0) return false;
        if (cfg.activeRunnerTimeoutMs == 0) return false;
        if (cfg.cadenceInvalidTimeoutMs == 0) return false;
        if (cfg.classificationHoldMs == 0) return false;
        if (cfg.speedDataMaxAgeMs == 0) return false;
        if (cfg.imuDataMaxAgeMs == 0) return false;
        if (cfg.maximumDistanceIntegrationIntervalMs == 0) return false;

        if (cfg.walkingCadenceMinimumSpm >= cfg.walkingCadenceMaximumSpm) return false;
        if (cfg.runningCadenceMinimumSpm >= cfg.runningCadenceMaximumSpm) return false;
        if (cfg.walkingSpeedSupportMaximumKmh >= cfg.runningSpeedSupportMinimumKmh) return false;
        if (cfg.classificationConfirmationStepCount < 1) return false;

        if (cfg.maximumSamplesPerUpdate < 1 || cfg.maximumSamplesPerUpdate > 128) return false;

        if (cfg.maximumInputBlockSpanUs == 0) return false;
        if (cfg.maximumImpactApplicationAgeMs == 0) return false;
        if ((cfg.maximumInputBlockSpanUs / 1000) > cfg.maximumImpactApplicationAgeMs) return false;

        return true;
    }

    bool isSpeedStateValid(const SpeedSensorState& speedState) const {
        if (!speedState.initialized || !speedState.measurementValid ||
            !std::isfinite(speedState.speedKmh) || speedState.speedKmh < 0.0f) {
            return false;
        }

        if (speedState.status == SpeedSensorStatus::Uninitialized ||
            speedState.status == SpeedSensorStatus::TimedOut ||
            speedState.status == SpeedSensorStatus::HardwareError) {
            return false;
        }

        bool isOperational = (speedState.status == SpeedSensorStatus::Ready ||
                              speedState.status == SpeedSensorStatus::AwaitingFirstPulse ||
                              speedState.status == SpeedSensorStatus::Measuring ||
                              speedState.status == SpeedSensorStatus::AwaitingInterval);

        if (!isOperational) {
            return false;
        }

        if (speedState.speedKmh <= kZeroSpeedEpsilonKmh) {
            // Confirmed stationary state: pulse-age exemption applies
            return true;
        }

        // Non-zero pulse-derived speed: the physically expected time between pulses grows
        // as speed drops (real T610 calibration: speedSensorKmhPerHz). A fixed timeout
        // is guaranteed to misfire below some speed - scale the tolerance with expected
        // pulse interval instead, with margin, never looser than 2x the configured floor.
        const float expectedIntervalMs = (config_.speedSensorKmhPerHz / speedState.speedKmh) * 1000.0f;
        // Widen the margin to absorb pulse jitter at low speed, but cap it so a genuinely
        // dead/faulted signal is never allowed to hang undetected for too long.
        constexpr uint32_t kDynamicMaxAgeCeilingMs = 2500;
        constexpr uint32_t kLowSpeedCeilingMs = 3500;
        constexpr float kLowSpeedZoneKmh = 3.0f;
        uint32_t dynamicMaxAgeMs = std::min(
            static_cast<uint32_t>(expectedIntervalMs * 2.0f),
            kDynamicMaxAgeCeilingMs);
        // Below kLowSpeedZoneKmh (an explicit low-speed/deceleration zone, kept generously
        // above beltMovingThresholdKmh regardless of its configured value), active deceleration
        // stretches pulse intervals far beyond what instantaneous speed predicts - including
        // the very last pulse before a full stop. Grant a generous ceiling here so the belt can
        // coast down to zero without tripping SpeedInvalid.
        const float lowSpeedZoneKmh = std::max(kLowSpeedZoneKmh, config_.beltMovingThresholdKmh * 3.0f);
        if (speedState.speedKmh <= lowSpeedZoneKmh) {
            dynamicMaxAgeMs = kLowSpeedCeilingMs;
        }
        const uint32_t effectiveMaxAgeMs = std::max(config_.speedDataMaxAgeMs, dynamicMaxAgeMs);

        return (speedState.lastPulseAgeMs <= effectiveMaxAgeMs);
    }

    void commitSnapshot(uint32_t nowMs) {
        RunnerDynamicsState snap;
        snap.initialized = initialized_;
        if (!initialized_) {
            snap.status = RunnerDynamicsStatus::Uninitialized;
        } else if (!inputDataValid_) {
            snap.status = RunnerDynamicsStatus::DegradedInput;
        } else if (presence_ == RunnerPresence::Active ||
                   presence_ == RunnerPresence::Candidate ||
                   presence_ == RunnerPresence::SideRailsCandidate ||
                   presence_ == RunnerPresence::SideRails) {
            snap.status = RunnerDynamicsStatus::Tracking;
        } else {
            snap.status = RunnerDynamicsStatus::Ready;
        }
        snap.snapshotTimestampMs = nowMs;

        snap.presence = presence_;
        snap.activity = activity_;
        snap.confidence = confidence_;
        snap.sideRailsCandidate = (presence_ == RunnerPresence::SideRailsCandidate);
        snap.activePresence = (presence_ == RunnerPresence::Active);
        snap.onSideRails = (presence_ == RunnerPresence::SideRails);

        snap.beltSpeedKmh = beltSpeedKmh_;
        snap.runnerSpeedKmh = runnerSpeedKmh_;
        snap.runnerSpeedValid = runnerSpeedValid_;
        snap.speedCreditEnabled = speedCreditEnabled_;

        snap.instantaneousCadenceSpm = instantaneousCadenceSpm_;
        snap.smoothedCadenceSpm = smoothedCadenceSpm_;
        snap.cadenceValid = cadenceValid_;
        snap.sessionStepCount = sessionStepCount_;
        snap.lifetimeQualifiedStepCount = lifetimeQualifiedStepCount_;
        snap.detectedImpactCount = detectedImpactCount_;
        snap.lastStepTimestampMs = lastStepTimestampMs_;
        snap.lastStepAgeMs = lastStepAgeMs_;
        snap.lastStepIntervalMs = lastStepIntervalMs_;
        snap.meanStepIntervalMs = meanStepIntervalMs_;
        snap.stepIntervalVariationPct = stepIntervalVariationPct_;
        snap.stepRegularityValid = stepRegularityValid_;

        snap.rawSignalG = rawSignalG_;
        snap.baselineG = baselineG_;
        snap.dynamicSignalG = dynamicSignalG_;
        snap.noiseFloorG = noiseFloorG_;
        snap.effectiveThresholdG = effectiveThresholdG_;
        snap.thresholdSaturated = thresholdSaturated_;
        snap.thresholdSaturationCount = thresholdSaturationCount_;
        snap.peakDetectorState = peakState_;
        snap.lastPeakMagnitudeG = lastPeakMagnitudeG_;

        snap.validatedDistanceKm = validatedDistanceKm_;
        snap.lastDistanceDeltaKm = lastDistanceDeltaKm_;
        snap.distanceAccumulationEnabled = distanceAccumulationEnabled_;
        snap.distanceValid = distanceValid_;
        snap.distancePauseReason = distancePauseReason;

        snap.imuInputValid = imuInputValid_;
        snap.imuDataAgeMs = hasValidImuData_ ? (nowMs - lastValidImuDataMs_) : UINT32_MAX;
        snap.speedInputValid = speedInputValid_;
        snap.csafeInputValid = csafeInputValid_;
        snap.inputDataValid = inputDataValid_;
        snap.provisionalResumeStepCount = provisionalResumeSteps_;

        snap.invalidSampleCount = invalidSampleCount_;
        snap.invalidSampleBlockCount = invalidSampleBlockCount_;
        snap.oversizedInputBlockCount = oversizedInputBlockCount_;
        snap.timestampDiscontinuityCount = timestampDiscontinuityCount_;
        snap.sequenceDiscontinuityCount = sequenceDiscontinuityCount_;
        snap.streamDiscontinuityCount = streamDiscontinuityCount_;
        snap.applicationTimestampRejectionCount = applicationTimestampRejectionCount_;
        snap.rejectedIntervalCount = rejectedIntervalCount_;
        snap.processedSampleCount = processedSampleCount_;
        snap.presenceTransitionCount = presenceTransitionCount_;
        snap.activityTransitionCount = activityTransitionCount_;

        portENTER_CRITICAL(&stateMux);
        publicState_ = snap;
        publicConfig_ = config_;
        portEXIT_CRITICAL(&stateMux);
    }

    void resetDiscontinuityInternal() {
        baselineInitialized_ = false;
        hasPreviousSample_ = false;
        lastSampleTimestampUs_ = 0;
        lastSampleSequence_ = 0;

        hasValidImuData_ = false;
        lastValidImuDataMs_ = 0;
        imuInputValid_ = false;
        inputDataValid_ = false;

        peakState_ = RunnerPeakDetectorState::Idle;
        peakValueG_ = 0.0f;
        peakTimestampUs_ = 0;
        lastEmittedPeakTimestampUs_ = 0;

        lastImpactTimestampUs_ = 0;
        hasImpactAnchor_ = false;

        cadenceValid_ = false;
        instantaneousCadenceSpm_ = 0.0f;
        smoothedCadenceSpm_ = 0.0f;

        validIntervalCount_ = 0;
        intervalIndex_ = 0;
        lastStepIntervalMs_ = 0;
        meanStepIntervalMs_ = 0;
        stepIntervalVariationPct_ = 0.0f;
        stepRegularityValid_ = false;

        hasValidStepApplicationTimestamp_ = false;
        qualifiedActiveSteps_ = 0;
        candidateStartTimeValid_ = false;
        if (presence_ != RunnerPresence::Unknown) {
            setPresence(RunnerPresence::Unknown);
        }
        if (activity_ != RunnerActivity::Unknown) {
            setActivity(RunnerActivity::Unknown, lastUpdateMs_);
        }

        pendingActivity_ = RunnerActivity::Unknown;
        pendingActivitySteps_ = 0;
        lastValidActivityEvidenceMs_ = 0;
        hasValidActivityEvidence_ = false;

        inResumeQualification_ = false;
        resumeAnchorTimestampUs_ = 0;
        resumeAnchorApplicationMs_ = 0;
        resumeApplicationTimeValid_ = false;
        provisionalResumeSteps_ = 0;
        provisionalIntervalCount_ = 0;

        speedCreditEnabled_ = false;
        runnerSpeedValid_ = false;
        runnerSpeedKmh_ = 0.0f;
        distanceAccumulationEnabled_ = false;

        distanceIntegrationInitialized_ = false;
        previousEndpointValid_ = false;
        lastDistanceDeltaKm_ = 0.0;
    }

    bool begin(const RunnerDynamicsConfig& cfg) {
        if (opMutex == nullptr) return false;
        if (xSemaphoreTake(opMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return false;
        }

        if (!validateConfiguration(cfg)) {
            xSemaphoreGive(opMutex);
            return false;
        }

        config_ = cfg;
        initialized_ = true;
        active_ = true;
        lastUpdateMs_ = 0;

        hasPreviousSample_ = false;
        lastSampleTimestampUs_ = 0;
        lastSampleSequence_ = 0;
        baselineInitialized_ = false;
        lastValidImuDataMs_ = 0;
        hasValidImuData_ = false;

        rawSignalG_ = 0.0f;
        baselineG_ = 0.0f;
        dynamicSignalG_ = 0.0f;
        noiseFloorG_ = config_.initialNoiseFloorG;
        effectiveThresholdG_ = config_.minimumImpactThresholdG;
        thresholdSaturated_ = false;

        // Lifecycle establishment resets all diagnostic and lifetime counters
        sessionStepCount_ = 0;
        lifetimeQualifiedStepCount_ = 0;
        detectedImpactCount_ = 0;
        rejectedIntervalCount_ = 0;
        invalidSampleCount_ = 0;
        invalidSampleBlockCount_ = 0;
        oversizedInputBlockCount_ = 0;
        timestampDiscontinuityCount_ = 0;
        sequenceDiscontinuityCount_ = 0;
        streamDiscontinuityCount_ = 0;
        applicationTimestampRejectionCount_ = 0;
        thresholdSaturationCount_ = 0;
        processedSampleCount_ = 0;

        presence_ = RunnerPresence::Unknown;
        activity_ = RunnerActivity::Unknown;
        presenceTransitionCount_ = 0;
        activityTransitionCount_ = 0;

        resetSessionInternal();
        distanceValid_ = true;

        commitSnapshot(0);
        xSemaphoreGive(opMutex);
        return true;
    }

    void end() {
        if (opMutex == nullptr) return;
        if (xSemaphoreTake(opMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;
        }

        active_ = false;
        initialized_ = false;
        distanceValid_ = false;

        setPresence(RunnerPresence::Unknown);
        setActivity(RunnerActivity::Unknown, lastUpdateMs_);

        runnerSpeedKmh_ = 0.0f;
        runnerSpeedValid_ = false;
        speedCreditEnabled_ = false;
        distanceAccumulationEnabled_ = false;
        lastDistanceDeltaKm_ = 0.0;
        previousEndpointValid_ = false;
        distanceIntegrationInitialized_ = false;

        hasValidStepApplicationTimestamp_ = false;
        hasImpactAnchor_ = false;
        candidateStartTimeValid_ = false;
        inResumeQualification_ = false;
        resumeApplicationTimeValid_ = false;
        provisionalResumeSteps_ = 0;
        provisionalIntervalCount_ = 0;

        pendingActivity_ = RunnerActivity::Unknown;
        pendingActivitySteps_ = 0;
        lastValidActivityEvidenceMs_ = 0;
        hasValidActivityEvidence_ = false;

        distancePauseReason = RunnerDistancePauseReason::UnknownState;

        commitSnapshot(lastUpdateMs_);
        xSemaphoreGive(opMutex);
    }

    void resetStepCounterInternal() {
        sessionStepCount_ = 0;
    }

    void resetValidatedDistanceInternal() {
        validatedDistanceKm_ = 0.0;
        lastDistanceDeltaKm_ = 0.0;
        lastDistanceIntegrationMs_ = lastUpdateMs_;
        distanceIntegrationInitialized_ = false;
        previousEndpointValid_ = false;
        previousRunnerSpeedKmh_ = 0.0f;
        distanceValid_ = initialized_;
    }

    void resetSessionInternal() {
        resetStepCounterInternal();
        resetValidatedDistanceInternal();

        setPresence(RunnerPresence::Unknown);
        setActivity(RunnerActivity::Unknown, lastUpdateMs_);
        confidence_ = RunnerDynamicsConfidence::Unavailable;

        instantaneousCadenceSpm_ = 0.0f;
        smoothedCadenceSpm_ = 0.0f;
        cadenceValid_ = false;
        lastStepIntervalMs_ = 0;
        meanStepIntervalMs_ = 0;
        stepIntervalVariationPct_ = 0.0f;
        stepRegularityValid_ = false;
        lastStepTimestampMs_ = 0;
        hasValidStepApplicationTimestamp_ = false;
        lastStepAgeMs_ = 0;

        lastImpactTimestampUs_ = 0;
        hasImpactAnchor_ = false;

        qualifiedActiveSteps_ = 0;
        candidateStartTimestampMs_ = 0;
        candidateStartTimeValid_ = false;

        peakState_ = RunnerPeakDetectorState::Idle;
        peakValueG_ = 0.0f;
        peakTimestampUs_ = 0;
        lastEmittedPeakTimestampUs_ = 0;
        lastPeakMagnitudeG_ = 0.0f;

        inResumeQualification_ = false;
        resumeAnchorTimestampUs_ = 0;
        resumeAnchorApplicationMs_ = 0;
        resumeApplicationTimeValid_ = false;
        resumeLastImpactTimestampUs_ = 0;
        provisionalResumeSteps_ = 0;
        provisionalIntervalCount_ = 0;

        intervalIndex_ = 0;
        validIntervalCount_ = 0;

        pendingActivity_ = RunnerActivity::Unknown;
        pendingActivitySteps_ = 0;
        lastActivityTransitionMs_ = 0;
        lastValidActivityEvidenceMs_ = 0;
        hasValidActivityEvidence_ = false;

        runnerSpeedKmh_ = 0.0f;
        runnerSpeedValid_ = false;
        speedCreditEnabled_ = false;
        distanceAccumulationEnabled_ = false;
        distancePauseReason = RunnerDistancePauseReason::UnknownState;
    }

    void resetStepCounter() {
        if (opMutex == nullptr) return;
        if (xSemaphoreTake(opMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
        resetStepCounterInternal();
        commitSnapshot(lastUpdateMs_);
        xSemaphoreGive(opMutex);
    }

    void resetValidatedDistance() {
        if (opMutex == nullptr) return;
        if (xSemaphoreTake(opMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
        resetValidatedDistanceInternal();
        commitSnapshot(lastUpdateMs_);
        xSemaphoreGive(opMutex);
    }

    void resetSession() {
        if (opMutex == nullptr) return;
        if (xSemaphoreTake(opMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
        resetSessionInternal();
        commitSnapshot(lastUpdateMs_);
        xSemaphoreGive(opMutex);
    }

    static bool computeMapd(const uint32_t* intervals, size_t count, float& mapdOut, uint32_t& meanOut) {
        if (intervals == nullptr || count < 2) {
            mapdOut = 0.0f;
            meanOut = 0;
            return false;
        }

        uint32_t sum = 0;
        for (size_t i = 0; i < count; ++i) {
            sum += intervals[i];
        }
        float mean = static_cast<float>(sum) / static_cast<float>(count);
        meanOut = static_cast<uint32_t>(mean + 0.5f);
        if (mean <= 0.001f) {
            mapdOut = 0.0f;
            return false;
        }

        float diffSum = 0.0f;
        for (size_t i = 0; i < count; ++i) {
            diffSum += std::abs(static_cast<float>(intervals[i]) - mean);
        }
        mapdOut = ((diffSum / static_cast<float>(count)) / mean) * 100.0f;
        return true;
    }

    void recordInterval(uint32_t intervalMs) {
        lastStepIntervalMs_ = intervalMs;
        intervalHistory_[intervalIndex_] = intervalMs;
        intervalIndex_ = (intervalIndex_ + 1U) % 4U;
        if (validIntervalCount_ < 4U) {
            validIntervalCount_++;
        }
        float mapd = 0.0f;
        uint32_t mean = 0;
        if (computeMapd(intervalHistory_, validIntervalCount_, mapd, mean)) {
            meanStepIntervalMs_ = mean;
            stepIntervalVariationPct_ = mapd;
            stepRegularityValid_ = (mapd <= config_.maximumActiveStepVariationPct);
        } else {
            meanStepIntervalMs_ = intervalMs;
            stepIntervalVariationPct_ = 0.0f;
            stepRegularityValid_ = false;
        }
    }

    void advanceActivityEvidence(RunnerActivity candidate, uint8_t stepCount, uint32_t stepAppTimestampMs, uint32_t nowMs) {
        if (candidate == RunnerActivity::Unknown) return;

        if (candidate == pendingActivity_) {
            pendingActivitySteps_ += stepCount;
            if (pendingActivitySteps_ >= config_.classificationConfirmationStepCount) {
                setActivity(candidate, nowMs);
                pendingActivity_ = RunnerActivity::Unknown;
                pendingActivitySteps_ = 0;
                lastValidActivityEvidenceMs_ = stepAppTimestampMs;
                hasValidActivityEvidence_ = true;
            } else if (candidate == activity_) {
                lastValidActivityEvidenceMs_ = stepAppTimestampMs;
                hasValidActivityEvidence_ = true;
            }
        } else {
            pendingActivity_ = candidate;
            pendingActivitySteps_ = stepCount;
            if (pendingActivitySteps_ >= config_.classificationConfirmationStepCount) {
                setActivity(candidate, nowMs);
                pendingActivity_ = RunnerActivity::Unknown;
                pendingActivitySteps_ = 0;
                lastValidActivityEvidenceMs_ = stepAppTimestampMs;
                hasValidActivityEvidence_ = true;
            } else if (candidate == activity_) {
                lastValidActivityEvidenceMs_ = stepAppTimestampMs;
                hasValidActivityEvidence_ = true;
            }
        }
    }

    RunnerActivity evaluateCurrentCandidate(float beltSpeedKmh) const {
        bool beltMoving = (speedInputValid_ && beltSpeedKmh >= config_.beltMovingThresholdKmh);
        if (presence_ != RunnerPresence::Active || !cadenceValid_ || instantaneousCadenceSpm_ <= 0.0f ||
            !stepRegularityValid_ || thresholdSaturated_ || !imuInputValid_ || !speedInputValid_ || !beltMoving) {
            return RunnerActivity::Unknown;
        }

        float c = smoothedCadenceSpm_ > 0.0f ? smoothedCadenceSpm_ : instantaneousCadenceSpm_;
        bool inWalk = (c >= config_.walkingCadenceMinimumSpm && c <= config_.walkingCadenceMaximumSpm);
        bool inRun = (c >= config_.runningCadenceMinimumSpm && c <= config_.runningCadenceMaximumSpm);

        if (inWalk && !inRun) {
            return RunnerActivity::Walking;
        } else if (inRun && !inWalk) {
            return RunnerActivity::Running;
        } else if (inWalk && inRun) {
            if (beltSpeedKmh <= config_.walkingSpeedSupportMaximumKmh) {
                return RunnerActivity::Walking;
            } else if (beltSpeedKmh >= config_.runningSpeedSupportMinimumKmh) {
                return RunnerActivity::Running;
            } else {
                return (activity_ != RunnerActivity::Unknown) ? activity_ : RunnerActivity::Walking;
            }
        }
        return RunnerActivity::Unknown;
    }

    void evaluateActivityHold(uint32_t nowMs) {
        if (presence_ == RunnerPresence::SideRails) {
            setActivity(RunnerActivity::SideRails, nowMs);
            pendingActivity_ = RunnerActivity::Unknown;
            pendingActivitySteps_ = 0;
            return;
        }
        if (presence_ == RunnerPresence::Inactive) {
            setActivity(RunnerActivity::Inactive, nowMs);
            pendingActivity_ = RunnerActivity::Unknown;
            pendingActivitySteps_ = 0;
            return;
        }
        if (presence_ != RunnerPresence::Active) {
            if (activity_ == RunnerActivity::Walking || activity_ == RunnerActivity::Running) {
                if (!hasValidActivityEvidence_ || (nowMs - lastValidActivityEvidenceMs_ > config_.classificationHoldMs)) {
                    setActivity(RunnerActivity::Unknown, nowMs);
                }
            } else {
                setActivity(RunnerActivity::Unknown, nowMs);
            }
            pendingActivity_ = RunnerActivity::Unknown;
            pendingActivitySteps_ = 0;
            return;
        }

        if (activity_ == RunnerActivity::Walking || activity_ == RunnerActivity::Running) {
            if (!hasValidActivityEvidence_ || (nowMs - lastValidActivityEvidenceMs_ > config_.classificationHoldMs)) {
                setActivity(RunnerActivity::Unknown, nowMs);
            }
        }
    }

    void processQualifiedImpact(uint32_t impactTimestampUs,
                                uint32_t nowMs,
                                uint32_t newestValidUs,
                                bool streamContinuous,
                                float beltSpeedKmh) {
        detectedImpactCount_++;

        uint32_t impactAgeUs = newestValidUs - impactTimestampUs; // Rollover safe
        bool timestampValid = false;
        uint32_t appTimestampMs = 0;

        if (streamContinuous &&
            impactAgeUs <= config_.maximumInputBlockSpanUs &&
            (impactAgeUs / 1000U) <= config_.maximumImpactApplicationAgeMs) {
            appTimestampMs = nowMs - (impactAgeUs / 1000U);
            timestampValid = true;
        } else {
            applicationTimestampRejectionCount_++;
        }

        // SideRails Resume Progression
        if (presence_ == RunnerPresence::SideRailsCandidate || presence_ == RunnerPresence::SideRails) {
            if (!inResumeQualification_) {
                if (!timestampValid) {
                    return; // Do not start resume with invalid application timestamp
                }
                inResumeQualification_ = true;
                resumeAnchorTimestampUs_ = impactTimestampUs;
                resumeAnchorApplicationMs_ = appTimestampMs;
                resumeApplicationTimeValid_ = true;
                resumeLastImpactTimestampUs_ = impactTimestampUs;
                provisionalResumeSteps_ = 0;
                provisionalIntervalCount_ = 0;
                return;
            }

            if (!timestampValid) {
                // Abort provisional progression on invalid timestamp
                inResumeQualification_ = false;
                resumeApplicationTimeValid_ = false;
                provisionalResumeSteps_ = 0;
                provisionalIntervalCount_ = 0;
                return;
            }

            uint32_t resumeIntervalUs = impactTimestampUs - resumeLastImpactTimestampUs_;
            uint32_t resumeIntervalMs = resumeIntervalUs / 1000U;
            resumeLastImpactTimestampUs_ = impactTimestampUs;

            bool resumeIntervalValid = (resumeIntervalUs != 0 &&
                                        resumeIntervalMs >= config_.minimumStepIntervalMs &&
                                        resumeIntervalMs <= config_.maximumStepIntervalMs);

            if (resumeIntervalValid) {
                if (provisionalIntervalCount_ < 2) {
                    provisionalIntervals_[provisionalIntervalCount_++] = resumeIntervalMs;
                }
                if (provisionalResumeSteps_ == 0) {
                    provisionalResumeSteps_ = 2; // Qualify pair
                } else {
                    provisionalResumeSteps_++;
                }

                if (provisionalResumeSteps_ >= config_.sideRailResumeQualifiedSteps) {
                    bool resumeRegular = true;
                    if (provisionalIntervalCount_ >= 2) {
                        float mapd = 0.0f;
                        uint32_t mean = 0;
                        if (computeMapd(provisionalIntervals_, provisionalIntervalCount_, mapd, mean)) {
                            resumeRegular = (mapd <= config_.maximumActiveStepVariationPct);
                        } else {
                            resumeRegular = false;
                        }
                    }

                    bool beltMoving = (speedInputValid_ && beltSpeedKmh >= config_.beltMovingThresholdKmh);

                    if (resumeRegular && !thresholdSaturated_ && imuInputValid_ && speedInputValid_ &&
                        beltMoving && timestampValid) {
                        sessionStepCount_ += provisionalResumeSteps_;
                        lifetimeQualifiedStepCount_ += provisionalResumeSteps_;
                        qualifiedActiveSteps_ = provisionalResumeSteps_;

                        for (uint8_t i = 0; i < provisionalIntervalCount_; ++i) {
                            recordInterval(provisionalIntervals_[i]);
                        }

                        if (qualifiedActiveSteps_ >= config_.minQualifiedStepsForCadence) {
                            float instCadence = 60000.0f / resumeIntervalMs;
                            instantaneousCadenceSpm_ = instCadence;
                            smoothedCadenceSpm_ = instCadence;
                            cadenceValid_ = true;
                        }

                        lastStepTimestampMs_ = appTimestampMs;
                        hasValidStepApplicationTimestamp_ = true;
                        lastStepAgeMs_ = 0;

                        setPresence(RunnerPresence::Active);
                        speedCreditEnabled_ = true;
                        runnerSpeedValid_ = true;
                        runnerSpeedKmh_ = beltSpeedKmh;
                        distanceAccumulationEnabled_ = true;

                        previousEndpointValid_ = false;
                        distanceIntegrationInitialized_ = false;
                        lastDistanceIntegrationMs_ = nowMs;

                        inResumeQualification_ = false;
                        resumeApplicationTimeValid_ = false;
                        uint8_t stepsToAdvance = provisionalResumeSteps_;
                        provisionalResumeSteps_ = 0;
                        provisionalIntervalCount_ = 0;

                        RunnerActivity cand = evaluateCurrentCandidate(beltSpeedKmh);
                        advanceActivityEvidence(cand, stepsToAdvance, appTimestampMs, nowMs);
                    }
                }
            } else {
                rejectedIntervalCount_++;
                inResumeQualification_ = true;
                resumeAnchorTimestampUs_ = impactTimestampUs;
                resumeAnchorApplicationMs_ = appTimestampMs;
                resumeApplicationTimeValid_ = true;
                resumeLastImpactTimestampUs_ = impactTimestampUs;
                provisionalResumeSteps_ = 0;
                provisionalIntervalCount_ = 0;
            }
            return;
        }

        // Active / Candidate Progression
        if (!hasImpactAnchor_) {
            lastImpactTimestampUs_ = impactTimestampUs;
            hasImpactAnchor_ = true;
            if (timestampValid && (presence_ == RunnerPresence::Unknown || presence_ == RunnerPresence::Inactive)) {
                setPresence(RunnerPresence::Candidate);
                qualifiedActiveSteps_ = 0;
                candidateStartTimestampMs_ = appTimestampMs;
                candidateStartTimeValid_ = true;
            }
            return;
        }

        uint32_t intervalUs = impactTimestampUs - lastImpactTimestampUs_;
        lastImpactTimestampUs_ = impactTimestampUs;
        uint32_t intervalMs = intervalUs / 1000U;

        bool intervalPlasible = (intervalUs != 0 &&
                                 intervalMs >= config_.minimumStepIntervalMs &&
                                 intervalMs <= config_.maximumStepIntervalMs);

        if (!intervalPlasible) {
            rejectedIntervalCount_++;
            if (presence_ == RunnerPresence::Candidate) {
                qualifiedActiveSteps_ = 0;
            }
            return;
        }

        recordInterval(intervalMs);

        if (presence_ == RunnerPresence::Candidate || presence_ == RunnerPresence::Unknown || presence_ == RunnerPresence::Inactive) {
            if (qualifiedActiveSteps_ == 0) {
                qualifiedActiveSteps_ = 2; // Qualify first pair
                if (timestampValid) {
                    if (!candidateStartTimeValid_) {
                        candidateStartTimestampMs_ = appTimestampMs;
                        candidateStartTimeValid_ = true;
                    }
                    if (presence_ != RunnerPresence::Candidate) {
                        setPresence(RunnerPresence::Candidate);
                    }
                }
            } else {
                qualifiedActiveSteps_++;
            }

            if (timestampValid) {
                lastStepTimestampMs_ = appTimestampMs;
                hasValidStepApplicationTimestamp_ = true;
                lastStepAgeMs_ = 0;
            }

            if (qualifiedActiveSteps_ >= config_.minQualifiedStepsForCadence) {
                float instCadence = 60000.0f / intervalMs;
                instantaneousCadenceSpm_ = instCadence;
                smoothedCadenceSpm_ = instCadence;
                cadenceValid_ = true;
            }

            bool beltMoving = (speedInputValid_ && beltSpeedKmh >= config_.beltMovingThresholdKmh);

            if (qualifiedActiveSteps_ >= config_.minQualifiedStepsForActive &&
                stepRegularityValid_ && !thresholdSaturated_ && imuInputValid_ && speedInputValid_ &&
                beltMoving && timestampValid && hasValidStepApplicationTimestamp_ && candidateStartTimeValid_) {
                const uint8_t committedInitialSteps = qualifiedActiveSteps_;
                sessionStepCount_ += committedInitialSteps;
                lifetimeQualifiedStepCount_ += committedInitialSteps;
                setPresence(RunnerPresence::Active);
                speedCreditEnabled_ = true;
                runnerSpeedValid_ = true;
                runnerSpeedKmh_ = beltSpeedKmh;
                distanceAccumulationEnabled_ = true;

                previousEndpointValid_ = false;
                distanceIntegrationInitialized_ = false;
                lastDistanceIntegrationMs_ = nowMs;
                candidateStartTimeValid_ = false;

                RunnerActivity cand = evaluateCurrentCandidate(beltSpeedKmh);
                if (timestampValid) {
                    advanceActivityEvidence(cand, committedInitialSteps, appTimestampMs, nowMs);
                }
            }
            return;
        }

        if (presence_ == RunnerPresence::Active) {
            sessionStepCount_++;
            lifetimeQualifiedStepCount_++;

            if (timestampValid) {
                lastStepTimestampMs_ = appTimestampMs;
                hasValidStepApplicationTimestamp_ = true;
                lastStepAgeMs_ = 0;
            }

            float instCadence = 60000.0f / intervalMs;
            instantaneousCadenceSpm_ = instCadence;
            if (cadenceValid_) {
                smoothedCadenceSpm_ = (smoothedCadenceSpm_ * (1.0f - config_.cadenceSmoothingAlpha)) +
                                      (instCadence * config_.cadenceSmoothingAlpha);
            } else if (qualifiedActiveSteps_ >= config_.minQualifiedStepsForCadence) {
                smoothedCadenceSpm_ = instCadence;
                cadenceValid_ = true;
            }

            RunnerActivity cand = evaluateCurrentCandidate(beltSpeedKmh);
            if (timestampValid) {
                advanceActivityEvidence(cand, 1, appTimestampMs, nowMs);
            }
        }
    }

    void updateTimeoutsAndAbsence(uint32_t nowMs, float beltSpeedKmh) {
        const bool beltMoving = (speedInputValid_ && beltSpeedKmh >= config_.beltMovingThresholdKmh);

        if (hasValidStepApplicationTimestamp_) {
            lastStepAgeMs_ = nowMs - lastStepTimestampMs_;
        }

        if (inResumeQualification_ && resumeApplicationTimeValid_) {
            if (nowMs - resumeAnchorApplicationMs_ > config_.sideRailResumeTimeoutMs) {
                inResumeQualification_ = false;
                resumeApplicationTimeValid_ = false;
                provisionalResumeSteps_ = 0;
                provisionalIntervalCount_ = 0;
            }
        }

        // Candidate Timeout
        if (presence_ == RunnerPresence::Candidate) {
            if (candidateStartTimeValid_) {
                if (nowMs - candidateStartTimestampMs_ > config_.candidateTimeoutMs) {
                    qualifiedActiveSteps_ = 0;
                    candidateStartTimeValid_ = false;
                    lastImpactTimestampUs_ = 0;
                    hasImpactAnchor_ = false;
                    validIntervalCount_ = 0;
                    if (speedInputValid_ && !beltMoving) {
                        setPresence(RunnerPresence::Inactive);
                        setActivity(RunnerActivity::Inactive, nowMs);
                    } else {
                        setPresence(RunnerPresence::Unknown);
                        setActivity(RunnerActivity::Unknown, nowMs);
                    }
                }
            } else {
                // Candidate without valid start time: defensive recovery
                setPresence(RunnerPresence::Unknown);
                setActivity(RunnerActivity::Unknown, nowMs);
                qualifiedActiveSteps_ = 0;
                candidateStartTimeValid_ = false;
                lastImpactTimestampUs_ = 0;
                hasImpactAnchor_ = false;
                validIntervalCount_ = 0;
                cadenceValid_ = false;
            }
        }

        if (presence_ == RunnerPresence::Active) {
            if (!imuInputValid_ || !speedInputValid_) {
                // If mandatory input becomes invalid, do not declare SideRails; transition to Unknown
                setPresence(RunnerPresence::Unknown);
                setActivity(RunnerActivity::Unknown, nowMs);
                runnerSpeedKmh_ = 0.0f;
                runnerSpeedValid_ = false;
                speedCreditEnabled_ = false;
                distanceAccumulationEnabled_ = false;
                distanceIntegrationInitialized_ = false;
                previousEndpointValid_ = false;
            } else if (hasValidStepApplicationTimestamp_ && lastStepAgeMs_ > config_.sideRailCreditGraceMs && beltMoving) {
                setPresence(RunnerPresence::SideRailsCandidate);
                runnerSpeedKmh_ = 0.0f;
                runnerSpeedValid_ = false;
                speedCreditEnabled_ = false;
                distanceAccumulationEnabled_ = false;
                distanceIntegrationInitialized_ = false;
                previousEndpointValid_ = false;
                distancePauseReason = RunnerDistancePauseReason::SideRailsCandidate;
            } else if (hasValidStepApplicationTimestamp_ && lastStepAgeMs_ > config_.activeRunnerTimeoutMs && !beltMoving) {
                setPresence(RunnerPresence::Inactive);
                setActivity(RunnerActivity::Inactive, nowMs);
                runnerSpeedKmh_ = 0.0f;
                runnerSpeedValid_ = false;
                speedCreditEnabled_ = false;
                distanceAccumulationEnabled_ = false;
                distanceIntegrationInitialized_ = false;
                previousEndpointValid_ = false;
                distancePauseReason = RunnerDistancePauseReason::Inactive;
            }
        } else if (presence_ == RunnerPresence::SideRailsCandidate) {
            if (!imuInputValid_ || !speedInputValid_) {
                setPresence(RunnerPresence::Unknown);
                setActivity(RunnerActivity::Unknown, nowMs);
                runnerSpeedKmh_ = 0.0f;
                runnerSpeedValid_ = false;
                speedCreditEnabled_ = false;
                distanceAccumulationEnabled_ = false;
                distanceIntegrationInitialized_ = false;
                previousEndpointValid_ = false;
                distancePauseReason = RunnerDistancePauseReason::UnknownState;
            } else if (hasValidStepApplicationTimestamp_ && lastStepAgeMs_ >= config_.sideRailTimeoutMs && beltMoving) {
                setPresence(RunnerPresence::SideRails);
                setActivity(RunnerActivity::SideRails, nowMs);
                distancePauseReason = RunnerDistancePauseReason::SideRails;
            } else if (!beltMoving) {
                setPresence(RunnerPresence::Inactive);
                setActivity(RunnerActivity::Inactive, nowMs);
                distancePauseReason = RunnerDistancePauseReason::BeltStopped;
            }
        } else if (presence_ == RunnerPresence::SideRails) {
            if (!imuInputValid_ || !speedInputValid_) {
                setPresence(RunnerPresence::Unknown);
                setActivity(RunnerActivity::Unknown, nowMs);
                runnerSpeedKmh_ = 0.0f;
                runnerSpeedValid_ = false;
                speedCreditEnabled_ = false;
                distanceAccumulationEnabled_ = false;
                distanceIntegrationInitialized_ = false;
                previousEndpointValid_ = false;
                distancePauseReason = RunnerDistancePauseReason::UnknownState;
            } else if (!beltMoving) {
                setPresence(RunnerPresence::Inactive);
                setActivity(RunnerActivity::Inactive, nowMs);
                distancePauseReason = RunnerDistancePauseReason::BeltStopped;
            }
        }

        if (hasValidStepApplicationTimestamp_ && lastStepAgeMs_ > config_.cadenceInvalidTimeoutMs) {
            cadenceValid_ = false;
        }

        evaluateActivityHold(nowMs);
    }

    void updateDistanceIntegration(uint32_t nowMs, float beltSpeedKmh) {
        if (!speedInputValid_) {
            distancePauseReason = RunnerDistancePauseReason::SpeedInvalid;
        } else if (!imuInputValid_) {
            distancePauseReason = RunnerDistancePauseReason::ImuInvalid;
        } else if (beltSpeedKmh < config_.beltMovingThresholdKmh) {
            distancePauseReason = RunnerDistancePauseReason::BeltStopped;
        } else if (presence_ == RunnerPresence::SideRailsCandidate) {
            distancePauseReason = RunnerDistancePauseReason::SideRailsCandidate;
        } else if (presence_ == RunnerPresence::SideRails) {
            distancePauseReason = RunnerDistancePauseReason::SideRails;
        } else if (presence_ == RunnerPresence::Candidate) {
            distancePauseReason = RunnerDistancePauseReason::Candidate;
        } else if (presence_ == RunnerPresence::Inactive) {
            distancePauseReason = RunnerDistancePauseReason::Inactive;
        } else if (presence_ == RunnerPresence::Unknown) {
            distancePauseReason = RunnerDistancePauseReason::UnknownState;
        } else {
            distancePauseReason = RunnerDistancePauseReason::None;
        }

        if (presence_ == RunnerPresence::Active && speedCreditEnabled_ && speedInputValid_ &&
            (beltSpeedKmh >= config_.beltMovingThresholdKmh)) {
            runnerSpeedKmh_ = beltSpeedKmh;
            runnerSpeedValid_ = true;
            distanceAccumulationEnabled_ = true;
        } else if (presence_ == RunnerPresence::SideRails || presence_ == RunnerPresence::SideRailsCandidate) {
            runnerSpeedKmh_ = 0.0f;
            runnerSpeedValid_ = false;
            distanceAccumulationEnabled_ = false;
        } else {
            runnerSpeedKmh_ = 0.0f;
            runnerSpeedValid_ = false;
            distanceAccumulationEnabled_ = false;
        }

        bool currentEndpointValid = (distanceAccumulationEnabled_ && runnerSpeedValid_ &&
                                     presence_ == RunnerPresence::Active && inputDataValid_ &&
                                     distancePauseReason == RunnerDistancePauseReason::None);

        if (!distanceIntegrationInitialized_) {
            lastDistanceIntegrationMs_ = nowMs;
            previousRunnerSpeedKmh_ = runnerSpeedKmh_;
            previousEndpointValid_ = currentEndpointValid;
            distanceIntegrationInitialized_ = true;
            lastDistanceDeltaKm_ = 0.0;
            return;
        }

        uint32_t dtMs = nowMs - lastDistanceIntegrationMs_;
        lastDistanceIntegrationMs_ = nowMs;

        if (dtMs > 0 && dtMs <= config_.maximumDistanceIntegrationIntervalMs &&
            previousEndpointValid_ && currentEndpointValid) {
            double avgSpeedKmh = 0.5 * (previousRunnerSpeedKmh_ + runnerSpeedKmh_);
            double deltaKm = (avgSpeedKmh * static_cast<double>(dtMs)) / 3600000.0;
            validatedDistanceKm_ += deltaKm;
            lastDistanceDeltaKm_ = deltaKm;
        } else {
            lastDistanceDeltaKm_ = 0.0;
        }

        previousRunnerSpeedKmh_ = runnerSpeedKmh_;
        previousEndpointValid_ = currentEndpointValid;
    }

    void updateConfidence(const CsafeState& csafeState) {
        if (!inputDataValid_) {
            confidence_ = RunnerDynamicsConfidence::Unavailable;
            return;
        }
        if (thresholdSaturated_) {
            confidence_ = RunnerDynamicsConfidence::Low;
            return;
        }

        bool csafeMatches = false;
        if (csafeInputValid_) {
            if (presence_ == RunnerPresence::Active &&
                (csafeState.reportedState == CsafeMachineState::InUse ||
                 csafeState.reportedState == CsafeMachineState::Starting)) {
                csafeMatches = true;
            } else if ((presence_ == RunnerPresence::SideRails || presence_ == RunnerPresence::Inactive) &&
                       (csafeState.reportedState == CsafeMachineState::Paused ||
                        csafeState.reportedState == CsafeMachineState::Ready ||
                        csafeState.reportedState == CsafeMachineState::Idle)) {
                csafeMatches = true;
            }
        }

        if (presence_ == RunnerPresence::Active && stepRegularityValid_) {
            confidence_ = csafeMatches ? RunnerDynamicsConfidence::High : RunnerDynamicsConfidence::Medium;
        } else if (presence_ == RunnerPresence::Active) {
            confidence_ = RunnerDynamicsConfidence::Medium;
        } else if (presence_ == RunnerPresence::SideRails || presence_ == RunnerPresence::SideRailsCandidate) {
            confidence_ = RunnerDynamicsConfidence::Medium;
        } else {
            confidence_ = RunnerDynamicsConfidence::Low;
        }
    }

    void update(const ImuSample* samples,
                size_t sampleCount,
                const SpeedSensorState& speedState,
                const CsafeState& csafeState,
                uint32_t nowMs) {
        if (opMutex == nullptr || xSemaphoreTake(opMutex, 0) != pdTRUE) {
            return;
        }

        if (!initialized_ || !active_) {
            xSemaphoreGive(opMutex);
            return;
        }

        lastUpdateMs_ = nowMs;

        // Speed Sensor Validation
        beltSpeedKmh_ = speedState.speedKmh;
        speedInputValid_ = isSpeedStateValid(speedState);
        csafeInputValid_ = (csafeState.initialized && csafeState.online && csafeState.machineStateFresh);

        // IMU Block Handling
        if (samples == nullptr && sampleCount > 0) {
            invalidSampleBlockCount_++;
            resetDiscontinuityInternal();
            updateTimeoutsAndAbsence(nowMs, beltSpeedKmh_);
            updateDistanceIntegration(nowMs, beltSpeedKmh_);
            updateConfidence(csafeState);
            commitSnapshot(nowMs);
            xSemaphoreGive(opMutex);
            return;
        }

        if (sampleCount == 0) {
            // Empty update preserves valid IMU data up to imuDataMaxAgeMs
            if (hasValidImuData_ && (nowMs - lastValidImuDataMs_ <= config_.imuDataMaxAgeMs)) {
                imuInputValid_ = true;
            } else {
                imuInputValid_ = false;
            }
            inputDataValid_ = (imuInputValid_ && speedInputValid_);

            updateTimeoutsAndAbsence(nowMs, beltSpeedKmh_);
            updateDistanceIntegration(nowMs, beltSpeedKmh_);
            updateConfidence(csafeState);
            commitSnapshot(nowMs);
            xSemaphoreGive(opMutex);
            return;
        }

        if (sampleCount > config_.maximumSamplesPerUpdate) {
            oversizedInputBlockCount_++;
            resetDiscontinuityInternal();
            updateTimeoutsAndAbsence(nowMs, beltSpeedKmh_);
            updateDistanceIntegration(nowMs, beltSpeedKmh_);
            updateConfidence(csafeState);
            commitSnapshot(nowMs);
            xSemaphoreGive(opMutex);
            return;
        }

        // Conservative Invalid-Sample Policy: Any invalid sample rejects entire block
        uint32_t blockInvalidCount = 0;
        for (size_t i = 0; i < sampleCount; ++i) {
            const ImuSample& s = samples[i];
            bool sampleFinite = (std::isfinite(s.accelVerticalG) &&
                                 std::isfinite(s.accelLongitudinalG) &&
                                 std::isfinite(s.accelLateralG));
            if (!s.accelValid || !sampleFinite) {
                blockInvalidCount++;
            }
        }

        if (blockInvalidCount > 0) {
            invalidSampleCount_ += blockInvalidCount;
            invalidSampleBlockCount_++;
            resetDiscontinuityInternal();
            updateTimeoutsAndAbsence(nowMs, beltSpeedKmh_);
            updateDistanceIntegration(nowMs, beltSpeedKmh_);
            updateConfidence(csafeState);
            commitSnapshot(nowMs);
            xSemaphoreGive(opMutex);
            return;
        }

        // Entire block is guaranteed valid and finite
        uint32_t firstValidUs = samples[0].timestampUs;
        uint32_t newestValidUs = samples[sampleCount - 1].timestampUs;
        bool streamContinuous = true;
        bool timestampDiscontinuityDetected = false;
        bool sequenceDiscontinuityDetected = false;

        if (hasPreviousSample_) {
            uint32_t crossGapUs = samples[0].timestampUs - lastSampleTimestampUs_;
            uint32_t seqDelta = samples[0].sequence - lastSampleSequence_;
            if (crossGapUs == 0 || crossGapUs > config_.maxSampleTimestampGapUs) {
                timestampDiscontinuityDetected = true;
            }
            if (seqDelta != 1U) {
                sequenceDiscontinuityDetected = true;
            }
        }

        for (size_t i = 1; i < sampleCount; ++i) {
            uint32_t gapUs = samples[i].timestampUs - samples[i - 1].timestampUs;
            uint32_t seqDelta = samples[i].sequence - samples[i - 1].sequence;
            if (gapUs == 0 || gapUs > config_.maxSampleTimestampGapUs) {
                timestampDiscontinuityDetected = true;
            }
            if (seqDelta != 1U) {
                sequenceDiscontinuityDetected = true;
            }
        }

        if (timestampDiscontinuityDetected) {
            timestampDiscontinuityCount_++;
        }
        if (sequenceDiscontinuityDetected) {
            sequenceDiscontinuityCount_++;
        }
        if (timestampDiscontinuityDetected || sequenceDiscontinuityDetected) {
            streamDiscontinuityCount_++;
            streamContinuous = false;
        }

        if (newestValidUs - firstValidUs > config_.maximumInputBlockSpanUs) {
            invalidSampleBlockCount_++;
            streamContinuous = false;
        }

        if (!streamContinuous) {
            resetDiscontinuityInternal();
            updateTimeoutsAndAbsence(nowMs, beltSpeedKmh_);
            updateDistanceIntegration(nowMs, beltSpeedKmh_);
            updateConfidence(csafeState);
            commitSnapshot(nowMs);
            xSemaphoreGive(opMutex);
            return;
        }

        // Mark IMU data fresh & valid
        lastValidImuDataMs_ = nowMs;
        hasValidImuData_ = true;
        imuInputValid_ = true;
        inputDataValid_ = (imuInputValid_ && speedInputValid_);

        // Process all samples in the verified block
        for (size_t i = 0; i < sampleCount; ++i) {
            const ImuSample& s = samples[i];
            processedSampleCount_++;

            // 1. Select raw treadmill-coordinate signal
            float rawG = 0.0f;
            switch (config_.signalAxis) {
                case RunnerSignalAxis::Vertical: rawG = s.accelVerticalG; break;
                case RunnerSignalAxis::Longitudinal: rawG = s.accelLongitudinalG; break;
                case RunnerSignalAxis::Lateral: rawG = s.accelLateralG; break;
                case RunnerSignalAxis::AccelerationMagnitude:
                    rawG = std::sqrt(s.accelLongitudinalG * s.accelLongitudinalG +
                                     s.accelLateralG * s.accelLateralG +
                                     s.accelVerticalG * s.accelVerticalG);
                    break;
            }
            rawSignalG_ = rawG;

            // 2. Baseline Initialization / Update
            if (!baselineInitialized_) {
                baselineG_ = rawG;
                dynamicSignalG_ = 0.0f;
                baselineInitialized_ = true;
                lastSampleTimestampUs_ = s.timestampUs;
                lastSampleSequence_ = s.sequence;
                hasPreviousSample_ = true;
                continue; // Do not feed initializing sample to peak logic
            }

            uint32_t sampleDeltaUs = s.timestampUs - lastSampleTimestampUs_;
            lastSampleTimestampUs_ = s.timestampUs;
            lastSampleSequence_ = s.sequence;
            hasPreviousSample_ = true;

            float dtSec = static_cast<float>(sampleDeltaUs) / 1000000.0f;
            float baseTauSec = config_.baselineTimeConstantMs / 1000.0f;
            float alphaBase = dtSec / (dtSec + baseTauSec);
            alphaBase = std::max(0.00001f, std::min(alphaBase, 1.0f));

            baselineG_ += alphaBase * (rawG - baselineG_);

            // 3. Signed dynamic acceleration
            float signedDynamicG = rawG - baselineG_;

            // 4. Apply Polarity
            switch (config_.signalPolarity) {
                case RunnerSignalPolarity::Positive: dynamicSignalG_ = signedDynamicG; break;
                case RunnerSignalPolarity::Negative: dynamicSignalG_ = -signedDynamicG; break;
                case RunnerSignalPolarity::Absolute: dynamicSignalG_ = std::abs(signedDynamicG); break;
            }

            // 5. Adaptive Noise Floor (Freeze on threshold crossing or active peak state)
            bool thresholdCrossed = (dynamicSignalG_ >= effectiveThresholdG_);
            if (!thresholdCrossed && peakState_ == RunnerPeakDetectorState::Idle) {
                float noiseTauSec = config_.noiseFloorTimeConstantMs / 1000.0f;
                float alphaNoise = dtSec / (dtSec + noiseTauSec);
                alphaNoise = std::max(0.00001f, std::min(alphaNoise, 1.0f));
                float dynamicMagnitudeG = std::abs(dynamicSignalG_);
                noiseFloorG_ += alphaNoise * (dynamicMagnitudeG - noiseFloorG_);
                noiseFloorG_ = std::max(0.0f, std::min(noiseFloorG_, config_.maximumNoiseFloorG));
            }

            // 6. Effective Threshold & Saturation
            float rawThreshold = std::max(config_.minimumImpactThresholdG, noiseFloorG_ * config_.noiseMultiplier);
            if (rawThreshold >= config_.maximumEffectiveThresholdG) {
                if (!thresholdSaturated_) {
                    thresholdSaturated_ = true;
                    thresholdSaturationCount_++;
                }
                effectiveThresholdG_ = config_.maximumEffectiveThresholdG;
            } else {
                thresholdSaturated_ = false;
                effectiveThresholdG_ = std::max(config_.minimumImpactThresholdG, rawThreshold);
            }

            // 7. Peak Detector State Machine (Dedicated lastEmittedPeakTimestampUs_ for refractory)
            uint32_t currentSampleUs = s.timestampUs;
            switch (peakState_) {
                case RunnerPeakDetectorState::Idle:
                    if (dynamicSignalG_ > effectiveThresholdG_) {
                        peakState_ = RunnerPeakDetectorState::Rising;
                        peakValueG_ = dynamicSignalG_;
                        peakTimestampUs_ = currentSampleUs;
                    }
                    break;

                case RunnerPeakDetectorState::Rising:
                    if (dynamicSignalG_ > peakValueG_) {
                        peakValueG_ = dynamicSignalG_;
                        peakTimestampUs_ = currentSampleUs;
                    } else if (dynamicSignalG_ < (peakValueG_ * 0.85f)) {
                        peakState_ = RunnerPeakDetectorState::Falling;
                    }
                    break;

                case RunnerPeakDetectorState::Falling:
                    if (dynamicSignalG_ < (effectiveThresholdG_ * 0.5f)) {
                        lastPeakMagnitudeG_ = peakValueG_;
                        lastEmittedPeakTimestampUs_ = peakTimestampUs_;
                        peakState_ = RunnerPeakDetectorState::Refractory;
                        processQualifiedImpact(peakTimestampUs_, nowMs, newestValidUs, streamContinuous, beltSpeedKmh_);
                    }
                    break;

                case RunnerPeakDetectorState::Refractory:
                    if (currentSampleUs - lastEmittedPeakTimestampUs_ >= (config_.refractoryIntervalMs * 1000U)) {
                        peakState_ = RunnerPeakDetectorState::Idle;
                    }
                    break;
            }
        }

        updateTimeoutsAndAbsence(nowMs, beltSpeedKmh_);
        updateDistanceIntegration(nowMs, beltSpeedKmh_);
        updateConfidence(csafeState);

        commitSnapshot(nowMs);
        xSemaphoreGive(opMutex);
    }

    RunnerDynamicsState getState() const {
        RunnerDynamicsState snap;
        portENTER_CRITICAL(&stateMux);
        snap = publicState_;
        portEXIT_CRITICAL(&stateMux);
        return snap;
    }

    RunnerDynamicsConfig configSnapshot() const {
        RunnerDynamicsConfig cfg;
        portENTER_CRITICAL(&stateMux);
        cfg = publicConfig_;
        portEXIT_CRITICAL(&stateMux);
        return cfg;
    }

    bool isReady() const {
        bool r = false;
        portENTER_CRITICAL(&stateMux);
        r = publicState_.initialized;
        portEXIT_CRITICAL(&stateMux);
        return r;
    }
};

RunnerDynamics::RunnerDynamics() : impl_(new Impl()) {}

RunnerDynamics::~RunnerDynamics() {
    if (impl_ != nullptr) {
        if (impl_->opMutex != nullptr) {
            xSemaphoreTake(impl_->opMutex, portMAX_DELAY);
            impl_->initialized_ = false;
            impl_->active_ = false;
            impl_->commitSnapshot(0);
            xSemaphoreGive(impl_->opMutex);
        }
        delete impl_;
        impl_ = nullptr;
    }
}

bool RunnerDynamics::begin(const RunnerDynamicsConfig& config) {
    if (impl_ == nullptr) return false;
    return impl_->begin(config);
}

void RunnerDynamics::end() {
    if (impl_ != nullptr) {
        impl_->end();
    }
}

void RunnerDynamics::update(const ImuSample* samples,
                            size_t sampleCount,
                            const SpeedSensorState& speedState,
                            const CsafeState& csafeState,
                            uint32_t nowMs) {
    if (impl_ != nullptr) {
        impl_->update(samples, sampleCount, speedState, csafeState, nowMs);
    }
}

RunnerDynamicsState RunnerDynamics::getState() const {
    if (impl_ == nullptr) return RunnerDynamicsState{};
    return impl_->getState();
}

RunnerDynamicsConfig RunnerDynamics::configSnapshot() const {
    if (impl_ == nullptr) return RunnerDynamicsConfig{};
    return impl_->configSnapshot();
}

void RunnerDynamics::resetStepCounter() {
    if (impl_ != nullptr) {
        impl_->resetStepCounter();
    }
}

void RunnerDynamics::resetValidatedDistance() {
    if (impl_ != nullptr) {
        impl_->resetValidatedDistance();
    }
}

void RunnerDynamics::resetSession() {
    if (impl_ != nullptr) {
        impl_->resetSession();
    }
}

bool RunnerDynamics::isReady() const {
    if (impl_ == nullptr) return false;
    return impl_->isReady();
}

const char* RunnerDynamics::version() {
    return "1.0.0";
}

} // namespace stridecontrol
