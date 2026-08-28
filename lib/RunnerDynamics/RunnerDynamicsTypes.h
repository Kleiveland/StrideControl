#pragma once

#include <stdint.h>

namespace stridecontrol {

enum class RunnerDynamicsStatus : uint8_t {
    Uninitialized,
    Ready,
    Tracking,
    DegradedInput
};

enum class RunnerPresence : uint8_t {
    Unknown,
    Inactive,
    Candidate,
    Active,
    SideRailsCandidate,
    SideRails
};

enum class RunnerActivity : uint8_t {
    Unknown,
    Inactive,
    Walking,
    Running,
    SideRails
};

enum class RunnerDynamicsConfidence : uint8_t {
    Unavailable,
    Low,
    Medium,
    High
};

enum class RunnerDistancePauseReason : uint8_t {
    None,
    BeltStopped,
    SpeedInvalid,
    ImuInvalid,
    Inactive,
    Candidate,
    SideRailsCandidate,
    SideRails,
    UnknownState
};

enum class RunnerSignalAxis : uint8_t {
    Vertical,
    Longitudinal,
    Lateral,
    AccelerationMagnitude
};

enum class RunnerSignalPolarity : uint8_t {
    Positive,
    Negative,
    Absolute
};

enum class RunnerPeakDetectorState : uint8_t {
    Idle,
    Rising,
    Falling,
    Refractory
};

struct RunnerDynamicsState {
    bool initialized = false;
    RunnerDynamicsStatus status = RunnerDynamicsStatus::Uninitialized;
    uint32_t snapshotTimestampMs = 0;

    // Presence & Activity
    RunnerPresence presence = RunnerPresence::Unknown;
    RunnerActivity activity = RunnerActivity::Unknown;
    RunnerDynamicsConfidence confidence = RunnerDynamicsConfidence::Unavailable;
    bool sideRailsCandidate = false;
    bool activePresence = false;
    bool onSideRails = false;

    // Speed
    float beltSpeedKmh = 0.0f;
    float runnerSpeedKmh = 0.0f;
    bool runnerSpeedValid = false;
    bool speedCreditEnabled = false;

    // Cadence & Steps
    float instantaneousCadenceSpm = 0.0f;
    float smoothedCadenceSpm = 0.0f;
    bool cadenceValid = false;
    uint32_t sessionStepCount = 0;
    uint32_t lifetimeQualifiedStepCount = 0;
    uint32_t detectedImpactCount = 0;
    uint32_t lastStepTimestampMs = 0;
    uint32_t lastStepAgeMs = 0;
    uint32_t lastStepIntervalMs = 0;
    uint32_t meanStepIntervalMs = 0;
    float stepIntervalVariationPct = 0.0f;
    bool stepRegularityValid = false;

    // Signal & Noise / Thresholds
    float rawSignalG = 0.0f;
    float baselineG = 0.0f;
    float dynamicSignalG = 0.0f;
    float noiseFloorG = 0.0f;
    float effectiveThresholdG = 0.0f;
    bool thresholdSaturated = false;
    uint32_t thresholdSaturationCount = 0;
    RunnerPeakDetectorState peakDetectorState = RunnerPeakDetectorState::Idle;
    float lastPeakMagnitudeG = 0.0f;

    // Distance
    double validatedDistanceKm = 0.0;
    double lastDistanceDeltaKm = 0.0;
    bool distanceAccumulationEnabled = false;
    bool distanceValid = false;
    RunnerDistancePauseReason distancePauseReason = RunnerDistancePauseReason::UnknownState;

    // Input Validity & Freshness
    bool imuInputValid = false;
    uint32_t imuDataAgeMs = UINT32_MAX;
    bool speedInputValid = false;
    bool csafeInputValid = false;
    bool inputDataValid = false;
    uint8_t provisionalResumeStepCount = 0;

    // Diagnostic & Stream Counters
    uint32_t invalidSampleCount = 0;
    uint32_t invalidSampleBlockCount = 0;
    uint32_t oversizedInputBlockCount = 0;
    uint32_t timestampDiscontinuityCount = 0;
    uint32_t sequenceDiscontinuityCount = 0;
    uint32_t streamDiscontinuityCount = 0;
    uint32_t applicationTimestampRejectionCount = 0;
    uint32_t rejectedIntervalCount = 0;
    uint32_t processedSampleCount = 0;
    uint32_t presenceTransitionCount = 0;
    uint32_t activityTransitionCount = 0;
};

const char* runnerDynamicsStatusName(RunnerDynamicsStatus status);
const char* runnerPresenceName(RunnerPresence presence);
const char* runnerActivityName(RunnerActivity activity);
const char* runnerConfidenceName(RunnerDynamicsConfidence conf);
const char* runnerDistancePauseReasonName(RunnerDistancePauseReason reason);
const char* runnerPeakDetectorStateName(RunnerPeakDetectorState state);

} // namespace stridecontrol