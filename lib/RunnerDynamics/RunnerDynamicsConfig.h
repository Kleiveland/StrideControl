#pragma once

#include <stdint.h>
#include <stddef.h>
#include "RunnerDynamicsTypes.h"

namespace stridecontrol {

struct RunnerDynamicsConfig {
    // Signal selection & Polarity
    RunnerSignalAxis signalAxis = RunnerSignalAxis::Vertical;
    RunnerSignalPolarity signalPolarity = RunnerSignalPolarity::Positive;

    // Baseline removal filter
    float baselineTimeConstantMs = 300.0f;
    uint32_t maxSampleTimestampGapUs = 500000;

    // Noise floor & Adaptive threshold limits
    float noiseFloorTimeConstantMs = 2000.0f;
    float initialNoiseFloorG = 0.01f;
    float minimumImpactThresholdG = 0.03f;
    float noiseMultiplier = 3.0f;
    float maximumNoiseFloorG = 1.0f;
    float maximumEffectiveThresholdG = 2.0f;

    // Step timing bounds & Refractory
    uint32_t refractoryIntervalMs = 220;
    uint32_t minimumStepIntervalMs = 240;
    uint32_t maximumStepIntervalMs = 1500;

    // Qualification & Cadence parameters
    uint8_t minQualifiedStepsForCadence = 3;
    uint8_t minQualifiedStepsForActive = 4;
    float cadenceSmoothingAlpha = 0.25f;
    uint32_t cadenceInvalidTimeoutMs = 1500;
    float maximumActiveStepVariationPct = 20.0f;

    // Timeouts & Candidate / Side-rail behavior
    uint32_t candidateTimeoutMs = 2500;
    uint32_t activeRunnerTimeoutMs = 1500;
    uint32_t sideRailCreditGraceMs = 750;
    uint32_t sideRailTimeoutMs = 2500;
    uint8_t sideRailResumeQualifiedSteps = 3;
    uint32_t sideRailResumeTimeoutMs = 3000;

    // Activity classification
    float walkingCadenceMinimumSpm = 70.0f;
    float walkingCadenceMaximumSpm = 145.0f;
    float runningCadenceMinimumSpm = 125.0f;
    float runningCadenceMaximumSpm = 230.0f;
    float walkingSpeedSupportMaximumKmh = 5.0f;
    float runningSpeedSupportMinimumKmh = 8.5f;
    uint8_t classificationConfirmationStepCount = 3;
    uint32_t classificationHoldMs = 1500;

    // Speed & Distance integration
    float beltMovingThresholdKmh = 0.5f;
    uint32_t speedDataMaxAgeMs = 1000;       // Floor only - see isSpeedStateValid() for the dynamic, speed-aware check
    float speedSensorKmhPerHz = 1.1148f;     // MUST match SpeedSensorConfig::kmhPerHz - real T610 tacho calibration
    uint32_t imuDataMaxAgeMs = 500;
    uint32_t maximumDistanceIntegrationIntervalMs = 500;
    size_t maximumSamplesPerUpdate = 64;

    // Input Block Bounds
    uint32_t maximumInputBlockSpanUs = 750000;
    uint32_t maximumImpactApplicationAgeMs = 1000;
};

} // namespace stridecontrol