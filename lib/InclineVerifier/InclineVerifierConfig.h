#pragma once

#include <cstdint>

namespace stridecontrol {

struct InclineVerifierConfig {
    // Angular Error Thresholds [Degrees]
    float maxAllowedVerificationErrorDeg = 0.35f;
    ///< Classification: ALGORITHMIC DEFAULT, NOT PHYSICALLY VALIDATED
    ///< Permissible angular discrepancy (~0.6% grade) before entering Diverging

    float mismatchThresholdDeg = 0.90f;
    ///< Classification: ALGORITHMIC DEFAULT, NOT PHYSICALLY VALIDATED
    ///< Critical angular discrepancy (~1.5% grade) for confirmed Mismatch

    // Target Tracking Tolerance [% Grade]
    float targetTrackingTolerancePct = 0.25f;
    ///< Classification: ALGORITHMIC DEFAULT, NOT PHYSICALLY VALIDATED
    ///< Allowable target-tracking discrepancy when stopped at target

    // Stability and Gating Durations [Milliseconds]
    uint32_t actuatorStationaryHoldMs = 1500U;
    ///< Classification: ALGORITHMIC DEFAULT, NOT PHYSICALLY VALIDATED
    ///< Duration incline motor must remain stationary before angle evaluation

    uint32_t imuStableHoldMs = 1000U;
    ///< Classification: ALGORITHMIC DEFAULT, NOT PHYSICALLY VALIDATED
    ///< Continuous duration ImuInterface must report ImuStability::Stable

    uint32_t divergencePersistenceMs = 2000U;
    ///< Classification: ALGORITHMIC DEFAULT, NOT PHYSICALLY VALIDATED
    ///< Continuous duration angular error must exceed threshold before Mismatch

    // Feature Flags
    bool enableImuVerification = true;
    ///< Classification: ARCHITECTURAL DECISION
    ///< Enables physical angle comparison; when false, target tracking remains active
};

} // namespace stridecontrol

