#pragma once

#include <cstdint>

namespace stridecontrol {

/**
 * @brief Software lifecycle and input-channel operational status.
 */
enum class InclineVerifierStatus : uint8_t {
    Uninitialized,  ///< Module has not completed begin() or has been terminated by end()
    Ready,          ///< Operational; at least one verification path (e.g. target tracking) can execute
    DegradedInput   ///< An input channel required for an expected current evaluation is invalid, stale, or faulted
};

/**
 * @brief Real-time physical incline verification state.
 */
enum class InclineVerificationState : uint8_t {
    Unavailable,    ///< Verification unavailable (IMU disabled, position untrusted, angle invalid, or sensor fault)
    NotHomed,       ///< InclineSensor homed == false; pulse accumulation is unanchored
    Moving,         ///< Incline actuator in active transit (inclineState.moving == true)
    Stabilizing,    ///< Actuator stationary, but actuator hold or IMU stability duration incomplete
    Verified,       ///< Settled deck; physical angle matches expected angle within allowed tolerance
    Diverging,      ///< Settled deck; physical angle exceeds allowed tolerance within confirmation window
    Mismatch        ///< Settled deck; physical angle persistently exceeds critical mismatch threshold
};

/**
 * @brief Physical verification confidence assessment.
 */
enum class InclineVerifierConfidence : uint8_t {
    Unavailable,    ///< Moving, stabilizing, not homed, untrusted, or input invalid
    Low,            ///< Active divergence or high residual variance
    Medium,         ///< Verified within tolerance, moderate stationary duration
    High            ///< Verified within tolerance, sustained IMU stability, homed and trusted tracker
};

/**
 * @brief Minimal, immutable command context input supplied to update().
 */
struct InclineVerificationCommandInput {
    float targetInclinePct = 0.0f;          ///< Target incline [% grade, 0.0 to 15.0]
    bool targetInclineValid = false;        ///< True if requested target is authoritative
    uint32_t commandSequence = 0;           ///< Monotonically increasing command sequence ID
    uint32_t commandTimestampMs = 0;        ///< Application timestamp of command issuance
    bool commandTimestampValid = false;     ///< True if commandTimestampMs is valid
};

/**
 * @brief Published telemetry snapshot for InclineVerifier.
 */
struct InclineVerifierState {
    InclineVerifierStatus status = InclineVerifierStatus::Uninitialized;
    InclineVerificationState verificationState = InclineVerificationState::Unavailable;

    uint32_t snapshotTimestampMs = 0;

    // Command Context Echo
    float targetInclinePct = 0.0f;
    bool targetInclineValid = false;
    uint32_t commandSequence = 0;
    uint32_t commandTimestampMs = 0;
    bool commandTimestampValid = false;

    // InclineSensor Independent Context Echo
    float trackedInclinePct = 0.0f;
    bool trackedInclineValid = false;       ///< inclineState.initialized && isfinite(estimatedInclinePct) && status != HardwareError
    bool inclineHomed = false;              ///< inclineState.homed
    bool inclinePositionTrusted = false;    ///< inclineState.positionTrusted
    bool inclineMoving = false;             ///< inclineState.moving

    // ImuInterface Physical Context Echo
    float relativeDeckAngleDeg = 0.0f;
    bool relativeDeckAngleValid = false;    ///< imuState.dataValid && imuState.deckAngleValid && !imuState.dataStale && isfinite(relativeDeckAngleDeg)
    bool imuStable = false;                 ///< imuState.stability == ImuStability::Stable

    // Derived Physical Reference
    float expectedDeckAngleDeg = 0.0f;      ///< atan(trackedInclinePct / 100.0) * (180.0 / PI)

    // Mathematical Comparison Errors (Signed Delta & Scalar Absolute)
    float targetTrackingDeltaPct = 0.0f;            ///< trackedInclinePct - targetInclinePct [% grade]
    float targetTrackingAbsoluteErrorPct = 0.0f;    ///< |targetTrackingDeltaPct| [% grade]

    float physicalVerificationDeltaDeg = 0.0f;      ///< relativeDeckAngleDeg - expectedDeckAngleDeg [°]
    float physicalVerificationAbsoluteErrorDeg = 0.0f; ///< |physicalVerificationDeltaDeg| [°]

    // Distinct Capability Availability Flags
    bool targetTrackingAvailable = false;       ///< targetInclineValid && trackedInclineValid
    bool physicalVerificationAvailable = false; ///< True strictly when physical angle verification conditions are fully satisfied

    // State-Driven Real-Time Flags (Unlatched)
    bool driftDetected = false;                 ///< True when verificationState is Diverging or Mismatch
    bool mismatchDetected = false;              ///< True when verificationState is Mismatch

    InclineVerifierConfidence confidence = InclineVerifierConfidence::Unavailable;

    // Rising-Edge Episode Diagnostic Counters
    uint32_t verifiedEpisodeCount = 0;
    uint32_t divergenceEpisodeCount = 0;
    uint32_t mismatchEpisodeCount = 0;
    uint32_t inputDegradedEpisodeCount = 0;
};

const char* inclineVerifierStatusName(InclineVerifierStatus status);
const char* inclineVerificationStateName(InclineVerificationState state);
const char* inclineVerifierConfidenceName(InclineVerifierConfidence confidence);

} // namespace stridecontrol

