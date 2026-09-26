#pragma once

#include <cstdint>
#include <cmath>
#include <array>
#include "../SettingsService/SettingsServiceTypes.h"
#include "../CsafeInterface/CsafeTypes.h"

namespace stridecontrol {

/**
 * @brief Circular log entry for transparent audit of automatic adaptations.
 */
struct SpeedAdaptationLogEntry {
    uint32_t timestampMs = 0;
    float measuredPhysicalSpeedKmh = 0.0f;
    float previousCommandKmh = 0.0f;
    float newCommandKmh = 0.0f;
    float deltaKmh = 0.0f;
    float accumulatedSteadySeconds = 0.0f;
    uint8_t confidencePct = 0;
};

/**
 * @brief Dynamically tracked speed candidate slot.
 *
 * NOTE: Strict single-precision float only. Zero double types.
 */
struct TrackedCandidate {
    float commandedSpeedKmh = 0.0f;       // Nominal console command (Y)
    float measuredSpeedMeanKmh = 0.0f;    // Running physical speed average (X)
    float measuredSpeedM2 = 0.0f;         // Welford variance accumulator
    uint32_t sampleCount = 0;             // Total steady ticks accumulated (50 Hz)
    uint32_t transientAnomalyTicks = 0;   // Rejected spike ticks (from pre-filter)
    uint32_t firstObservedMs = 0;
    uint32_t lastObservedMs = 0;
    uint8_t updateCountLifetime = 0;      // Lifetime EWMA adaptations applied
    bool active = false;
};

/**
 * @brief Configuration tuning parameters for SpeedLearningTracker.
 */
struct SpeedLearningConfig {
    float minCommandSettledSeconds = 8.0f;      // Settling time after command change
    float minSteadySecondsToCommit = 45.0f;     // Minimum accumulated steady time
    float targetConfidenceSeconds = 120.0f;     // Full-time confidence benchmark
    float maxStdDevKmh = 0.08f;                 // Maximum allowable standard deviation
    float transientOutlierThresholdKmh = 0.40f; // 1-tick spike rejection gate
    float maxTransientOutlierRatio = 0.015f;    // Max 1.5% anomalous ticks allowed
    float hybridMergeAbsKmh = 0.30f;            // Hybrid merge distance (absolute)
    float hybridMergePct = 0.025f;              // Hybrid merge distance (2.5%)
    float ewmaBaseAlpha = 0.15f;                // Maximum base EWMA step size
    float maxAdaptationStepKmh = 0.30f;         // Strict upper clamp per session
    float maxBaselineSafetyPct = 15.0f;         // Immutable ROM envelope (±15%)
    float maxBaselineSafetyAbsKmh = 2.00f;      // Immutable ROM envelope (±2.0 km/h)
    float crossPointToleranceKmh = 0.40f;       // Cross-point neighbor tolerance
    uint8_t minConfidenceToCommit = 75;         // Minimum confidence to auto-commit
    float minAdaptationDeltaKmh = 0.02f;        // Minimum delta to consider table dirty (item 8)
    float newPointPredictionToleranceKmh = 0.40f; // Redundant new-point insertion skip tolerance
};

/**
 * @brief Fully autonomous background calibration learning tracker.
 *
 * Runs passively in the 50 Hz control loop. Dynamically tracks held speeds, applies
 * a 3-tap median pre-filter, accumulates Welford variance across the session, and
 * commits confidence-adaptive EWMA refinements to SpeedConfig only when the belt is stopped.
 */
class SpeedLearningTracker {
public:
    static constexpr size_t kMaxTrackedCandidates = 10;
    static constexpr size_t kMaxAuditLogEntries = 8;

    explicit SpeedLearningTracker(const SpeedLearningConfig& config = SpeedLearningConfig{});

    /**
     * @brief Periodic update called in ControlRuntime::runTaskLoop() at 50 Hz.
     */
    void update(
        float measuredPhysicalSpeedKmh,
        bool speedMeasurementValid,
        float activeCommandedSpeedKmh,
        CsafeMachineState csafeState,
        bool controllerBusy,
        bool estopActive,
        bool rampPreFireActive,
        uint32_t nowMs
    );

    /**
     * @brief Evaluates whether the belt has reached stationary state to trigger auto-commit.
     * @return true if an automatic EWMA calibration update was committed to activeConfigInOut.
     */
    bool checkBeltStoppedTrigger(
        float measuredSpeedKmh,
        CsafeMachineState csafeState,
        SpeedConfig& activeConfigInOut,
        uint32_t nowMs
    );

    /**
     * @brief Sets the active SpeedConfig for cross-point gradient verification.
     */
    void setActiveSpeedConfig(const SpeedConfig& activeConfig);

    /**
     * @brief Resets all active candidates for the current session.
     */
    void resetSession();

    // Read-only queries for Service Console audit log
    const std::array<TrackedCandidate, kMaxTrackedCandidates>& getTrackedCandidates() const { return candidates_; }
    uint8_t getTrackedCandidateCount() const { return candidateCount_; }
    uint8_t getAuditLog(SpeedAdaptationLogEntry* outEntries, uint8_t maxEntries) const;
    uint8_t getAuditLogCount() const { return auditLogCount_; }

private:
    // 3-tap median pre-filter (single-precision float)
    static float filterMedian3(float s0, float s1, float s2);

    // Dynamic candidate management
    int8_t findOrAllocateCandidateIndex(float commandedSpeedKmh, uint32_t nowMs);
    void evictLowestConfidenceCandidate();

    // Confidence & safety evaluations
    uint8_t calculateConfidence(const TrackedCandidate& cand) const;

    /**
     * @brief Predicts the expected console command for a given physical speed
     *        using piecewise linear interpolation and linear edge extrapolation.
     */
    float predictCommand(const SpeedConfig& config, float measuredKmh) const;

    /**
     * @brief Cross-point consistency verification.
     *
     * - Interpolated points: Compares candidate command against linear interpolation of
     *   surrounding neighbor points (tolerance: ±0.40 km/h; slope: [0.75, 1.35]).
     * - Extrapolated points (outside [X_0, X_{N-1}]): Uses linear slope continuation
     *   from the outermost known segment (Y_{N-1} + m_outer * (S - X_{N-1})), bounded
     *   by strict ascending monotonicity and the immutable factory baseline anchor.
     * - Uncalibrated fallback (<2 points): Compares against nominal 1:1 factory identity.
     */
    bool evaluateCrossPointConsistency(float measuredKmh, float commandKmh) const;

    /**
     * @brief Absolute safety guardrail evaluated strictly against immutable flash ROM baseline.
     * Never evaluated against mutable/persisted NVS values to prevent multi-session calibration creep.
     */
    bool evaluateFactoryBaselineSafety(float measuredKmh, float commandKmh) const;

    // EWMA calculation
    float computeAdaptiveAlpha(uint8_t confidencePct, uint8_t priorUpdates) const;

    // Logging
    void appendAuditLog(const SpeedAdaptationLogEntry& entry);

    SpeedLearningConfig config_;
    SpeedConfig activeConfig_{};

    // Immutable factory baseline table (stored in ROM, never modified)
    static const SpeedConfig kFactorySpeedBaseline;

    // Dynamic candidates array (bounded to 10 slots)
    std::array<TrackedCandidate, kMaxTrackedCandidates> candidates_{};
    uint8_t candidateCount_ = 0;

    // Pre-filter history buffer
    std::array<float, 3> medianBuffer_{0.0f, 0.0f, 0.0f};
    uint8_t medianCount_ = 0;

    // Steady-state command tracking
    float lastCommandedSpeedKmh_ = 0.0f;
    uint32_t commandStableSinceMs_ = 0;

    // Belt stop debounce
    uint32_t stationarySinceMs_ = 0;
    bool stationaryActive_ = false;

    // Read-only circular audit log
    std::array<SpeedAdaptationLogEntry, kMaxAuditLogEntries> auditLog_{};
    uint8_t auditLogHead_ = 0;
    uint8_t auditLogCount_ = 0;
};

} // namespace stridecontrol
