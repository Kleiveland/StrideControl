#pragma once

#include <cstdint>
#include <cmath>
#include <array>
#include "../InclineCalibration/InclineCalibrationTypes.h"
#include "../InclineSensor/InclineSensorTypes.h"

namespace stridecontrol {

enum class InclineCommissioningPhase : uint8_t {
    Idle = 0,
    Homing,
    MeasuringPoint,
    Complete,
    Failed,
    TimedOut,
    Aborted
};

struct InclineCommissioningConfig {
    uint32_t homingTimeoutMs = 60000;              // Max time for physical homing to 0%
    uint32_t pointTimeoutMs = 45000;               // Max time for a point measurement to settle
    float directionConfirmAngleDeg = 0.10f;        // Minimum angle delta to qualify correct direction
    uint32_t directionConfirmMs = 300;             // Continuous duration required for direction qualification
    float arrivalTolerancePct = 0.50f;             // Pulse sensor tolerance to consider target reached
    float stabilityAngleToleranceDeg = 0.05f;      // Max allowable IMU angle jitter during stabilization
    uint32_t stableDurationMs = 1500;              // Required quiet stabilization window
    float maxAchievableInclinePct = 15.0f;         // Physical console maximum
};

class InclineCommissioningTracker {
public:
    explicit InclineCommissioningTracker(const InclineCommissioningConfig& config = InclineCommissioningConfig{});

    void beginHoming(uint32_t nowMs);
    void onHomedConfirmed(uint32_t nowMs); // Caller invokes after InclineSensor::confirmHomedAtZero() succeeds
    void beginMeasurePoint(float commandedPct, InclineDirection expectedDirection, uint32_t nowMs);
    void update(float pulseSensorReportedPct, float imuRelativeDeckAngleDeg, bool imuDataValid, uint32_t nowMs);
    void abort(uint32_t nowMs);
    void reset();

    InclineCommissioningPhase phase() const;
    bool currentPointDirectionConfirmed() const; // True once IMU confirmed movement in expectedDirection for this point
    InclineConfig buildCandidateConfig() const;  // Assembles all confirmed points into an InclineConfig, once all target points have been measured
    bool timedOut() const;

    float currentCommandedPct() const;
    InclineDirection currentExpectedDirection() const;
    uint8_t pointCount() const;

private:
    InclineCommissioningConfig config_;
    InclineCommissioningPhase phase_ = InclineCommissioningPhase::Idle;

    float currentCommandedPct_ = 0.0f;
    InclineDirection currentExpectedDirection_ = InclineDirection::Unknown;

    uint32_t homingStartMs_ = 0;
    uint32_t pointStartMs_ = 0;

    bool startAngleRecorded_ = false;
    float startAngleDeg_ = 0.0f;

    uint32_t directionSinceMs_ = 0;
    bool directionActive_ = false;
    bool directionConfirmed_ = false;

    uint32_t stableSinceMs_ = 0;
    bool stableActive_ = false;
    float lastSampleAngleDeg_ = 0.0f;

    std::array<InclineCalibrationPoint, kMaxInclineCalibrationPoints> points_{};
    uint8_t pointCount_ = 0;
    uint32_t lastCompleteMs_ = 0;
};

} // namespace stridecontrol

