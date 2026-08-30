#include "InclineVerifier.h"

#include <cmath>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace stridecontrol {

static constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;

const char* inclineVerifierStatusName(InclineVerifierStatus status) {
    switch (status) {
        case InclineVerifierStatus::Uninitialized: return "UNINITIALIZED";
        case InclineVerifierStatus::Ready:         return "READY";
        case InclineVerifierStatus::DegradedInput: return "DEGRADED_INPUT";
    }
    return "UNKNOWN";
}

const char* inclineVerificationStateName(InclineVerificationState state) {
    switch (state) {
        case InclineVerificationState::Unavailable: return "UNAVAILABLE";
        case InclineVerificationState::NotHomed:    return "NOT_HOMED";
        case InclineVerificationState::Moving:      return "MOVING";
        case InclineVerificationState::Stabilizing: return "STABILIZING";
        case InclineVerificationState::Verified:    return "VERIFIED";
        case InclineVerificationState::Diverging:   return "DIVERGING";
        case InclineVerificationState::Mismatch:    return "MISMATCH";
    }
    return "UNKNOWN";
}

const char* inclineVerifierConfidenceName(InclineVerifierConfidence confidence) {
    switch (confidence) {
        case InclineVerifierConfidence::Unavailable: return "UNAVAILABLE";
        case InclineVerifierConfidence::Low:         return "LOW";
        case InclineVerifierConfidence::Medium:      return "MEDIUM";
        case InclineVerifierConfidence::High:        return "HIGH";
    }
    return "UNKNOWN";
}

static bool validateConfiguration(const InclineVerifierConfig& cfg) {
    if (!std::isfinite(cfg.maxAllowedVerificationErrorDeg) ||
        !std::isfinite(cfg.mismatchThresholdDeg) ||
        !std::isfinite(cfg.targetTrackingTolerancePct)) {
        return false;
    }
    if (cfg.maxAllowedVerificationErrorDeg <= 0.0f) {
        return false;
    }
    if (cfg.mismatchThresholdDeg <= cfg.maxAllowedVerificationErrorDeg) {
        return false;
    }
    if (cfg.targetTrackingTolerancePct <= 0.0f) {
        return false;
    }
    if (cfg.actuatorStationaryHoldMs == 0U ||
        cfg.imuStableHoldMs == 0U ||
        cfg.divergencePersistenceMs == 0U) {
        return false;
    }
    return true;
}

struct InclineVerifier::Impl {
    SemaphoreHandle_t opMutex = nullptr;
    mutable portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

    bool initialized_ = false;
    bool active_ = false;
    InclineVerifierConfig config_{};

    InclineVerifierState publicState_{};
    InclineVerifierConfig publicConfig_{};

    InclineVerifierStatus status_ = InclineVerifierStatus::Uninitialized;
    InclineVerificationState verificationState_ = InclineVerificationState::Unavailable;
    InclineVerifierConfidence confidence_ = InclineVerifierConfidence::Unavailable;

    bool previousMoving_ = false;
    bool previousInputUsable_ = false;

    uint32_t stationaryStartTimestampMs_ = 0;
    bool stationaryTimerValid_ = false;

    uint32_t divergenceStartTimestampMs_ = 0;
    bool divergenceTimerActive_ = false;

    uint32_t lastCommandSequence_ = 0;
    bool hasLastCommandSequence_ = false;

    uint32_t verifiedEpisodeCount_ = 0;
    uint32_t divergenceEpisodeCount_ = 0;
    uint32_t mismatchEpisodeCount_ = 0;
    uint32_t inputDegradedEpisodeCount_ = 0;

    Impl() {
        opMutex = xSemaphoreCreateMutex();
    }

    ~Impl() {
        if (opMutex != nullptr) {
            vSemaphoreDelete(opMutex);
            opMutex = nullptr;
        }
    }

    void setStatus(InclineVerifierStatus nextStatus) {
        if (status_ != InclineVerifierStatus::DegradedInput && nextStatus == InclineVerifierStatus::DegradedInput) {
            inputDegradedEpisodeCount_++;
        }
        status_ = nextStatus;
    }

    void setVerificationState(InclineVerificationState nextState) {
        if (verificationState_ != InclineVerificationState::Verified && nextState == InclineVerificationState::Verified) {
            verifiedEpisodeCount_++;
        }
        if (verificationState_ != InclineVerificationState::Diverging && nextState == InclineVerificationState::Diverging) {
            divergenceEpisodeCount_++;
        }
        if (verificationState_ != InclineVerificationState::Mismatch && nextState == InclineVerificationState::Mismatch) {
            mismatchEpisodeCount_++;
        }
        verificationState_ = nextState;
    }

    void commitSnapshot(uint32_t nowMs,
                        const InclineVerificationCommandInput& commandInput,
                        const InclineState& inclineState,
                        const ImuState& imuState,
                        bool targetTrackingAvailable,
                        bool physicalVerificationAvailable,
                        float expectedDeckAngleDeg,
                        float targetTrackingDeltaPct,
                        float targetTrackingAbsoluteErrorPct,
                        float physicalVerificationDeltaDeg,
                        float physicalVerificationAbsoluteErrorDeg,
                        bool driftDetected,
                        bool mismatchDetected) {
        InclineVerifierState snap;
        snap.status = status_;
        snap.verificationState = verificationState_;
        snap.snapshotTimestampMs = nowMs;

        snap.targetInclinePct = commandInput.targetInclinePct;
        snap.targetInclineValid = commandInput.targetInclineValid;
        snap.commandSequence = commandInput.commandSequence;
        snap.commandTimestampMs = commandInput.commandTimestampMs;
        snap.commandTimestampValid = commandInput.commandTimestampValid;

        snap.trackedInclinePct = inclineState.estimatedInclinePct;
        snap.trackedInclineValid = (inclineState.initialized &&
                                    std::isfinite(inclineState.estimatedInclinePct) &&
                                    inclineState.status != InclineStatus::Uninitialized &&
                                    inclineState.status != InclineStatus::HardwareError);
        snap.inclineHomed = inclineState.homed;
        snap.inclinePositionTrusted = inclineState.positionTrusted;
        snap.inclineMoving = inclineState.moving;

        snap.relativeDeckAngleDeg = imuState.relativeDeckAngleDeg;
        snap.relativeDeckAngleValid = (imuState.initialized &&
                                       imuState.dataValid &&
                                       imuState.deckAngleValid &&
                                       !imuState.dataStale &&
                                       std::isfinite(imuState.relativeDeckAngleDeg) &&
                                       imuState.status != ImuStatus::HardwareError);
        snap.imuStable = (imuState.stability == ImuStability::Stable);

        snap.expectedDeckAngleDeg = expectedDeckAngleDeg;
        snap.targetTrackingDeltaPct = targetTrackingDeltaPct;
        snap.targetTrackingAbsoluteErrorPct = targetTrackingAbsoluteErrorPct;
        snap.physicalVerificationDeltaDeg = physicalVerificationDeltaDeg;
        snap.physicalVerificationAbsoluteErrorDeg = physicalVerificationAbsoluteErrorDeg;

        snap.targetTrackingAvailable = targetTrackingAvailable;
        snap.physicalVerificationAvailable = physicalVerificationAvailable;
        snap.driftDetected = driftDetected;
        snap.mismatchDetected = mismatchDetected;
        snap.confidence = confidence_;

        snap.verifiedEpisodeCount = verifiedEpisodeCount_;
        snap.divergenceEpisodeCount = divergenceEpisodeCount_;
        snap.mismatchEpisodeCount = mismatchEpisodeCount_;
        snap.inputDegradedEpisodeCount = inputDegradedEpisodeCount_;

        portENTER_CRITICAL(&stateMux);
        publicState_ = snap;
        publicConfig_ = config_;
        portEXIT_CRITICAL(&stateMux);
    }

    bool begin(const InclineVerifierConfig& config) {
        if (opMutex == nullptr) return false;
        if (xSemaphoreTake(opMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return false;
        }

        if (!validateConfiguration(config)) {
            xSemaphoreGive(opMutex);
            return false;
        }

        config_ = config;
        initialized_ = true;
        active_ = true;

        status_ = InclineVerifierStatus::Ready;
        verificationState_ = InclineVerificationState::Unavailable;
        confidence_ = InclineVerifierConfidence::Unavailable;

        previousMoving_ = false;
        previousInputUsable_ = false;
        stationaryStartTimestampMs_ = 0;
        stationaryTimerValid_ = false;
        divergenceStartTimestampMs_ = 0;
        divergenceTimerActive_ = false;
        lastCommandSequence_ = 0;
        hasLastCommandSequence_ = false;

        verifiedEpisodeCount_ = 0;
        divergenceEpisodeCount_ = 0;
        mismatchEpisodeCount_ = 0;
        inputDegradedEpisodeCount_ = 0;

        InclineVerificationCommandInput emptyCmd{};
        InclineState emptyIncline{};
        ImuState emptyImu{};
        commitSnapshot(0, emptyCmd, emptyIncline, emptyImu, false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false);

        xSemaphoreGive(opMutex);
        return true;
    }

    void end() {
        if (opMutex == nullptr) return;
        if (xSemaphoreTake(opMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;
        }

        initialized_ = false;
        active_ = false;
        status_ = InclineVerifierStatus::Uninitialized;
        verificationState_ = InclineVerificationState::Unavailable;
        confidence_ = InclineVerifierConfidence::Unavailable;

        stationaryTimerValid_ = false;
        divergenceTimerActive_ = false;
        hasLastCommandSequence_ = false;

        InclineVerificationCommandInput emptyCmd{};
        InclineState emptyIncline{};
        ImuState emptyImu{};
        commitSnapshot(0, emptyCmd, emptyIncline, emptyImu, false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false);

        xSemaphoreGive(opMutex);
    }

    void update(const InclineVerificationCommandInput& commandInput,
                const InclineState& inclineState,
                const ImuState& imuState,
                uint32_t nowMs) {
        if (opMutex == nullptr || xSemaphoreTake(opMutex, 0) != pdTRUE) {
            return;
        }

        if (!initialized_ || !active_) {
            xSemaphoreGive(opMutex);
            return;
        }

        // Track new command sequence to reset divergence timing on target change
        if (commandInput.targetInclineValid) {
            if (!hasLastCommandSequence_ || commandInput.commandSequence != lastCommandSequence_) {
                divergenceTimerActive_ = false;
                divergenceStartTimestampMs_ = 0;
                lastCommandSequence_ = commandInput.commandSequence;
                hasLastCommandSequence_ = true;
            }
        }

        // 1. Incline input validation
        const bool inclineInitialized = inclineState.initialized;
        const bool inclineHardwareError = (inclineState.status == InclineStatus::HardwareError);
        const bool inclineUninitialized = (inclineState.status == InclineStatus::Uninitialized);
        const bool trackedInclineFinite = std::isfinite(inclineState.estimatedInclinePct);
        const bool trackedInclineValid = (inclineInitialized &&
                                          trackedInclineFinite &&
                                          !inclineUninitialized &&
                                          !inclineHardwareError);
        const bool inclineHomed = inclineState.homed;
        const bool inclinePositionTrusted = inclineState.positionTrusted;
        const bool actuatorMoving = (inclineState.moving ||
                                     inclineState.status == InclineStatus::MovingUp ||
                                     inclineState.status == InclineStatus::MovingDown ||
                                     inclineState.status == InclineStatus::QualifyingMovement);
        const bool actuatorStationary = (!actuatorMoving && inclineState.status != InclineStatus::MovingDirectionUnknown);

        // 2. IMU input validation
        const bool imuInitialized = imuState.initialized;
        const bool imuHardwareError = (imuState.status == ImuStatus::HardwareError);
        const bool imuAngleFinite = std::isfinite(imuState.relativeDeckAngleDeg);
        const bool relativeDeckAngleValid = (imuInitialized &&
                                             imuState.dataValid &&
                                             imuState.deckAngleValid &&
                                             !imuState.dataStale &&
                                             imuAngleFinite &&
                                             !imuHardwareError);
        const bool imuStable = (imuState.stability == ImuStability::Stable);

        // 3. Stationary timer lifecycle
        const bool currentInputUsable = trackedInclineValid && (!config_.enableImuVerification || relativeDeckAngleValid);
        if (!previousInputUsable_ && currentInputUsable && actuatorStationary) {
            stationaryStartTimestampMs_ = nowMs;
            stationaryTimerValid_ = true;
            divergenceTimerActive_ = false;
            divergenceStartTimestampMs_ = 0;
        } else if (previousMoving_ && !actuatorMoving) {
            stationaryStartTimestampMs_ = nowMs;
            stationaryTimerValid_ = true;
            divergenceTimerActive_ = false;
            divergenceStartTimestampMs_ = 0;
        } else if (actuatorMoving) {
            stationaryTimerValid_ = false;
            divergenceTimerActive_ = false;
            divergenceStartTimestampMs_ = 0;
        } else if (!stationaryTimerValid_ && actuatorStationary) {
            stationaryStartTimestampMs_ = nowMs;
            stationaryTimerValid_ = true;
        }

        previousMoving_ = actuatorMoving;
        previousInputUsable_ = currentInputUsable;

        const bool actuatorHoldComplete = (stationaryTimerValid_ && (nowMs - stationaryStartTimestampMs_ >= config_.actuatorStationaryHoldMs));
        const bool imuStabilityHoldComplete = (imuStable && (imuState.stabilityDurationMs >= config_.imuStableHoldMs));

        // 4. Mathematical Comparison Calculations
        const float targetInclinePct = commandInput.targetInclinePct;
        const bool targetInclineValid = commandInput.targetInclineValid && std::isfinite(targetInclinePct);
        const float trackedInclinePct = inclineState.estimatedInclinePct;
        const float relativeDeckAngleDeg = imuState.relativeDeckAngleDeg;

        float expectedDeckAngleDeg = 0.0f;
        if (trackedInclineValid) {
            expectedDeckAngleDeg = atanf(trackedInclinePct / 100.0f) * kRadToDeg;
        }

        float targetTrackingDeltaPct = 0.0f;
        float targetTrackingAbsoluteErrorPct = 0.0f;
        const bool targetTrackingAvailable = (targetInclineValid && trackedInclineValid);
        if (targetTrackingAvailable) {
            targetTrackingDeltaPct = trackedInclinePct - targetInclinePct;
            targetTrackingAbsoluteErrorPct = std::fabs(targetTrackingDeltaPct);
        }

        float physicalVerificationDeltaDeg = 0.0f;
        float physicalVerificationAbsoluteErrorDeg = 0.0f;
        physicalVerificationDeltaDeg = relativeDeckAngleDeg - expectedDeckAngleDeg;
        physicalVerificationAbsoluteErrorDeg = std::fabs(physicalVerificationDeltaDeg);

        // 5. Physical Verification Availability
        const bool physicalVerificationAvailable = config_.enableImuVerification &&
                                                   inclineInitialized &&
                                                   trackedInclineValid &&
                                                   inclineHomed &&
                                                   inclinePositionTrusted &&
                                                   actuatorStationary &&
                                                   actuatorHoldComplete &&
                                                   imuInitialized &&
                                                   relativeDeckAngleValid &&
                                                   imuStabilityHoldComplete;

        // 6. Deterministic Physical State Priority Evaluation
        if (!inclineHomed) {
            setVerificationState(InclineVerificationState::NotHomed);
            divergenceTimerActive_ = false;
            divergenceStartTimestampMs_ = 0;
        } else if (actuatorMoving) {
            setVerificationState(InclineVerificationState::Moving);
            divergenceTimerActive_ = false;
            divergenceStartTimestampMs_ = 0;
        } else if (!config_.enableImuVerification ||
                   !inclinePositionTrusted ||
                   !relativeDeckAngleValid ||
                   inclineHardwareError ||
                   imuHardwareError) {
            setVerificationState(InclineVerificationState::Unavailable);
            divergenceTimerActive_ = false;
            divergenceStartTimestampMs_ = 0;
        } else if (!actuatorHoldComplete || !imuStabilityHoldComplete) {
            setVerificationState(InclineVerificationState::Stabilizing);
            divergenceTimerActive_ = false;
            divergenceStartTimestampMs_ = 0;
        } else {
            // Physical verification is available and active
            if (physicalVerificationAbsoluteErrorDeg >= config_.maxAllowedVerificationErrorDeg) {
                if (!divergenceTimerActive_) {
                    divergenceStartTimestampMs_ = nowMs;
                    divergenceTimerActive_ = true;
                }

                uint32_t divergenceElapsedMs = nowMs - divergenceStartTimestampMs_;
                if (physicalVerificationAbsoluteErrorDeg >= config_.mismatchThresholdDeg &&
                    divergenceElapsedMs >= config_.divergencePersistenceMs) {
                    setVerificationState(InclineVerificationState::Mismatch);
                } else {
                    setVerificationState(InclineVerificationState::Diverging);
                }
            } else {
                divergenceTimerActive_ = false;
                divergenceStartTimestampMs_ = 0;
                setVerificationState(InclineVerificationState::Verified);
            }
        }

        // 7. Software / Input Operational Status Determination
        if (!trackedInclineValid || (config_.enableImuVerification && (!imuInitialized || imuHardwareError || !relativeDeckAngleValid))) {
            setStatus(InclineVerifierStatus::DegradedInput);
        } else {
            setStatus(InclineVerifierStatus::Ready);
        }

        // 8. Confidence Assessment
        if (!physicalVerificationAvailable) {
            confidence_ = InclineVerifierConfidence::Unavailable;
        } else if (verificationState_ == InclineVerificationState::Diverging ||
                   verificationState_ == InclineVerificationState::Mismatch) {
            confidence_ = InclineVerifierConfidence::Low;
        } else if (verificationState_ == InclineVerificationState::Verified) {
            if (imuState.stabilityDurationMs >= config_.imuStableHoldMs * 2U) {
                confidence_ = InclineVerifierConfidence::High;
            } else {
                confidence_ = InclineVerifierConfidence::Medium;
            }
        } else {
            confidence_ = InclineVerifierConfidence::Unavailable;
        }

        const bool driftDetected = (verificationState_ == InclineVerificationState::Diverging ||
                                    verificationState_ == InclineVerificationState::Mismatch);
        const bool mismatchDetected = (verificationState_ == InclineVerificationState::Mismatch);

        commitSnapshot(nowMs,
                       commandInput,
                       inclineState,
                       imuState,
                       targetTrackingAvailable,
                       physicalVerificationAvailable,
                       expectedDeckAngleDeg,
                       targetTrackingDeltaPct,
                       targetTrackingAbsoluteErrorPct,
                       physicalVerificationDeltaDeg,
                       physicalVerificationAbsoluteErrorDeg,
                       driftDetected,
                       mismatchDetected);

        xSemaphoreGive(opMutex);
    }

    InclineVerifierState getState() const {
        InclineVerifierState snap;
        portENTER_CRITICAL(&stateMux);
        snap = publicState_;
        portEXIT_CRITICAL(&stateMux);
        return snap;
    }

    InclineVerifierConfig configSnapshot() const {
        InclineVerifierConfig cfg;
        portENTER_CRITICAL(&stateMux);
        cfg = publicConfig_;
        portEXIT_CRITICAL(&stateMux);
        return cfg;
    }

    bool isReady() const {
        bool r = false;
        portENTER_CRITICAL(&stateMux);
        r = publicState_.status == InclineVerifierStatus::Ready;
        portEXIT_CRITICAL(&stateMux);
        return r;
    }
};

InclineVerifier::InclineVerifier() : impl_(new Impl()) {}

InclineVerifier::~InclineVerifier() {
    if (impl_ != nullptr) {
        if (impl_->opMutex != nullptr) {
            xSemaphoreTake(impl_->opMutex, portMAX_DELAY);
            impl_->initialized_ = false;
            impl_->active_ = false;
            xSemaphoreGive(impl_->opMutex);
        }
        delete impl_;
        impl_ = nullptr;
    }
}

bool InclineVerifier::begin(const InclineVerifierConfig& config) {
    if (impl_ == nullptr) return false;
    return impl_->begin(config);
}

void InclineVerifier::end() {
    if (impl_ != nullptr) {
        impl_->end();
    }
}

void InclineVerifier::update(const InclineVerificationCommandInput& commandInput,
                             const InclineState& inclineState,
                             const ImuState& imuState,
                             uint32_t nowMs) {
    if (impl_ != nullptr) {
        impl_->update(commandInput, inclineState, imuState, nowMs);
    }
}

InclineVerifierState InclineVerifier::getState() const {
    if (impl_ == nullptr) return InclineVerifierState{};
    return impl_->getState();
}

InclineVerifierConfig InclineVerifier::configSnapshot() const {
    if (impl_ == nullptr) return InclineVerifierConfig{};
    return impl_->configSnapshot();
}

bool InclineVerifier::isReady() const {
    if (impl_ == nullptr) return false;
    return impl_->isReady();
}

const char* InclineVerifier::version() {
    return "1.0.0";
}

} // namespace stridecontrol
