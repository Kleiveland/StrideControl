#include "VirtualTreadmill.h"
#include <cmath>
#include <algorithm>

namespace stridecontrol {

VirtualTreadmill::VirtualTreadmill(const VirtualTreadmillConfig& config)
    : config_(config) {
    resetModel(0);
}

void VirtualTreadmill::resetModel(uint32_t initialTimeMs) {
    resetModelUs(static_cast<uint64_t>(initialTimeMs) * 1000ULL);
    lastTimestampMs_ = initialTimeMs;
}

void VirtualTreadmill::resetModelUs(uint64_t initialTimeUs) {
    initialized_ = true;
    lastTickIndex_ = 0;
    lastScenarioTimeUs_ = initialTimeUs;
    lastTimestampMs_ = static_cast<uint32_t>(initialTimeUs / 1000ULL);

    actualSpeedKmh_ = 0.0f;
    targetSpeedKmh_ = 0.0f;
    actualInclinePct_ = 0.0f;
    targetInclinePct_ = 0.0f;
    odometerKm_ = 0.0;

    fractionalTachoPulses_ = 0.0;
    totalTachoPulses_ = 0;
    fractionalInclinePulses_ = 0.0;
    totalInclinePulses_ = 0;
    totalSignedInclinePulses_ = 0;

    eStopActive_ = false;
    runnerLocation_ = VirtualRunnerLocation::Absent;
    cadenceSpm_ = 0;
    heartRateBpm_ = 0;
    activeFaultsMask_ = 0;

    eventQueueCount_ = 0;
    for (size_t i = 0; i < kEventQueueCapacity; ++i) {
        eventQueue_[i] = VirtualButtonEvent{};
    }

    tachoOutput_ = TachoOutput{};
    inclineOutput_ = InclineFeedbackOutput{};
    consoleOutput_ = ConsoleOutput{};
    biometricOutput_ = BiometricOutput{};
}

bool VirtualTreadmill::tick(const SimulationTick& tick) {
    if (!initialized_) {
        return false;
    }

    // 1. Monotonic tickIndex check: Must increment by exactly 1
    if (tick.tickIndex != lastTickIndex_ + 1) {
        return false;
    }

    // 2. Monotonic timestamp check
    const uint32_t deltaUs = tick.deltaUs != 0 ? tick.deltaUs : tick.deltaMs * 1000UL;
    if (deltaUs == 0) {
        return false;
    }
    const uint64_t scenarioTimeUs = tick.scenarioTimeUs != 0 ? tick.scenarioTimeUs : static_cast<uint64_t>(tick.timestampMs) * 1000ULL;
    const uint32_t deltaMs = tick.deltaMs != 0 ? tick.deltaMs : (deltaUs / 1000UL);

    if (lastTickIndex_ > 0) {
        if (scenarioTimeUs <= lastScenarioTimeUs_) {
            return false;
        }
    }

    // Check E-Stop stuck fault
    if (isFaultActive(VirtualTreadmillFault::EStopStuckActive)) {
        eStopActive_ = true;
    }

    const float prevSpeed = actualSpeedKmh_;
    const float prevIncline = actualInclinePct_;

    // 3. Speed dynamics
    updateSpeed(deltaMs);

    // 4. Incline dynamics
    updateIncline(deltaMs);

    // 5. Tacho pulse synthesis & odometer integration
    updateTachoPulses(prevSpeed, actualSpeedKmh_, deltaUs, scenarioTimeUs);

    // 6. Incline pulse synthesis
    updateInclinePulses(prevIncline, actualInclinePct_);

    // 7. Console event drain into output DTO
    consoleOutput_.eventCount = 0;
    for (uint8_t i = 0; i < eventQueueCount_ && i < ConsoleOutput::kMaxEvents; ++i) {
        consoleOutput_.events[i] = eventQueue_[i];
        consoleOutput_.eventCount++;
    }
    eventQueueCount_ = 0;
    consoleOutput_.eStopEngaged = eStopActive_;

    // 8. Biometric DTO generation with fault isolation
    biometricOutput_.runnerLocation = runnerLocation_;
    biometricOutput_.cadenceSpm = cadenceSpm_;
    if (isFaultActive(VirtualTreadmillFault::HeartRateDropout)) {
        biometricOutput_.heartRateBpm = 0;
        biometricOutput_.signalValid = false;
    } else {
        biometricOutput_.heartRateBpm = heartRateBpm_;
        biometricOutput_.signalValid = true;
    }

    // Advance state tracking
    lastTickIndex_ = tick.tickIndex;
    lastScenarioTimeUs_ = scenarioTimeUs;
    lastTimestampMs_ = static_cast<uint32_t>(scenarioTimeUs / 1000ULL);

    return true;
}

void VirtualTreadmill::updateSpeed(uint32_t deltaMs) {
    const float dtSec = static_cast<float>(deltaMs) / 1000.0f;
    float effectiveTarget = targetSpeedKmh_;

    // When E-Stop is active, target is forced to 0.0 with eStopDeceleration
    if (eStopActive_) {
        effectiveTarget = 0.0f;
        const float maxDecel = config_.eStopDecelerationKmhPerSec * dtSec;
        if (actualSpeedKmh_ > 0.0f) {
            actualSpeedKmh_ -= maxDecel;
            if (actualSpeedKmh_ < 0.0f) {
                actualSpeedKmh_ = 0.0f;
            }
        }
        return;
    }

    // Clamp effective target within physical bounds
    if (effectiveTarget > config_.maxSpeedKmh) {
        effectiveTarget = config_.maxSpeedKmh;
    } else if (effectiveTarget < 0.0f) {
        effectiveTarget = 0.0f;
    }

    if (actualSpeedKmh_ < effectiveTarget) {
        const float maxAccel = config_.accelerationKmhPerSec * dtSec;
        actualSpeedKmh_ += maxAccel;
        if (actualSpeedKmh_ > effectiveTarget) {
            actualSpeedKmh_ = effectiveTarget;
        }
    } else if (actualSpeedKmh_ > effectiveTarget) {
        const float maxDecel = config_.normalDecelerationKmhPerSec * dtSec;
        actualSpeedKmh_ -= maxDecel;
        if (actualSpeedKmh_ < effectiveTarget) {
            actualSpeedKmh_ = effectiveTarget;
        }
    }

    // Clamp absolute limits
    if (actualSpeedKmh_ < 0.0f) {
        actualSpeedKmh_ = 0.0f;
    } else if (actualSpeedKmh_ > config_.maxSpeedKmh) {
        actualSpeedKmh_ = config_.maxSpeedKmh;
    }
}

void VirtualTreadmill::updateIncline(uint32_t deltaMs) {
    // Incline motor stall fault freezes incline motion
    if (isFaultActive(VirtualTreadmillFault::InclineMotorStall)) {
        return;
    }

    const float dtSec = static_cast<float>(deltaMs) / 1000.0f;
    float effectiveTarget = targetInclinePct_;

    if (effectiveTarget > config_.inclineMaxPct) {
        effectiveTarget = config_.inclineMaxPct;
    } else if (effectiveTarget < config_.inclineMinPct) {
        effectiveTarget = config_.inclineMinPct;
    }

    if (actualInclinePct_ < effectiveTarget) {
        const float maxTransit = config_.inclineTransitPctPerSec * dtSec;
        actualInclinePct_ += maxTransit;
        if (actualInclinePct_ > effectiveTarget) {
            actualInclinePct_ = effectiveTarget;
        }
    } else if (actualInclinePct_ > effectiveTarget) {
        const float maxTransit = config_.inclineTransitPctPerSec * dtSec;
        actualInclinePct_ -= maxTransit;
        if (actualInclinePct_ < effectiveTarget) {
            actualInclinePct_ = effectiveTarget;
        }
    }

    // Final boundary clamp
    if (actualInclinePct_ < config_.inclineMinPct) {
        actualInclinePct_ = config_.inclineMinPct;
    } else if (actualInclinePct_ > config_.inclineMaxPct) {
        actualInclinePct_ = config_.inclineMaxPct;
    }
}

void VirtualTreadmill::updateTachoPulses(float prevSpeed, float currSpeed, uint32_t deltaUs, uint64_t scenarioTimeUs) {
    // Trapezoidal distance integration: average speed over deltaUs
    const double avgSpeedKmh = static_cast<double>(prevSpeed + currSpeed) * 0.5;
    const double dtHours = static_cast<double>(deltaUs) / 3600000000.0;
    const double deltaKm = avgSpeedKmh * dtHours;

    if (deltaKm > 0.0) {
        odometerKm_ += deltaKm;
    }

    // Clear edge outputs for this tick
    tachoOutput_.edgeCount = 0;
    for (size_t i = 0; i < kMaxTachoEdgesPerTick; ++i) {
        tachoOutput_.edgeTimestampUs[i] = 0;
    }

    // Check tacho fault isolation
    const bool tachoFault = isFaultActive(VirtualTreadmillFault::TachoLostSignal);

    if (tachoFault) {
        tachoOutput_.pulsesThisTick = 0;
        tachoOutput_.totalPulses = totalTachoPulses_;
        tachoOutput_.fractionalRemainder = fractionalTachoPulses_;
        tachoOutput_.instantaneousSpeedKmh = currSpeed;
        tachoOutput_.signalValid = false;
        return;
    }

    // Accumulate fractional pulses based on configured tacho calibration
    const double pulsesToAdd = deltaKm * config_.tachoPulsesPerKm;
    const double phi0 = fractionalTachoPulses_;
    const double phiNew = phi0 + pulsesToAdd;

    const uint64_t wholePulses = static_cast<uint64_t>(phiNew);
    fractionalTachoPulses_ = phiNew - static_cast<double>(wholePulses);
    totalTachoPulses_ += wholePulses;

    tachoOutput_.pulsesThisTick = wholePulses;
    tachoOutput_.totalPulses = totalTachoPulses_;
    tachoOutput_.fractionalRemainder = fractionalTachoPulses_;
    tachoOutput_.instantaneousSpeedKmh = currSpeed;
    tachoOutput_.signalValid = true;

    // Schedule intra-tick edge timestamps using exact quadratic ramp solution
    if (wholePulses > 0 && pulsesToAdd > 0.0) {
        const uint64_t tStartUs = scenarioTimeUs - deltaUs;
        const double v0 = static_cast<double>(prevSpeed);
        const double v1 = static_cast<double>(currSpeed);

        for (uint64_t p = 1; p <= wholePulses; ++p) {
            const double xi = (static_cast<double>(p) - phi0) / pulsesToAdd;
            double tauUs = 0.0;

            if ((v0 + v1) <= 0.0001) {
                tauUs = static_cast<double>(deltaUs) * xi;
            } else {
                const double D = (1.0 - xi) * (v0 * v0) + xi * (v1 * v1);
                const double sqrtTerm = std::sqrt(D > 0.0 ? D : 0.0);
                const double denom = v0 + sqrtTerm;
                if (denom > 1e-9) {
                    tauUs = static_cast<double>(deltaUs) * ((v0 + v1) * xi) / denom;
                } else {
                    tauUs = static_cast<double>(deltaUs) * xi;
                }
            }

            if (tauUs < 0.0) tauUs = 0.0;
            if (tauUs > static_cast<double>(deltaUs)) tauUs = static_cast<double>(deltaUs);

            const uint32_t edgeTime = static_cast<uint32_t>(tStartUs + static_cast<uint64_t>(std::round(tauUs)));

            if (tachoOutput_.edgeCount < kMaxTachoEdgesPerTick) {
                tachoOutput_.edgeTimestampUs[tachoOutput_.edgeCount++] = edgeTime;
            } else {
                tachoOutput_.droppedEdgesCount++;
            }
        }
    }
}

void VirtualTreadmill::updateInclinePulses(float prevIncline, float currIncline) {
    const float deltaIncline = currIncline - prevIncline;
    const float absDelta = std::abs(deltaIncline);

    InclineDirection dir = InclineDirection::Unknown;
    if (deltaIncline > 0.0001f) {
        dir = InclineDirection::Up;
    } else if (deltaIncline < -0.0001f) {
        dir = InclineDirection::Down;
    }

    const bool pulseFault = isFaultActive(VirtualTreadmillFault::InclinePulseLost);

    if (pulseFault || dir == InclineDirection::Unknown) {
        inclineOutput_.direction = dir;
        inclineOutput_.pulsesThisTick = 0;
        inclineOutput_.totalSignedPulses = totalSignedInclinePulses_;
        inclineOutput_.totalPulses = totalInclinePulses_;
        inclineOutput_.fractionalRemainder = fractionalInclinePulses_;
        inclineOutput_.physicalInclinePct = currIncline;
        inclineOutput_.moving = (dir != InclineDirection::Unknown);
        inclineOutput_.signalValid = !pulseFault;
        return;
    }

    const double pulsesToAdd = static_cast<double>(absDelta) * config_.inclinePulsesPerPct;
    fractionalInclinePulses_ += pulsesToAdd;

    const uint64_t wholePulses = static_cast<uint64_t>(fractionalInclinePulses_);
    fractionalInclinePulses_ -= static_cast<double>(wholePulses);
    totalInclinePulses_ += wholePulses;

    if (dir == InclineDirection::Up) {
        totalSignedInclinePulses_ += static_cast<int64_t>(wholePulses);
    } else {
        totalSignedInclinePulses_ -= static_cast<int64_t>(wholePulses);
    }

    inclineOutput_.direction = dir;
    inclineOutput_.pulsesThisTick = wholePulses;
    inclineOutput_.totalSignedPulses = totalSignedInclinePulses_;
    inclineOutput_.totalPulses = totalInclinePulses_;
    inclineOutput_.fractionalRemainder = fractionalInclinePulses_;
    inclineOutput_.physicalInclinePct = currIncline;
    inclineOutput_.moving = true;
    inclineOutput_.signalValid = true;
}

void VirtualTreadmill::setTargetSpeedKmh(float speedKmh) {
    if (speedKmh < 0.0f) {
        targetSpeedKmh_ = 0.0f;
    } else if (speedKmh > config_.maxSpeedKmh) {
        targetSpeedKmh_ = config_.maxSpeedKmh;
    } else {
        targetSpeedKmh_ = speedKmh;
    }
}

void VirtualTreadmill::setTargetInclinePct(float inclinePct) {
    if (inclinePct < config_.inclineMinPct) {
        targetInclinePct_ = config_.inclineMinPct;
    } else if (inclinePct > config_.inclineMaxPct) {
        targetInclinePct_ = config_.inclineMaxPct;
    } else {
        targetInclinePct_ = inclinePct;
    }
}

void VirtualTreadmill::setEmergencyStop(bool active) {
    eStopActive_ = active;
}

void VirtualTreadmill::setRunnerLocation(VirtualRunnerLocation location) {
    runnerLocation_ = location;
}

void VirtualTreadmill::setCadenceSpm(uint16_t spm) {
    cadenceSpm_ = spm;
}

void VirtualTreadmill::setHeartRateBpm(uint8_t bpm) {
    heartRateBpm_ = bpm;
}

bool VirtualTreadmill::enqueueButtonEvent(VirtualButtonId button, VirtualButtonState state, uint32_t timestampMs) {
    if (eventQueueCount_ >= kEventQueueCapacity) {
        return false;
    }
    eventQueue_[eventQueueCount_] = VirtualButtonEvent{button, state, timestampMs};
    eventQueueCount_++;
    return true;
}

void VirtualTreadmill::setFault(VirtualTreadmillFault fault, bool active) {
    const uint16_t bit = static_cast<uint16_t>(fault);
    if (active) {
        activeFaultsMask_ |= bit;
    } else {
        activeFaultsMask_ &= ~bit;
    }
}

bool VirtualTreadmill::isFaultActive(VirtualTreadmillFault fault) const {
    const uint16_t bit = static_cast<uint16_t>(fault);
    return (activeFaultsMask_ & bit) != 0;
}

void VirtualTreadmill::clearAllFaults() {
    activeFaultsMask_ = 0;
}

const char* VirtualTreadmill::version() {
    return "VirtualTreadmill/1.0.0";
}

} // namespace stridecontrol\n