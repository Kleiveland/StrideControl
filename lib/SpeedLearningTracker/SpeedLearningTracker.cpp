#include "SpeedLearningTracker.h"
#include "../SpeedCalibration/SpeedCalibration.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace stridecontrol {

const SpeedConfig SpeedLearningTracker::kFactorySpeedBaseline = []() {
    SpeedConfig cfg{};
    cfg.maxAchievableSpeedKmh = 25.0f;
    cfg.maxAchievableSpeedVerified = false;
    cfg.commandMapValid = false;
    cfg.sensorCalibrationFactor = 1.0f;
    cfg.pointCount = 0;
    return cfg;
}();

SpeedLearningTracker::SpeedLearningTracker(const SpeedLearningConfig& config)
    : config_(config) {
    resetSession();
}

void SpeedLearningTracker::setActiveSpeedConfig(const SpeedConfig& activeConfig) {
    activeConfig_ = activeConfig;
}

void SpeedLearningTracker::resetSession() {
    for (auto& cand : candidates_) {
        cand = TrackedCandidate{};
    }
    candidateCount_ = 0;
    medianCount_ = 0;
    lastCommandedSpeedKmh_ = 0.0f;
    commandStableSinceMs_ = 0;
    stationaryActive_ = false;
    stationarySinceMs_ = 0;
}

float SpeedLearningTracker::filterMedian3(float s0, float s1, float s2) {
    if (s0 > s1) std::swap(s0, s1);
    if (s1 > s2) std::swap(s1, s2);
    if (s0 > s1) std::swap(s0, s1);
    return s1;
}

float SpeedLearningTracker::computeAdaptiveAlpha(uint8_t confidencePct, uint8_t priorUpdates) const {
    const float confWeight = static_cast<float>(confidencePct) / 100.0f;
    const float histWeight = 1.0f / (1.0f + 0.15f * static_cast<float>(priorUpdates));
    const float alpha = config_.ewmaBaseAlpha * confWeight * histWeight;
    return std::max(0.02f, std::min(config_.ewmaBaseAlpha, alpha));
}

int8_t SpeedLearningTracker::findOrAllocateCandidateIndex(float commandedSpeedKmh, uint32_t nowMs) {
    // 1. Hybrid merge distance check with existing candidates
    for (size_t i = 0; i < kMaxTrackedCandidates; ++i) {
        if (candidates_[i].active) {
            const float mergeTol = std::max(config_.hybridMergeAbsKmh, config_.hybridMergePct * candidates_[i].commandedSpeedKmh);
            if (std::fabs(commandedSpeedKmh - candidates_[i].commandedSpeedKmh) <= mergeTol) {
                return static_cast<int8_t>(i);
            }
        }
    }

    // 2. Find empty slot
    for (size_t i = 0; i < kMaxTrackedCandidates; ++i) {
        if (!candidates_[i].active) {
            candidates_[i].active = true;
            candidates_[i].commandedSpeedKmh = commandedSpeedKmh;
            candidates_[i].measuredSpeedMeanKmh = 0.0f;
            candidates_[i].measuredSpeedM2 = 0.0f;
            candidates_[i].sampleCount = 0;
            candidates_[i].transientAnomalyTicks = 0;
            candidates_[i].firstObservedMs = nowMs;
            candidates_[i].lastObservedMs = nowMs;
            candidates_[i].updateCountLifetime = 0;
            candidateCount_++;
            return static_cast<int8_t>(i);
        }
    }

    // 3. All 10 slots active: evict lowest confidence candidate
    evictLowestConfidenceCandidate();

    for (size_t i = 0; i < kMaxTrackedCandidates; ++i) {
        if (!candidates_[i].active) {
            candidates_[i].active = true;
            candidates_[i].commandedSpeedKmh = commandedSpeedKmh;
            candidates_[i].measuredSpeedMeanKmh = 0.0f;
            candidates_[i].measuredSpeedM2 = 0.0f;
            candidates_[i].sampleCount = 0;
            candidates_[i].transientAnomalyTicks = 0;
            candidates_[i].firstObservedMs = nowMs;
            candidates_[i].lastObservedMs = nowMs;
            candidates_[i].updateCountLifetime = 0;
            candidateCount_++;
            return static_cast<int8_t>(i);
        }
    }

    return -1;
}

void SpeedLearningTracker::evictLowestConfidenceCandidate() {
    int8_t lowestIdx = -1;
    uint32_t minSamples = UINT32_MAX;

    for (size_t i = 0; i < kMaxTrackedCandidates; ++i) {
        if (candidates_[i].active) {
            if (candidates_[i].sampleCount < minSamples) {
                minSamples = candidates_[i].sampleCount;
                lowestIdx = static_cast<int8_t>(i);
            }
        }
    }

    // Only evict if the lowest candidate has less than 15s (750 ticks) of steady data
    if (lowestIdx >= 0 && minSamples < 750) {
        candidates_[lowestIdx] = TrackedCandidate{};
        if (candidateCount_ > 0) {
            candidateCount_--;
        }
    }
}

uint8_t SpeedLearningTracker::calculateConfidence(const TrackedCandidate& cand) const {
    if (!cand.active || cand.sampleCount < 100) {
        return 0;
    }

    const float steadySec = static_cast<float>(cand.sampleCount) / 50.0f;
    if (steadySec < config_.minSteadySecondsToCommit) {
        return 0;
    }

    const float anomalyRatio = static_cast<float>(cand.transientAnomalyTicks) / static_cast<float>(cand.sampleCount);
    if (anomalyRatio > config_.maxTransientOutlierRatio) {
        return 0;
    }

    const float variance = cand.measuredSpeedM2 / static_cast<float>(cand.sampleCount - 1);
    if (!std::isfinite(variance) || variance < 0.0f) {
        return 0;
    }
    const float stdDev = std::sqrt(variance);
    if (stdDev > config_.maxStdDevKmh) {
        return 0;
    }

    float timeRatio = (steadySec - config_.minSteadySecondsToCommit) /
                      std::max(1.0f, (config_.targetConfidenceSeconds - config_.minSteadySecondsToCommit));
    timeRatio = std::max(0.0f, std::min(1.0f, timeRatio));
    const float timeWeight = 0.70f + 0.30f * timeRatio;

    float varRatio = (stdDev - 0.03f) / 0.05f;
    varRatio = std::max(0.0f, std::min(1.0f, varRatio));
    const float varWeight = 1.0f - 0.40f * varRatio;

    const float conf = timeWeight * varWeight * 100.0f;
    return static_cast<uint8_t>(std::round(std::max(0.0f, std::min(100.0f, conf))));
}

float SpeedLearningTracker::predictCommand(const SpeedConfig& config, float measuredKmh) const {
    if (!config.commandMapValid || config.pointCount < 2) {
        return std::max(0.8f, std::min(25.0f, measuredKmh));
    }
    const size_t count = config.pointCount;
    const float x0 = config.points[0].measuredPhysicalSpeedKmh;
    const float y0 = config.points[0].treadmillCommandKmh;
    const float xLast = config.points[count - 1].measuredPhysicalSpeedKmh;
    const float yLast = config.points[count - 1].treadmillCommandKmh;

    // Case 1: Interpolated between X_0 and X_{N-1}
    if (measuredKmh >= x0 && measuredKmh <= xLast) {
        for (size_t i = 0; i < count - 1; ++i) {
            const float segX0 = config.points[i].measuredPhysicalSpeedKmh;
            const float segX1 = config.points[i + 1].measuredPhysicalSpeedKmh;
            if (measuredKmh >= segX0 && measuredKmh <= segX1) {
                const float segY0 = config.points[i].treadmillCommandKmh;
                const float segY1 = config.points[i + 1].treadmillCommandKmh;
                const float segDx = segX1 - segX0;
                if (segDx <= 0.001f) return segY0;
                return segY0 + ((segY1 - segY0) / segDx) * (measuredKmh - segX0);
            }
        }
        return yLast;
    }

    // Case 2: Upper Extrapolation (measuredKmh > X_{N-1})
    if (measuredKmh > xLast) {
        const float xPen = config.points[count - 2].measuredPhysicalSpeedKmh;
        const float yPen = config.points[count - 2].treadmillCommandKmh;
        const float segDx = xLast - xPen;
        if (segDx <= 0.001f) return yLast;
        const float mUpper = (yLast - yPen) / segDx;
        return yLast + mUpper * (measuredKmh - xLast);
    }

    // Case 3: Lower Extrapolation (measuredKmh < X_0)
    const float x1 = config.points[1].measuredPhysicalSpeedKmh;
    const float y1 = config.points[1].treadmillCommandKmh;
    const float segDx = x1 - x0;
    if (segDx <= 0.001f) return y0;
    const float mLower = (y1 - y0) / segDx;
    return y0 + mLower * (measuredKmh - x0);
}

bool SpeedLearningTracker::evaluateCrossPointConsistency(float measuredKmh, float commandKmh) const {
    if (!std::isfinite(measuredKmh) || !std::isfinite(commandKmh) || measuredKmh <= 0.0f || commandKmh <= 0.0f) {
        return false;
    }

    const size_t count = activeConfig_.pointCount;
    if (!activeConfig_.commandMapValid || count < 2) {
        // Uncalibrated / single-point fallback: compare against nominal 1:1 factory identity
        const float cPred = std::max(0.8f, std::min(25.0f, measuredKmh));
        return std::fabs(commandKmh - cPred) <= config_.crossPointToleranceKmh;
    }

    const float cPred = predictCommand(activeConfig_, measuredKmh);
    if (std::fabs(commandKmh - cPred) > config_.crossPointToleranceKmh) {
        return false;
    }

    const float x0 = activeConfig_.points[0].measuredPhysicalSpeedKmh;
    const float y0 = activeConfig_.points[0].treadmillCommandKmh;
    const float xLast = activeConfig_.points[count - 1].measuredPhysicalSpeedKmh;
    const float yLast = activeConfig_.points[count - 1].treadmillCommandKmh;

    // Case 1: Interpolated between X_0 and X_{N-1}
    if (measuredKmh >= x0 && measuredKmh <= xLast) {
        for (size_t i = 0; i < count - 1; ++i) {
            const float segX0 = activeConfig_.points[i].measuredPhysicalSpeedKmh;
            const float segX1 = activeConfig_.points[i + 1].measuredPhysicalSpeedKmh;
            if (measuredKmh >= segX0 && measuredKmh <= segX1) {
                const float segY0 = activeConfig_.points[i].treadmillCommandKmh;
                const float segY1 = activeConfig_.points[i + 1].treadmillCommandKmh;

                if (measuredKmh - segX0 > 0.05f) {
                    const float m1 = (commandKmh - segY0) / (measuredKmh - segX0);
                    if (m1 < 0.75f || m1 > 1.35f) return false;
                }
                if (segX1 - measuredKmh > 0.05f) {
                    const float m2 = (segY1 - commandKmh) / (segX1 - measuredKmh);
                    if (m2 < 0.75f || m2 > 1.35f) return false;
                }
                return true;
            }
        }
    }

    // Case 2: Upper Extrapolation (measuredKmh > X_{N-1})
    if (measuredKmh > xLast) {
        const float xPen = activeConfig_.points[count - 2].measuredPhysicalSpeedKmh;
        const float yPen = activeConfig_.points[count - 2].treadmillCommandKmh;
        const float segDx = xLast - xPen;
        if (segDx <= 0.001f) return false;

        const float mUpper = (yLast - yPen) / segDx;
        if (mUpper < 0.75f || mUpper > 1.35f) return false;
        return commandKmh > yLast;
    }

    // Case 3: Lower Extrapolation (measuredKmh < X_0)
    if (measuredKmh < x0) {
        const float x1 = activeConfig_.points[1].measuredPhysicalSpeedKmh;
        const float y1 = activeConfig_.points[1].treadmillCommandKmh;
        const float segDx = x1 - x0;
        if (segDx <= 0.001f) return false;

        const float mLower = (y1 - y0) / segDx;
        if (mLower < 0.75f || mLower > 1.35f) return false;
        return commandKmh < y0;
    }

    return false;
}


bool SpeedLearningTracker::evaluateFactoryBaselineSafety(float measuredKmh, float commandKmh) const {
    if (!std::isfinite(measuredKmh) || !std::isfinite(commandKmh) || measuredKmh <= 0.0f || commandKmh <= 0.0f) {
        return false;
    }
    // Evaluated strictly against immutable factory ROM baseline (nominal 1:1 identity)
    const float cFactory = std::max(0.8f, std::min(25.0f, measuredKmh));
    const float delta = std::fabs(commandKmh - cFactory);
    const float pctError = (delta / cFactory) * 100.0f;

    return (pctError <= config_.maxBaselineSafetyPct) && (delta <= config_.maxBaselineSafetyAbsKmh);
}

void SpeedLearningTracker::update(
    float measuredPhysicalSpeedKmh,
    bool speedMeasurementValid,
    float activeCommandedSpeedKmh,
    CsafeMachineState csafeState,
    bool controllerBusy,
    bool estopActive,
    bool rampPreFireActive,
    uint32_t nowMs
) {
    // 1. Validate preconditions
    if (csafeState != CsafeMachineState::InUse ||
        !speedMeasurementValid ||
        !std::isfinite(measuredPhysicalSpeedKmh) ||
        measuredPhysicalSpeedKmh <= 0.0f ||
        controllerBusy ||
        estopActive ||
        rampPreFireActive ||
        !std::isfinite(activeCommandedSpeedKmh) ||
        activeCommandedSpeedKmh < 0.8f ||
        activeCommandedSpeedKmh > 25.0f) {
        // Reset command settling if conditions invalidated
        commandStableSinceMs_ = nowMs;
        medianCount_ = 0;
        return;
    }

    // 2. Track command stability
    if (std::fabs(activeCommandedSpeedKmh - lastCommandedSpeedKmh_) > 0.05f) {
        lastCommandedSpeedKmh_ = activeCommandedSpeedKmh;
        commandStableSinceMs_ = nowMs;
        medianCount_ = 0;
        return;
    }

    // 3. Settling time gate
    const uint32_t settleMs = static_cast<uint32_t>(config_.minCommandSettledSeconds * 1000.0f);
    if (nowMs - commandStableSinceMs_ < settleMs) {
        return;
    }

    // 4. Pre-filter Stage 1: 3-tap median
    medianBuffer_[0] = medianBuffer_[1];
    medianBuffer_[1] = medianBuffer_[2];
    medianBuffer_[2] = measuredPhysicalSpeedKmh;
    if (medianCount_ < 3) {
        medianCount_++;
        return;
    }
    const float filteredSpeed = filterMedian3(medianBuffer_[0], medianBuffer_[1], medianBuffer_[2]);

    // 5. Find or allocate candidate slot
    const int8_t candIdx = findOrAllocateCandidateIndex(activeCommandedSpeedKmh, nowMs);
    if (candIdx < 0) {
        return;
    }
    auto& cand = candidates_[candIdx];

    // 6. Pre-filter Stage 2: Gated innovation outlier check
    if (cand.sampleCount >= 50) {
        const float delta = std::fabs(filteredSpeed - cand.measuredSpeedMeanKmh);
        if (delta > config_.transientOutlierThresholdKmh) {
            cand.transientAnomalyTicks++;
            cand.lastObservedMs = nowMs;
            return; // Skip Welford update for this tick
        }
    }

    // 7. Welford online update
    cand.sampleCount++;
    const float d1 = filteredSpeed - cand.measuredSpeedMeanKmh;
    cand.measuredSpeedMeanKmh += d1 / static_cast<float>(cand.sampleCount);
    const float d2 = filteredSpeed - cand.measuredSpeedMeanKmh;
    cand.measuredSpeedM2 += d1 * d2;
    cand.lastObservedMs = nowMs;
}

bool SpeedLearningTracker::checkBeltStoppedTrigger(
    float measuredSpeedKmh,
    CsafeMachineState csafeState,
    SpeedConfig& activeConfigInOut,
    uint32_t nowMs
) {
    const bool beltStationary = (measuredSpeedKmh < 0.5f) && (csafeState != CsafeMachineState::InUse);

    if (beltStationary) {
        if (!stationaryActive_) {
            stationaryActive_ = true;
            stationarySinceMs_ = nowMs;
        } else if (nowMs - stationarySinceMs_ >= 3000) {
            // Confirmed stopped! Reset debounce so commit runs once per stop
            stationaryActive_ = false;

            bool anyCommitted = false;
            for (size_t i = 0; i < kMaxTrackedCandidates; ++i) {
                auto& cand = candidates_[i];
                if (!cand.active) continue;

                const uint8_t conf = calculateConfidence(cand);
                if (conf < config_.minConfidenceToCommit) continue;

                const float measuredKmh = cand.measuredSpeedMeanKmh;
                const float commandedKmh = cand.commandedSpeedKmh;

                // Validate safety against immutable factory baseline and cross-point consistency
                if (!evaluateFactoryBaselineSafety(measuredKmh, commandedKmh)) continue;
                if (!evaluateCrossPointConsistency(measuredKmh, commandedKmh)) continue;

                // Candidate qualifies! Find existing point in activeConfigInOut or insert
                int8_t matchIdx = -1;
                for (size_t p = 0; p < activeConfigInOut.pointCount; ++p) {
                    const float tol = std::max(config_.hybridMergeAbsKmh, config_.hybridMergePct * activeConfigInOut.points[p].measuredPhysicalSpeedKmh);
                    if (std::fabs(measuredKmh - activeConfigInOut.points[p].measuredPhysicalSpeedKmh) <= tol) {
                        matchIdx = static_cast<int8_t>(p);
                        break;
                    }
                }

                float prevCmd = measuredKmh; // Nominal identity fallback
                float newCmd = commandedKmh;

                if (matchIdx >= 0) {
                    prevCmd = activeConfigInOut.points[matchIdx].treadmillCommandKmh;
                    const float alpha = computeAdaptiveAlpha(conf, cand.updateCountLifetime);
                    float delta = alpha * (commandedKmh - prevCmd);
                    delta = std::max(-config_.maxAdaptationStepKmh, std::min(config_.maxAdaptationStepKmh, delta));

                    // Dirty flag requirement (item 8): skip if EWMA delta is negligible (no NVS write)
                    if (std::fabs(delta) < config_.minAdaptationDeltaKmh) {
                        continue;
                    }

                    newCmd = prevCmd + delta;
                    activeConfigInOut.points[matchIdx].measuredPhysicalSpeedKmh = measuredKmh;
                    activeConfigInOut.points[matchIdx].treadmillCommandKmh = newCmd;
                } else {
                    // New-point insertion case:
                    // If current table already has valid interpolation/extrapolation (pointCount >= 2),
                    // check whether candidate's own measured value already matches what the current table predicts.
                    if (activeConfigInOut.commandMapValid && activeConfigInOut.pointCount >= 2) {
                        const float cPred = predictCommand(activeConfigInOut, measuredKmh);
                        if (std::fabs(commandedKmh - cPred) <= config_.newPointPredictionToleranceKmh) {
                            // Table already predicts this speed within tolerance: skip redundant point insertion
                            continue;
                        }
                    }

                    if (activeConfigInOut.pointCount >= kMaxSpeedCalibrationPoints) {
                        continue; // Table full
                    }

                    prevCmd = measuredKmh; // Factory identity baseline
                    const float alpha = computeAdaptiveAlpha(conf, 0);
                    float delta = alpha * (commandedKmh - prevCmd);
                    delta = std::max(-config_.maxAdaptationStepKmh, std::min(config_.maxAdaptationStepKmh, delta));
                    newCmd = prevCmd + delta;

                    const size_t insertIdx = activeConfigInOut.pointCount++;
                    activeConfigInOut.points[insertIdx].measuredPhysicalSpeedKmh = measuredKmh;
                    activeConfigInOut.points[insertIdx].treadmillCommandKmh = newCmd;
                }

                // Sort points ascending by measuredPhysicalSpeedKmh
                std::sort(activeConfigInOut.points.begin(), activeConfigInOut.points.begin() + activeConfigInOut.pointCount,
                    [](const SpeedCalibrationPoint& a, const SpeedCalibrationPoint& b) {
                        return a.measuredPhysicalSpeedKmh < b.measuredPhysicalSpeedKmh;
                    });

                // Update validity flag if >= 2 points
                if (activeConfigInOut.pointCount >= 2) {
                    bool strictlyAscending = true;
                    for (size_t p = 1; p < activeConfigInOut.pointCount; ++p) {
                        if (activeConfigInOut.points[p].measuredPhysicalSpeedKmh <= activeConfigInOut.points[p - 1].measuredPhysicalSpeedKmh ||
                            activeConfigInOut.points[p].treadmillCommandKmh <= activeConfigInOut.points[p - 1].treadmillCommandKmh) {
                            strictlyAscending = false;
                            break;
                        }
                    }
                    activeConfigInOut.commandMapValid = strictlyAscending;
                }

                cand.updateCountLifetime++;

                // Append audit log
                SpeedAdaptationLogEntry entry{};
                entry.timestampMs = nowMs;
                entry.measuredPhysicalSpeedKmh = measuredKmh;
                entry.previousCommandKmh = prevCmd;
                entry.newCommandKmh = newCmd;
                entry.deltaKmh = newCmd - prevCmd;
                entry.accumulatedSteadySeconds = static_cast<float>(cand.sampleCount) / 50.0f;
                entry.confidencePct = conf;
                appendAuditLog(entry);

                anyCommitted = true;
            }

            resetSession();
            return anyCommitted;
        }
    } else {
        stationaryActive_ = false;
        stationarySinceMs_ = 0;
    }

    return false;
}

void SpeedLearningTracker::appendAuditLog(const SpeedAdaptationLogEntry& entry) {
    auditLog_[auditLogHead_] = entry;
    auditLogHead_ = (auditLogHead_ + 1) % kMaxAuditLogEntries;
    if (auditLogCount_ < kMaxAuditLogEntries) {
        auditLogCount_++;
    }
}

uint8_t SpeedLearningTracker::getAuditLog(SpeedAdaptationLogEntry* outEntries, uint8_t maxEntries) const {
    if (!outEntries || maxEntries == 0) return 0;
    const uint8_t n = std::min(auditLogCount_, maxEntries);
    for (uint8_t i = 0; i < n; ++i) {
        // Most recent first: index (head - 1 - i + kMaxAuditLogEntries) % kMaxAuditLogEntries
        const uint8_t idx = (auditLogHead_ + kMaxAuditLogEntries - 1 - i) % kMaxAuditLogEntries;
        outEntries[i] = auditLog_[idx];
    }
    return n;
}

} // namespace stridecontrol
