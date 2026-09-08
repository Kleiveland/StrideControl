#include "WorkoutExpander.h"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace stridecontrol {

bool WorkoutExpander::expand(
    const WorkoutDefinition& input,
    ExpandedWorkout& output,
    char* errorMsg,
    size_t errorMsgLen,
    float maxAchievableSpeedKmh
) {
    auto setError = [&](const char* msg) {
        output = ExpandedWorkout{};
        if (errorMsg != nullptr && errorMsgLen > 0) {
            strncpy(errorMsg, msg, errorMsgLen - 1);
            errorMsg[errorMsgLen - 1] = '\0';
        }
        return false;
    };

    // 1. Initial reset
    output = ExpandedWorkout{};

    // 2. Validate basic definition
    if (input.segmentCount == 0 || input.segmentCount > MAX_SEGMENTS_PER_WORKOUT) {
        return setError("Segment count out of range");
    }

    // 3. Pre-validate all segments and steps & pre-calculate total required steps
    size_t totalRequiredSteps = 0;
    for (size_t segIdx = 0; segIdx < input.segmentCount; ++segIdx) {
        const WorkoutSegment& seg = input.segments[segIdx];

        if (seg.stepCount == 0 || seg.stepCount > MAX_STEPS_PER_GROUP) {
            return setError("Invalid segment step count");
        }

        bool isZeroDurationBypassed = false;
        if (seg.type == SegmentType::SINGLE_STEP && seg.stepCount == 1) {
            if ((segIdx == 0 && seg.steps[0].role == StepRole::WARMUP && seg.steps[0].durationValue == 0) ||
                (segIdx + 1 == input.segmentCount && seg.steps[0].role == StepRole::COOLDOWN && seg.steps[0].durationValue == 0)) {
                isZeroDurationBypassed = true;
            }
        }

        bool nextIsCooldown = (segIdx + 1 < input.segmentCount) &&
                              (input.segments[segIdx + 1].stepCount > 0) &&
                              (input.segments[segIdx + 1].steps[0].role == StepRole::COOLDOWN);

        if (seg.type == SegmentType::SINGLE_STEP) {
            if (seg.repetitions != 1) {
                return setError("Single step segment repetitions must be 1");
            }
            if (!isZeroDurationBypassed) {
                totalRequiredSteps += 1;
            }
        } else if (seg.type == SegmentType::REPEATING_GROUP) {
            if (seg.repetitions < 1) {
                return setError("Repeating group repetitions must be >= 1");
            }
            size_t segSteps = static_cast<size_t>(seg.repetitions) * seg.stepCount;
            // If the repeating group ends with REST and is immediately followed by COOLDOWN,
            // the final repetition's trailing REST is stripped.
            if (nextIsCooldown && seg.steps[seg.stepCount - 1].role == StepRole::REST) {
                segSteps -= 1;
            }
            totalRequiredSteps += segSteps;
        } else {
            return setError("Unsupported segment type");
        }

        for (size_t stepIdx = 0; stepIdx < seg.stepCount; ++stepIdx) {
            const WorkoutStep& st = seg.steps[stepIdx];

            if (st.durationValue == 0 && !isZeroDurationBypassed) {
                return setError("Step duration must be greater than zero");
            }

            if (st.durationType != DurationType::TIME_SECONDS && st.durationType != DurationType::METERS) {
                return setError("Unsupported duration type");
            }

            if (st.speedMode != SpeedMode::FIXED && st.speedMode != SpeedMode::FREE) {
                return setError("Unsupported speed mode");
            }

            if (st.role != StepRole::WARMUP && st.role != StepRole::WORK &&
                st.role != StepRole::REST && st.role != StepRole::COOLDOWN) {
                return setError("Unsupported step role");
            }
        }
    }

    if (totalRequiredSteps == 0) {
        return setError("Workout contains no steps");
    }

    if (totalRequiredSteps > MAX_EXPANDED_WORKOUT_STEPS) {
        return setError("Expanded workout exceeds step limit");
    }

    // 4. Perform deterministic expansion
    output.workoutId = input.id;
    strncpy(output.workoutName, input.name, sizeof(output.workoutName) - 1);
    output.workoutName[sizeof(output.workoutName) - 1] = '\0';

    uint8_t outStepIdx = 0;
    uint64_t totalDurationAccumulator = 0;
    float lastExplicitSpeedKmh = 0.0f;
    bool hasPriorExplicitSpeed = false;

    for (size_t segIdx = 0; segIdx < input.segmentCount; ++segIdx) {
        const WorkoutSegment& seg = input.segments[segIdx];

        bool isZeroDurationBypassed = false;
        if (seg.type == SegmentType::SINGLE_STEP && seg.stepCount == 1) {
            if ((segIdx == 0 && seg.steps[0].role == StepRole::WARMUP && seg.steps[0].durationValue == 0) ||
                (segIdx + 1 == input.segmentCount && seg.steps[0].role == StepRole::COOLDOWN && seg.steps[0].durationValue == 0)) {
                isZeroDurationBypassed = true;
            }
        }

        if (isZeroDurationBypassed) {
            // Skipped boundary segment: must not consume expanded-step capacity or produce timeline entries
            continue;
        }

        bool nextIsCooldown = (segIdx + 1 < input.segmentCount) &&
                              (input.segments[segIdx + 1].stepCount > 0) &&
                              (input.segments[segIdx + 1].steps[0].role == StepRole::COOLDOWN);

        if (seg.type == SegmentType::SINGLE_STEP) {
            const WorkoutStep& srcStep = seg.steps[0];
            ExpandedStep& outStep = output.steps[outStepIdx];

            outStep.stepIndex = outStepIdx;
            outStep.repNumber = 1;
            outStep.totalRepsInGroup = 1;
            outStep.segmentId = seg.id;
            outStep.originalStepId = srcStep.id;
            outStep.role = srcStep.role;
            outStep.durationType = srcStep.durationType;
            outStep.durationValue = srcStep.durationValue;
            outStep.speedMode = srcStep.speedMode;
            outStep.targetInclinePct = srcStep.targetInclinePct;
            outStep.setIncline = srcStep.setIncline;

            // Speed determination
            if (srcStep.speedMode == SpeedMode::FIXED) {
                if (srcStep.targetSpeedKmh > maxAchievableSpeedKmh) {
                    return setError("Target speed exceeds maximum achievable speed");
                }
                if (srcStep.targetSpeedKmh < 0.5f) {
                    return setError("Target speed below minimum speed");
                }
                outStep.targetSpeedKmh = srcStep.targetSpeedKmh;
                lastExplicitSpeedKmh = srcStep.targetSpeedKmh;
                hasPriorExplicitSpeed = true;
            } else { // SpeedMode::FREE
                if (srcStep.targetSpeedKmh >= 0.5f) {
                    if (srcStep.targetSpeedKmh > maxAchievableSpeedKmh) {
                        return setError("Target speed exceeds maximum achievable speed");
                    }
                    outStep.targetSpeedKmh = srcStep.targetSpeedKmh;
                    lastExplicitSpeedKmh = srcStep.targetSpeedKmh;
                    hasPriorExplicitSpeed = true;
                } else if (srcStep.role == StepRole::REST && hasPriorExplicitSpeed) {
                    outStep.targetSpeedKmh = lastExplicitSpeedKmh;
                } else {
                    outStep.targetSpeedKmh = 0.0f;
                }
            }

            uint32_t stepEstDuration = 0;
            if (outStep.durationType == DurationType::TIME_SECONDS) {
                stepEstDuration = outStep.durationValue;
            } else { // METERS
                float effectiveSpeed = (outStep.speedMode == SpeedMode::FREE || outStep.targetSpeedKmh < 0.5f)
                                           ? 10.0f
                                           : outStep.targetSpeedKmh;
                if (effectiveSpeed < 0.5f) effectiveSpeed = 0.5f;
                stepEstDuration = static_cast<uint32_t>(std::ceil((static_cast<double>(outStep.durationValue) * 3.6) / effectiveSpeed));
            }
            totalDurationAccumulator += stepEstDuration;
            outStepIdx++;
        } else if (seg.type == SegmentType::REPEATING_GROUP) {
            for (uint16_t rep = 0; rep < seg.repetitions; ++rep) {
                bool isFinalRep = (rep == seg.repetitions - 1);

                for (size_t s = 0; s < seg.stepCount; ++s) {
                    const WorkoutStep& srcStep = seg.steps[s];

                    // Strip redundant final REST on the last rep if immediately followed by COOLDOWN
                    if (isFinalRep && (s == seg.stepCount - 1) && (srcStep.role == StepRole::REST) && nextIsCooldown) {
                        continue;
                    }

                    ExpandedStep& outStep = output.steps[outStepIdx];

                    outStep.stepIndex = outStepIdx;
                    outStep.repNumber = static_cast<uint16_t>(rep + 1);
                    outStep.totalRepsInGroup = seg.repetitions;
                    outStep.segmentId = seg.id;
                    outStep.originalStepId = srcStep.id;
                    outStep.role = srcStep.role;
                    outStep.durationType = srcStep.durationType;
                    outStep.durationValue = srcStep.durationValue;
                    outStep.speedMode = srcStep.speedMode;
                    outStep.targetInclinePct = srcStep.targetInclinePct;
                    outStep.setIncline = srcStep.setIncline;

                    // Speed calculation & validation
                    if (srcStep.role == StepRole::WORK) {
                        float calcSpeed = seg.startSpeedKmh + (static_cast<float>(rep) * seg.speedProgressionPerRepKmh);
                        if (calcSpeed > maxAchievableSpeedKmh) {
                            return setError("Target speed exceeds maximum achievable speed");
                        }
                        if (calcSpeed < 0.5f) {
                            return setError("Target speed below minimum speed");
                        }
                        outStep.targetSpeedKmh = calcSpeed;
                        lastExplicitSpeedKmh = calcSpeed;
                        hasPriorExplicitSpeed = true;
                    } else if (srcStep.role == StepRole::REST) {
                        if (srcStep.speedMode == SpeedMode::FIXED && srcStep.targetSpeedKmh >= 0.5f) {
                            if (srcStep.targetSpeedKmh > maxAchievableSpeedKmh) {
                                return setError("Target speed exceeds maximum achievable speed");
                            }
                            outStep.targetSpeedKmh = srcStep.targetSpeedKmh;
                            lastExplicitSpeedKmh = srcStep.targetSpeedKmh;
                            hasPriorExplicitSpeed = true;
                        } else if (srcStep.speedMode == SpeedMode::FREE && srcStep.targetSpeedKmh >= 0.5f) {
                            if (srcStep.targetSpeedKmh > maxAchievableSpeedKmh) {
                                return setError("Target speed exceeds maximum achievable speed");
                            }
                            outStep.targetSpeedKmh = srcStep.targetSpeedKmh;
                            lastExplicitSpeedKmh = srcStep.targetSpeedKmh;
                            hasPriorExplicitSpeed = true;
                        } else if (hasPriorExplicitSpeed) {
                            // Inherited REST speed from nearest prior explicit-speed step
                            // (which reflects the current rep's progressive WORK speed if WORK preceded REST)
                            outStep.targetSpeedKmh = lastExplicitSpeedKmh;
                        } else {
                            // FREE / manual fallback when no preceding explicit speed exists
                            outStep.targetSpeedKmh = 0.0f;
                            outStep.speedMode = SpeedMode::FREE;
                        }
                    } else {
                        // Other roles in repeating group
                        if (srcStep.targetSpeedKmh >= 0.5f) {
                            if (srcStep.targetSpeedKmh > maxAchievableSpeedKmh) {
                                return setError("Target speed exceeds maximum achievable speed");
                            }
                            outStep.targetSpeedKmh = srcStep.targetSpeedKmh;
                            lastExplicitSpeedKmh = srcStep.targetSpeedKmh;
                            hasPriorExplicitSpeed = true;
                        } else if (hasPriorExplicitSpeed) {
                            outStep.targetSpeedKmh = lastExplicitSpeedKmh;
                        } else {
                            outStep.targetSpeedKmh = 0.0f;
                            outStep.speedMode = SpeedMode::FREE;
                        }
                    }

                    uint32_t stepEstDuration = 0;
                    if (outStep.durationType == DurationType::TIME_SECONDS) {
                        stepEstDuration = outStep.durationValue;
                    } else { // METERS
                        float effectiveSpeed = (outStep.speedMode == SpeedMode::FREE || outStep.targetSpeedKmh < 0.5f)
                                                   ? 10.0f
                                                   : outStep.targetSpeedKmh;
                        if (effectiveSpeed < 0.5f) effectiveSpeed = 0.5f;
                        stepEstDuration = static_cast<uint32_t>(std::ceil((static_cast<double>(outStep.durationValue) * 3.6) / effectiveSpeed));
                    }
                    totalDurationAccumulator += stepEstDuration;
                    outStepIdx++;
                }
            }
        }
    }

    if (totalDurationAccumulator > UINT32_MAX) {
        return setError("Estimated duration exceeds 32-bit limit");
    }

    output.totalSteps = outStepIdx;
    output.totalEstimatedDurationSeconds = static_cast<uint32_t>(totalDurationAccumulator);

    if (errorMsg != nullptr && errorMsgLen > 0) {
        errorMsg[0] = '\0';
    }

    return true;
}

} // namespace stridecontrol


