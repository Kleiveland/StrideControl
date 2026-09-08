#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include "SettingsTypes.h"

namespace stridecontrol {

static constexpr size_t MAX_EXPANDED_WORKOUT_STEPS = 96;

// Order strictly by size to minimize struct padding:
// 4-byte fields: durationValue, targetSpeedKmh
// 2-byte fields: repNumber, totalRepsInGroup, segmentId, originalStepId
// 1-byte fields: stepIndex, targetInclinePct, role, durationType, speedMode, setIncline
struct ExpandedStep {
    uint32_t durationValue = 0;
    float targetSpeedKmh = 0.0f;
    uint16_t repNumber = 0;
    uint16_t totalRepsInGroup = 0;
    uint16_t segmentId = 0;
    uint16_t originalStepId = 0;
    uint8_t stepIndex = 0;
    uint8_t targetInclinePct = 0;
    StepRole role = StepRole::WORK;
    DurationType durationType = DurationType::TIME_SECONDS;
    SpeedMode speedMode = SpeedMode::FIXED;
    bool setIncline = false;
};

struct ExpandedWorkout {
    uint16_t workoutId = 0;
    uint32_t totalEstimatedDurationSeconds = 0;
    char workoutName[MAX_WORKOUT_NAME_LENGTH + 1] = {};
    uint8_t totalSteps = 0;
    std::array<ExpandedStep, MAX_EXPANDED_WORKOUT_STEPS> steps{};
};

} // namespace stridecontrol

