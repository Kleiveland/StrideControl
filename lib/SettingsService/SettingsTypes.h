#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace stridecontrol {

constexpr size_t MAX_USERS = 4;
constexpr size_t MAX_WORKOUTS_PER_USER = 16;
constexpr size_t MAX_SEGMENTS_PER_WORKOUT = 16;
constexpr size_t MAX_STEPS_PER_GROUP = 8;
constexpr size_t MAX_WORKOUT_NAME_LENGTH = 31;
constexpr size_t MAX_USER_NAME_LENGTH = 16;

enum class DurationType : uint8_t {
    TIME_SECONDS = 0,
    METERS = 1
};

enum class SpeedMode : uint8_t {
    FIXED = 0,
    FREE = 1
};

enum class StepRole : uint8_t {
    WARMUP = 0,
    WORK = 1,
    REST = 2,
    COOLDOWN = 3
};

enum class SegmentType : uint8_t {
    SINGLE_STEP = 0,
    REPEATING_GROUP = 1
};

struct WorkoutStep {
    uint16_t id = 0;
    StepRole role = StepRole::WORK;
    DurationType durationType = DurationType::TIME_SECONDS;
    uint32_t durationValue = 0; // seconds or meters
    SpeedMode speedMode = SpeedMode::FIXED;
    float targetSpeedKmh = 0.0f;
    uint8_t targetInclinePct = 0;
    bool setIncline = false;

    WorkoutStep() = default;
    WorkoutStep(uint16_t id_, StepRole role_, DurationType durType_, uint32_t durVal_,
                SpeedMode spdMode_, float spdKmh_, uint8_t incPct_ = 0, bool setInc_ = false)
        : id(id_), role(role_), durationType(durType_), durationValue(durVal_),
          speedMode(spdMode_), targetSpeedKmh(spdKmh_), targetInclinePct(incPct_), setIncline(setInc_) {}
};

struct WorkoutSegment {
    uint16_t id = 0;
    SegmentType type = SegmentType::SINGLE_STEP;
    uint16_t repetitions = 1; // 1 if SINGLE_STEP, >=1 if REPEATING_GROUP
    float startSpeedKmh = 0.0f;
    float speedProgressionPerRepKmh = 0.0f;
    std::array<WorkoutStep, MAX_STEPS_PER_GROUP> steps{};
    uint8_t stepCount = 0;
};

struct WorkoutDefinition {
    uint16_t id = 0;
    char name[MAX_WORKOUT_NAME_LENGTH + 1] = {};
    std::array<WorkoutSegment, MAX_SEGMENTS_PER_WORKOUT> segments{};
    uint8_t segmentCount = 0;
    uint32_t lastUsedTimestamp = 0;
};

struct UserProfile {
    uint8_t id = 0;
    char name[MAX_USER_NAME_LENGTH + 1] = {};
    float hvileSpeedKmh = 6.0f;
    float dragSpeedKmh = 15.0f;
    std::array<float, 8> speedQuickKeys{};
    std::array<uint8_t, 8> inclineQuickKeys{};
    std::array<WorkoutDefinition, MAX_WORKOUTS_PER_USER> workouts{};
    uint8_t workoutCount = 0;
    uint16_t selectedWorkoutId = 0;
    std::array<uint16_t, 2> recentWorkoutIds{};
};

struct SystemSettings {
    uint8_t schemaVersion = 1;
    std::array<UserProfile, MAX_USERS> users{};
};

// String conversion helpers for serialization
const char* durationTypeName(DurationType type);
const char* speedModeName(SpeedMode mode);
const char* stepRoleName(StepRole role);
const char* segmentTypeName(SegmentType type);

DurationType parseDurationType(const char* str);
SpeedMode parseSpeedMode(const char* str);
StepRole parseStepRole(const char* str);
SegmentType parseSegmentType(const char* str);

} // namespace stridecontrol

