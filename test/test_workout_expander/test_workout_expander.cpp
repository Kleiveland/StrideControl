#include <unity.h>
#ifdef ARDUINO
#include <Arduino.h>
#include <WiFi.h>
#endif
#include <cmath>
#include <cstring>
#include "WorkoutExecutionTypes.h"
#include "WorkoutExpander.h"
#include "WorkoutEngine.h"

using namespace stridecontrol;

static WorkoutDefinition createWorkout321() {
    WorkoutDefinition w{};
    w.id = 101;
    strncpy(w.name, "3-2-1 X 2", sizeof(w.name) - 1);
    w.segmentCount = 3;

    // Segment 0: Warmup
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {10, StepRole::WARMUP, DurationType::TIME_SECONDS, 600, SpeedMode::FREE, 9.0f, 0, false};

    // Segment 1: Interval (3-2-1 repeating group x2)
    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 2;
    w.segments[1].startSpeedKmh = 15.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.0f;
    w.segments[1].stepCount = 5;
    w.segments[1].steps[0] = {20, StepRole::WORK, DurationType::TIME_SECONDS, 180, SpeedMode::FIXED, 15.0f, 0, false};
    w.segments[1].steps[1] = {21, StepRole::REST, DurationType::TIME_SECONDS, 90,  SpeedMode::FREE,  6.0f,  0, false};
    w.segments[1].steps[2] = {22, StepRole::WORK, DurationType::TIME_SECONDS, 120, SpeedMode::FIXED, 15.0f, 0, false};
    w.segments[1].steps[3] = {23, StepRole::REST, DurationType::TIME_SECONDS, 90,  SpeedMode::FREE,  6.0f,  0, false};
    w.segments[1].steps[4] = {24, StepRole::WORK, DurationType::TIME_SECONDS, 60,  SpeedMode::FIXED, 15.0f, 0, false};

    // Segment 2: Cooldown
    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {30, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};

    return w;
}

static WorkoutDefinition createWorkout45_15() {
    WorkoutDefinition w{};
    w.id = 102;
    strncpy(w.name, "45 / 15 PROG", sizeof(w.name) - 1);
    w.segmentCount = 3;

    // Segment 0: Warmup
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {10, StepRole::WARMUP, DurationType::TIME_SECONDS, 600, SpeedMode::FREE, 9.0f, 0, false};

    // Segment 1: Interval (10 reps of 45s work @ 14.0 km/h + 0.2 km/h progression / 15s rest @ 6.0 km/h)
    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 10;
    w.segments[1].startSpeedKmh = 14.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.2f;
    w.segments[1].stepCount = 2;
    w.segments[1].steps[0] = {20, StepRole::WORK, DurationType::TIME_SECONDS, 45, SpeedMode::FIXED, 14.0f, 0, false};
    w.segments[1].steps[1] = {21, StepRole::REST, DurationType::TIME_SECONDS, 15, SpeedMode::FIXED, 6.0f,  0, false};

    // Segment 2: Cooldown
    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {30, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};

    return w;
}

void test_expansion_321_workout() {
    WorkoutDefinition w = createWorkout321();
    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    bool ok = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_STRING("", err);
    TEST_ASSERT_EQUAL_UINT16(101, output.workoutId);
    TEST_ASSERT_EQUAL_STRING("3-2-1 X 2", output.workoutName);

    // 1 warmup + (5 steps * 2 reps) + 1 cooldown = 12 steps (last step of group is WORK, so no REST stripped)
    TEST_ASSERT_EQUAL_UINT8(12, output.totalSteps);

    // Verify step index sequentiality and segment/step ID preservation
    for (uint8_t i = 0; i < 12; ++i) {
        TEST_ASSERT_EQUAL_UINT8(i, output.steps[i].stepIndex);
    }

    // Warmup
    TEST_ASSERT_EQUAL(StepRole::WARMUP, output.steps[0].role);
    TEST_ASSERT_EQUAL_UINT16(1, output.steps[0].repNumber);
    TEST_ASSERT_EQUAL_UINT16(1, output.steps[0].totalRepsInGroup);
    TEST_ASSERT_EQUAL_UINT16(1, output.steps[0].segmentId);
    TEST_ASSERT_EQUAL_UINT16(10, output.steps[0].originalStepId);

    // Rep 1 Interval (steps 1..5)
    TEST_ASSERT_EQUAL_UINT16(1, output.steps[1].repNumber);
    TEST_ASSERT_EQUAL_UINT16(2, output.steps[1].totalRepsInGroup);
    TEST_ASSERT_EQUAL_UINT16(2, output.steps[1].segmentId);
    TEST_ASSERT_EQUAL_UINT16(20, output.steps[1].originalStepId);
    TEST_ASSERT_EQUAL(StepRole::WORK, output.steps[1].role);

    // Rep 2 Interval (steps 6..10)
    TEST_ASSERT_EQUAL_UINT16(2, output.steps[6].repNumber);
    TEST_ASSERT_EQUAL_UINT16(2, output.steps[6].totalRepsInGroup);
    TEST_ASSERT_EQUAL_UINT16(2, output.steps[6].segmentId);
    TEST_ASSERT_EQUAL_UINT16(20, output.steps[6].originalStepId);
    TEST_ASSERT_EQUAL(StepRole::WORK, output.steps[6].role);

    // Cooldown (step 11)
    TEST_ASSERT_EQUAL(StepRole::COOLDOWN, output.steps[11].role);
    TEST_ASSERT_EQUAL_UINT16(1, output.steps[11].repNumber);
    TEST_ASSERT_EQUAL_UINT16(1, output.steps[11].totalRepsInGroup);
    TEST_ASSERT_EQUAL_UINT16(3, output.steps[11].segmentId);
    TEST_ASSERT_EQUAL_UINT16(30, output.steps[11].originalStepId);

    // Total estimated duration: 600 + (180+90+120+90+60)*2 + 300 = 600 + 1080 + 300 = 1980s
    TEST_ASSERT_EQUAL_UINT32(1980, output.totalEstimatedDurationSeconds);
}

void test_expansion_45_15_progression_and_final_rest_stripping() {
    WorkoutDefinition w = createWorkout45_15();
    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    bool ok = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_TRUE(ok);

    // 1 warmup + (9 reps * 2 steps) + (1 rep * 1 WORK step, final REST stripped) + 1 cooldown = 21 steps
    TEST_ASSERT_EQUAL_UINT8(21, output.totalSteps);

    // Verify step index sequentiality
    for (uint8_t i = 0; i < 21; ++i) {
        TEST_ASSERT_EQUAL_UINT8(i, output.steps[i].stepIndex);
    }

    // Reps 1 to 9: Both WORK and REST preserved
    for (uint16_t rep = 0; rep < 9; ++rep) {
        uint8_t workIdx = 1 + rep * 2;
        uint8_t restIdx = 1 + rep * 2 + 1;

        float expectedWorkSpeed = 14.0f + (static_cast<float>(rep) * 0.2f);
        TEST_ASSERT_TRUE(std::abs(output.steps[workIdx].targetSpeedKmh - expectedWorkSpeed) < 0.001f);
        TEST_ASSERT_EQUAL(StepRole::WORK, output.steps[workIdx].role);
        TEST_ASSERT_EQUAL_UINT16(rep + 1, output.steps[workIdx].repNumber);
        TEST_ASSERT_EQUAL_UINT16(10, output.steps[workIdx].totalRepsInGroup);

        TEST_ASSERT_TRUE(std::abs(output.steps[restIdx].targetSpeedKmh - 6.0f) < 0.001f);
        TEST_ASSERT_EQUAL(StepRole::REST, output.steps[restIdx].role);
        TEST_ASSERT_EQUAL_UINT16(rep + 1, output.steps[restIdx].repNumber);
        TEST_ASSERT_EQUAL_UINT16(10, output.steps[restIdx].totalRepsInGroup);
    }

    // Rep 10: WORK step present @ 15.8 km/h (14.0 + 9 * 0.2)
    uint8_t rep10WorkIdx = 19;
    TEST_ASSERT_EQUAL(StepRole::WORK, output.steps[rep10WorkIdx].role);
    TEST_ASSERT_EQUAL_UINT16(10, output.steps[rep10WorkIdx].repNumber);
    TEST_ASSERT_TRUE(std::abs(output.steps[rep10WorkIdx].targetSpeedKmh - 15.8f) < 0.001f);

    // Final step is COOLDOWN (step 20), confirming rep 10 REST was stripped before COOLDOWN
    uint8_t cooldownIdx = 20;
    TEST_ASSERT_EQUAL(StepRole::COOLDOWN, output.steps[cooldownIdx].role);
    TEST_ASSERT_EQUAL_UINT16(1, output.steps[cooldownIdx].repNumber);
    TEST_ASSERT_EQUAL_UINT16(3, output.steps[cooldownIdx].segmentId);

    // Total estimated duration: 600 + (45+15)*9 + 45 + 300 = 600 + 540 + 45 + 300 = 1485s
    TEST_ASSERT_EQUAL_UINT32(1485, output.totalEstimatedDurationSeconds);
}

void test_expansion_inherited_rest_speed_progression() {
    // Test that a REST step without explicit speed inherits the progressive WORK speed for each repetition
    WorkoutDefinition w{};
    w.id = 103;
    strncpy(w.name, "INHERITED PROG", sizeof(w.name) - 1);
    w.segmentCount = 2;

    // Segment 0: Warmup (Explicit speed 10.0)
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::TIME_SECONDS, 300, SpeedMode::FIXED, 10.0f, 0, false};

    // Segment 1: Repeating Group (3 reps, startSpeed 12.0, progression 0.5 per rep)
    // Step 0: WORK (FIXED 12.0)
    // Step 1: REST (FREE, targetSpeed 0.0 -> inherits WORK speed for that repetition)
    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 3;
    w.segments[1].startSpeedKmh = 12.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.5f;
    w.segments[1].stepCount = 2;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, 60, SpeedMode::FIXED, 12.0f, 0, false};
    w.segments[1].steps[1] = {3, StepRole::REST, DurationType::TIME_SECONDS, 30, SpeedMode::FREE,  0.0f,  0, false};

    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    bool ok = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_TRUE(ok);
    // Not followed by cooldown, so final REST is preserved -> 1 warmup + (3 * 2) = 7 steps
    TEST_ASSERT_EQUAL_UINT8(7, output.totalSteps);

    // Rep 1 (rep index 0): WORK = 12.0, REST inherits 12.0
    TEST_ASSERT_TRUE(std::abs(output.steps[1].targetSpeedKmh - 12.0f) < 0.001f);
    TEST_ASSERT_TRUE(std::abs(output.steps[2].targetSpeedKmh - 12.0f) < 0.001f);

    // Rep 2 (rep index 1): WORK = 12.5, REST inherits 12.5
    TEST_ASSERT_TRUE(std::abs(output.steps[3].targetSpeedKmh - 12.5f) < 0.001f);
    TEST_ASSERT_TRUE(std::abs(output.steps[4].targetSpeedKmh - 12.5f) < 0.001f);

    // Rep 3 (rep index 2): WORK = 13.0, REST inherits 13.0
    TEST_ASSERT_TRUE(std::abs(output.steps[5].targetSpeedKmh - 13.0f) < 0.001f);
    TEST_ASSERT_TRUE(std::abs(output.steps[6].targetSpeedKmh - 13.0f) < 0.001f);
}

void test_expansion_free_manual_fallback_no_prior_speed() {
    // Test that when no preceding explicit speed exists, REST falls back to FREE / 0.0 km/h cleanly
    WorkoutDefinition w{};
    w.id = 104;
    strncpy(w.name, "NO PRIOR SPEED", sizeof(w.name) - 1);
    w.segmentCount = 2;

    // Segment 0: REST with FREE and 0.0 km/h speed (no prior speed in workout)
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::REST, DurationType::TIME_SECONDS, 60, SpeedMode::FREE, 0.0f, 0, false};

    // Segment 1: WORK with FIXED 11.0 km/h
    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::SINGLE_STEP;
    w.segments[1].repetitions = 1;
    w.segments[1].stepCount = 1;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, 120, SpeedMode::FIXED, 11.0f, 0, false};

    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    bool ok = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(2, output.totalSteps);

    // Step 0: REST has fallback to FREE / 0.0 km/h
    TEST_ASSERT_EQUAL(StepRole::REST, output.steps[0].role);
    TEST_ASSERT_EQUAL(SpeedMode::FREE, output.steps[0].speedMode);
    TEST_ASSERT_TRUE(std::abs(output.steps[0].targetSpeedKmh - 0.0f) < 0.001f);

    // Step 1: WORK is FIXED 11.0 km/h
    TEST_ASSERT_EQUAL(StepRole::WORK, output.steps[1].role);
    TEST_ASSERT_EQUAL(SpeedMode::FIXED, output.steps[1].speedMode);
    TEST_ASSERT_TRUE(std::abs(output.steps[1].targetSpeedKmh - 11.0f) < 0.001f);
}

void test_expansion_max_achievable_speed_validation() {
    // Test that speed exceeding maxAchievableSpeedKmh is rejected with error, not silently clamped
    WorkoutDefinition w{};
    w.id = 105;
    strncpy(w.name, "SPEED CEILING", sizeof(w.name) - 1);
    w.segmentCount = 1;

    // Segment 0: Repeating group with progression reaching 25.5 km/h (> default 25.0 max)
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::REPEATING_GROUP;
    w.segments[0].repetitions = 2;
    w.segments[0].startSpeedKmh = 24.5f;
    w.segments[0].speedProgressionPerRepKmh = 1.0f; // Rep 1 = 24.5, Rep 2 = 25.5 (> 25.0)
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WORK, DurationType::TIME_SECONDS, 60, SpeedMode::FIXED, 24.5f, 0, false};

    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    // 1. Default constraint (25.0 km/h) should fail
    bool okDefault = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_FALSE(okDefault);
    TEST_ASSERT_EQUAL_STRING("Target speed exceeds maximum achievable speed", err);
    TEST_ASSERT_EQUAL_UINT8(0, output.totalSteps);

    // 2. Custom lower constraint (20.0 km/h) should fail on rep 1 (24.5 > 20.0)
    memset(err, 0, sizeof(err));
    bool okCustomLower = WorkoutExpander::expand(w, output, err, sizeof(err), 20.0f);
    TEST_ASSERT_FALSE(okCustomLower);
    TEST_ASSERT_EQUAL_STRING("Target speed exceeds maximum achievable speed", err);
    TEST_ASSERT_EQUAL_UINT8(0, output.totalSteps);

    // 3. Custom higher constraint (26.0 km/h) should succeed
    memset(err, 0, sizeof(err));
    bool okCustomHigher = WorkoutExpander::expand(w, output, err, sizeof(err), 26.0f);
    TEST_ASSERT_TRUE(okCustomHigher);
    TEST_ASSERT_EQUAL_UINT8(2, output.totalSteps);
    TEST_ASSERT_TRUE(std::abs(output.steps[0].targetSpeedKmh - 24.5f) < 0.001f);
    TEST_ASSERT_TRUE(std::abs(output.steps[1].targetSpeedKmh - 25.5f) < 0.001f);
}

void test_expansion_step_limit() {
    // 97 steps total (1 warmup + 95 interval steps + 1 cooldown)
    WorkoutDefinition w{};
    w.id = 106;
    w.segmentCount = 3;

    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};

    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 95; // 95 * 1 = 95 steps (WORK)
    w.segments[1].startSpeedKmh = 12.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.0f;
    w.segments[1].stepCount = 1;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, 60, SpeedMode::FIXED, 12.0f, 0, false};

    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {3, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};

    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    bool ok = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_STRING("Expanded workout exceeds step limit", err);
    TEST_ASSERT_EQUAL_UINT8(0, output.totalSteps);
    TEST_ASSERT_EQUAL_UINT16(0, output.workoutId);
}

void test_expansion_exact_capacity() {
    // Exactly 96 steps total (1 warmup + 94 interval steps + 1 cooldown)
    WorkoutDefinition w{};
    w.id = 107;
    strncpy(w.name, "96-STEP CAPACITY", sizeof(w.name) - 1);
    w.segmentCount = 3;

    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};

    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 94; // 94 * 1 = 94 steps
    w.segments[1].startSpeedKmh = 12.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.0f;
    w.segments[1].stepCount = 1;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, 60, SpeedMode::FIXED, 12.0f, 0, false};

    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {3, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};

    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    bool ok = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_STRING("", err);
    TEST_ASSERT_EQUAL_UINT8(96, output.totalSteps);
    TEST_ASSERT_EQUAL_UINT8(95, output.steps[95].stepIndex);
}

void test_expansion_meter_duration() {
    // 1000m @ 10.0 km/h fixed -> 360 seconds.
    // 1000m @ FREE -> uses 10.0 km/h fallback -> 360 seconds.
    WorkoutDefinition w{};
    w.id = 108;
    w.segmentCount = 3;

    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::METERS, 1000, SpeedMode::FIXED, 10.0f, 0, false};

    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::SINGLE_STEP;
    w.segments[1].repetitions = 1;
    w.segments[1].stepCount = 1;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::METERS, 1000, SpeedMode::FREE, 6.0f, 0, false};

    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {3, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};

    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    bool ok = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_TRUE(ok);
    // Warmup: (1000 * 3.6) / 10.0 = 360s
    // Work: (1000 * 3.6) / 10.0 (FREE fallback) = 360s
    // Cooldown: 300s
    // Total = 360 + 360 + 300 = 1020s
    TEST_ASSERT_EQUAL_UINT32(1020, output.totalEstimatedDurationSeconds);
}

void test_expansion_invalid_input() {
    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    // 1. Zero repetitions in repeating group
    WorkoutDefinition w1 = createWorkout321();
    w1.segments[1].repetitions = 0;
    TEST_ASSERT_FALSE(WorkoutExpander::expand(w1, output, err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT8(0, output.totalSteps);

    // 2. Zero duration in step (WORK step in repeating group)
    WorkoutDefinition w2 = createWorkout321();
    w2.segments[1].steps[0].durationValue = 0;
    TEST_ASSERT_FALSE(WorkoutExpander::expand(w2, output, err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT8(0, output.totalSteps);

    // 3. Unsupported duration type
    WorkoutDefinition w3 = createWorkout321();
    w3.segments[0].steps[0].durationType = static_cast<DurationType>(99);
    TEST_ASSERT_FALSE(WorkoutExpander::expand(w3, output, err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT8(0, output.totalSteps);

    // 4. Fixed speed below minimum 0.5 km/h
    WorkoutDefinition w4 = createWorkout321();
    w4.segments[0].steps[0].speedMode = SpeedMode::FIXED;
    w4.segments[0].steps[0].targetSpeedKmh = 0.2f;
    TEST_ASSERT_FALSE(WorkoutExpander::expand(w4, output, err, sizeof(err)));
    TEST_ASSERT_EQUAL_UINT8(0, output.totalSteps);
    TEST_ASSERT_EQUAL_STRING("Target speed below minimum speed", err);
}

void test_expansion_repeating_group_zero_duration_cooldown_final_rest_stripping() {
    WorkoutDefinition w{};
    w.id = 105;
    strncpy(w.name, "4X4 ZERO CD", sizeof(w.name) - 1);
    w.segmentCount = 3;

    // Segment 0: Warmup (600s)
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::TIME_SECONDS, 600, SpeedMode::FREE, 6.0f, 0, false};

    // Segment 1: Repeating WORK/REST group (reps = 4 > 1)
    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 4;
    w.segments[1].startSpeedKmh = 14.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.0f;
    w.segments[1].stepCount = 2;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, 240, SpeedMode::FIXED, 14.0f, 0, false};
    w.segments[1].steps[1] = {3, StepRole::REST, DurationType::TIME_SECONDS, 180, SpeedMode::FIXED, 6.0f, 0, false};

    // Segment 2: Zero-duration final Cooldown
    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {4, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 0, SpeedMode::FREE, 6.0f, 0, false};

    ExpandedWorkout& output = WorkoutEngine::instance().getExpandedWorkout();
    char err[64] = {};

    bool ok = WorkoutExpander::expand(w, output, err, sizeof(err));
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_STRING("", err);

    // 1 warmup + (3 reps * 2 steps) + (1 final rep * 1 WORK step, final REST stripped) + 0 cooldown (bypassed) = 1 + 6 + 1 + 0 = 8 steps
    TEST_ASSERT_EQUAL_UINT8(8, output.totalSteps);

    // Step 0: Warmup
    TEST_ASSERT_EQUAL(StepRole::WARMUP, output.steps[0].role);

    // Final step (step 7): Expected final expanded step is WORK, not REST
    TEST_ASSERT_EQUAL(StepRole::WORK, output.steps[7].role);
    TEST_ASSERT_EQUAL_UINT16(4, output.steps[7].repNumber);
    TEST_ASSERT_EQUAL_UINT32(240, output.steps[7].durationValue);

    // Verify zero-duration Cooldown emits no ExpandedStep
    for (uint8_t i = 0; i < output.totalSteps; ++i) {
        TEST_ASSERT_NOT_EQUAL(StepRole::COOLDOWN, output.steps[i].role);
    }
}

void run_all_workout_expander_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_expansion_321_workout);
    RUN_TEST(test_expansion_45_15_progression_and_final_rest_stripping);
    RUN_TEST(test_expansion_repeating_group_zero_duration_cooldown_final_rest_stripping);
    RUN_TEST(test_expansion_inherited_rest_speed_progression);
    RUN_TEST(test_expansion_free_manual_fallback_no_prior_speed);
    RUN_TEST(test_expansion_max_achievable_speed_validation);
    RUN_TEST(test_expansion_step_limit);
    RUN_TEST(test_expansion_exact_capacity);
    RUN_TEST(test_expansion_meter_duration);
    RUN_TEST(test_expansion_invalid_input);
    UNITY_END();
}

#if defined(ARDUINO)
void setup() {
    Serial.begin(115200);
    delay(2000);
    run_all_workout_expander_tests();
}
void loop() {}
#else
int main(int argc, char** argv) {
    run_all_workout_expander_tests();
    return 0;
}
#endif

