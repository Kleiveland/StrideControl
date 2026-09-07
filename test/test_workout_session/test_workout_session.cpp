#include <unity.h>
#ifdef ARDUINO
#include <Arduino.h>
#include <WiFi.h>
#endif
#include <cmath>
#include <cstring>
#include "WorkoutSession.h"
#include "WorkoutExpander.h"

using namespace stridecontrol;

// Helper to create a multi-step expanded workout for testing
static ExpandedWorkout createTestExpandedWorkout() {
    ExpandedWorkout ew{};
    ew.workoutId = 42;
    strncpy(ew.workoutName, "SESSION TEST", sizeof(ew.workoutName) - 1);
    ew.totalSteps = 5;

    // Step 0: Warmup (Time, 300s, FIXED 9.0 km/h, 1% incline)
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WARMUP;
    ew.steps[0].durationType = DurationType::TIME_SECONDS;
    ew.steps[0].durationValue = 300;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 9.0f;
    ew.steps[0].setIncline = true;
    ew.steps[0].targetInclinePct = 1;
    ew.steps[0].repNumber = 1;
    ew.steps[0].totalRepsInGroup = 1;

    // Step 1: Work Rep 1 (Distance, 1000m, FIXED 14.0 km/h, 2% incline)
    ew.steps[1].stepIndex = 1;
    ew.steps[1].role = StepRole::WORK;
    ew.steps[1].durationType = DurationType::METERS;
    ew.steps[1].durationValue = 1000;
    ew.steps[1].speedMode = SpeedMode::FIXED;
    ew.steps[1].targetSpeedKmh = 14.0f;
    ew.steps[1].setIncline = true;
    ew.steps[1].targetInclinePct = 2;
    ew.steps[1].repNumber = 1;
    ew.steps[1].totalRepsInGroup = 2;

    // Step 2: Rest Rep 1 (Time, 60s, FREE, 6.0 km/h fallback)
    ew.steps[2].stepIndex = 2;
    ew.steps[2].role = StepRole::REST;
    ew.steps[2].durationType = DurationType::TIME_SECONDS;
    ew.steps[2].durationValue = 60;
    ew.steps[2].speedMode = SpeedMode::FREE;
    ew.steps[2].targetSpeedKmh = 6.0f;
    ew.steps[2].setIncline = false;
    ew.steps[2].targetInclinePct = 0;
    ew.steps[2].repNumber = 1;
    ew.steps[2].totalRepsInGroup = 2;

    // Step 3: Work Rep 2 (Distance, 1000m, FIXED 14.0 km/h, 2% incline)
    ew.steps[3].stepIndex = 3;
    ew.steps[3].role = StepRole::WORK;
    ew.steps[3].durationType = DurationType::METERS;
    ew.steps[3].durationValue = 1000;
    ew.steps[3].speedMode = SpeedMode::FIXED;
    ew.steps[3].targetSpeedKmh = 14.0f;
    ew.steps[3].setIncline = true;
    ew.steps[3].targetInclinePct = 2;
    ew.steps[3].repNumber = 2;
    ew.steps[3].totalRepsInGroup = 2;

    // Step 4: Cooldown (Time, 120s, FREE, 8.0 km/h fallback)
    ew.steps[4].stepIndex = 4;
    ew.steps[4].role = StepRole::COOLDOWN;
    ew.steps[4].durationType = DurationType::TIME_SECONDS;
    ew.steps[4].durationValue = 120;
    ew.steps[4].speedMode = SpeedMode::FREE;
    ew.steps[4].targetSpeedKmh = 8.0f;
    ew.steps[4].setIncline = false;
    ew.steps[4].targetInclinePct = 0;
    ew.steps[4].repNumber = 1;
    ew.steps[4].totalRepsInGroup = 1;

    return ew;
}

static ApplicationSnapshot makeAppSnapshot(float beltSpeedKmh, double distanceKm, bool speedCredit = true) {
    ApplicationSnapshot snap{};
    snap.speed.speedKmh = beltSpeedKmh;
    snap.runner.validatedDistanceKm = distanceKm;
    snap.runner.speedCreditEnabled = speedCredit;
    return snap;
}

// 1. Reject null or empty workout
void test_session_reject_null_or_empty() {
    WorkoutSession session;
    session.begin();

    TEST_ASSERT_FALSE(session.armWorkout(nullptr, 1000));

    ExpandedWorkout emptyWk{};
    emptyWk.totalSteps = 0;
    TEST_ASSERT_FALSE(session.armWorkout(&emptyWk, 1000));
}

// 2. Arm workout without declaring belt running
void test_session_arm_workout_state() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();

    TEST_ASSERT_TRUE(session.armWorkout(&ew, 1000));
    WorkoutSessionSnapshot snap = session.getSnapshot();

    TEST_ASSERT_EQUAL(WorkoutSessionState::Armed, snap.state);
    TEST_ASSERT_TRUE(snap.active);
    TEST_ASSERT_FALSE(snap.suspended);
    TEST_ASSERT_EQUAL_UINT16(42, snap.workoutId);
    TEST_ASSERT_EQUAL_UINT8(0, snap.currentStepIndex);
    TEST_ASSERT_EQUAL_UINT8(5, snap.totalStepCount);
    TEST_ASSERT_EQUAL_UINT32(0, snap.totalElapsedTimeMs);
}

// 3. Enter first step after validated physical start
void test_session_activate_on_belt_start() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Belt still stopped (< 0.5 km/h)
    ApplicationSnapshot stoppedSnap = makeAppSnapshot(0.0f, 0.0);
    session.update(stoppedSnap, 1100);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Armed, session.getSnapshot().state);

    // Belt starts moving (1.0 km/h >= 0.5 km/h)
    ApplicationSnapshot movingSnap = makeAppSnapshot(1.0f, 0.0);
    session.update(movingSnap, 1200);

    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, snap.state);
    TEST_ASSERT_EQUAL_UINT8(0, snap.currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::WARMUP, snap.currentRole);

    WorkoutCommandIntent intent = session.getPendingCommandIntent();
    TEST_ASSERT_TRUE(intent.hasSpeedTarget);
    TEST_ASSERT_TRUE(std::abs(intent.targetSpeedKmh - 9.0f) < 0.001f);
    TEST_ASSERT_TRUE(intent.hasInclineTarget);
    TEST_ASSERT_EQUAL_UINT8(1, intent.targetInclinePct);
}

// 4. FREE speed mode must never emit speed target
void test_session_free_speed_mode_no_target() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Start belt & advance to Step 2 (REST, FREE mode)
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.advanceToNextStep(1000 + 1000); // to Step 1 (WORK)
    session.advanceToNextStep(1000 + 2000); // to Step 2 (REST, SpeedMode::FREE)

    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL_UINT8(2, snap.currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::REST, snap.currentRole);
    TEST_ASSERT_EQUAL(SpeedMode::FREE, snap.currentStep.speedMode);

    // Assert that intent and snapshot do NOT emit speed target
    TEST_ASSERT_FALSE(snap.hasSpeedTarget);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, snap.targetSpeedKmh);

    WorkoutCommandIntent intent = session.getPendingCommandIntent();
    TEST_ASSERT_FALSE(intent.hasSpeedTarget);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, intent.targetSpeedKmh);
}

// 5. Time-step completion
void test_session_time_step_completion() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Start belt
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);

    // Step 0 duration is 300s (300,000 ms)
    // Advance 299 seconds (299,000 ms) -> still on Step 0
    session.update(makeAppSnapshot(9.0f, 0.5), 1000 + 299000);
    TEST_ASSERT_EQUAL_UINT8(0, session.getSnapshot().currentStepIndex);
    TEST_ASSERT_EQUAL_UINT32(299000, session.getSnapshot().stepElapsedMs);
    TEST_ASSERT_EQUAL_UINT32(1000, session.getSnapshot().stepRemainingMs);

    // Advance 1 more second -> Step 0 completes, Step 1 begins!
    session.update(makeAppSnapshot(9.0f, 0.51), 1000 + 300000);
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL_UINT8(1, snap.currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::WORK, snap.currentRole);
    TEST_ASSERT_EQUAL_UINT32(0, snap.stepElapsedMs);
}

// 6. Distance-step completion from authoritative distance input
void test_session_distance_step_completion() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Start belt & complete warmup (Step 0)
    session.update(makeAppSnapshot(9.0f, 10.0), 1000);
    session.update(makeAppSnapshot(9.0f, 10.5), 1000 + 300000);

    // Now on Step 1: Distance step (1000m = 1.0 km)
    // Entry distance was 10.5 km.
    TEST_ASSERT_EQUAL_UINT8(1, session.getSnapshot().currentStepIndex);

    // Distance advances to 11.0 km (0.5 km completed)
    session.update(makeAppSnapshot(14.0f, 11.0), 1000 + 330000);
    TEST_ASSERT_EQUAL_UINT8(1, session.getSnapshot().currentStepIndex);
    TEST_ASSERT_TRUE(std::abs(session.getSnapshot().stepElapsedValidatedDistanceKm - 0.5) < 0.001);
    TEST_ASSERT_TRUE(std::abs(session.getSnapshot().stepRemainingValidatedDistanceKm - 0.5) < 0.001);

    // Distance reaches 11.5 km (1.0 km completed) -> Step 1 completes, Step 2 begins!
    session.update(makeAppSnapshot(14.0f, 11.5), 1000 + 360000);
    TEST_ASSERT_EQUAL_UINT8(2, session.getSnapshot().currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::REST, session.getSnapshot().currentRole);
}

// 7. Pause freezes time and distance baseline
void test_session_pause_freezes_time_and_distance() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.update(makeAppSnapshot(9.0f, 0.1), 1000 + 50000); // 50s in
    TEST_ASSERT_EQUAL_UINT32(50000, session.getSnapshot().stepElapsedMs);

    // Explicit suspend
    TEST_ASSERT_TRUE(session.suspend(1000 + 50000));
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, session.getSnapshot().state);
    TEST_ASSERT_TRUE(session.getSnapshot().suspended);

    // Updates while suspended (e.g. 60 seconds passing with belt stopped)
    session.update(makeAppSnapshot(0.0f, 0.1), 1000 + 110000);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, session.getSnapshot().state);
    // Time must remain strictly frozen at 50,000 ms!
    TEST_ASSERT_EQUAL_UINT32(50000, session.getSnapshot().stepElapsedMs);
    TEST_ASSERT_EQUAL_UINT32(50000, session.getSnapshot().totalElapsedTimeMs);
}

// 8. Resume preserves exact progress and resumes countdown
void test_session_resume_preserves_progress() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.update(makeAppSnapshot(9.0f, 0.1), 1000 + 50000);
    session.suspend(1000 + 50000);

    // Pause elapsed 60s
    uint32_t resumeTime = 1000 + 110000;
    TEST_ASSERT_TRUE(session.resume(resumeTime));
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);
    TEST_ASSERT_FALSE(session.getSnapshot().suspended);

    // 10s after resume (timestamp resumeTime + 10000)
    session.update(makeAppSnapshot(9.0f, 0.12), resumeTime + 10000);
    // Elapsed should be exactly 50s + 10s = 60s (60,000 ms)
    TEST_ASSERT_EQUAL_UINT32(60000, session.getSnapshot().stepElapsedMs);
    TEST_ASSERT_EQUAL_UINT32(60000, session.getSnapshot().totalElapsedTimeMs);
}

// 9. Delayed update handles multiple elapsed seconds
void test_session_delayed_update() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);

    // Call update after 5000ms in a single tick
    session.update(makeAppSnapshot(9.0f, 0.05), 6000);
    TEST_ASSERT_EQUAL_UINT32(5000, session.getSnapshot().stepElapsedMs);
    TEST_ASSERT_EQUAL_UINT32(5000, session.getSnapshot().totalElapsedTimeMs);
}

// 10. uint32_t rollover handling
void test_session_uint32_rollover() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();

    uint32_t startMs = 0xFFFFFFF0; // 16 ms before rollover
    session.armWorkout(&ew, startMs);
    session.update(makeAppSnapshot(9.0f, 0.0), startMs);

    uint32_t afterRolloverMs = 0x00000020; // 32 ms after rollover -> total dt = 48 ms
    session.update(makeAppSnapshot(9.0f, 0.0), afterRolloverMs);

    TEST_ASSERT_EQUAL_UINT32(48, session.getSnapshot().stepElapsedMs);
    TEST_ASSERT_EQUAL_UINT32(48, session.getSnapshot().totalElapsedTimeMs);
}

// 11. Cut drag enters Hvile (REST) and records partial completion
void test_session_cut_drag() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Start workout and advance to Step 1 (WORK / Drag)
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.update(makeAppSnapshot(9.0f, 0.5), 1000 + 300000);
    TEST_ASSERT_EQUAL_UINT8(1, session.getSnapshot().currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::WORK, session.getSnapshot().currentRole);

    // Runner cuts drag at 1000 + 320000
    TEST_ASSERT_TRUE(session.cutDrag(1000 + 320000));

    WorkoutSessionSnapshot snap = session.getSnapshot();
    // Must advance to Step 2 (REST / Hvile)
    TEST_ASSERT_EQUAL_UINT8(2, snap.currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::REST, snap.currentRole);
    TEST_ASSERT_TRUE(snap.isPartialDrag);
    TEST_ASSERT_EQUAL_UINT8(1, snap.partialDragCount);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, snap.state); // Does NOT enter physical Pause
}

// 12. Extend Hvile (REST) by 30 seconds
void test_session_extend_rest() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Advance to Step 2 (REST, original duration 60s)
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.advanceToNextStep(1000 + 10000); // to step 1
    session.advanceToNextStep(1000 + 20000); // to step 2 (REST)
    TEST_ASSERT_EQUAL_UINT8(2, session.getSnapshot().currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::REST, session.getSnapshot().currentRole);

    // Extend rest by 30s
    TEST_ASSERT_TRUE(session.extendRest(30));
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_TRUE(snap.isRestExtended);
    TEST_ASSERT_EQUAL_UINT32(30, snap.restExtensionSeconds);

    // Step remaining should now reflect 60s + 30s = 90s (90,000 ms)
    TEST_ASSERT_EQUAL_UINT32(90000, snap.stepRemainingMs);

    // Original ExpandedWorkout struct must be completely unchanged!
    TEST_ASSERT_EQUAL_UINT32(60, ew.steps[2].durationValue);
}

// 13. Nedjogg expiry enters CompletionPending without stopping or completing
void test_session_nedjogg_completion_pending() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Advance to Step 4 (COOLDOWN / Nedjogg, 120s)
    session.update(makeAppSnapshot(8.0f, 0.0), 1000);
    session.advanceToNextStep(1000 + 1000); // step 1
    session.advanceToNextStep(1000 + 2000); // step 2
    session.advanceToNextStep(1000 + 3000); // step 3
    session.advanceToNextStep(1000 + 4000); // step 4 (COOLDOWN)
    TEST_ASSERT_EQUAL_UINT8(4, session.getSnapshot().currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::COOLDOWN, session.getSnapshot().currentRole);

    // Complete Nedjogg 120s
    session.update(makeAppSnapshot(8.0f, 2.0), 1000 + 4000 + 120000);

    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::CompletionPending, snap.state);
    TEST_ASSERT_TRUE(snap.completionPending);
    TEST_ASSERT_TRUE(snap.active);
    TEST_ASSERT_FALSE(snap.suspended);
}

// 14. Finalization after physical stop
void test_session_finalize_session() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(8.0f, 0.0), 1000);

    // Finalize session
    TEST_ASSERT_TRUE(session.finalizeSession(2000));
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Completed, snap.state);
    TEST_ASSERT_FALSE(snap.active);
    TEST_ASSERT_FALSE(session.getPendingCommandIntent().hasSpeedTarget);
}

// 15. Abort session
void test_session_abort() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(8.0f, 0.0), 1000);

    TEST_ASSERT_TRUE(session.abortSession(2000));
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Aborted, snap.state);
    TEST_ASSERT_FALSE(snap.active);
    TEST_ASSERT_FALSE(session.getPendingCommandIntent().hasSpeedTarget);
}

// 16. Repeated pause/resume calls are idempotent
void test_session_pause_resume_idempotent() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);

    TEST_ASSERT_TRUE(session.suspend(1100));
    TEST_ASSERT_FALSE(session.suspend(1200)); // already suspended

    TEST_ASSERT_TRUE(session.resume(1300));
    TEST_ASSERT_FALSE(session.resume(1400)); // already running
}

// 17. Stop hierarchy: 1x Stop (Pause), 2x Stop (10s continuation), 3x Stop (Finalize)
void test_session_stop_hierarchy_1x_2x_3x() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);

    // 1x Stop: Suspends session (Pause)
    session.registerPhysicalStop(2000);
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, snap.state);
    TEST_ASSERT_EQUAL_UINT8(1, snap.physicalStopCount);
    TEST_ASSERT_FALSE(snap.continuationWindowActive);

    // Belt restarts -> resumes and clears stop count
    session.update(makeAppSnapshot(9.0f, 0.0), 2500);
    snap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, snap.state);
    TEST_ASSERT_EQUAL_UINT8(0, snap.physicalStopCount);

    // 1st stop again
    session.registerPhysicalStop(3000);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, session.getSnapshot().state);
    TEST_ASSERT_EQUAL_UINT8(1, session.getSnapshot().physicalStopCount);

    // 2nd stop within pause -> 10s continuation window opened, targets reset
    session.registerPhysicalStop(4000);
    snap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, snap.state);
    TEST_ASSERT_EQUAL_UINT8(2, snap.physicalStopCount);
    TEST_ASSERT_TRUE(snap.continuationWindowActive);
    TEST_ASSERT_EQUAL_UINT32(10000, snap.continuationWindowRemainingMs);
    TEST_ASSERT_FALSE(session.getPendingCommandIntent().hasSpeedTarget);

    // 3rd stop -> Immediately finalizes session
    session.registerPhysicalStop(5000);
    snap = session.getSnapshot();
    TEST_ASSERT_EQUAL(WorkoutSessionState::Completed, snap.state);
    TEST_ASSERT_FALSE(snap.active);
}

// 18. Stop hierarchy: 2x Stop continuation window expiry
void test_session_continuation_window_expiry() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);

    // 1x Stop then 2x Stop
    session.registerPhysicalStop(2000);
    session.registerPhysicalStop(3000);
    TEST_ASSERT_TRUE(session.getSnapshot().continuationWindowActive);

    // Time advances by 5 seconds (5,000 ms) -> still active
    session.update(makeAppSnapshot(0.0f, 0.0), 3000 + 5000);
    TEST_ASSERT_TRUE(session.getSnapshot().continuationWindowActive);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, session.getSnapshot().state);

    // Time advances past 10 seconds (10,001 ms) -> auto-finalizes
    session.update(makeAppSnapshot(0.0f, 0.0), 3000 + 10001);
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_FALSE(snap.continuationWindowActive);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Completed, snap.state);
    TEST_ASSERT_FALSE(snap.active);
}

// 19. Emergency stop handling
void test_session_emergency_stop() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);

    // E-stop occurs
    session.registerEmergencyStop(2000);
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_TRUE(snap.isEmergencyStopped);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, snap.state);
    TEST_ASSERT_FALSE(session.getPendingCommandIntent().hasSpeedTarget);

    // If belt spins / moves while E-stop is active, session does NOT resume
    session.update(makeAppSnapshot(5.0f, 0.0), 3000);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, session.getSnapshot().state);

    // E-stop cleared
    session.registerEmergencyStopCleared();
    TEST_ASSERT_FALSE(session.getSnapshot().isEmergencyStopped);

    // Belt starts moving -> now resumes!
    session.update(makeAppSnapshot(5.0f, 0.0), 4000);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);
}

// 20. Interval to Manual mode state preservation
void test_session_interval_to_manual_state_preservation() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Run for 60s and 0.2 km
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.update(makeAppSnapshot(9.0f, 0.2), 1000 + 60000);

    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL_UINT32(60000, snap.totalElapsedTimeMs);
    TEST_ASSERT_EQUAL_UINT32(60000, snap.activeRunningTimeMs);
    TEST_ASSERT_TRUE(std::abs(snap.totalValidatedDistanceKm - 0.2) < 0.001);

    // User aborts interval mode to switch to manual running
    session.abortSession(1000 + 60000);
    WorkoutSessionSnapshot postAbortSnap = session.getSnapshot();

    TEST_ASSERT_EQUAL(WorkoutSessionState::Aborted, postAbortSnap.state);
    // Cumulative metrics must be preserved for training session continuity
    TEST_ASSERT_EQUAL_UINT32(60000, postAbortSnap.totalElapsedTimeMs);
    TEST_ASSERT_EQUAL_UINT32(60000, postAbortSnap.activeRunningTimeMs);
    TEST_ASSERT_TRUE(std::abs(postAbortSnap.totalValidatedDistanceKm - 0.2) < 0.001);
}

// 21. Remaining-drag speed adjustment: Accept shift
void test_session_remaining_drag_speed_adjustment_accept() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Start belt & advance to Step 1 (WORK Rep 1, planned 14.0 km/h)
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.advanceToNextStep(1000 + 1000); // step 1 (WORK)

    TEST_ASSERT_EQUAL_UINT8(1, session.getSnapshot().currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::WORK, session.getSnapshot().currentRole);

    // Runner changes speed to 15.0 km/h (+1.0 km/h delta)
    session.reportWorkSpeedAdjustment(15.0f);

    // Advance to Step 2 (REST Rep 1)
    session.advanceToNextStep(1000 + 2000);
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL_UINT8(2, snap.currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::REST, snap.currentRole);

    // Prompt must be active with suggested delta +1.0 km/h
    TEST_ASSERT_TRUE(snap.speedAdjustmentPromptActive);
    TEST_ASSERT_TRUE(std::abs(snap.suggestedSpeedDeltaKmh - 1.0f) < 0.01f);

    // REST step is FREE mode -> no speed target emitted
    TEST_ASSERT_FALSE(snap.hasSpeedTarget);

    // Runner accepts the prompt
    session.acceptSpeedAdjustmentShift();
    TEST_ASSERT_FALSE(session.getSnapshot().speedAdjustmentPromptActive);
    TEST_ASSERT_TRUE(std::abs(session.getSnapshot().appliedWorkSpeedShiftKmh - 1.0f) < 0.01f);

    // Advance to Step 3 (WORK Rep 2, planned 14.0 km/h)
    session.advanceToNextStep(1000 + 3000);
    snap = session.getSnapshot();
    TEST_ASSERT_EQUAL_UINT8(3, snap.currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::WORK, snap.currentRole);

    // Shift of +1.0 km/h must be applied to this WORK step -> 15.0 km/h
    TEST_ASSERT_TRUE(snap.hasSpeedTarget);
    TEST_ASSERT_TRUE(std::abs(snap.targetSpeedKmh - 15.0f) < 0.01f);
    TEST_ASSERT_TRUE(std::abs(session.getPendingCommandIntent().targetSpeedKmh - 15.0f) < 0.01f);
}

// 22. Remaining-drag speed adjustment: Reject or Timeout
void test_session_remaining_drag_speed_adjustment_reject_and_timeout() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Step 0 -> Step 1 (WORK)
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.advanceToNextStep(1000 + 1000);

    // Speed adjustment +1.5 km/h
    session.reportWorkSpeedAdjustment(15.5f);

    // Step 2 (REST) -> Prompt active
    session.advanceToNextStep(1000 + 2000);
    TEST_ASSERT_TRUE(session.getSnapshot().speedAdjustmentPromptActive);

    // Reject shift
    session.rejectSpeedAdjustmentShift();
    TEST_ASSERT_FALSE(session.getSnapshot().speedAdjustmentPromptActive);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, session.getSnapshot().appliedWorkSpeedShiftKmh);

    // Next WORK step stays at planned 14.0 km/h
    session.advanceToNextStep(1000 + 3000);
    TEST_ASSERT_TRUE(std::abs(session.getSnapshot().targetSpeedKmh - 14.0f) < 0.01f);

    // Test prompt timeout during another session
    WorkoutSession session2;
    session2.begin();
    session2.armWorkout(&ew, 1000);
    session2.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session2.advanceToNextStep(1000 + 1000); // Step 1 (WORK)
    session2.reportWorkSpeedAdjustment(16.0f); // +2.0 km/h
    session2.advanceToNextStep(1000 + 2000); // Step 2 (REST)

    TEST_ASSERT_TRUE(session2.getSnapshot().speedAdjustmentPromptActive);

    // 15 seconds elapse without runner interaction -> prompt expires
    session2.update(makeAppSnapshot(6.0f, 1.0), 1000 + 2000 + 15001);
    TEST_ASSERT_FALSE(session2.getSnapshot().speedAdjustmentPromptActive);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, session2.getSnapshot().appliedWorkSpeedShiftKmh);
}

// 23. Immutability: Zero mutation of ExpandedWorkout
void test_session_immutability() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    ExpandedWorkout ewCopy = ew;

    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    session.advanceToNextStep(2000);
    session.reportWorkSpeedAdjustment(16.0f);
    session.cutDrag(3000);
    session.acceptSpeedAdjustmentShift();
    session.extendRest(30);
    session.finalizeSession(4000);

    // Byte-by-byte comparison to verify immutability
    TEST_ASSERT_EQUAL_UINT8_ARRAY(reinterpret_cast<const uint8_t*>(&ewCopy),
                                  reinterpret_cast<const uint8_t*>(&ew),
                                  sizeof(ExpandedWorkout));
}

// 24. Distance step overshoot rollover
void test_session_distance_step_overshoot_rollover() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew{};
    ew.workoutId = 201;
    ew.totalSteps = 2;

    // Step 0: Distance 1000m (1.000 km), FIXED 12.0 km/h
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WORK;
    ew.steps[0].durationType = DurationType::METERS;
    ew.steps[0].durationValue = 1000;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 12.0f;

    // Step 1: Distance 1000m (1.000 km), FIXED 12.0 km/h
    ew.steps[1].stepIndex = 1;
    ew.steps[1].role = StepRole::WORK;
    ew.steps[1].durationType = DurationType::METERS;
    ew.steps[1].durationValue = 1000;
    ew.steps[1].speedMode = SpeedMode::FIXED;
    ew.steps[1].targetSpeedKmh = 12.0f;

    ExpandedWorkout ewCopy = ew;

    session.armWorkout(&ew, 1000);
    // Start step 0 at distance 10.000 km
    session.update(makeAppSnapshot(12.0f, 10.000), 1000);
    TEST_ASSERT_EQUAL_UINT8(0, session.getSnapshot().currentStepIndex);

    // Sensor distance jumps to 11.018 km (target 1.000 km reached + 0.018 km overshoot)
    session.update(makeAppSnapshot(12.0f, 11.018), 1000 + 300000);

    // Must advance to Step 1
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL_UINT8(1, snap.currentStepIndex);
    // Step 1 must show 0.018 km elapsed (or remaining 0.982 km)
    TEST_ASSERT_TRUE(std::abs(snap.stepElapsedValidatedDistanceKm - 0.018) < 0.001);
    TEST_ASSERT_TRUE(std::abs(snap.stepRemainingValidatedDistanceKm - 0.982) < 0.001);

    // ExpandedWorkout must remain completely unchanged
    TEST_ASSERT_EQUAL_UINT8_ARRAY(reinterpret_cast<const uint8_t*>(&ewCopy),
                                  reinterpret_cast<const uint8_t*>(&ew),
                                  sizeof(ExpandedWorkout));
}

// 25. Intent latching and clear
void test_session_intent_latching_and_clear() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew{};
    ew.workoutId = 202;
    ew.totalSteps = 1;
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WORK;
    ew.steps[0].durationType = DurationType::TIME_SECONDS;
    ew.steps[0].durationValue = 300;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 12.0f;
    ew.steps[0].setIncline = true;
    ew.steps[0].targetInclinePct = 3;

    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(12.0f, 0.0), 1000);

    // Initial step entry produces hasSpeedTarget == true and hasInclineTarget == true
    WorkoutCommandIntent intent = session.getPendingCommandIntent();
    TEST_ASSERT_TRUE(intent.hasSpeedTarget);
    TEST_ASSERT_TRUE(std::abs(intent.targetSpeedKmh - 12.0f) < 0.01f);
    TEST_ASSERT_TRUE(intent.hasInclineTarget);
    TEST_ASSERT_EQUAL_UINT8(3, intent.targetInclinePct);

    // Clear intent
    session.clearPendingCommandIntent();
    intent = session.getPendingCommandIntent();
    TEST_ASSERT_FALSE(intent.hasSpeedTarget);
    TEST_ASSERT_FALSE(intent.hasInclineTarget);

    // Unchanged update() in the same step
    session.update(makeAppSnapshot(12.0f, 0.02), 2000);
    intent = session.getPendingCommandIntent();
    TEST_ASSERT_FALSE(intent.hasSpeedTarget);
    TEST_ASSERT_FALSE(intent.hasInclineTarget);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);
}

// 26. Speed adjustment prompt forced close on step exit
void test_session_speed_adjustment_prompt_forced_close_on_step_exit() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew{};
    ew.workoutId = 203;
    ew.totalSteps = 3;

    // Step 0: WORK 60s, 12.0 km/h
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WORK;
    ew.steps[0].durationType = DurationType::TIME_SECONDS;
    ew.steps[0].durationValue = 60;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 12.0f;

    // Step 1: REST 10s, FREE mode
    ew.steps[1].stepIndex = 1;
    ew.steps[1].role = StepRole::REST;
    ew.steps[1].durationType = DurationType::TIME_SECONDS;
    ew.steps[1].durationValue = 10;
    ew.steps[1].speedMode = SpeedMode::FREE;
    ew.steps[1].targetSpeedKmh = 6.0f;

    // Step 2: WORK 60s, 12.0 km/h
    ew.steps[2].stepIndex = 2;
    ew.steps[2].role = StepRole::WORK;
    ew.steps[2].durationType = DurationType::TIME_SECONDS;
    ew.steps[2].durationValue = 60;
    ew.steps[2].speedMode = SpeedMode::FIXED;
    ew.steps[2].targetSpeedKmh = 12.0f;

    ExpandedWorkout ewCopy = ew;

    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(12.0f, 0.0), 1000);

    // Report valid speed adjustment during WORK (+1.0 km/h)
    session.reportWorkSpeedAdjustment(13.0f);

    // Advance into REST (Step 1)
    session.advanceToNextStep(1000 + 1000);
    TEST_ASSERT_EQUAL_UINT8(1, session.getSnapshot().currentStepIndex);
    TEST_ASSERT_TRUE(session.getSnapshot().speedAdjustmentPromptActive);

    // Advance REST to completion (10s duration completes before 15s prompt timeout)
    session.update(makeAppSnapshot(6.0f, 0.0), 1000 + 1000 + 10000);

    // Step 2 is now active
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL_UINT8(2, snap.currentStepIndex);
    TEST_ASSERT_EQUAL(StepRole::WORK, snap.currentRole);

    // Prompt forced closed
    TEST_ASSERT_FALSE(snap.speedAdjustmentPromptActive);
    // No speed shift applied
    TEST_ASSERT_EQUAL_FLOAT(0.0f, snap.appliedWorkSpeedShiftKmh);
    TEST_ASSERT_TRUE(std::abs(snap.targetSpeedKmh - 12.0f) < 0.01f);

    // A later accept call must NOT apply the closed prompt
    session.acceptSpeedAdjustmentShift();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, session.getSnapshot().appliedWorkSpeedShiftKmh);
    TEST_ASSERT_TRUE(std::abs(session.getSnapshot().targetSpeedKmh - 12.0f) < 0.01f);

    // Immutability
    TEST_ASSERT_EQUAL_UINT8_ARRAY(reinterpret_cast<const uint8_t*>(&ewCopy),
                                  reinterpret_cast<const uint8_t*>(&ew),
                                  sizeof(ExpandedWorkout));
}

// 27. Motion gating jitter debounce
void test_session_motion_gating_jitter_debounce() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew = createTestExpandedWorkout();
    session.armWorkout(&ew, 1000);

    // Start running
    session.update(makeAppSnapshot(9.0f, 0.0), 1000);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);

    // Jitter 1: 0.4 km/h for 200 ms
    session.update(makeAppSnapshot(0.4f, 0.0), 1200);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);

    // Speed recovers to 0.6 km/h before 1000 ms
    session.update(makeAppSnapshot(0.6f, 0.0), 1500);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);

    // Jitter 2: 0.4 km/h again for 200 ms (< 1000 ms continuous)
    session.update(makeAppSnapshot(0.4f, 0.0), 2100);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);
    // Recovers to 0.6 km/h before 1000 ms
    session.update(makeAppSnapshot(0.6f, 0.0), 2300);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);
    TEST_ASSERT_EQUAL_UINT8(0, session.getSnapshot().physicalStopCount);

    // Sustained low speed: 0.2 km/h continuously
    session.update(makeAppSnapshot(0.2f, 0.0), 3000); // start of continuous low speed window
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);

    // 999 ms later -> still Running!
    session.update(makeAppSnapshot(0.2f, 0.0), 3999);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);

    // At 1000 ms -> Suspended!
    session.update(makeAppSnapshot(0.2f, 0.0), 4000);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, session.getSnapshot().state);
    // Low-speed suspension does not increment stop count
    TEST_ASSERT_EQUAL_UINT8(0, session.getSnapshot().physicalStopCount);

    // Explicit stop remains immediate (no debounce)
    session.resume(5000);
    session.update(makeAppSnapshot(9.0f, 0.0), 5100);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Running, session.getSnapshot().state);

    session.registerPhysicalStop(5150);
    TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, session.getSnapshot().state);
    TEST_ASSERT_EQUAL_UINT8(1, session.getSnapshot().physicalStopCount);
}

// 28. Multiple speed adjustments in one WORK step: net delta
void test_session_multiple_speed_adjustments_net_delta() {
    WorkoutSession session;
    session.begin();
    ExpandedWorkout ew{};
    ew.workoutId = 205;
    ew.totalSteps = 3;

    // Step 0: WORK 12.0 km/h
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WORK;
    ew.steps[0].durationType = DurationType::TIME_SECONDS;
    ew.steps[0].durationValue = 60;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 12.0f;

    // Step 1: REST 6.0 km/h FREE
    ew.steps[1].stepIndex = 1;
    ew.steps[1].role = StepRole::REST;
    ew.steps[1].durationType = DurationType::TIME_SECONDS;
    ew.steps[1].durationValue = 60;
    ew.steps[1].speedMode = SpeedMode::FREE;
    ew.steps[1].targetSpeedKmh = 6.0f;

    // Step 2: WORK 12.0 km/h
    ew.steps[2].stepIndex = 2;
    ew.steps[2].role = StepRole::WORK;
    ew.steps[2].durationType = DurationType::TIME_SECONDS;
    ew.steps[2].durationValue = 60;
    ew.steps[2].speedMode = SpeedMode::FIXED;
    ew.steps[2].targetSpeedKmh = 12.0f;

    ExpandedWorkout ewCopy = ew;

    session.armWorkout(&ew, 1000);
    session.update(makeAppSnapshot(12.0f, 0.0), 1000);

    // Multiple adjustments in same WORK step: 12.5 then 13.0
    session.reportWorkSpeedAdjustment(12.5f);
    session.reportWorkSpeedAdjustment(13.0f);

    // Advance to REST
    session.advanceToNextStep(1000 + 1000);
    TEST_ASSERT_TRUE(session.getSnapshot().speedAdjustmentPromptActive);

    // Accept shift
    session.acceptSpeedAdjustmentShift();
    TEST_ASSERT_TRUE(std::abs(session.getSnapshot().appliedWorkSpeedShiftKmh - 1.0f) < 0.01f);

    // Advance to next WORK step
    session.advanceToNextStep(1000 + 2000);
    WorkoutSessionSnapshot snap = session.getSnapshot();
    TEST_ASSERT_EQUAL_UINT8(2, snap.currentStepIndex);
    // Must be exactly 13.0 km/h (planned 12.0 + net delta 1.0, not +1.5)
    TEST_ASSERT_TRUE(std::abs(snap.targetSpeedKmh - 13.0f) < 0.01f);
    TEST_ASSERT_TRUE(std::abs(session.getPendingCommandIntent().targetSpeedKmh - 13.0f) < 0.01f);

    // Immutability
    TEST_ASSERT_EQUAL_UINT8_ARRAY(reinterpret_cast<const uint8_t*>(&ewCopy),
                                  reinterpret_cast<const uint8_t*>(&ew),
                                  sizeof(ExpandedWorkout));
}

// 29. Target reissue after 2x stop and E-stop
void test_session_target_reissue_after_2x_stop_and_estop() {
    ExpandedWorkout ew{};
    ew.workoutId = 206;
    ew.totalSteps = 1;
    ew.steps[0].stepIndex = 0;
    ew.steps[0].role = StepRole::WORK;
    ew.steps[0].durationType = DurationType::TIME_SECONDS;
    ew.steps[0].durationValue = 300;
    ew.steps[0].speedMode = SpeedMode::FIXED;
    ew.steps[0].targetSpeedKmh = 14.0f;
    ew.steps[0].setIncline = true;
    ew.steps[0].targetInclinePct = 2;

    // --- Scenario A: 2x Stop continuation restart ---
    {
        WorkoutSession sessionA;
        sessionA.begin();
        sessionA.armWorkout(&ew, 1000);

        // Enter Running
        sessionA.update(makeAppSnapshot(14.0f, 0.0), 1000);
        TEST_ASSERT_EQUAL(WorkoutSessionState::Running, sessionA.getSnapshot().state);

        // Consume and clear initial intent
        sessionA.clearPendingCommandIntent();
        TEST_ASSERT_FALSE(sessionA.getPendingCommandIntent().hasSpeedTarget);
        TEST_ASSERT_FALSE(sessionA.getPendingCommandIntent().hasInclineTarget);

        // Trigger 2x Stop sequence
        sessionA.registerPhysicalStop(2000);
        sessionA.registerPhysicalStop(2500);
        TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, sessionA.getSnapshot().state);
        TEST_ASSERT_TRUE(sessionA.getSnapshot().continuationWindowActive);

        // Clear transport intent
        sessionA.clearPendingCommandIntent();
        TEST_ASSERT_FALSE(sessionA.getPendingCommandIntent().hasSpeedTarget);

        // Restart physical belt motion within continuation window (e.g. at 3500 ms, window expires at 12500 ms)
        sessionA.update(makeAppSnapshot(14.0f, 0.0), 3500);

        // Session resumes and reissues targets
        TEST_ASSERT_EQUAL(WorkoutSessionState::Running, sessionA.getSnapshot().state);
        WorkoutCommandIntent reissuedIntent = sessionA.getPendingCommandIntent();
        TEST_ASSERT_TRUE(reissuedIntent.hasSpeedTarget);
        TEST_ASSERT_TRUE(std::abs(reissuedIntent.targetSpeedKmh - 14.0f) < 0.01f);
        TEST_ASSERT_TRUE(reissuedIntent.hasInclineTarget);
        TEST_ASSERT_EQUAL_UINT8(2, reissuedIntent.targetInclinePct);
    }

    // --- Scenario B: E-stop assertion, explicit clear, and physical restart ---
    {
        WorkoutSession sessionB;
        sessionB.begin();
        sessionB.armWorkout(&ew, 1000);

        // Enter Running
        sessionB.update(makeAppSnapshot(14.0f, 0.0), 1000);
        TEST_ASSERT_EQUAL(WorkoutSessionState::Running, sessionB.getSnapshot().state);

        // Consume and clear initial intent
        sessionB.clearPendingCommandIntent();
        TEST_ASSERT_FALSE(sessionB.getPendingCommandIntent().hasSpeedTarget);

        // Assert E-stop
        sessionB.registerEmergencyStop(2000);
        TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, sessionB.getSnapshot().state);
        TEST_ASSERT_TRUE(sessionB.getSnapshot().isEmergencyStopped);

        // Verify no automatic restart while E-stop asserted
        sessionB.update(makeAppSnapshot(5.0f, 0.0), 2500);
        TEST_ASSERT_EQUAL(WorkoutSessionState::Suspended, sessionB.getSnapshot().state);

        // Explicitly clear E-stop
        sessionB.registerEmergencyStopCleared();
        TEST_ASSERT_FALSE(sessionB.getSnapshot().isEmergencyStopped);

        // Clear transport intent
        sessionB.clearPendingCommandIntent();
        TEST_ASSERT_FALSE(sessionB.getPendingCommandIntent().hasSpeedTarget);

        // Restart physical belt motion
        sessionB.update(makeAppSnapshot(14.0f, 0.0), 3500);

        // Session resumes and reissues targets
        TEST_ASSERT_EQUAL(WorkoutSessionState::Running, sessionB.getSnapshot().state);
        WorkoutCommandIntent reissuedIntent = sessionB.getPendingCommandIntent();
        TEST_ASSERT_TRUE(reissuedIntent.hasSpeedTarget);
        TEST_ASSERT_TRUE(std::abs(reissuedIntent.targetSpeedKmh - 14.0f) < 0.01f);
        TEST_ASSERT_TRUE(reissuedIntent.hasInclineTarget);
        TEST_ASSERT_EQUAL_UINT8(2, reissuedIntent.targetInclinePct);
    }
}

void run_all_workout_session_tests() {
    UNITY_BEGIN();
    RUN_TEST(test_session_reject_null_or_empty);
    RUN_TEST(test_session_arm_workout_state);
    RUN_TEST(test_session_activate_on_belt_start);
    RUN_TEST(test_session_free_speed_mode_no_target);
    RUN_TEST(test_session_time_step_completion);
    RUN_TEST(test_session_distance_step_completion);
    RUN_TEST(test_session_pause_freezes_time_and_distance);
    RUN_TEST(test_session_resume_preserves_progress);
    RUN_TEST(test_session_delayed_update);
    RUN_TEST(test_session_uint32_rollover);
    RUN_TEST(test_session_cut_drag);
    RUN_TEST(test_session_extend_rest);
    RUN_TEST(test_session_nedjogg_completion_pending);
    RUN_TEST(test_session_finalize_session);
    RUN_TEST(test_session_abort);
    RUN_TEST(test_session_pause_resume_idempotent);
    RUN_TEST(test_session_stop_hierarchy_1x_2x_3x);
    RUN_TEST(test_session_continuation_window_expiry);
    RUN_TEST(test_session_emergency_stop);
    RUN_TEST(test_session_interval_to_manual_state_preservation);
    RUN_TEST(test_session_remaining_drag_speed_adjustment_accept);
    RUN_TEST(test_session_remaining_drag_speed_adjustment_reject_and_timeout);
    RUN_TEST(test_session_immutability);
    RUN_TEST(test_session_distance_step_overshoot_rollover);
    RUN_TEST(test_session_intent_latching_and_clear);
    RUN_TEST(test_session_speed_adjustment_prompt_forced_close_on_step_exit);
    RUN_TEST(test_session_motion_gating_jitter_debounce);
    RUN_TEST(test_session_multiple_speed_adjustments_net_delta);
    RUN_TEST(test_session_target_reissue_after_2x_stop_and_estop);
    UNITY_END();
}

#if defined(ARDUINO)
void setup() {
    Serial.begin(115200);
    delay(2000);
    run_all_workout_session_tests();
}
void loop() {}
#else
int main(int argc, char** argv) {
    run_all_workout_session_tests();
    return 0;
}
#endif
