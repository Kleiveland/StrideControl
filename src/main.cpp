#include <Arduino.h>
#include "FileSystemManager.h"
#include "SettingsService.h"
#include "NetworkManager.h"
#include "WebServerManager.h"

#if defined(STRIDECONTROL_TESTBENCH)
#include "TestbenchControlRuntime.h"
#else
// Core Control & Domain Dependencies
#include "ConsoleInterface.h"
#include "SpeedCalibration.h"
#include "DiagnosticsService.h"
#include "SpeedSensor.h"
#include "InclineSensor.h"
#include "ApplicationOrchestrator.h"
#include "SystemManager.h"

// Additional Production Sensor Dependencies
#include "ImuInterface.h"
#include "CsafeInterface.h"
#include "RunnerDynamics.h"
#include "InclineVerifier.h"
#include "MaintenanceService.h"
#include "BleManager.h"
#include "HeartRateClient.h"
#include "RscService.h"
#include "FtmsService.h"
#endif

static stridecontrol::NetworkManager s_networkManager;
static stridecontrol::WebServerManager s_webServerManager;

#if defined(STRIDECONTROL_TESTBENCH)
static stridecontrol::TestbenchControlRuntime s_testbenchRuntime;

class TestbenchTelemetryProvider : public stridecontrol::ITelemetryProvider {
public:
    explicit TestbenchTelemetryProvider(stridecontrol::TestbenchControlRuntime& runtime)
        : runtime_(runtime) {}

    bool getTelemetry(stridecontrol::TelemetryReport& report) const override {
        const uint32_t nowMs = millis();
        const stridecontrol::TestbenchTelemetry telem = runtime_.getTelemetry(nowMs);
        report.timestampMs = telem.snapshot.timestampMs;
        report.authority = telem.authoritative;
        report.actualSpeedKmh = telem.snapshot.speed.speedKmh;
        report.actualInclinePct = telem.snapshot.incline.estimatedInclinePct;
        report.runnerDistanceKm = telem.snapshot.runner.validatedDistanceKm;
        report.sessionState = stridecontrol::workoutSessionStateName(telem.sessionSnapshot.state);
        report.stepIndex = telem.sessionSnapshot.currentStepIndex;
        report.stepRemainingMs = telem.sessionSnapshot.stepRemainingMs;
        report.targetSpeedKmh = telem.simTargetSpeedKmh;
        report.targetInclinePct = telem.simTargetInclinePct;
        report.runnerSpeedKmh = telem.snapshot.runner.runnerSpeedKmh;
        report.beltDistanceKm = runtime_.getComposite().getVirtualTreadmill().getOdometerKm();
        report.runnerPresence = stridecontrol::runnerPresenceName(telem.snapshot.runner.presence);
        report.droppedEventsCount = runtime_.getComposite().getDroppedEventsCount();
        report.heartRateBpm = telem.snapshot.heartRate.heartRateBpm;
        report.heartRateValid = telem.snapshot.heartRate.heartRateValid;
        report.totalElapsedTimeMs = telem.sessionSnapshot.totalElapsedTimeMs;
        return true;
    }

private:
    stridecontrol::TestbenchControlRuntime& runtime_;
};
static TestbenchTelemetryProvider s_telemetryProvider(s_testbenchRuntime);

#else
// Global Hardware & Control Instances
static stridecontrol::ConsoleInterface s_console;
static stridecontrol::SpeedCalibration s_speedCalibration;
static stridecontrol::DiagnosticsService s_diagnosticsService;
static stridecontrol::SpeedSensor s_speedSensor;
static stridecontrol::InclineSensor s_inclineSensor;
static stridecontrol::ApplicationOrchestrator s_orchestrator;
static stridecontrol::SystemManager s_systemManager(s_console, s_speedCalibration, s_diagnosticsService);

class ProductionTelemetryProvider : public stridecontrol::ITelemetryProvider {
public:
    ProductionTelemetryProvider(
        stridecontrol::ApplicationOrchestrator& orchestrator,
        stridecontrol::SystemManager& systemManager
    ) : orchestrator_(orchestrator), systemManager_(systemManager) {}

    bool getTelemetry(stridecontrol::TelemetryReport& report) const override {
        const uint32_t nowMs = millis();
        const stridecontrol::ApplicationSnapshot snap = orchestrator_.getSnapshot();
        const stridecontrol::WorkoutSessionSnapshot sessSnap =
            systemManager_.getControlRuntime().getSessionSnapshot();
        const stridecontrol::StagedTargets staged =
            systemManager_.getControlRuntime().getStagedTargets();

        report.timestampMs = snap.timestampMs;
        report.authority = stridecontrol::ControlRuntime::isSnapshotAuthoritative(snap, nowMs);
        report.actualSpeedKmh = snap.speed.speedKmh;
        report.actualInclinePct = snap.incline.estimatedInclinePct;
        report.runnerDistanceKm = snap.runner.validatedDistanceKm;
        report.sessionState = stridecontrol::workoutSessionStateName(sessSnap.state);
        report.stepIndex = sessSnap.currentStepIndex;
        report.stepRemainingMs = sessSnap.stepRemainingMs;
        report.targetSpeedKmh = sessSnap.hasSpeedTarget ? sessSnap.targetSpeedKmh : staged.speedKmh;
        report.targetInclinePct = sessSnap.hasInclineTarget ? static_cast<float>(sessSnap.targetInclinePct) : staged.inclinePct;
        report.runnerSpeedKmh = snap.runner.runnerSpeedKmh;
        report.beltDistanceKm = snap.runner.validatedDistanceKm;
        report.runnerPresence = stridecontrol::runnerPresenceName(snap.runner.presence);
        report.droppedEventsCount = 0;
        report.heartRateBpm = snap.heartRate.heartRateBpm;
        report.heartRateValid = snap.heartRate.heartRateValid;
        report.totalElapsedTimeMs = sessSnap.totalElapsedTimeMs;
        return true;
    }

private:
    stridecontrol::ApplicationOrchestrator& orchestrator_;
    stridecontrol::SystemManager& systemManager_;
};
static ProductionTelemetryProvider s_telemetryProvider(s_orchestrator, s_systemManager);
#endif

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.println("         StrideControl Firmware Initializing      ");
#if defined(STRIDECONTROL_TESTBENCH)
    Serial.println("   Profile: SEEED_XIAO_ESP32S3 (Testbench Mode)   ");
#else
    Serial.println("   Profile: ESP32-S3-DEVKITC-1-N16R8 (Production) ");
#endif
    Serial.println("==================================================");

    // 1. Mount LittleFS Filesystem for UI Hosting & Configuration
    stridecontrol::FileSystemManager::instance().begin();

    // 2. Initialize Atomic Settings Service
    stridecontrol::SettingsService::instance().begin();

    // 3. Initialize Network & Web Subsystems
    s_networkManager.begin();
    s_webServerManager.begin(&s_telemetryProvider);

#if defined(STRIDECONTROL_TESTBENCH)
    // 4. Initialize Testbench Control Runtime
    s_testbenchRuntime.begin();
    s_webServerManager.attachSimulatorRuntime(&s_testbenchRuntime);

    // 5. Build Sample Workout Definition (Step 0: Warmup 10s @ 5 km/h 1%, Step 1: Work 15s @ 10 km/h 2%)
    stridecontrol::WorkoutDefinition workoutDef{};
    workoutDef.id = 1;
    strncpy(workoutDef.name, "Testbench", stridecontrol::MAX_WORKOUT_NAME_LENGTH);
    workoutDef.segmentCount = 2;

    workoutDef.segments[0].id = 101;
    workoutDef.segments[0].type = stridecontrol::SegmentType::SINGLE_STEP;
    workoutDef.segments[0].repetitions = 1;
    workoutDef.segments[0].stepCount = 1;
    workoutDef.segments[0].steps[0] = stridecontrol::WorkoutStep(
        1, stridecontrol::StepRole::WARMUP, stridecontrol::DurationType::TIME_SECONDS,
        10, stridecontrol::SpeedMode::FIXED, 5.0f, 1, true
    );

    workoutDef.segments[1].id = 102;
    workoutDef.segments[1].type = stridecontrol::SegmentType::SINGLE_STEP;
    workoutDef.segments[1].repetitions = 1;
    workoutDef.segments[1].stepCount = 1;
    workoutDef.segments[1].steps[0] = stridecontrol::WorkoutStep(
        2, stridecontrol::StepRole::WORK, stridecontrol::DurationType::TIME_SECONDS,
        15, stridecontrol::SpeedMode::FIXED, 10.0f, 2, true
    );

    const bool armOk = s_testbenchRuntime.armWorkout(workoutDef, millis());
    Serial.printf("[Testbench] Sample workout armed: %s\n", armOk ? "SUCCESS" : "FAILED");

    // 6. Start Dedicated Core 0 Testbench Control Task
    const bool taskOk = s_testbenchRuntime.startControlTask();
    Serial.printf("[Testbench] TestbenchControlRuntime task start: %s\n", taskOk ? "SUCCESS" : "FAILED");
    Serial.println("[Testbench] Testbench mode active: TreadmillSimulator closed-loop host running on Core 0.");
#else
    // 4. Initialize Sensors & Subsystem Orchestration
    s_console.begin();

    // Configure SpeedSensor (XIAO D0 / GPIO 1) with internal pullup
    stridecontrol::SpeedSensorConfig speedConfig;
    speedConfig.inputPin = GPIO_NUM_1;
    speedConfig.useInternalPullup = true;
    const bool speedOk = s_speedSensor.begin(speedConfig);

    // Configure InclineSensor (XIAO D1 / GPIO 2) with internal pullup
    stridecontrol::InclineSensorConfig inclineConfig;
    inclineConfig.inputPin = GPIO_NUM_2;
    inclineConfig.useInternalPullup = true;
    const bool inclineOk = s_inclineSensor.begin(inclineConfig, stridecontrol::InclineCalibration{});

    Serial.printf("[Sensors] SpeedSensor begin: %s (Pin %d, Pullup)\n", speedOk ? "SUCCESS" : "FAILED", speedConfig.inputPin);
    Serial.printf("[Sensors] InclineSensor begin: %s (Pin %d, Pullup)\n", inclineOk ? "SUCCESS" : "FAILED", inclineConfig.inputPin);

    stridecontrol::ApplicationOrchestratorDependencies orchestratorDeps{};
    orchestratorDeps.speedSensor = &s_speedSensor;
    orchestratorDeps.inclineSensor = &s_inclineSensor;
    orchestratorDeps.diagnosticsService = &s_diagnosticsService;
    s_orchestrator.begin(orchestratorDeps);

    // 5. Initialize System Manager and Start Dedicated Core 0 Control Task
    s_systemManager.begin(millis());
    s_systemManager.startControlTask(&s_orchestrator);

    Serial.println("[System] Production hardware profile active.");
#endif
}

void loop() {
    s_networkManager.update();

#if defined(STRIDECONTROL_TESTBENCH)
    const uint32_t nowMs = millis();

    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'q' || c == 'Q') {
            Serial.println("[SerialCmd] QuickStart triggered");
            s_testbenchRuntime.triggerQuickStart(nowMs);
        } else if (c == 's' || c == 'S') {
            Serial.println("[SerialCmd] Stop triggered");
            s_testbenchRuntime.triggerStop(nowMs);
        } else if (c == '+') {
            Serial.println("[SerialCmd] SpeedPlus triggered");
            s_testbenchRuntime.stepSimSpeed(true, nowMs);
        } else if (c == '-') {
            Serial.println("[SerialCmd] SpeedMinus triggered");
            s_testbenchRuntime.stepSimSpeed(false, nowMs);
        }
    }

    static uint32_t s_lastDiagMs = 0;
    if (nowMs - s_lastDiagMs >= 500) {
        s_lastDiagMs = nowMs;
        const stridecontrol::TestbenchTelemetry telem = s_testbenchRuntime.getTelemetry(nowMs);
        const char* stateStr = stridecontrol::workoutSessionStateName(telem.sessionSnapshot.state);
        const char* roleStr = stridecontrol::stepRoleName(telem.sessionSnapshot.currentRole);
        const uint32_t remSeconds = telem.sessionSnapshot.stepRemainingMs / 1000;
        const double distMeters = telem.snapshot.runner.validatedDistanceKm * 1000.0;

        Serial.printf("[Telemetry] Time:%u | State:%s | Step:%u (%s) | Rem:%us | TargetSpd:%.1f km/h | SimSpd:%.2f km/h | TargetInc:%.0f%% | SimInc:%.0f%% | Dist:%.1f m | Auth:%s\n",
            telem.snapshot.timestampMs,
            stateStr,
            telem.sessionSnapshot.currentStepIndex,
            roleStr,
            remSeconds,
            telem.simTargetSpeedKmh,
            telem.snapshot.speed.speedKmh,
            telem.simTargetInclinePct,
            telem.snapshot.incline.estimatedInclinePct,
            distMeters,
            telem.authoritative ? "TRUE" : "FALSE"
        );
    }
#else
    s_systemManager.update();

    static uint32_t s_lastDiagMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - s_lastDiagMs >= 2000) {
        s_lastDiagMs = nowMs;
        const stridecontrol::ApplicationSnapshot snap = s_orchestrator.getSnapshot();
        const bool authoritative = stridecontrol::ControlRuntime::isSnapshotAuthoritative(snap, nowMs);
        const uint32_t lostAuthCount = s_systemManager.getControlRuntime().getLostAuthorityCount();
        const uint32_t minFreeStack = s_orchestrator.getMinFreeStackBytes();

        Serial.printf("[Telemetry] Seq:%u Time:%u ms | Speed: init=%d, stat=%s, val=%.2f km/h | Incline: init=%d, stat=%s, val=%.1f%% | Auth:%s | LostAuthCount:%u | StackHeadroom:%u B\n",
            snap.sequenceNumber,
            snap.timestampMs,
            snap.speed.initialized ? 1 : 0,
            stridecontrol::speedSensorStatusName(snap.speed.status),
            snap.speed.speedKmh,
            snap.incline.initialized ? 1 : 0,
            stridecontrol::inclineStatusName(snap.incline.status),
            snap.incline.estimatedInclinePct,
            authoritative ? "TRUE" : "FALSE",
            lostAuthCount,
            minFreeStack
        );
    }
#endif

    delay(10);
}