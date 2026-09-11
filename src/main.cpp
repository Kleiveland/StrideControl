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
        const auto runnerLoc = runtime_.getComposite().getVirtualTreadmill().getRunnerLocation();
        const auto runnerPresence = telem.snapshot.runner.presence;
        if (runnerPresence == stridecontrol::RunnerPresence::Active) {
            report.runnerPresence = "ACTIVE";
        } else if (runnerLoc == stridecontrol::VirtualRunnerLocation::OnBelt) {
            report.runnerPresence = "Present / OnBelt";
        } else if (runnerLoc == stridecontrol::VirtualRunnerLocation::OnSideRails) {
            report.runnerPresence = "SIDE_RAILS";
        } else {
            report.runnerPresence = stridecontrol::runnerPresenceName(runnerPresence);
        }
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

#if !defined(STRIDECONTROL_TESTBENCH)
static stridecontrol::ImuInterface s_imuInterface;
static stridecontrol::CsafeInterface s_csafeInterface;
static stridecontrol::RunnerDynamics s_runnerDynamics;
static stridecontrol::InclineVerifier s_inclineVerifier;
static stridecontrol::MaintenanceService s_maintenanceService;
static stridecontrol::BleManager s_bleManager;
static stridecontrol::HeartRateClient s_heartRateClient;
static stridecontrol::RscService s_rscService;
static stridecontrol::FtmsService s_ftmsService;
#endif

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
    // 4. Initialize Testbench Control Runtime (Clean Idle boot contract)
    s_testbenchRuntime.begin();
    s_webServerManager.attachSimulatorRuntime(&s_testbenchRuntime);

    // 5. Start Dedicated Core 0 Testbench Control Task
    const bool taskOk = s_testbenchRuntime.startControlTask();
    Serial.printf("[Testbench] TestbenchControlRuntime task start: %s\n", taskOk ? "SUCCESS" : "FAILED");
    Serial.println("[Testbench] Testbench mode active: TreadmillSimulator closed-loop host running on Core 0.");
#else
    // 4. Initialize Sensors & Subsystem Orchestration
    s_console.begin();

    // Configure SpeedSensor per DESIGN_GUIDE.md §4.3 (integrated GPIO allocation): GPIO 3
    stridecontrol::SpeedSensorConfig speedConfig;
    speedConfig.inputPin = GPIO_NUM_3;
    speedConfig.useInternalPullup = true;
    const bool speedOk = s_speedSensor.begin(speedConfig);

    // Configure InclineSensor per DESIGN_GUIDE.md §4.3: GPIO 14
    // NOTE: GPIO 2 is reserved exclusively for TXS0108E OE per §4.3 - must never be reused.
    stridecontrol::InclineSensorConfig inclineConfig;
    inclineConfig.inputPin = GPIO_NUM_14;
    inclineConfig.useInternalPullup = true;
    const bool inclineOk = s_inclineSensor.begin(inclineConfig, stridecontrol::InclineCalibration{});

    Serial.printf("[Sensors] SpeedSensor begin: %s (Pin %d, Pullup)\n", speedOk ? "SUCCESS" : "FAILED", speedConfig.inputPin);
    Serial.printf("[Sensors] InclineSensor begin: %s (Pin %d, Pullup)\n", inclineOk ? "SUCCESS" : "FAILED", inclineConfig.inputPin);

    const bool imuOk = s_imuInterface.begin();
    const bool csafeOk = s_csafeInterface.begin();
    const bool runnerDynamicsOk = s_runnerDynamics.begin();
    const bool inclineVerifierOk = s_inclineVerifier.begin();
    s_maintenanceService.begin(&stridecontrol::SettingsService::instance());

    s_bleManager.attachServices(&s_rscService, &s_ftmsService);
    const stridecontrol::BleConfig bleConfig{};
    const bool bleOk = s_bleManager.begin(bleConfig);
    const bool hrOk = s_heartRateClient.begin(bleConfig, &s_bleManager);

    Serial.printf("[Sensors] ImuInterface begin: %s\n", imuOk ? "SUCCESS" : "FAILED");
    Serial.printf("[Sensors] CsafeInterface begin: %s\n", csafeOk ? "SUCCESS" : "FAILED");
    Serial.printf("[Sensors] RunnerDynamics begin: %s\n", runnerDynamicsOk ? "SUCCESS" : "FAILED");
    Serial.printf("[Sensors] InclineVerifier begin: %s\n", inclineVerifierOk ? "SUCCESS" : "FAILED");
    Serial.printf("[BLE] BleManager begin: %s\n", bleOk ? "SUCCESS" : "FAILED");
    Serial.printf("[BLE] HeartRateClient begin: %s\n", hrOk ? "SUCCESS" : "FAILED");

    stridecontrol::ApplicationOrchestratorDependencies orchestratorDeps{};
    orchestratorDeps.speedSensor = &s_speedSensor;
    orchestratorDeps.inclineSensor = &s_inclineSensor;
    orchestratorDeps.diagnosticsService = &s_diagnosticsService;
    orchestratorDeps.imuInterface = &s_imuInterface;
    orchestratorDeps.csafeInterface = &s_csafeInterface;
    orchestratorDeps.runnerDynamics = &s_runnerDynamics;
    orchestratorDeps.inclineVerifier = &s_inclineVerifier;
    orchestratorDeps.maintenanceService = &s_maintenanceService;
    orchestratorDeps.bleManager = &s_bleManager;
    orchestratorDeps.hrClient = &s_heartRateClient;
    orchestratorDeps.bleConfig = bleConfig;
    s_orchestrator.begin(orchestratorDeps);

    // 5. Initialize System Manager and Start Dedicated Core 0 Control Task
    s_systemManager.begin(millis());
    s_systemManager.startControlTask(&s_orchestrator);
    s_webServerManager.attachCommandStager(&s_systemManager.getControlRuntime());

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
            s_testbenchRuntime.setSimRunner(stridecontrol::VirtualRunnerMode::RunningOnBelt, 180, 0.35f, true);
        } else if (c == 's' || c == 'S') {
            Serial.println("[SerialCmd] Stop triggered");
            s_testbenchRuntime.triggerStop(nowMs);
            s_testbenchRuntime.setSimRunner(stridecontrol::VirtualRunnerMode::RunningOnBelt, 0, 0.0f, true);
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
        const auto runnerLoc = s_testbenchRuntime.getComposite().getVirtualTreadmill().getRunnerLocation();
        const char* presenceStr = (telem.snapshot.runner.presence == stridecontrol::RunnerPresence::Active) ? "ACTIVE" :
            (runnerLoc == stridecontrol::VirtualRunnerLocation::OnBelt ? "Present / OnBelt" :
            (runnerLoc == stridecontrol::VirtualRunnerLocation::OnSideRails ? "SIDE_RAILS" : stridecontrol::runnerPresenceName(telem.snapshot.runner.presence)));
        const uint16_t cadence = s_testbenchRuntime.getComposite().getVirtualTreadmill().getCadenceSpm();

        Serial.printf("[Telemetry] Time:%u | State:%s | Presence:%s | Cadence:%u | Step:%u (%s) | Rem:%us | TargetSpd:%.1f km/h | SimSpd:%.2f km/h | TargetInc:%.0f%% | SimInc:%.0f%% | Dist:%.1f m | Auth:%s\n",
            telem.snapshot.timestampMs,
            stateStr,
            presenceStr,
            cadence,
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