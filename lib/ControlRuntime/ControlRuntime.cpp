#include "ControlRuntime.h"
#include <cmath>
#include <atomic>
#include <Arduino.h>
#include "../SettingsService/SettingsService.h"

namespace stridecontrol {

ControlRuntime::ControlRuntime(
    ConsoleInterface& console,
    SpeedCalibration& calibration,
    DiagnosticsService& diagnostics
)
    : controller_(console, calibration, diagnostics),
      adapter_(controller_),
      session_(),
      dispatcher_(),
      coordinator_() {}

ControlRuntime::~ControlRuntime() {
    end();
}

bool ControlRuntime::begin(const WorkoutSessionConfig& sessionConfig) {
    if (initialized_) {
        return true;
    }

    if (commandQueue_ == nullptr) {
        commandQueue_ = xQueueCreate(kCommandQueueDepth, sizeof(ControlCommand));
    }

    controller_.begin();
    session_.begin(sessionConfig);
    dispatcher_.begin();

    initialized_ = true;
    lastAuthoritativeTimestampMs_ = 0;
    lostAuthorityCount_ = 0;
    authorityLostReported_ = false;
    return true;
}

void ControlRuntime::end() {
    stopControlTask(1000);

    if (commandQueue_ != nullptr) {
        vQueueDelete(commandQueue_);
        commandQueue_ = nullptr;
    }

    if (initialized_) {
        session_.end();
        controller_.end();
        initialized_ = false;
    }
}

bool ControlRuntime::isSnapshotAuthoritative(
    const ApplicationSnapshot& snapshot,
    uint32_t nowMs
) {
    static bool s_wasAuthoritative = false;
    const char* dropReason = nullptr;
    char reasonBuf[96]{};

    // 1. Initialized state check: Reject default or unpopulated snapshots
    if (snapshot.timestampMs == 0 && snapshot.sequenceNumber == 0) {
        dropReason = "Unpopulated (timestamp=0, seq=0)";
    }
    // 2. Rollover-safe snapshot age calculation
    else if ((nowMs - snapshot.timestampMs) > kMaxSnapshotAgeMs) {
        const uint32_t ageMs = nowMs - snapshot.timestampMs;
        snprintf(reasonBuf, sizeof(reasonBuf), "Age: %lu ms (limit %u ms)",
                 static_cast<unsigned long>(ageMs), static_cast<unsigned int>(kMaxSnapshotAgeMs));
        dropReason = reasonBuf;
    }
    // 3. System health check: Reject critical or fatal fault severities
    else if (snapshot.health.highestSeverity == FaultSeverity::Critical ||
             snapshot.health.highestSeverity == FaultSeverity::Fatal) {
        snprintf(reasonBuf, sizeof(reasonBuf), "Severity: %d",
                 static_cast<int>(snapshot.health.highestSeverity));
        dropReason = reasonBuf;
    }
    // 4. Speed sensor check with operational stopped-speed exemption
    else if (!snapshot.speed.initialized) {
        dropReason = "SpeedNotInit";
    } else if (snapshot.speed.status == SpeedSensorStatus::HardwareError ||
               snapshot.speed.status == SpeedSensorStatus::Uninitialized) {
        snprintf(reasonBuf, sizeof(reasonBuf), "SpeedStatus: %d",
                 static_cast<int>(snapshot.speed.status));
        dropReason = reasonBuf;
    } else if (snapshot.speed.speedKmh > 0.1f &&
               (!snapshot.speed.measurementValid ||
                snapshot.speed.status != SpeedSensorStatus::Measuring)) {
        snprintf(reasonBuf, sizeof(reasonBuf), "SpeedInvalid (valid=%d, status=%d, spd=%.2f km/h)",
                 static_cast<int>(snapshot.speed.measurementValid),
                 static_cast<int>(snapshot.speed.status),
                 snapshot.speed.speedKmh);
        dropReason = reasonBuf;
    }
    // 5. Incline sensor basic health
    else if (!snapshot.incline.initialized ||
             snapshot.incline.status == InclineStatus::HardwareError) {
        snprintf(reasonBuf, sizeof(reasonBuf), "InclineStatus: init=%d, stat=%d",
                 static_cast<int>(snapshot.incline.initialized),
                 static_cast<int>(snapshot.incline.status));
        dropReason = reasonBuf;
    }
    // 6. Runner dynamics check when moving
    else if (snapshot.runner.initialized && snapshot.speed.speedKmh > 0.1f &&
             (snapshot.runner.distancePauseReason == RunnerDistancePauseReason::SpeedInvalid ||
              snapshot.runner.distancePauseReason == RunnerDistancePauseReason::ImuInvalid)) {
        snprintf(reasonBuf, sizeof(reasonBuf), "PauseReason: %d",
                 static_cast<int>(snapshot.runner.distancePauseReason));
        dropReason = reasonBuf;
    }

    const bool authoritative = (dropReason == nullptr);

    if (s_wasAuthoritative && !authoritative) {
        Serial.printf("[Authority] DROPPED: Reason: %s | Time: %lu ms | Seq: %lu | Now: %lu ms\n",
                      dropReason ? dropReason : "Unknown",
                      static_cast<unsigned long>(snapshot.timestampMs),
                      static_cast<unsigned long>(snapshot.sequenceNumber),
                      static_cast<unsigned long>(nowMs));
    }
    s_wasAuthoritative = authoritative;

    return authoritative;
}

void ControlRuntime::update(const ApplicationSnapshot& snapshot, uint32_t nowMs) {
    if (!initialized_) {
        return;
    }

    // 1. Always update the physical controller so active hardware execution can complete or report failure
    controller_.update(nowMs);

    const bool authoritative = isSnapshotAuthoritative(snapshot, nowMs);

    if (authoritative) {
        lastAuthoritativeTimestampMs_ = nowMs;
        authorityLostReported_ = false;

        // 2. Execute sequenced coordinator tick (session.update followed by dispatcher.update)
        coordinator_.tick(session_, dispatcher_, adapter_, snapshot, nowMs);
    } else {
        lostAuthorityCount_++;

        // 3. LOST SNAPSHOT POLICY:
        // - Do NOT call coordinator_.tick()
        // - Do NOT call dispatcher_.update() (prevents target ingestion & delivery)
        // - Do NOT clear session intent or dispatcher staging
        // - Do NOT issue physical speed or incline commands

        // 4. Apply explicit existing WorkoutSession suspension contract:
        // If the session is actively Running, freeze/suspend it so stale time is
        // not counted when telemetry recovers later.
        if (session_.getSnapshot().state == WorkoutSessionState::Running) {
            session_.suspend(nowMs);
        }
    }
}

bool ControlRuntime::armWorkout(const ExpandedWorkout* workout, uint32_t nowMs) {
    if (!initialized_ || workout == nullptr || workout->totalSteps == 0) {
        return false;
    }
    // Prevent workout loading or rebinding while session is active
    if (session_.isActive()) {
        return false;
    }
    return session_.armWorkout(workout, nowMs);
}

bool ControlRuntime::abortWorkout(uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    return session_.abortSession(nowMs);
}

bool ControlRuntime::finalizeWorkout(uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    return session_.finalizeSession(nowMs);
}

bool ControlRuntime::canSafelyModifyWorkoutEngine() const {
    return !session_.isActive();
}

bool ControlRuntime::startControlTask(ApplicationOrchestrator* orchestrator) {
    if (taskRunning_ || taskHandle_ != nullptr) {
        return true;
    }
    if (orchestrator == nullptr) {
        return false;
    }

    orchestrator_ = orchestrator;
    stopRequested_ = false;
    taskRunning_ = false;

    exitSem_ = xSemaphoreCreateBinary();
    if (exitSem_ == nullptr) {
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        taskEntry,
        "ControlRuntime",
        kTaskStackSize,
        this,
        kTaskPriority,
        &taskHandle_,
        kTaskCore // Pinned strictly to Core 0 (PRO_CPU)
    );

    if (result != pdPASS) {
        vSemaphoreDelete(exitSem_);
        exitSem_ = nullptr;
        taskHandle_ = nullptr;
        return false;
    }

    return true;
}

bool ControlRuntime::stopControlTask(uint32_t timeoutMs) {
    if (!taskRunning_ && taskHandle_ == nullptr) {
        return true;
    }

    stopRequested_ = true;

    bool cleanExit = false;
    if (exitSem_ != nullptr) {
        cleanExit = (xSemaphoreTake(exitSem_, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
    }

    if (taskHandle_ != nullptr) {
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }
    taskRunning_ = false;
    stopRequested_ = false;

    if (exitSem_ != nullptr) {
        vSemaphoreDelete(exitSem_);
        exitSem_ = nullptr;
    }

    return cleanExit;
}

bool ControlRuntime::isTaskRunning() const {
    return taskRunning_;
}

void ControlRuntime::taskEntry(void* param) {
    auto* self = static_cast<ControlRuntime*>(param);
    if (self != nullptr) {
        self->runTaskLoop();
    }
}

void ControlRuntime::runTaskLoop() {
    taskRunning_ = true;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(kPeriodMs);

    while (!stopRequested_) {
        const uint32_t nowMs = millis();

        // 0. Drain staged external commands up to batch limit (8)
        processQueuedCommands(nowMs);

        ApplicationSnapshot snap{};
        if (orchestrator_ != nullptr) {
            snap = orchestrator_->getSnapshot();
        }

        this->update(snap, nowMs);

        const uint32_t executionDurationMs = millis() - nowMs;
        loopCount_++;
        if (executionDurationMs > kPeriodMs) {
            overrunCount_++;
        }

        const BaseType_t delaySuccess = xTaskDelayUntil(&lastWakeTime, periodTicks);
        if (delaySuccess != pdTRUE) {
            deadlineMissCount_++;
        }
    }

    taskRunning_ = false;
    if (exitSem_ != nullptr) {
        xSemaphoreGive(exitSem_);
    }
    vTaskSuspend(NULL);
}

void ControlRuntime::processQueuedCommands(uint32_t nowMs) {
    ControlCommand cmd{};
    size_t processed = 0;
    while (commandQueue_ != nullptr && xQueueReceive(commandQueue_, &cmd, 0) == pdTRUE && processed < 8) {
        const uint32_t cmdNowMs = (cmd.timestampMs != 0) ? cmd.timestampMs : nowMs;
        switch (cmd.type) {
            case ControlCommandType::QuickStart:
                controller_.submitSpeedTarget(1.0f, cmdNowMs);
                break;
            case ControlCommandType::Stop:
                controller_.submitStop(cmdNowMs);
                session_.registerPhysicalStop(cmdNowMs);
                break;
            case ControlCommandType::Pause:
                session_.suspend(cmdNowMs);
                break;
            case ControlCommandType::Resume:
                session_.resume(cmdNowMs);
                break;
            case ControlCommandType::SetSpeed:
                dispatcher_.stageSpeedTarget(cmd.data.target.speedKmh);
                break;
            case ControlCommandType::SetIncline:
                dispatcher_.stageInclineTarget(cmd.data.target.inclinePct);
                break;
            case ControlCommandType::StepSpeed: {
                const float currentSpd = controller_.getSnapshot().acceptedPhysicalSpeedTargetKmh;
                dispatcher_.stepSpeedTarget(cmd.data.stepSpeed.deltaSpeedKmh, currentSpd);
                break;
            }
            case ControlCommandType::StepIncline: {
                const float currentInc = controller_.getSnapshot().acceptedInclineTargetPct;
                dispatcher_.stepInclineTarget(cmd.data.stepIncline.deltaInclinePct, currentInc);
                break;
            }
            case ControlCommandType::ArmWorkout: {
                const WorkoutDefinition* def = SettingsService::instance().findWorkout(
                    cmd.data.arm.userId, cmd.data.arm.workoutId);
                if (def != nullptr && workoutEngine_.loadWorkout(*def)) {
                    session_.armWorkout(&workoutEngine_.getExpandedWorkout(), cmdNowMs, cmd.data.arm.userId);
                    Serial.printf("[ControlRuntime] Armed workout id=%u for user=%u\n", cmd.data.arm.workoutId, cmd.data.arm.userId);
                } else {
                    Serial.printf("[ControlRuntime] FAILED to arm workout id=%u for user=%u (def=%p)\n", cmd.data.arm.workoutId, cmd.data.arm.userId, def);
                }
                break;
            }
            case ControlCommandType::CancelWorkout:
                session_.abortSession(cmdNowMs);
                break;
            case ControlCommandType::FinalizeWorkout:
                session_.finalizeSession(cmdNowMs);
                break;
            case ControlCommandType::SetGuiMode:
                session_.setDesiredGuiMode(cmd.data.guiMode.userId, cmd.data.guiMode.isManual);
                break;
            case ControlCommandType::None:
            default:
                break;
        }
        processed++;
    }
}

WorkoutSessionSnapshot ControlRuntime::getSessionSnapshot() const {
    return session_.getSnapshot();
}

TreadmillControllerSnapshot ControlRuntime::getControllerSnapshot() const {
    return controller_.getSnapshot();
}

StagedTargets ControlRuntime::getStagedTargets() const {
    return dispatcher_.getStagedTargets();
}

bool ControlRuntime::isSessionActive() const {
    return session_.isActive();
}

bool ControlRuntime::isSessionSuspended() const {
    return session_.isSuspended();
}

bool ControlRuntime::isControllerReady() const {
    return controller_.isReady();
}

bool ControlRuntime::isControllerBusy() const {
    return controller_.isBusy();
}

uint32_t ControlRuntime::getLostAuthorityCount() const {
    return lostAuthorityCount_;
}

uint32_t ControlRuntime::getLoopCount() const {
    return loopCount_;
}

uint32_t ControlRuntime::getDeadlineMissCount() const {
    return deadlineMissCount_;
}

uint32_t ControlRuntime::getOverrunCount() const {
    return overrunCount_;
}

const char* ControlRuntime::version() {
    return "ControlRuntime/1.0.0";
}

} // namespace stridecontrol

