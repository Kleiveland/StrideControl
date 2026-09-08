#include "ControlRuntime.h"
#include <cmath>

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
    // 1. Initialized state check: Reject default or unpopulated snapshots
    if (snapshot.timestampMs == 0 && snapshot.sequenceNumber == 0) {
        return false;
    }

    // 2. Rollover-safe snapshot age calculation
    const uint32_t ageMs = nowMs - snapshot.timestampMs;
    if (ageMs > kMaxSnapshotAgeMs) {
        return false;
    }

    // 3. System health check: Reject critical or fatal fault severities
    if (snapshot.health.highestSeverity == FaultSeverity::Critical ||
        snapshot.health.highestSeverity == FaultSeverity::Fatal) {
        return false;
    }

    // 4. Speed sensor check with operational stopped-speed exemption
    if (!snapshot.speed.initialized) {
        return false;
    }
    if (snapshot.speed.status == SpeedSensorStatus::HardwareError ||
        snapshot.speed.status == SpeedSensorStatus::Uninitialized) {
        return false;
    }

    if (snapshot.speed.speedKmh > 0.1f) {
        // When belt is moving, speed pulses must be actively measuring and valid
        if (!snapshot.speed.measurementValid ||
            snapshot.speed.status != SpeedSensorStatus::Measuring) {
            return false;
        }
    } else {
        // Standstill exemption: At operational standstill (speed <= 0.1 km/h),
        // pulse intervals are not generated. AwaitingFirstPulse and TimedOut
        // are accepted as valid stopped belt states.
    }

    // 5. Incline sensor basic health
    if (!snapshot.incline.initialized ||
        snapshot.incline.status == InclineStatus::HardwareError) {
        return false;
    }

    // 6. Runner dynamics check when moving
    if (snapshot.runner.initialized && snapshot.speed.speedKmh > 0.1f) {
        if (snapshot.runner.distancePauseReason == RunnerDistancePauseReason::SpeedInvalid ||
            snapshot.runner.distancePauseReason == RunnerDistancePauseReason::ImuInvalid) {
            return false;
        }
    }

    return true;
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

