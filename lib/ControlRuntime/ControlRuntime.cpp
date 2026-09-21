#include "ControlRuntime.h"
#include <cmath>
#include <atomic>
#include <Arduino.h>
#include "../SettingsService/SettingsService.h"
#include "../DiagnosticsLog/DiagnosticsLog.h"

namespace stridecontrol {

ControlRuntime::ControlRuntime(
    ConsoleInterface& console,
    SpeedCalibration& calibration,
    DiagnosticsService& diagnostics
)
    : diagnostics_(&diagnostics),
      console_(console),
      calibration_(calibration),
      controller_(console, calibration, diagnostics),
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
    rampTestTracker_.reset();
    inclineTracker_.reset();

    initialized_ = true;
    lastAuthoritativeTimestampMs_ = 0;
    lostAuthorityCount_ = 0;
    authorityLostReported_ = false;
    connectionWarningActive_ = false;
    wasAuthoritative_ = false;
    previousCsafeQualifiedState_ = CsafeMachineState::Unknown;
    portENTER_CRITICAL(&snapshotMux_);
    publishedSessionSnapshot_ = WorkoutSessionSnapshot{};
    publishedControllerSnapshot_ = TreadmillControllerSnapshot{};
    portEXIT_CRITICAL(&snapshotMux_);

    portENTER_CRITICAL(&inclineCommandContextMux_);
    publishedInclineCommandContext_ = InclineVerificationCommandInput{};
    portEXIT_CRITICAL(&inclineCommandContextMux_);

    portENTER_CRITICAL(&commandExecutionStatusMux_);
    publishedCommandExecutionStatus_ = CommandExecutionStatus{};
    portEXIT_CRITICAL(&commandExecutionStatusMux_);
    lastReportedAbortedRequestId_ = 0;

    publishSnapshots();

    return true;
}

void ControlRuntime::end() {
    stopControlTask(1000);

    if (commandQueue_ != nullptr) {
        vQueueDelete(commandQueue_);
        commandQueue_ = nullptr;
    }

    if (initialized_) {
        dispatcher_.clearAllTargets();
        session_.end();
        controller_.end();
        initialized_ = false;
        previousCsafeQualifiedState_ = CsafeMachineState::Unknown;
        csafeStateInitialized_ = false;

        portENTER_CRITICAL(&snapshotMux_);
        publishedSessionSnapshot_ = WorkoutSessionSnapshot{};
        publishedControllerSnapshot_ = TreadmillControllerSnapshot{};
        portEXIT_CRITICAL(&snapshotMux_);

        portENTER_CRITICAL(&inclineCommandContextMux_);
        publishedInclineCommandContext_ = InclineVerificationCommandInput{};
        portEXIT_CRITICAL(&inclineCommandContextMux_);

        portENTER_CRITICAL(&commandExecutionStatusMux_);
        publishedCommandExecutionStatus_ = CommandExecutionStatus{};
        portEXIT_CRITICAL(&commandExecutionStatusMux_);
        lastReportedAbortedRequestId_ = 0;

        rampTestTracker_.reset();
        inclineTracker_.reset();
    }
}

bool ControlRuntime::isSnapshotAuthoritative(
    const ApplicationSnapshot& snapshot,
    uint32_t nowMs,
    bool* wasAuthoritative
) {
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

    if (wasAuthoritative != nullptr) {
        if (*wasAuthoritative && !authoritative) {
            Serial.printf("[Authority] DROPPED: Reason: %s | Time: %lu ms | Seq: %lu | Now: %lu ms\n",
                          dropReason ? dropReason : "Unknown",
                          static_cast<unsigned long>(snapshot.timestampMs),
                          static_cast<unsigned long>(snapshot.sequenceNumber),
                          static_cast<unsigned long>(nowMs));
            DiagnosticsLog::instance().addEntryf("[Authority] DROPPED: Reason: %s | Time: %lu ms | Seq: %lu | Now: %lu ms",
                                                 dropReason ? dropReason : "Unknown",
                                                 static_cast<unsigned long>(snapshot.timestampMs),
                                                 static_cast<unsigned long>(snapshot.sequenceNumber),
                                                 static_cast<unsigned long>(nowMs));
        }
        *wasAuthoritative = authoritative;
    }

    return authoritative;
}

void ControlRuntime::update(const ApplicationSnapshot& snapshot, uint32_t nowMs) {
    if (!initialized_) {
        return;
    }

    // 1. Always update the physical controller so active hardware execution can complete or report failure
    controller_.update(nowMs);

    rampTestTracker_.update(
        snapshot.speed.speedKmh,
        snapshot.speed.measurementValid,
        nowMs
    );

    if (rampTestTracker_.targetDispatchRequested()) {
        TargetContext ctx;
        ctx.origin = TargetOrigin::Commissioning;
        ctx.timestampMs = nowMs;
        dispatcher_.stageSpeedTarget(rampTestTracker_.targetSpeedKmh(), ctx);
        rampTestTracker_.acknowledgeTargetDispatched();
    }

    inclineTracker_.update(
        snapshot.incline.estimatedInclinePct,
        snapshot.imu.relativeDeckAngleDeg,
        snapshot.imu.dataValid,
        nowMs
    );

    if (inclineTracker_.isHomedSettled()) {
        requestInclineCommissioningAction(true, true);
        inclineTracker_.onHomedConfirmed(nowMs);
    }

    // E-Stop Detection (authoritative hardware GPIO 18, independent of snapshot authority)
    const bool currentEstop = console_.isEmergencyStopActive();
    if (currentEstop != previousEstopActive_) {
        previousEstopActive_ = currentEstop;
        if (currentEstop) {
            session_.registerEmergencyStop(nowMs);
            DiagnosticsLog::instance().addEntryf("[Safety] Emergency Stop engaged at %lu ms", static_cast<unsigned long>(nowMs));
            Serial.printf("[Safety] Emergency Stop engaged at %lu ms\n", static_cast<unsigned long>(nowMs));
        } else {
            session_.registerEmergencyStopCleared();
            DiagnosticsLog::instance().addEntryf("[Safety] Emergency Stop cleared at %lu ms", static_cast<unsigned long>(nowMs));
            Serial.printf("[Safety] Emergency Stop cleared at %lu ms\n", static_cast<unsigned long>(nowMs));
        }
    }

    const bool authoritative = isSnapshotAuthoritative(snapshot, nowMs, &wasAuthoritative_);

    if (authoritative) {
        lastAuthoritativeTimestampMs_ = nowMs;
        authorityLostReported_ = false;
        authorityLostSinceMs_ = 0; // Reset the loss streak - authority has recovered
        connectionWarningActive_ = false;

        // CSAFE Stop Hierarchy Detection
        const bool csafeValid = snapshot.csafe.initialized &&
                                snapshot.csafe.online &&
                                snapshot.csafe.machineStateFresh &&
                                snapshot.csafe.qualifiedState != CsafeMachineState::Unknown;
        if (csafeValid) {
            if (!csafeStateInitialized_) {
                // Re-baseline without firing transitions (initial connect or reconnect)
                previousCsafeQualifiedState_ = snapshot.csafe.qualifiedState;
                csafeStateInitialized_ = true;
            } else if (snapshot.csafe.qualifiedState != previousCsafeQualifiedState_) {
                const bool estopEngaged = currentEstop || session_.getSnapshot().isEmergencyStopped;
                // Physical Stop 1: InUse -> Paused
                if (previousCsafeQualifiedState_ == CsafeMachineState::InUse &&
                    snapshot.csafe.qualifiedState == CsafeMachineState::Paused) {
                    if (!estopEngaged) {
                        session_.registerPhysicalStop(nowMs);
                    }
                }
                // Physical Stop 2: Paused -> Ready
                else if (previousCsafeQualifiedState_ == CsafeMachineState::Paused &&
                         snapshot.csafe.qualifiedState == CsafeMachineState::Ready) {
                    if (!estopEngaged) {
                        session_.registerPhysicalStop(nowMs);
                    }
                }
                // Physical Stop 3: InUse -> Ready (Direct reset / intentional stop)
                else if (previousCsafeQualifiedState_ == CsafeMachineState::InUse &&
                         snapshot.csafe.qualifiedState == CsafeMachineState::Ready) {
                    if (!estopEngaged) {
                        session_.registerPhysicalStop(nowMs);
                    }
                }
                previousCsafeQualifiedState_ = snapshot.csafe.qualifiedState;
            }
        } else {
            // Communication loss or stale telemetry: mark uninitialized so reconnect re-baselines
            csafeStateInitialized_ = false;
        }

        // 3rd Physical Stop Detection and Keypad Routing
        PhysicalButtonEvent btnEvent{};
        while (console_.receivePhysicalButtonEvent(btnEvent)) {
            if (btnEvent.action != PhysicalButtonAction::Pressed) {
                continue;
            }
            const uint32_t eventTs = (btnEvent.timestampMs != 0) ? btnEvent.timestampMs : nowMs;

            if (btnEvent.button == ButtonId::Stop) {
                const bool estopEngaged = currentEstop || session_.getSnapshot().isEmergencyStopped;
                if (!estopEngaged &&
                    session_.getSnapshot().continuationWindowActive &&
                    csafeValid &&
                    snapshot.csafe.qualifiedState == CsafeMachineState::Ready) {
                    session_.registerPhysicalStop(eventTs);
                }
            } else if (btnEvent.button == ButtonId::QuickStart) {
                // Physical QuickStart is handled by treadmill hardware independently.
                // Observed passively via CSAFE InUse and belt movement in WorkoutSession::update().
            } else if (btnEvent.button == ButtonId::SpeedPlus || btnEvent.button == ButtonId::SpeedMinus) {
                if (rampTestTracker_.active()) continue;
                const float delta = (btnEvent.button == ButtonId::SpeedPlus) ? 0.1f : -0.1f;
                const float acceptedTarget = controller_.getSnapshot().acceptedPhysicalSpeedTargetKmh;
                const float beltMovingThreshold = session_.getConfig().beltMovingThresholdKmh;
                const float currentSpd = (acceptedTarget > beltMovingThreshold) ? acceptedTarget : snapshot.speed.speedKmh;
                const float newSpeed = (currentSpd + delta > 0.0f) ? (currentSpd + delta) : 0.0f;
                session_.reportWorkSpeedAdjustment(newSpeed);
            } else if (btnEvent.button == ButtonId::InclinePlus || btnEvent.button == ButtonId::InclineMinus) {
                // Physical incline adjustment is handled by treadmill hardware independently.
                // Observed passively via snapshot.incline.estimatedInclinePct; no outbound command.
            }
        }

        // 2. Execute sequenced coordinator tick (session.update followed by dispatcher.update)
        coordinator_.tick(session_, dispatcher_, adapter_, snapshot, nowMs);
    } else {
        lostAuthorityCount_++;
        if (authorityLostSinceMs_ == 0) {
            authorityLostSinceMs_ = nowMs;
        }
        connectionWarningActive_ = (nowMs - authorityLostSinceMs_) >= kAuthorityLossWarningThresholdMs;
        if (connectionWarningActive_ && !authorityLostReported_) {
            DiagnosticsLog::instance().addEntryf(
                "Authority lost for >= %lums, showing reconnect indicator",
                static_cast<unsigned long>(nowMs - authorityLostSinceMs_));
            authorityLostReported_ = true;
        }

        csafeStateInitialized_ = false;

        // Unconditionally drain physical button queue even when authority is lost, but do not trigger Stop
        PhysicalButtonEvent btnEvent{};
        while (console_.receivePhysicalButtonEvent(btnEvent)) {
            // Drained without action while unauthoritative
        }

        // 3. LOST SNAPSHOT POLICY:
        // - Do NOT call coordinator_.tick()
        // - Do NOT call dispatcher_.update() (prevents target ingestion & delivery)
        // - Do NOT clear session intent or dispatcher staging
        // - Do NOT issue physical speed or incline commands
        // - Do NOT suspend session_: telemetry jitter must freeze data integration,
        //   but must never freeze the athlete's workout session state while the belt moves.
    }

    // 4. Publish latest incline verification command input and command execution status for cross-core consumers
    publishInclineVerificationCommandInput();
    publishCommandExecutionStatus();
    publishSnapshots();
}

bool ControlRuntime::armWorkout(const ExpandedWorkout* workout, uint32_t nowMs) {
    if (!initialized_ || workout == nullptr || workout->totalSteps == 0) {
        return false;
    }
    // Prevent workout loading or rebinding while session or ramp test is active
    if (session_.isActive() || rampTestTracker_.active()) {
        return false;
    }
    dispatcher_.clearForNewSession();
    const bool armed = session_.armWorkout(workout, nowMs);
    if (armed) {
        publishSnapshots();
    }
    return armed;
}

bool ControlRuntime::abortWorkout(uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    dispatcher_.clearWorkoutTargets();
    const bool aborted = session_.abortSession(nowMs);
    if (aborted) {
        publishSnapshots();
    }
    return aborted;
}

bool ControlRuntime::finalizeWorkout(uint32_t nowMs) {
    if (!initialized_) {
        return false;
    }
    dispatcher_.clearWorkoutTargets();
    const bool finalized = session_.finalizeSession(nowMs);
    if (finalized) {
        publishSnapshots();
    }
    return finalized;
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
        // Task must be removed either way to avoid a permanently stuck reference, but a
        // forced (non-clean) deletion means the task may still have been mid-execution and
        // could reference exitSem_ - do not delete it in that case.
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }
    taskRunning_ = false;
    stopRequested_ = false;

    if (cleanExit) {
        if (exitSem_ != nullptr) {
            vSemaphoreDelete(exitSem_);
            exitSem_ = nullptr;
        }
    } else if (diagnostics_ != nullptr) {
        // Forced termination: report it, and deliberately leak exitSem_ rather than risk a
        // use-after-free on a task that may not have actually stopped touching it yet.
        diagnostics_->reportFault(FaultCode::SystemWatchdogWarning, millis());
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
                if (!rampTestTracker_.active()) {
                    controller_.submitSpeedTarget(1.0f, cmdNowMs);
                }
                break;
            case ControlCommandType::Stop:
                rampTestTracker_.abort(cmdNowMs);
                inclineTracker_.abort(cmdNowMs);
                controller_.submitStop(cmdNowMs);
                break;
            case ControlCommandType::Pause:
                session_.suspend(cmdNowMs);
                break;
            case ControlCommandType::Resume:
                if (!session_.isActive() && !rampTestTracker_.active()) {
                    controller_.submitSpeedTarget(1.0f, cmdNowMs);
                }
                break;
            case ControlCommandType::SetSpeed: {
                if (rampTestTracker_.active()) {
                    break;
                }
                TargetContext ctx;
                if (session_.isActive()) {
                    ctx.origin = TargetOrigin::SessionManualAdjustment;
                    ctx.sessionGeneration = session_.getSessionGeneration();
                    ctx.stepIndex = session_.getCurrentStepIndex();
                } else {
                    ctx.origin = TargetOrigin::StandaloneManual;
                }
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stageSpeedTarget(cmd.data.target.speedKmh, ctx);
                session_.reportWorkSpeedAdjustment(cmd.data.target.speedKmh);
                break;
            }
            case ControlCommandType::SetIncline: {
                TargetContext ctx;
                if (session_.isActive()) {
                    ctx.origin = TargetOrigin::SessionManualAdjustment;
                    ctx.sessionGeneration = session_.getSessionGeneration();
                    ctx.stepIndex = session_.getCurrentStepIndex();
                } else {
                    ctx.origin = TargetOrigin::StandaloneManual;
                }
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stageInclineTarget(cmd.data.target.inclinePct, ctx);
                break;
            }
            case ControlCommandType::StepSpeed: {
                if (rampTestTracker_.active()) {
                    break;
                }
                const float acceptedTarget = controller_.getSnapshot().acceptedPhysicalSpeedTargetKmh;
                // Anchor on the actual, observed belt speed whenever the accepted target isn't yet a
                // confirmed, sane baseline (e.g. 0.0 right after startup while the belt may already be
                // moving) - never invent a target relative to a value we haven't actually confirmed.
                const float observedSpd = (orchestrator_ != nullptr) ? orchestrator_->getSnapshot().speed.speedKmh : 0.0f;
                // Belt moving threshold from WorkoutSessionConfig
                const float beltMovingThreshold = session_.getConfig().beltMovingThresholdKmh;
                const bool usedObserved = (acceptedTarget <= beltMovingThreshold);
                const float currentSpd = usedObserved ? observedSpd : acceptedTarget;

                DiagnosticsLog::instance().addEntryf(
                    "[StepSpeed] delta=%.2f: accepted=%.2f, observed=%.2f, threshold=%.2f -> baseline=%.2f (%s)",
                    cmd.data.stepSpeed.deltaSpeedKmh, acceptedTarget, observedSpd, beltMovingThreshold, currentSpd,
                    usedObserved ? "FALLBACK to observedSpd" : "used acceptedTarget");
                Serial.printf(
                    "[StepSpeed] delta=%.2f: accepted=%.2f, observed=%.2f, threshold=%.2f -> baseline=%.2f (%s)\n",
                    cmd.data.stepSpeed.deltaSpeedKmh, acceptedTarget, observedSpd, beltMovingThreshold, currentSpd,
                    usedObserved ? "FALLBACK to observedSpd" : "used acceptedTarget");
                TargetContext ctx;
                if (session_.isActive()) {
                    ctx.origin = TargetOrigin::SessionManualAdjustment;
                    ctx.sessionGeneration = session_.getSessionGeneration();
                    ctx.stepIndex = session_.getCurrentStepIndex();
                } else {
                    ctx.origin = TargetOrigin::StandaloneManual;
                }
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stepSpeedTarget(cmd.data.stepSpeed.deltaSpeedKmh, currentSpd, ctx);
                session_.reportWorkSpeedAdjustment(currentSpd + cmd.data.stepSpeed.deltaSpeedKmh);
                break;
            }
            case ControlCommandType::StepIncline: {
                const float currentInc = controller_.getSnapshot().acceptedInclineTargetPct;
                TargetContext ctx;
                if (session_.isActive()) {
                    ctx.origin = TargetOrigin::SessionManualAdjustment;
                    ctx.sessionGeneration = session_.getSessionGeneration();
                    ctx.stepIndex = session_.getCurrentStepIndex();
                } else {
                    ctx.origin = TargetOrigin::StandaloneManual;
                }
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stepInclineTarget(cmd.data.stepIncline.deltaInclinePct, currentInc, ctx);
                break;
            }
            case ControlCommandType::ArmWorkout: {
                if (session_.isActive() || rampTestTracker_.active()) {
                    Serial.println("[ControlRuntime] Cannot arm workout: session or ramp test already active");
                    break;
                }
                const WorkoutDefinition* def = SettingsService::instance().findWorkout(
                    cmd.data.arm.userId, cmd.data.arm.workoutId);
                if (def != nullptr && workoutEngine_.loadWorkout(*def)) {
                    dispatcher_.clearForNewSession();
                    session_.armWorkout(&workoutEngine_.getExpandedWorkout(), cmdNowMs, cmd.data.arm.userId);
                    Serial.printf("[ControlRuntime] Armed workout id=%u for user=%u\n", cmd.data.arm.workoutId, cmd.data.arm.userId);
                } else {
                    Serial.printf("[ControlRuntime] FAILED to arm workout id=%u for user=%u (def=%p)\n", cmd.data.arm.workoutId, cmd.data.arm.userId, def);
                }
                break;
            }
            case ControlCommandType::CancelWorkout:
                dispatcher_.clearWorkoutTargets();
                session_.abortSession(cmdNowMs);
                break;
            case ControlCommandType::FinalizeWorkout:
                dispatcher_.clearWorkoutTargets();
                session_.finalizeSession(cmdNowMs);
                break;
            case ControlCommandType::CutDrag:
                session_.cutDrag(cmdNowMs);
                break;
            case ControlCommandType::SkipToNextDrag:
                session_.skipToNextDrag(cmdNowMs);
                break;
            case ControlCommandType::ExtendRest:
                session_.extendRest();
                break;
            case ControlCommandType::AcceptSpeedShift:
                session_.acceptSpeedAdjustmentShift();
                break;
            case ControlCommandType::RejectSpeedShift:
                session_.rejectSpeedAdjustmentShift();
                break;
            case ControlCommandType::StartRampCalibrationTest:
                if (session_.isActive()) {
                    Serial.println("[ControlRuntime] Cannot start ramp test: session already active");
                    break;
                }
                if (rampTestTracker_.active()) {
                    Serial.println("[ControlRuntime] Cannot start ramp test: ramp test already active");
                    break;
                }
                rampTestTracker_.beginTest(
                    cmd.data.rampTest.startSpeedKmh,
                    cmd.data.rampTest.targetSpeedKmh,
                    cmdNowMs
                );
                break;
            case ControlCommandType::SetGuiMode:
                session_.setDesiredGuiMode(cmd.data.guiMode.userId, cmd.data.guiMode.isManual, cmdNowMs);
                break;
            case ControlCommandType::SelectUser: {
                const uint8_t selectedId = cmd.data.selectUser.userId;
                session_.selectUser(selectedId);
                publishSnapshots();

                if (orchestrator_ != nullptr) {
                    HeartRateClient* hrClient = orchestrator_->getHeartRateClient();
                    if (hrClient != nullptr) {
                        const auto* settings = SettingsService::instance().getActiveSettings();
                        if (settings != nullptr) {
                            for (size_t i = 0; i < MAX_USERS; ++i) {
                                if (settings->users[i].id == selectedId) {
                                    BleConfig bleCfg = orchestrator_->getBleConfig();
                                    const char* mac = settings->users[i].preferredHrMac;
                                    strncpy(bleCfg.preferredHrMac, mac, sizeof(bleCfg.preferredHrMac) - 1);
                                    bleCfg.preferredHrMac[sizeof(bleCfg.preferredHrMac) - 1] = '\0';
                                    bleCfg.autoConnectHr = (mac[0] != '\0');
                                    orchestrator_->updateBleConfig(bleCfg);
                                    break;
                                }
                            }
                        }
                    }
                }
                break;
            }
            case ControlCommandType::StartInclineHoming: {
                if (session_.isActive()) {
                    Serial.println("[ControlRuntime] Cannot start incline homing: session already active");
                    break;
                }
                if (rampTestTracker_.active()) {
                    Serial.println("[ControlRuntime] Cannot start incline homing: ramp test already active");
                    break;
                }
                const auto phase = inclineTracker_.phase();
                if (phase != InclineCommissioningPhase::Idle &&
                    phase != InclineCommissioningPhase::Aborted &&
                    phase != InclineCommissioningPhase::TimedOut &&
                    phase != InclineCommissioningPhase::Failed &&
                    phase != InclineCommissioningPhase::Complete) {
                    Serial.println("[ControlRuntime] Cannot start incline homing: incline commissioning already active");
                    break;
                }
                inclineTracker_.beginHoming(cmdNowMs);
                TargetContext ctx;
                ctx.origin = TargetOrigin::Commissioning;
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stageInclineTarget(0.0f, ctx);
                break;
            }
            case ControlCommandType::StartInclineMeasurePoint: {
                if (session_.isActive()) {
                    Serial.println("[ControlRuntime] Cannot start incline measurement: session already active");
                    break;
                }
                if (rampTestTracker_.active()) {
                    Serial.println("[ControlRuntime] Cannot start incline measurement: ramp test already active");
                    break;
                }
                if (inclineTracker_.pointCount() == 0) {
                    Serial.println("[ControlRuntime] Cannot start incline measurement: homing not confirmed");
                    break;
                }
                const auto phase = inclineTracker_.phase();
                if (phase != InclineCommissioningPhase::Idle &&
                    phase != InclineCommissioningPhase::Complete) {
                    Serial.println("[ControlRuntime] Cannot start incline measurement: incline commissioning already active");
                    break;
                }
                const float cmdPct = cmd.data.inclineMeasurePoint.commandedPct;
                const auto dir = static_cast<InclineDirection>(cmd.data.inclineMeasurePoint.expectedDirection);
                inclineTracker_.beginMeasurePoint(cmdPct, dir, cmdNowMs);
                TargetContext ctx;
                ctx.origin = TargetOrigin::Commissioning;
                ctx.timestampMs = cmdNowMs;
                dispatcher_.stageInclineTarget(cmdPct, ctx);
                break;
            }
            case ControlCommandType::SaveInclineCalibration: {
                InclineConfig candidate = inclineTracker_.buildCandidateConfig();
                if (candidate.commandMapValid) {
                    SettingsService::instance().saveInclineConfig(candidate);
                }
                break;
            }
            case ControlCommandType::AbortInclineCommissioning: {
                inclineTracker_.abort(cmdNowMs);
                break;
            }
            case ControlCommandType::None:
            default:
                break;
        }
        processed++;
    }
}

WorkoutSessionSnapshot ControlRuntime::getSessionSnapshot() const {
    WorkoutSessionSnapshot snap{};
    portENTER_CRITICAL(&snapshotMux_);
    snap = publishedSessionSnapshot_;
    portEXIT_CRITICAL(&snapshotMux_);
    return snap;
}

TreadmillControllerSnapshot ControlRuntime::getControllerSnapshot() const {
    TreadmillControllerSnapshot snap{};
    portENTER_CRITICAL(&snapshotMux_);
    snap = publishedControllerSnapshot_;
    portEXIT_CRITICAL(&snapshotMux_);
    return snap;
}

void ControlRuntime::publishSnapshots() {
    const WorkoutSessionSnapshot sessSnap = session_.getSnapshot();
    const TreadmillControllerSnapshot ctrlSnap = controller_.getSnapshot();
    portENTER_CRITICAL(&snapshotMux_);
    publishedSessionSnapshot_ = sessSnap;
    publishedControllerSnapshot_ = ctrlSnap;
    portEXIT_CRITICAL(&snapshotMux_);
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

InclineVerificationCommandInput ControlRuntime::getInclineVerificationCommandInput() const {
    InclineVerificationCommandInput input{};
    portENTER_CRITICAL(&inclineCommandContextMux_);
    input = publishedInclineCommandContext_;
    portEXIT_CRITICAL(&inclineCommandContextMux_);
    return input;
}

void ControlRuntime::publishInclineVerificationCommandInput() {
    const TreadmillControllerSnapshot ctrlSnap = controller_.getSnapshot();
    InclineVerificationCommandInput input{};
    input.targetInclinePct = ctrlSnap.acceptedInclineTargetPct;
    input.targetInclineValid = ctrlSnap.inclineTargetValid;
    input.commandSequence = ctrlSnap.acceptedInclineTargetSequence;
    input.commandTimestampMs = ctrlSnap.acceptedInclineTargetTimestampMs;
    input.commandTimestampValid = ctrlSnap.inclineTargetValid && ctrlSnap.acceptedInclineTargetTimestampMs != 0;

    portENTER_CRITICAL(&inclineCommandContextMux_);
    publishedInclineCommandContext_ = input;
    portEXIT_CRITICAL(&inclineCommandContextMux_);
}

void ControlRuntime::publishCommandExecutionStatus() {
    const TreadmillControllerSnapshot ctrlSnap = controller_.getSnapshot();
    CommandExecutionStatus status{};
    if (ctrlSnap.interruptedRequestValid && ctrlSnap.interruptedRequestId != 0 &&
        ctrlSnap.interruptedRequestId != lastReportedAbortedRequestId_) {
        status.lastCommandAborted = true;
        status.abortedRequestId = ctrlSnap.interruptedRequestId;
        lastReportedAbortedRequestId_ = ctrlSnap.interruptedRequestId;
    }
    portENTER_CRITICAL(&commandExecutionStatusMux_);
    publishedCommandExecutionStatus_ = status;
    portEXIT_CRITICAL(&commandExecutionStatusMux_);
}

CommandExecutionStatus ControlRuntime::getCommandExecutionStatus() const {
    CommandExecutionStatus status{};
    portENTER_CRITICAL(&commandExecutionStatusMux_);
    status = publishedCommandExecutionStatus_;
    portEXIT_CRITICAL(&commandExecutionStatusMux_);
    return status;
}

void ControlRuntime::requestInclineCommissioningAction(bool confirmHomed, bool zeroImu) {
    if (orchestrator_ != nullptr) {
        orchestrator_->requestInclineCommissioningAction(confirmHomed, zeroImu);
    }
}

float ControlRuntime::getMaxAchievableSpeedKmh() const {
    return calibration_.getMaxAchievableSpeedKmh();
}

bool ControlRuntime::isMaxAchievableSpeedVerified() const {
    return calibration_.isMaxAchievableSpeedVerified();
}

const char* ControlRuntime::version() {
    return "ControlRuntime/1.0.0";
}

} // namespace stridecontrol

