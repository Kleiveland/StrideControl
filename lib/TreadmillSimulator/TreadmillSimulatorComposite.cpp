#include "TreadmillSimulatorComposite.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace stridecontrol {

// =============================================================================
// StagedStimulusMailbox Implementation
// =============================================================================

bool StagedStimulusMailbox::enqueueEvent(const StagedDiscreteEvent& event) {
    portENTER_CRITICAL(&mux_);
    if (count_ >= kEventCapacity) {
        droppedEventsCount_++;
        portEXIT_CRITICAL(&mux_);
        return false;
    }
    queue_[head_] = event;
    head_ = (head_ + 1) % kEventCapacity;
    count_++;
    portEXIT_CRITICAL(&mux_);
    return true;
}

void StagedStimulusMailbox::updateRunnerState(const StagedContinuousRunnerState& state) {
    portENTER_CRITICAL(&mux_);
    runnerState_ = state;
    runnerState_.dirty = true;
    portEXIT_CRITICAL(&mux_);
}

size_t StagedStimulusMailbox::drainEvents(StagedDiscreteEvent* outBuffer, size_t maxEvents,
                                         StagedContinuousRunnerState& outRunner, bool& outRunnerDirty) {
    portENTER_CRITICAL(&mux_);
    size_t n = 0;
    while (count_ > 0 && n < maxEvents) {
        outBuffer[n++] = queue_[tail_];
        tail_ = (tail_ + 1) % kEventCapacity;
        count_--;
    }
    outRunnerDirty = runnerState_.dirty;
    if (outRunnerDirty) {
        outRunner = runnerState_;
        runnerState_.dirty = false;
    }
    portEXIT_CRITICAL(&mux_);
    return n;
}

uint32_t StagedStimulusMailbox::getDroppedEventsCount() const {
    portENTER_CRITICAL(&mux_);
    uint32_t d = droppedEventsCount_;
    portEXIT_CRITICAL(&mux_);
    return d;
}

// =============================================================================
// TreadmillSimulatorComposite Implementation
// =============================================================================

TreadmillSimulatorComposite::TreadmillSimulatorComposite(
    SpeedSensor& speedSensor,
    InclineSensor& inclineSensor,
    ImuInterface& imu,
    const VirtualTreadmillConfig& config
)
    : treadmill_(config),
      speedAdapter_(speedSensor),
      inclineAdapter_(inclineSensor),
      consoleAdapter_(treadmill_),
      runnerAdapter_(imu) {}

void TreadmillSimulatorComposite::tick(const SimulationTick& simTick, float deckAngleDeg) {
    // 1. Drain staged stimuli under minimal critical section (copy-then-process)
    StagedDiscreteEvent events[StagedStimulusMailbox::kEventCapacity];
    StagedContinuousRunnerState runnerState{};
    bool runnerDirty = false;
    size_t eventCount = mailbox_.drainEvents(events, StagedStimulusMailbox::kEventCapacity, runnerState, runnerDirty);

    // 2. Process discrete events outside critical section
    for (size_t i = 0; i < eventCount; ++i) {
        applyDiscreteEvent(events[i]);
    }

    // 3. Process continuous runner state outside critical section
    if (runnerDirty) {
        runnerAdapter_.setMode(runnerState.mode);
        runnerAdapter_.setCadenceSpm(runnerState.cadenceSpm);
        runnerAdapter_.setImpactMagnitudeG(runnerState.impactMagnitudeG);
        runnerAdapter_.setSignalValid(runnerState.imuSignalValid);

        if (runnerState.mode == VirtualRunnerMode::RunningOnBelt) {
            treadmill_.setRunnerLocation(VirtualRunnerLocation::OnBelt);
        } else if (runnerState.mode == VirtualRunnerMode::OnSideRails) {
            treadmill_.setRunnerLocation(VirtualRunnerLocation::OnSideRails);
        } else {
            treadmill_.setRunnerLocation(VirtualRunnerLocation::Absent);
        }
        treadmill_.setCadenceSpm(runnerState.cadenceSpm);
    }

    // 4. Advance physical authority
    treadmill_.tick(simTick);

    // 5. Forward passive outputs into sensor drivers
    speedAdapter_.processTacho(treadmill_.getTachoOutput());
    inclineAdapter_.processFeedback(treadmill_.getInclineOutput(), simTick.timestampMs);
    runnerAdapter_.processTick(simTick, deckAngleDeg);
}

void TreadmillSimulatorComposite::applyDiscreteEvent(const StagedDiscreteEvent& ev) {
    TreadmillCommand cmd{};
    switch (ev.type) {
        case StagedDiscreteEvent::Type::QuickStart:
            cmd.type = CommandType::PressButton;
            cmd.button = ButtonId::QuickStart;
            consoleAdapter_.onCommandIntent(cmd);
            break;

        case StagedDiscreteEvent::Type::Stop:
            cmd.type = CommandType::PressButton;
            cmd.button = ButtonId::Stop;
            consoleAdapter_.onCommandIntent(cmd);
            break;

        case StagedDiscreteEvent::Type::EmergencyStop:
            consoleAdapter_.onEmergencyStop(true);
            break;

        case StagedDiscreteEvent::Type::SpeedPlus:
            cmd.type = CommandType::PressSpeedPlus;
            consoleAdapter_.onCommandIntent(cmd);
            break;

        case StagedDiscreteEvent::Type::SpeedMinus:
            cmd.type = CommandType::PressSpeedMinus;
            consoleAdapter_.onCommandIntent(cmd);
            break;

        case StagedDiscreteEvent::Type::InclinePlus:
            cmd.type = CommandType::PressButton;
            cmd.button = ButtonId::InclinePlus;
            consoleAdapter_.onCommandIntent(cmd);
            break;

        case StagedDiscreteEvent::Type::InclineMinus:
            cmd.type = CommandType::PressButton;
            cmd.button = ButtonId::InclineMinus;
            consoleAdapter_.onCommandIntent(cmd);
            break;

        case StagedDiscreteEvent::Type::SetSpeed:
            cmd.type = CommandType::SetSpeed;
            cmd.value = ev.paramValue;
            consoleAdapter_.onCommandIntent(cmd);
            break;

        case StagedDiscreteEvent::Type::SetIncline:
            cmd.type = CommandType::SetIncline;
            cmd.value = ev.paramValue;
            consoleAdapter_.onCommandIntent(cmd);
            break;

        default:
            break;
    }
}

bool TreadmillSimulatorComposite::stageQuickStart(uint32_t nowMs) {
    StagedDiscreteEvent ev{};
    ev.type = StagedDiscreteEvent::Type::QuickStart;
    ev.timestampMs = nowMs;
    return mailbox_.enqueueEvent(ev);
}

bool TreadmillSimulatorComposite::stageStop(uint32_t nowMs) {
    StagedDiscreteEvent ev{};
    ev.type = StagedDiscreteEvent::Type::Stop;
    ev.timestampMs = nowMs;
    return mailbox_.enqueueEvent(ev);
}

bool TreadmillSimulatorComposite::stageEmergencyStop(uint32_t nowMs) {
    StagedDiscreteEvent ev{};
    ev.type = StagedDiscreteEvent::Type::EmergencyStop;
    ev.timestampMs = nowMs;
    return mailbox_.enqueueEvent(ev);
}

bool TreadmillSimulatorComposite::stageSpeedTarget(float speedKmh, uint32_t nowMs) {
    StagedDiscreteEvent ev{};
    ev.type = StagedDiscreteEvent::Type::SetSpeed;
    ev.paramValue = speedKmh;
    ev.timestampMs = nowMs;
    return mailbox_.enqueueEvent(ev);
}

bool TreadmillSimulatorComposite::stageInclineTarget(float inclinePct, uint32_t nowMs) {
    StagedDiscreteEvent ev{};
    ev.type = StagedDiscreteEvent::Type::SetIncline;
    ev.paramValue = inclinePct;
    ev.timestampMs = nowMs;
    return mailbox_.enqueueEvent(ev);
}

bool TreadmillSimulatorComposite::stageSpeedStep(bool positive, uint32_t nowMs) {
    StagedDiscreteEvent ev{};
    ev.type = positive ? StagedDiscreteEvent::Type::SpeedPlus : StagedDiscreteEvent::Type::SpeedMinus;
    ev.timestampMs = nowMs;
    return mailbox_.enqueueEvent(ev);
}

bool TreadmillSimulatorComposite::stageInclineStep(bool positive, uint32_t nowMs) {
    StagedDiscreteEvent ev{};
    ev.type = positive ? StagedDiscreteEvent::Type::InclinePlus : StagedDiscreteEvent::Type::InclineMinus;
    ev.timestampMs = nowMs;
    return mailbox_.enqueueEvent(ev);
}

void TreadmillSimulatorComposite::stageRunner(VirtualRunnerMode mode, uint16_t cadenceSpm, float magnitudeG, bool valid) {
    StagedContinuousRunnerState st{};
    st.mode = mode;
    st.cadenceSpm = cadenceSpm;
    st.impactMagnitudeG = magnitudeG;
    st.imuSignalValid = valid;
    mailbox_.updateRunnerState(st);
}

uint32_t TreadmillSimulatorComposite::getDroppedEventsCount() const {
    return mailbox_.getDroppedEventsCount();
}

bool TreadmillSimulatorComposite::submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) {
    (void)nowMs;
    TreadmillCommand cmd{};
    cmd.type = CommandType::SetSpeed;
    cmd.value = targetSpeedKmh;
    return consoleAdapter_.onCommandIntent(cmd);
}

bool TreadmillSimulatorComposite::submitInclineTarget(float targetInclinePct, uint32_t nowMs) {
    (void)nowMs;
    TreadmillCommand cmd{};
    cmd.type = CommandType::SetIncline;
    cmd.value = targetInclinePct;
    return consoleAdapter_.onCommandIntent(cmd);
}

bool TreadmillSimulatorComposite::submitStop(uint32_t nowMs) {
    (void)nowMs;
    TreadmillCommand cmd{};
    cmd.type = CommandType::PressButton;
    cmd.button = ButtonId::Stop;
    return consoleAdapter_.onCommandIntent(cmd);
}

bool TreadmillSimulatorComposite::isBusy() const {
    return false;
}

bool TreadmillSimulatorComposite::isReady() const {
    return true;
}

const char* TreadmillSimulatorComposite::version() {
    return "TreadmillSimulatorComposite/1.0.0";
}

} // namespace stridecontrol

