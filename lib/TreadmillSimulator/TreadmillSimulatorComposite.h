#pragma once

#include <cstdint>
#include <cstddef>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "../WorkoutDispatcher/IWorkoutTargetSink.h"
#include "../SpeedSensor/SpeedSensor.h"
#include "../InclineSensor/InclineSensor.h"
#include "../ConsoleInterface/ConsoleInterface.h"
#include "../ImuInterface/ImuInterface.h"
#include "VirtualTreadmill.h"
#include "VirtualSpeedSensorAdapter.h"
#include "VirtualInclineAdapter.h"
#include "VirtualConsoleAdapter.h"
#include "VirtualRunnerAdapter.h"

namespace stridecontrol {

struct StagedDiscreteEvent {
    enum class Type : uint8_t {
        None = 0,
        QuickStart,
        Stop,
        EmergencyStop,
        SpeedPlus,
        SpeedMinus,
        InclinePlus,
        InclineMinus,
        SetSpeed,
        SetIncline
    };
    Type type = Type::None;
    float paramValue = 0.0f;
    uint32_t timestampMs = 0;
};

struct StagedContinuousRunnerState {
    VirtualRunnerMode mode = VirtualRunnerMode::RunningOnBelt;
    uint16_t cadenceSpm = 180;
    float impactMagnitudeG = 0.35f;
    bool imuSignalValid = true;
    bool dirty = false;
};

class StagedStimulusMailbox {
public:
    static constexpr size_t kEventCapacity = 16;

    bool enqueueEvent(const StagedDiscreteEvent& event);
    void updateRunnerState(const StagedContinuousRunnerState& state);
    size_t drainEvents(StagedDiscreteEvent* outBuffer, size_t maxEvents,
                       StagedContinuousRunnerState& outRunner, bool& outRunnerDirty);
    uint32_t getDroppedEventsCount() const;

private:
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    StagedDiscreteEvent queue_[kEventCapacity]{};
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    uint32_t droppedEventsCount_ = 0;
    StagedContinuousRunnerState runnerState_{};
};

/**
 * @brief Passive structural aggregator integrating VirtualTreadmill and its 4 verified
 *        boundary adapters into production sensor drivers.
 */
class TreadmillSimulatorComposite : public IWorkoutTargetSink {
public:
    TreadmillSimulatorComposite(
        SpeedSensor& speedSensor,
        InclineSensor& inclineSensor,
        ImuInterface& imu,
        const VirtualTreadmillConfig& config = VirtualTreadmillConfig{}
    );
    virtual ~TreadmillSimulatorComposite() = default;

    TreadmillSimulatorComposite(const TreadmillSimulatorComposite&) = delete;
    TreadmillSimulatorComposite& operator=(const TreadmillSimulatorComposite&) = delete;

    void tick(const SimulationTick& simTick, float deckAngleDeg = 0.0f);

    // Stimulus staging
    bool stageQuickStart(uint32_t nowMs = 0);
    bool stageStop(uint32_t nowMs = 0);
    bool stageEmergencyStop(uint32_t nowMs = 0);
    bool stageSpeedTarget(float speedKmh, uint32_t nowMs = 0);
    bool stageInclineTarget(float inclinePct, uint32_t nowMs = 0);
    bool stageSpeedStep(bool positive, uint32_t nowMs = 0);
    bool stageInclineStep(bool positive, uint32_t nowMs = 0);
    void stageRunner(VirtualRunnerMode mode, uint16_t cadenceSpm = 180, float magnitudeG = 0.35f, bool valid = true);

    uint32_t getDroppedEventsCount() const;

    // IWorkoutTargetSink implementation
    bool submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) override;
    bool submitInclineTarget(float targetInclinePct, uint32_t nowMs) override;
    bool submitStop(uint32_t nowMs);
    bool isBusy() const override;
    bool isReady() const override;

    // Inspectors
    const VirtualTreadmill& getVirtualTreadmill() const { return treadmill_; }
    VirtualTreadmill& getVirtualTreadmill() { return treadmill_; }
    const VirtualRunnerAdapter& getRunnerAdapter() const { return runnerAdapter_; }
    VirtualRunnerAdapter& getRunnerAdapter() { return runnerAdapter_; }

    static const char* version();

private:
    void applyDiscreteEvent(const StagedDiscreteEvent& ev);

    VirtualTreadmill treadmill_;
    VirtualSpeedSensorAdapter speedAdapter_;
    VirtualInclineAdapter inclineAdapter_;
    VirtualConsoleAdapter consoleAdapter_;
    VirtualRunnerAdapter runnerAdapter_;

    StagedStimulusMailbox mailbox_;
};

} // namespace stridecontrol

