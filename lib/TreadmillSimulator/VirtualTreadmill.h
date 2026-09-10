#pragma once

#include <cstdint>
#include <cstddef>

#include "../InclineSensor/InclineSensorTypes.h"

namespace stridecontrol {

// =============================================================================
// SIMULATION TIME CONTRACT
// =============================================================================

struct SimulationTick {
    uint64_t tickIndex = 0;     // Monotonic step index, increments by exactly 1
    uint64_t scenarioTimeUs = 0;// Monotonic scenario timestamp in microseconds
    uint32_t deltaUs = 0;       // Scenario step elapsed time in microseconds
    uint32_t timestampMs = 0;   // Monotonic timestamp with rollover-safe math (backward compat)
    uint32_t deltaMs = 0;       // Scenario step elapsed time (nominal 20 ms @ 50 Hz) (backward compat)

    SimulationTick() = default;
    SimulationTick(uint64_t idx, uint32_t tsMs, uint32_t dtMs)
        : tickIndex(idx),
          scenarioTimeUs(static_cast<uint64_t>(tsMs) * 1000ULL),
          deltaUs(dtMs * 1000UL),
          timestampMs(tsMs),
          deltaMs(dtMs) {}
    SimulationTick(uint64_t idx, uint64_t scUs, uint32_t dtUs)
        : tickIndex(idx),
          scenarioTimeUs(scUs),
          deltaUs(dtUs),
          timestampMs(static_cast<uint32_t>(scUs / 1000ULL)),
          deltaMs(dtUs / 1000UL) {}
};

// =============================================================================
// PASSIVE OUTPUT CONTRACTS (Hardware-Facing DTOs)
// =============================================================================

static constexpr size_t kMaxTachoEdgesPerTick = 3;

struct TachoOutput {
    uint64_t pulsesThisTick = 0;
    uint64_t totalPulses = 0;
    double fractionalRemainder = 0.0;
    float instantaneousSpeedKmh = 0.0f;
    bool signalValid = true;
    uint32_t edgeTimestampUs[kMaxTachoEdgesPerTick]{};
    uint8_t edgeCount = 0;
    uint32_t droppedEdgesCount = 0;
};

constexpr InclineDirection InclineDirectionNone = InclineDirection::Unknown;

struct InclineFeedbackOutput {
    InclineDirection direction = InclineDirection::Unknown;
    uint64_t pulsesThisTick = 0;
    int64_t totalSignedPulses = 0;
    uint64_t totalPulses = 0;
    double fractionalRemainder = 0.0;
    float physicalInclinePct = 0.0f;
    bool moving = false;
    bool signalValid = true;
};

enum class VirtualButtonId : uint8_t {
    None = 0,
    QuickStart,
    Stop,
    SpeedPlus,
    SpeedMinus,
    InclinePlus,
    InclineMinus,
    Enter,
    Clear
};

enum class VirtualButtonState : uint8_t {
    Released = 0,
    Pressed  = 1
};

struct VirtualButtonEvent {
    VirtualButtonId button = VirtualButtonId::None;
    VirtualButtonState state = VirtualButtonState::Released;
    uint32_t timestampMs = 0;

    VirtualButtonEvent() = default;
    VirtualButtonEvent(VirtualButtonId b, VirtualButtonState s, uint32_t ts)
        : button(b), state(s), timestampMs(ts) {}
};

struct ConsoleOutput {
    static constexpr size_t kMaxEvents = 8;
    VirtualButtonEvent events[kMaxEvents]{};
    uint8_t eventCount = 0;
    bool eStopEngaged = false;
};

enum class VirtualRunnerLocation : uint8_t {
    Absent      = 0,
    OnBelt      = 1,
    OnSideRails = 2
};

struct BiometricOutput {
    VirtualRunnerLocation runnerLocation = VirtualRunnerLocation::Absent;
    uint16_t cadenceSpm = 0;
    uint8_t heartRateBpm = 0;
    bool signalValid = true;
};

// =============================================================================
// FAULT INJECTION FLAGS
// =============================================================================

enum class VirtualTreadmillFault : uint16_t {
    None                    = 0,
    TachoLostSignal         = 1 << 0,
    InclineMotorStall       = 1 << 1,
    InclinePulseLost        = 1 << 2,
    ImuSensorFreeze         = 1 << 3,
    HeartRateDropout        = 1 << 4,
    EStopStuckActive        = 1 << 5
};

// =============================================================================
// CONFIGURATION CONTRACT
// =============================================================================

struct VirtualTreadmillConfig {
    // Physical limits
    float maxSpeedKmh = 22.0f;
    float minSpeedKmh = 0.5f;

    // Dynamics (Provisional defaults unless calibrated)
    float accelerationKmhPerSec = 1.0f;         // Provisional default: 1.0 km/h/s
    float normalDecelerationKmhPerSec = 1.2f;   // Provisional default: 1.2 km/h/s
    float eStopDecelerationKmhPerSec = 3.5f;    // Provisional default: 3.5 km/h/s

    // Incline limits & dynamics (Reused from InclineSensorConfig.h: 0.0% to 15.0%)
    float inclineMinPct = 0.0f;
    float inclineMaxPct = 15.0f;
    float inclineTransitPctPerSec = 0.35f;      // Provisional default: 0.35 %/s

    // Pulse calibrations (Provisional defaults)
    double tachoPulsesPerKm = 3600.0 / 1.1148;  // T610 default calibration: ~3229.28 pulses/km (1.1148 km/h/Hz)
    double inclinePulsesPerPct = 3086.0;        // T610 default calibration: 3086 pulses/%
};

// =============================================================================
// VIRTUAL TREADMILL CORE ENGINE
// =============================================================================

class VirtualTreadmill {
public:
    explicit VirtualTreadmill(const VirtualTreadmillConfig& config = VirtualTreadmillConfig{});
    ~VirtualTreadmill() = default;

    VirtualTreadmill(const VirtualTreadmill&) = delete;
    VirtualTreadmill& operator=(const VirtualTreadmill&) = delete;

    // Lifecycle and reset
    void resetModel(uint32_t initialTimeMs = 0);
    void resetModelUs(uint64_t initialTimeUs = 0);

    // Monotonic simulation step
    bool tick(const SimulationTick& tick);

    // Stimulus and command inputs
    void setTargetSpeedKmh(float speedKmh);
    void setTargetInclinePct(float inclinePct);
    void setEmergencyStop(bool active);
    void setRunnerLocation(VirtualRunnerLocation location);
    void setCadenceSpm(uint16_t spm);
    void setHeartRateBpm(uint8_t bpm);
    bool enqueueButtonEvent(VirtualButtonId button, VirtualButtonState state, uint32_t timestampMs);

    // Fault injection
    void setFault(VirtualTreadmillFault fault, bool active);
    bool isFaultActive(VirtualTreadmillFault fault) const;
    void clearAllFaults();

    // Passive Output DTO Accessors (Hardware-facing state)
    const TachoOutput& getTachoOutput() const { return tachoOutput_; }
    const InclineFeedbackOutput& getInclineOutput() const { return inclineOutput_; }
    const ConsoleOutput& getConsoleOutput() const { return consoleOutput_; }
    const BiometricOutput& getBiometricOutput() const { return biometricOutput_; }

    // Physical state inspectors
    float getActualSpeedKmh() const { return actualSpeedKmh_; }
    float getTargetSpeedKmh() const { return targetSpeedKmh_; }
    float getActualInclinePct() const { return actualInclinePct_; }
    float getTargetInclinePct() const { return targetInclinePct_; }
    double getOdometerKm() const { return odometerKm_; }
    VirtualRunnerLocation getRunnerLocation() const { return runnerLocation_; }
    uint16_t getCadenceSpm() const { return cadenceSpm_; }
    bool isEStopActive() const { return eStopActive_; }
    bool isBeltMoving() const { return actualSpeedKmh_ > 0.001f; }
    uint64_t getTickCount() const { return lastTickIndex_; }
    uint32_t getLastTimestampMs() const { return lastTimestampMs_; }
    uint64_t getLastScenarioTimeUs() const { return lastScenarioTimeUs_; }
    uint64_t getTotalPhysicalPulses() const { return totalTachoPulses_; }

    const VirtualTreadmillConfig& getConfig() const { return config_; }

    static const char* version();

private:
    void updateSpeed(uint32_t deltaMs);
    void updateIncline(uint32_t deltaMs);
    void updateTachoPulses(float prevSpeed, float currSpeed, uint32_t deltaUs, uint64_t scenarioTimeUs);
    void updateInclinePulses(float prevIncline, float currIncline);

    VirtualTreadmillConfig config_;

    // Timekeeping & sequencing
    bool initialized_ = false;
    uint64_t lastTickIndex_ = 0;
    uint64_t lastScenarioTimeUs_ = 0;
    uint32_t lastTimestampMs_ = 0;

    // Physical state
    float actualSpeedKmh_ = 0.0f;
    float targetSpeedKmh_ = 0.0f;
    float actualInclinePct_ = 0.0f;
    float targetInclinePct_ = 0.0f;
    double odometerKm_ = 0.0;

    // Pulse synthesis fractional accumulators
    double fractionalTachoPulses_ = 0.0;
    uint64_t totalTachoPulses_ = 0;
    double fractionalInclinePulses_ = 0.0;
    uint64_t totalInclinePulses_ = 0;
    int64_t totalSignedInclinePulses_ = 0;

    // Hardware status & biometrics
    bool eStopActive_ = false;
    VirtualRunnerLocation runnerLocation_ = VirtualRunnerLocation::Absent;
    uint16_t cadenceSpm_ = 0;
    uint8_t heartRateBpm_ = 0;
    uint16_t activeFaultsMask_ = 0;

    // Button event queue
    static constexpr size_t kEventQueueCapacity = 8;
    VirtualButtonEvent eventQueue_[kEventQueueCapacity]{};
    uint8_t eventQueueCount_ = 0;

    // Output DTOs
    TachoOutput tachoOutput_{};
    InclineFeedbackOutput inclineOutput_{};
    ConsoleOutput consoleOutput_{};
    BiometricOutput biometricOutput_{};
};

} // namespace stridecontrol\n