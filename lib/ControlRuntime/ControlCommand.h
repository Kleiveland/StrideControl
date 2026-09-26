#pragma once

#include <cstdint>
#include <cstddef>
#include "../InclineCommissioningTracker/InclineCommissioningTracker.h"
#include "../SpeedCalibration/SpeedCalibrationTypes.h"
#include "../SettingsService/SettingsServiceTypes.h"
#include "../SpeedLearningTracker/SpeedLearningTracker.h"

namespace stridecontrol {

enum class ControlCommandType : uint8_t {
    None = 0,
    QuickStart,
    Stop,
    Pause,
    Resume,
    SetSpeed,
    SetIncline,
    StepSpeed,
    StepIncline,
    ArmWorkout,
    CancelWorkout,
    FinalizeWorkout,
    CutDrag,
    SkipToNextDrag,
    ExtendRest,
    AcceptSpeedShift,
    RejectSpeedShift,
    StartRampCalibrationTest,
    SetGuiMode,
    SelectUser,
    StartInclineHoming,
    StartInclineMeasurePoint,
    SaveInclineCalibration,
    AbortInclineCommissioning
};

struct ControlCommand {
    ControlCommandType type{ControlCommandType::None};
    uint32_t timestampMs{0};
    union {
        struct {
            float speedKmh;
            float inclinePct;
        } target;
        struct {
            float deltaSpeedKmh;
        } stepSpeed;
        struct {
            float deltaInclinePct;
        } stepIncline;
        struct {
            uint16_t workoutId;
            uint8_t userId;
        } arm;
        struct {
            uint8_t userId;
            bool isManual;
        } guiMode;
        struct {
            uint8_t userId;
        } selectUser;
        struct {
            float startSpeedKmh;
            float targetSpeedKmh;
        } rampTest;
        struct {
            float commandedPct;
            uint8_t expectedDirection; // cast to/from InclineDirection
        } inclineMeasurePoint;
    } data{};
};

class IControlCommandStager {
public:
    virtual ~IControlCommandStager() = default;
    virtual bool stageCommand(const ControlCommand& cmd) = 0;
    virtual bool isRampTestActive() const { return false; }
    virtual bool isRampTestComplete() const { return false; }
    virtual bool didRampTestTimeOut() const { return false; }
    virtual uint32_t getRampTestDeadTimeMs() const { return 0; }
    virtual uint32_t getRampTestTotalMs() const { return 0; }
    virtual float getMaxAchievableSpeedKmh() const { return 25.0f; }
    virtual bool isMaxAchievableSpeedVerified() const { return false; }
    virtual InclineCommissioningPhase getInclineCommissioningPhase() const { return InclineCommissioningPhase::Idle; }
    virtual uint8_t getInclineCommissioningPointCount() const { return 0; }
    virtual bool didInclineCommissioningTimeOut() const { return false; }
    virtual void onSpeedConfigUpdated(const SpeedConfig& config) {}
    virtual SpeedCalibrationResult calculateSpeedCommand(float physicalSpeedKmh) const {
        return SpeedCalibrationResult(physicalSpeedKmh, physicalSpeedKmh, SpeedCalibrationResultStatus::IdentityFallback);
    }
    virtual uint8_t getSpeedAdaptationLog(SpeedAdaptationLogEntry* outEntries, uint8_t maxEntries) const { return 0; }
};

} // namespace stridecontrol
