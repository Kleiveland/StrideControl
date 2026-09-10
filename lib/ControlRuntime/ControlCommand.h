#pragma once

#include <cstdint>
#include <cstddef>

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
    CancelWorkout
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
    } data{};
};

class IControlCommandStager {
public:
    virtual ~IControlCommandStager() = default;
    virtual bool stageCommand(const ControlCommand& cmd) = 0;
};

} // namespace stridecontrol
