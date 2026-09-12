#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include "../ConsoleInterface/ConsoleTypes.h"
#include "VirtualTreadmill.h"

namespace stridecontrol {

/**
 * @brief Behavioral adapter translating StrideControl console command intents
 *        into VirtualTreadmill physical dynamics according to T610 user behavior.
 * 
 * Captures command intents only (no electrical/matrix/pull-up simulation):
 * - QuickStart: Starts at 1.0 km/h, clears E-Stop latch
 * - SpeedPlus / SpeedMinus: +/- 0.1 km/h step, clamped [0.8, 22.0]
 * - InclinePlus / InclineMinus: +/- 0.5 % step, clamped [0.0, 15.0]
 * - SetSpeed / SetIncline: Direct target setting clamped to valid ranges
 * - Stop: Ramps speed target to 0.0 km/h (pause)
 * - EmergencyStop: Sets E-Stop latch and zeroes speed target
 */
class VirtualConsoleAdapter : public ICommandIntentSink {
public:
    explicit VirtualConsoleAdapter(VirtualTreadmill& treadmill)
        : treadmill_(treadmill) {}

    bool onCommandIntent(const TreadmillCommand& cmd) override {
        totalCommandsReceived_++;
        lastCommand_ = cmd;

        // If E-Stop is active, only QuickStart or E-Stop release can clear/override
        if (treadmill_.isEStopActive()) {
            if (cmd.type == CommandType::PressButton && cmd.button == ButtonId::QuickStart) {
                treadmill_.setEmergencyStop(false);
                treadmill_.setTargetSpeedKmh(1.0f);
                treadmill_.setTargetInclinePct(0.0f);
                resetResumeDefaults();
                consecutiveStopPresses_ = 0;
                quickStartCount_++;
                return true;
            }
            if (cmd.type == CommandType::PressButton && cmd.button == ButtonId::Stop) {
                stopCount_++;
                return true;
            }
            // All other commands rejected while E-Stop is engaged
            return false;
        }

        switch (cmd.type) {
            case CommandType::SetSpeed: {
                speedCommandsCount_++;
                float target = std::round(cmd.value * 10.0f) / 10.0f;
                if (target <= 0.0f) {
                    treadmill_.setTargetSpeedKmh(0.0f);
                } else {
                    target = std::max(0.8f, std::min(22.0f, target));
                    treadmill_.setTargetSpeedKmh(target);
                }
                return true;
            }

            case CommandType::SetIncline: {
                inclineCommandsCount_++;
                float target = std::round(cmd.value * 2.0f) / 2.0f;
                target = std::max(0.0f, std::min(15.0f, target));
                treadmill_.setTargetInclinePct(target);
                return true;
            }

            case CommandType::PressSpeedPlus: {
                speedCommandsCount_++;
                float current = treadmill_.getTargetSpeedKmh();
                float next = (current < 0.8f) ? 0.8f : (std::round((current + 0.1f) * 10.0f) / 10.0f);
                next = std::min(22.0f, next);
                treadmill_.setTargetSpeedKmh(next);
                return true;
            }

            case CommandType::PressSpeedMinus: {
                speedCommandsCount_++;
                float current = treadmill_.getTargetSpeedKmh();
                float next = std::max(0.8f, std::round((current - 0.1f) * 10.0f) / 10.0f);
                treadmill_.setTargetSpeedKmh(next);
                return true;
            }

            case CommandType::PressButton: {
                switch (cmd.button) {
                    case ButtonId::QuickStart: {
                        quickStartCount_++;
                        consecutiveStopPresses_ = 0;
                        treadmill_.setEmergencyStop(false);
                        treadmill_.setTargetSpeedKmh(preStopTargetSpeedKmh_);
                        treadmill_.setTargetInclinePct(preStopTargetInclinePct_);
                        return true;
                    }

                    case ButtonId::Stop: {
                        stopCount_++;
                        consecutiveStopPresses_++;
                        if (consecutiveStopPresses_ == 1) {
                            const float currentTarget = treadmill_.getTargetSpeedKmh();
                            if (currentTarget > 0.0f) {
                                preStopTargetSpeedKmh_ = currentTarget;
                                preStopTargetInclinePct_ = treadmill_.getTargetInclinePct();
                            }
                        } else {
                            resetResumeDefaults();
                        }
                        treadmill_.setTargetSpeedKmh(0.0f);
                        return true;
                    }

                    case ButtonId::SpeedPlus: {
                        speedCommandsCount_++;
                        float current = treadmill_.getTargetSpeedKmh();
                        float next = (current < 0.8f) ? 0.8f : (std::round((current + 0.1f) * 10.0f) / 10.0f);
                        next = std::min(22.0f, next);
                        treadmill_.setTargetSpeedKmh(next);
                        return true;
                    }

                    case ButtonId::SpeedMinus: {
                        speedCommandsCount_++;
                        float current = treadmill_.getTargetSpeedKmh();
                        float next = std::max(0.8f, std::round((current - 0.1f) * 10.0f) / 10.0f);
                        treadmill_.setTargetSpeedKmh(next);
                        return true;
                    }

                    case ButtonId::InclinePlus: {
                        inclineCommandsCount_++;
                        float current = treadmill_.getTargetInclinePct();
                        float next = std::min(15.0f, std::round((current + 0.5f) * 2.0f) / 2.0f);
                        treadmill_.setTargetInclinePct(next);
                        return true;
                    }

                    case ButtonId::InclineMinus: {
                        inclineCommandsCount_++;
                        float current = treadmill_.getTargetInclinePct();
                        float next = std::max(0.0f, std::round((current - 0.5f) * 2.0f) / 2.0f);
                        treadmill_.setTargetInclinePct(next);
                        return true;
                    }

                    case ButtonId::InstantSpeed:
                    case ButtonId::InstantIncline:
                    case ButtonId::Enter:
                    case ButtonId::Clear:
                    case ButtonId::Num0:
                    case ButtonId::Num1:
                    case ButtonId::Num2:
                    case ButtonId::Num3:
                    case ButtonId::Num4:
                    case ButtonId::Num5:
                    case ButtonId::Num6:
                    case ButtonId::Num7:
                    case ButtonId::Num8:
                    case ButtonId::Num9:
                        return true;

                    default:
                        return false;
                }
            }

            default:
                return false;
        }
    }

    void onEmergencyStop(bool active) override {
        emergencyStopCount_++;
        treadmill_.setEmergencyStop(active);
        if (active) {
            treadmill_.setTargetSpeedKmh(0.0f);
        }
    }

    uint32_t getTotalCommandsReceived() const { return totalCommandsReceived_; }
    uint32_t getQuickStartCount() const { return quickStartCount_; }
    uint32_t getStopCount() const { return stopCount_; }
    uint32_t getEmergencyStopCount() const { return emergencyStopCount_; }
    uint32_t getSpeedCommandsCount() const { return speedCommandsCount_; }
    uint32_t getInclineCommandsCount() const { return inclineCommandsCount_; }
    const TreadmillCommand& getLastCommand() const { return lastCommand_; }

private:
    void resetResumeDefaults() {
        preStopTargetSpeedKmh_ = 1.0f;
        preStopTargetInclinePct_ = 0.0f;
    }

    VirtualTreadmill& treadmill_;
    uint32_t totalCommandsReceived_ = 0;
    uint32_t quickStartCount_ = 0;
    uint32_t stopCount_ = 0;
    uint32_t emergencyStopCount_ = 0;
    uint32_t speedCommandsCount_ = 0;
    uint32_t inclineCommandsCount_ = 0;
    float preStopTargetSpeedKmh_ = 1.0f;
    float preStopTargetInclinePct_ = 0.0f;
    uint8_t consecutiveStopPresses_ = 0;
    TreadmillCommand lastCommand_{};
};

} // namespace stridecontrol
