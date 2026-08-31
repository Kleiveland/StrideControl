#pragma once

#include <cstdint>
#include "../ConsoleInterface/ConsoleTypes.h"
#include "../SpeedCalibration/SpeedCalibrationTypes.h"

namespace stridecontrol {

/**
 * @brief Controller operational lifecycle and state.
 */
enum class TreadmillControllerState : uint8_t {
    Uninitialized,    ///< begin() not yet called or end() completed
    Idle,             ///< Ready, no command in flight
    WaitingForResult, ///< Command submitted, awaiting terminal event
    Completed,        ///< Latest correlated command completed successfully
    CommandFailed,    ///< Latest correlated command failed/aborted/rejected
    Faulted           ///< Mandatory dependency not ready
};

/**
 * @brief Lightweight, copy-safe snapshot of TreadmillController state and targets.
 */
struct TreadmillControllerSnapshot {
    TreadmillControllerState state = TreadmillControllerState::Uninitialized;

    // Speed context
    float acceptedPhysicalSpeedTargetKmh = 0.0f;
    bool physicalSpeedTargetValid = false;
    float correctedConsoleSpeedKmh = 0.0f;
    bool correctedConsoleSpeedValid = false;
    SpeedCalibrationResultStatus lastCalibrationStatus = SpeedCalibrationResultStatus::InvalidInput;

    // Incline context
    float acceptedInclineTargetPct = 0.0f;
    bool inclineTargetValid = false;

    // Active command tracking
    uint32_t activeRequestId = 0;
    bool activeRequestValid = false;
    CommandType activeCommandType = CommandType::PressButton;

    // Latest terminal command outcome & validities
    ConsoleOutcome latestOutcome = ConsoleOutcome::NormalSingle;
    bool latestOutcomeValid = false;
    CommandStatus latestCommandStatus = CommandStatus::Completed;
    bool latestCommandStatusValid = false;

    // State & timing
    uint32_t lastUpdateTimestampMs = 0;
    bool ready = false;
    bool busy = false;
};

} // namespace stridecontrol
