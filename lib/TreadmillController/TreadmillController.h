#pragma once

#include <cstdint>
#include <cstddef>
#include "../ConsoleInterface/ConsoleInterface.h"
#include "../SpeedCalibration/SpeedCalibration.h"
#include "../DiagnosticsService/DiagnosticsService.h"
#include "TreadmillControllerTypes.h"

namespace stridecontrol {

/**
 * @brief Lightweight, synchronous Core 0 command-coordination layer.
 *
 * Coordinates high-level physical speed and incline requests, applies SpeedCalibration,
 * and submits non-blocking macro commands to ConsoleInterface.
 *
 * @note CALLER CONCURRENCY CONTRACT:
 * - TreadmillController does not create an internal FreeRTOS task, mutex, or queue.
 * - All public methods (begin, end, submitSpeedTarget, submitInclineTarget, submitStop,
 *   update, getSnapshot, isReady, isBusy) MUST be serialized and called from one owning
 *   Core 0 execution context (e.g. SystemManager / main loop).
 * - Concurrent callers across tasks/cores are strictly prohibited.
 * - Injected dependency references (ConsoleInterface, SpeedCalibration, DiagnosticsService)
 *   must outlive TreadmillController.
 * - All ConsoleInterface interactions are verified non-blocking (waitTicks = 0).
 */
class TreadmillController {
public:
    TreadmillController(
        ConsoleInterface& console,
        SpeedCalibration& calibration,
        DiagnosticsService& diagnostics
    );

    ~TreadmillController() = default;

    TreadmillController(const TreadmillController&) = delete;
    TreadmillController& operator=(const TreadmillController&) = delete;

    bool begin();
    void end();

    bool submitSpeedTarget(
        float desiredPhysicalSpeedKmh,
        uint32_t requestTimestampMs
    );

    bool submitInclineTarget(
        float targetInclinePct,
        uint32_t requestTimestampMs
    );

    bool submitStop(uint32_t requestTimestampMs);

    void update(uint32_t nowMs);

    TreadmillControllerSnapshot getSnapshot() const;

    bool isReady() const;
    bool isBusy() const;

    static const char* version();

private:
    ConsoleInterface& console_;
    SpeedCalibration& calibration_;
    DiagnosticsService& diagnostics_;

    TreadmillControllerSnapshot snapshot_{};
    bool initialized_ = false;
    uint32_t nextRequestId_ = 0;
    bool consoleFaultReported_ = false;
};

} // namespace stridecontrol
