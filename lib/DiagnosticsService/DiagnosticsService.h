#pragma once

#include <cstddef>
#include <cstdint>
#include <bitset>
#include <freertos/FreeRTOS.h>
#include "DiagnosticsServiceTypes.h"

namespace stridecontrol {

/**
 * @brief Static, thread-safe system fault and health registry.
 *
 * Implements zero-heap allocation, FreeRTOS cross-core spinlocks,
 * and edge-triggered circular event logging.
 */
class DiagnosticsService {
public:
    DiagnosticsService();
    ~DiagnosticsService() = default;

    DiagnosticsService(const DiagnosticsService&) = delete;
    DiagnosticsService& operator=(const DiagnosticsService&) = delete;

    void begin();
    void end();

    void reportFault(FaultCode code, uint32_t timestampMs);
    void clearFault(FaultCode code, uint32_t timestampMs);

    bool hasActiveFaults() const;
    uint32_t getActiveFaultCount() const;
    bool isFaultActive(FaultCode code) const;
    FaultSeverity getHighestActiveSeverity() const;

    SystemHealthSnapshot getSnapshot() const;
    SystemHealthSnapshot getSnapshot(uint32_t timestampMs) const;

    static const char* version();

private:
    mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;

    bool initialized_ = false;
    uint32_t activeFaultCount_ = 0;
    uint32_t totalLoggedEvents_ = 0;

    std::bitset<kFaultCodeCount> activeFaults_{};

    FaultEvent eventBuffer_[kEventHistorySize]{};
    size_t bufferHead_ = 0;
    size_t bufferCount_ = 0;
};

} // namespace stridecontrol
