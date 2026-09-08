#pragma once

#include <cstdint>
#include <cstddef>
#include <freertos/FreeRTOS.h>
#include "../BleServer/BleServerTypes.h"
#include "../ApplicationSnapshot/ApplicationSnapshot.h"

// Forward declarations in global namespace (Zero NimBLE header leakage)
class NimBLEServer;
class NimBLEService;
class NimBLECharacteristic;

namespace stridecontrol {

class FtmsTreadmillCallbackAdapter;

/**
 * @brief Telemetry-only GATT server implementation for Fitness Machine Service (0x1826).
 *
 * Implements Treadmill Data (0x2ACD, Notify) and Fitness Machine Feature (0x2ACC, Read).
 * Strictly telemetry-only for V1; Control Point, Status, and Supported Ranges are omitted.
 * Driven strictly by ApplicationSnapshot telemetry on Core 0 with zero dynamic heap allocation.
 */
class FtmsService {
public:
    FtmsService();
    ~FtmsService();

    // Non-copyable, non-movable
    FtmsService(const FtmsService&) = delete;
    FtmsService& operator=(const FtmsService&) = delete;
    FtmsService(FtmsService&&) = delete;
    FtmsService& operator=(FtmsService&&) = delete;

    bool begin(::NimBLEServer* pServer);
    void end();

    void update(uint32_t nowMs, const ApplicationSnapshot& snapshot);
    void handleDisconnect(uint16_t connectionHandle);

    FtmsServerStateDto getState() const;

private:
    friend class FtmsTreadmillCallbackAdapter;
    void handleSubscribe(uint16_t connectionHandle, uint16_t subValue);
    size_t packTreadmillData(const ApplicationSnapshot& snapshot, uint8_t* outPayload, size_t maxLen) const;

    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

    ::NimBLEServer* pServer_{nullptr};
    ::NimBLEService* pService_{nullptr};
    ::NimBLECharacteristic* pTreadmillDataChar_{nullptr};
    ::NimBLECharacteristic* pFeatureChar_{nullptr};

    BleSubscriptionEntry subscribers_[kMaxTrackedSubscribers]{};
    FtmsServerStateDto state_{};
    uint32_t lastNotificationMs_{0};
};

} // namespace stridecontrol

