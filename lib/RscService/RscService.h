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

class RscMeasurementCallbackAdapter;

/**
 * @brief Telemetry-only GATT server implementation for Running Speed and Cadence Service (0x1814).
 *
 * Implements RSC Measurement (0x2A53, Notify), RSC Feature (0x2A54, Read), and Sensor Location (0x2A5D, Read).
 * Driven strictly by ApplicationSnapshot telemetry on Core 0 with zero dynamic heap allocation.
 */
class RscService {
public:
    RscService();
    ~RscService();

    // Non-copyable, non-movable
    RscService(const RscService&) = delete;
    RscService& operator=(const RscService&) = delete;
    RscService(RscService&&) = delete;
    RscService& operator=(RscService&&) = delete;

    bool begin(::NimBLEServer* pServer);
    void end();

    void update(uint32_t nowMs, const ApplicationSnapshot& snapshot);
    void handleDisconnect(uint16_t connectionHandle);

    RscServerStateDto getState() const;

private:
    friend class RscMeasurementCallbackAdapter;
    void handleSubscribe(uint16_t connectionHandle, uint16_t subValue);
    size_t packRscMeasurement(const ApplicationSnapshot& snapshot, uint8_t* outPayload, size_t maxLen) const;

    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

    ::NimBLEServer* pServer_{nullptr};
    ::NimBLEService* pService_{nullptr};
    ::NimBLECharacteristic* pMeasurementChar_{nullptr};
    ::NimBLECharacteristic* pFeatureChar_{nullptr};
    ::NimBLECharacteristic* pSensorLocationChar_{nullptr};

    BleSubscriptionEntry subscribers_[kMaxTrackedSubscribers]{};
    RscServerStateDto state_{};
    uint32_t lastNotificationMs_{0};
};

} // namespace stridecontrol

