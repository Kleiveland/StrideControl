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

class HeartRateMeasurementCallbackAdapter;

/**
 * @brief Telemetry-only GATT server implementation for the standard Heart Rate Service (0x180D).
 *
 * Re-broadcasts heart rate already read from an external strap via HeartRateClient, as a
 * standalone peripheral service - so any connected app (e.g. Zwift) can use StrideControl as
 * its heart rate source directly, without a separate Bluetooth connection to the strap.
 * Implements Heart Rate Measurement (0x2A37, Notify) only - Body Sensor Location and Heart
 * Rate Control Point are optional and intentionally omitted for V1.
 */
class HeartRateService {
public:
    HeartRateService();
    ~HeartRateService();

    HeartRateService(const HeartRateService&) = delete;
    HeartRateService& operator=(const HeartRateService&) = delete;
    HeartRateService(HeartRateService&&) = delete;
    HeartRateService& operator=(HeartRateService&&) = delete;

    bool begin(::NimBLEServer* pServer);
    void end();

    void update(uint32_t nowMs, const ApplicationSnapshot& snapshot);
    void handleDisconnect(uint16_t connectionHandle);

    HeartRateServerStateDto getState() const;

private:
    friend class HeartRateMeasurementCallbackAdapter;
    void handleSubscribe(uint16_t connectionHandle, uint16_t subValue);
    size_t packHeartRateMeasurement(const ApplicationSnapshot& snapshot, uint8_t* outPayload, size_t maxLen) const;

    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

    ::NimBLEServer* pServer_{nullptr};
    ::NimBLEService* pService_{nullptr};
    ::NimBLECharacteristic* pMeasurementChar_{nullptr};

    BleSubscriptionEntry subscribers_[kMaxTrackedSubscribers]{};
    HeartRateServerStateDto state_{};
    uint32_t lastNotificationMs_{0};
    static constexpr uint32_t kHrNotificationIntervalMs = 1000; // 1 Hz, standard HR update rate
};

} // namespace stridecontrol

