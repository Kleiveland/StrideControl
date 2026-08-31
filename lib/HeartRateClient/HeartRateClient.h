#pragma once

#include <cstdint>
#include "../BluetoothTypes/BluetoothTypes.h"
#include "../SettingsService/SettingsServiceTypes.h"
#include "HeartRateClientTypes.h"

namespace stridecontrol {

/**
 * @brief Central BLE client for standard Heart Rate Service (0x180D).
 *
 * Runs non-blocking state updates on Core 0. Produces HeartRateState
 * consumed by ApplicationSnapshot and WorkoutSession.
 */
class HeartRateClient {
public:
    HeartRateClient();
    ~HeartRateClient();

    // Non-copyable, non-movable
    HeartRateClient(const HeartRateClient&) = delete;
    HeartRateClient& operator=(const HeartRateClient&) = delete;
    HeartRateClient(HeartRateClient&&) = delete;
    HeartRateClient& operator=(HeartRateClient&&) = delete;

    bool begin(const BleConfig& config);
    void end();

    void update(uint32_t nowMs);
    void updateConfig(const BleConfig& config);

    void startScan();
    void stopScan();
    void disconnect();

    HeartRateState getState() const;
    HeartRateClientMetrics getMetrics() const;
    HeartRateInternalState getInternalState() const;

private:
    BleConfig config_{};
    HeartRateState state_{};
    HeartRateClientMetrics metrics_{};
    HeartRateInternalState internalState_{HeartRateInternalState::Idle};

    uint32_t lastUpdateMs_{0};
    uint32_t stateEntryMs_{0};
    uint32_t lastBatteryPollMs_{0};
};

} // namespace stridecontrol
