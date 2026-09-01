#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include "../BluetoothTypes/BluetoothTypes.h"
#include "../SettingsService/SettingsServiceTypes.h"
#include "../BleManager/BleManagerTypes.h"
#include "HeartRateClientTypes.h"

// Forward declarations in global namespace (Zero NimBLE header leakage)
class NimBLEClient;
class NimBLERemoteCharacteristic;
class NimBLEAdvertisedDevice;

namespace stridecontrol {

class BleManager;

/**
 * @brief Central BLE client for standard Heart Rate Service (0x180D).
 *
 * Subscribes to Heart Rate Measurement characteristic (0x2A37) and produces
 * HeartRateState telemetry consumed by ApplicationSnapshot and WorkoutSession.
 *
 * ==============================================================================
 * STRICT V1 LIFETIME & THREADING CONTRACT:
 * ==============================================================================
 * 1. Storage Lifetime:
 *    HeartRateClient instance storage must remain allocated and valid until
 *    BleManager::end() has returned (system lifetime).
 *
 * 2. Unregister Semantics:
 *    unregisterScanListener() removes future dispatch eligibility only; in-flight
 *    callbacks on the NimBLE host task are not waited on synchronously.
 *
 * 3. Asynchronous Disconnect:
 *    NimBLEClient::disconnect() is asynchronous; full callback quiescence is
 *    guaranteed only after BleManager::end() / NimBLEDevice::deinit(true).
 *
 * 4. Lifecycle & Action Serialization:
 *    begin(), update(), startScan(), stopScan(), disconnect(), and end() must be
 *    externally serialized on the application lifecycle task on Core 0.
 *
 * 5. Non-Blocking Callback Discipline:
 *    Callbacks running on the NimBLE host task only update internal spinlock-protected
 *    state and set pending flags; they NEVER perform blocking BLE calls.
 * ==============================================================================
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

    bool begin(const BleConfig& config, BleManager* bleManager);
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
    friend class HeartRateClientCallbacks;
    static void onScanResultTrampoline(const BleScanResult& result, void* context);
    static void onHrNotifyTrampoline(::NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify);

    void handleScanResult(const BleScanResult& result);
    void handleNotification(const uint8_t* pData, size_t length);
    void handleConnect();
    void handleDisconnect();

    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

    ::NimBLEClient* client_{nullptr};
    ::NimBLERemoteCharacteristic* hrChar_{nullptr};
    BleManager* bleManager_{nullptr};

    BleConfig config_{};
    HeartRateState state_{};
    HeartRateClientMetrics metrics_{};
    HeartRateInternalState internalState_{HeartRateInternalState::Idle};

    char pendingConnectAddress_[18]{};
    uint8_t pendingConnectAddressType_{0};
    char pendingConnectName_[32]{};
    int32_t pendingRssiDbm_{0};
    bool connectPending_{false};

    uint16_t pendingHeartRateBpm_{0};
    bool samplePending_{false};

    bool disconnectPending_{false};
    bool connectEventPending_{false};

    uint32_t lastDataReceivedMs_{0};
    uint32_t stateEntryTimestampMs_{0};

    bool isShuttingDown_{false};
    bool isTransitioningLifecycle_{false};
};

} // namespace stridecontrol
