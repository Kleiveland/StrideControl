#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include "../BluetoothTypes/BluetoothTypes.h"
#include "../SettingsService/SettingsServiceTypes.h"
#include "BleManagerTypes.h"

// Forward declarations in global namespace
class NimBLEScan;
class NimBLEAdvertisedDevice;

namespace stridecontrol {

class BleScanCallbackAdapter;

/**
 * @brief Dual-role Bluetooth Low Energy subsystem coordinator.
 *
 * Runs non-blocking on Core 0. Owns adapter lifecycle, GAP roles (Central scan,
 * Peripheral advertise), and coordinates HeartRateClient, FtmsService, and RscService.
 *
 * ==============================================================================
 * STRICT V1 LIFETIME & THREADING CONTRACT:
 * ==============================================================================
 * 1. Lifecycle Serialization:
 *    begin() and end() must be externally serialized (e.g. from the main application
 *    orchestrator lifecycle task on Core 0).
 *
 * 2. Listener Context Lifetime:
 *    - Listener context storage (e.g. HeartRateClient) must remain valid until
 *      BleManager::end() has returned.
 *    - Runtime destruction of a registered listener context before BleManager::end()
 *      returns is prohibited.
 *    - After BleManager::end() returns, NimBLE callback quiescence is established
 *      and listener contexts may be safely destroyed before or after BleManager itself.
 *
 * 3. Unregister Semantics:
 *    unregisterScanListener() removes future dispatch eligibility only; in-flight
 *    completion on another task/core is not guaranteed.
 *
 * 4. Teardown Calling Context:
 *    BleManager::end() must NEVER be called from within a NimBLE host task callback
 *    or listener callback (deadlock hazard in nimble_port_stop).
 * ==============================================================================
 */
class BleManager {
public:
    BleManager();
    ~BleManager();

    // Non-copyable, non-movable
    BleManager(const BleManager&) = delete;
    BleManager& operator=(const BleManager&) = delete;
    BleManager(BleManager&&) = delete;
    BleManager& operator=(BleManager&&) = delete;

    bool begin(const BleConfig& config);
    void end();

    void update(uint32_t nowMs);
    void updateConfig(const BleConfig& config);

    bool startAdvertising();
    void stopAdvertising();

    bool registerScanListener(BleScanCallback callback, void* context);
    bool unregisterScanListener(BleScanCallback callback, void* context);

    bool startScan();
    void stopScan();

    BleState getState() const;
    BleManagerMetrics getMetrics() const;
    BleCoordinatorInternalState getInternalState() const;

private:
    friend class BleScanCallbackAdapter;
    void handleAdvertisedDevice(::NimBLEAdvertisedDevice* advertisedDevice);

    struct ScanListenerEntry {
        BleScanCallback callback{nullptr};
        void* context{nullptr};
    };

    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

    bool isTransitioningLifecycle_{false};
    bool isShuttingDown_{false};

    ScanListenerEntry scanListeners_[4]{};
    ::NimBLEScan* scan_{nullptr};

    BleConfig config_{};
    BleState state_{};
    BleManagerMetrics metrics_{};

    BleCoordinatorInternalState internalState_{
        BleCoordinatorInternalState::Uninitialized
    };

    uint32_t lastUpdateMs_{0};
    uint32_t stateEntryMs_{0};
};

} // namespace stridecontrol
