#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include "../BluetoothTypes/BluetoothTypes.h"
#include "../SettingsService/SettingsServiceTypes.h"
#include "BleManagerTypes.h"

// Forward declaration in global namespace
class NimBLEScan;

namespace stridecontrol {

/**
 * @brief Dual-role Bluetooth Low Energy subsystem coordinator.
 *
 * Runs non-blocking on Core 0. Owns adapter lifecycle, GAP roles (Central scan,
 * Peripheral advertise), and coordinates HeartRateClient, FtmsService, and RscService.
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
    struct ScanListenerEntry {
        BleScanCallback callback{nullptr};
        void* context{nullptr};
    };

    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

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
