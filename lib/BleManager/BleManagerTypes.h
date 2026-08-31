#pragma once

#include <cstdint>
#include "../BluetoothTypes/BluetoothTypes.h"

namespace stridecontrol {

/**
 * @brief Internal detailed lifecycle state for the BLE coordinator.
 */
enum class BleCoordinatorInternalState : uint8_t {
    Uninitialized,
    InitializingStack,
    Standby,
    AdvertisingOnly,
    ScanningOnly,
    DualRoleActive,
    StackErrorRecovery,
    Suspended
};

/**
 * @brief Read-only internal operational metrics for BLE coordinator.
 */
struct BleManagerMetrics {
    uint32_t stackInitCount = 0;
    uint32_t scanStartCount = 0;
    uint32_t advertisingStartCount = 0;
    uint32_t stackErrorEvents = 0;
    uint8_t activePeripheralConnections = 0;
    uint8_t activeCentralConnections = 0;
};

/**
 * @brief POD describing a discovered BLE advertiser.
 */
struct BleScanResult {
    char address[18]{};
    char name[32]{};
    int32_t rssiDbm{0};
    bool advertisesHeartRateService{false};
};

/**
 * @brief Scan callback signature for registered listeners.
 */
using BleScanCallback = void (*)(const BleScanResult& result, void* context);

} // namespace stridecontrol
