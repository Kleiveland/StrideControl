#pragma once

#include <cstdint>
#include <cstddef>
#include "../BluetoothTypes/BluetoothTypes.h"

namespace stridecontrol {

class RscService;
class FtmsService;

// Shared connection pool budget (CONFIG_BT_NIMBLE_MAX_CONNECTIONS = 3)
static constexpr uint16_t kMaxCentralConnections    = 1;     // HeartRateClient
static constexpr uint16_t kMaxPeripheralConnections = 2;     // e.g. Zwift + Companion/Watch
static constexpr uint16_t kTotalMaxConnections       = kMaxCentralConnections + kMaxPeripheralConnections; // 3
static constexpr uint16_t kInvalidConnectionHandle  = 0xFFFF;

// Maximum pending server disconnect handles queued in the Core 0 mailbox
static constexpr size_t kMaxPendingServerDisconnects = 4;

// Bluetooth SIG Generic Running Walking Sensor Appearance (Category 17, Sub-category 0)
static constexpr uint16_t kBleAppearanceGenericRunningWalkingSensor = 0x0440;

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
    uint8_t addressType{0};                     ///< Exact NimBLE advertiser-reported address type.
    char name[32]{};
    int32_t rssiDbm{0};
    bool advertisesHeartRateService{false};
};

/**
 * @brief Scan callback signature for registered listeners.
 */
using BleScanCallback = void (*)(const BleScanResult& result, void* context);

} // namespace stridecontrol
