#pragma once

#include <cstdint>
#include "../BluetoothTypes/BluetoothTypes.h"

namespace stridecontrol {

/**
 * @brief Internal driver timing constants for HeartRateClient lifecycle and data freshness.
 */
struct HeartRateClientTiming {
    static constexpr uint32_t DATA_TIMEOUT_MS = 5000;           ///< Time without sample before heartRateValid becomes false
    static constexpr uint32_t SCAN_DURATION_MS = 5000;          ///< Duration of an active background scan window
    static constexpr uint32_t DISCOVERY_SCAN_DURATION_MS = 8000;///< Duration of an active discovery scan window
    static constexpr uint32_t RECONNECT_COOLDOWN_MS = 3000;     ///< Cooldown before retry after disconnect/failure (base case)
    static constexpr uint32_t RECONNECT_COOLDOWN_TIER1_MS = 30000; ///< Cooldown after 3+ consecutive failures (30s)
    static constexpr uint32_t RECONNECT_COOLDOWN_TIER2_MS = 60000; ///< Cooldown after 10+ consecutive failures (60s)
    static constexpr uint32_t RECONNECT_FAILURES_TIER1 = 3;     ///< Consecutive failures threshold for Tier 1 cooldown
    static constexpr uint32_t RECONNECT_FAILURES_TIER2 = 10;    ///< Consecutive failures threshold for Tier 2 cooldown
    static constexpr uint32_t BATTERY_POLL_INTERVAL_MS = 60000; ///< Interval between Battery Service (0x180F) reads
};

/**
 * @brief Internal sub-states for the Heart Rate Client connection cycle.
 */
enum class HeartRateInternalState : uint8_t {
    Idle,
    InitiatingScan,
    Scanning,
    Connecting,
    DiscoveringServices,
    SubscribingHr,
    ConnectedStreaming,
    Disconnecting,
    CooldownWait,
    ErrorRecovery
};

/**
 * @brief Read-only internal operational metrics for diagnostics.
 */
struct HeartRateClientMetrics {
    uint32_t totalSamplesReceived = 0;
    uint32_t droppedOrInvalidPackets = 0;
    uint32_t connectionAttempts = 0;
    uint32_t reconnectAttempts = 0;
    uint32_t successfulConnections = 0;
    uint32_t disconnectEvents = 0;
    int32_t lastRssiDbm = 0;                                    ///< Signed signal strength in dBm (typically negative)
};

} // namespace stridecontrol

