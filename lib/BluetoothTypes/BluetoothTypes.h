#pragma once

#include <cstdint>
#include <climits>

namespace stridecontrol {

/**
 * @brief Connection state of the central Heart Rate Client.
 */
enum class HeartRateConnectionState : uint8_t {
    Disconnected,
    Scanning,
    Connecting,
    Connected,
    Failed
};

/**
 * @brief Operational state of the BleManager adapter controller.
 */
enum class BleManagerState : uint8_t {
    Uninitialized,
    Ready,
    Scanning,
    Advertising,
    Active,
    Faulted
};

/**
 * @brief Passive DTO representing Heart Rate sensor state and telemetry.
 */
struct HeartRateState {
    bool initialized = false;
    bool heartRateValid = false;
    uint8_t heartRateBpm = 0;

    uint32_t lastSampleTimestampMs = 0;
    uint32_t dataAgeMs = UINT32_MAX;

    uint8_t batteryPercent = 0;
    bool batteryPercentValid = false;

    HeartRateConnectionState connectionState = HeartRateConnectionState::Disconnected;

    char sensorName[32] = {};
    char sensorAddress[18] = {};
};

/**
 * @brief Passive DTO representing Bluetooth stack health and connection state.
 */
struct BleState {
    bool initialized = false;
    BleManagerState state = BleManagerState::Uninitialized;

    bool isScanning = false;
    bool centralConnected = false;
    bool peripheralAdvertising = false;

    uint8_t ftmsClientCount = 0;
    uint8_t rscClientCount = 0;

    uint32_t lastEventTimestampMs = 0;
};

} // namespace stridecontrol

