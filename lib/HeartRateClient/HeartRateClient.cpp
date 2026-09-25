#include "HeartRateClient.h"
#include <Arduino.h>
#include "../BleManager/BleManager.h"
#include <NimBLEDevice.h>
#include <NimBLEClient.h>
#include <NimBLERemoteService.h>
#include <NimBLERemoteCharacteristic.h>
#include <cstring>
#include <string>
#include "../DiagnosticsLog/DiagnosticsLog.h"

namespace stridecontrol {

class HeartRateClientCallbacks;

static portMUX_TYPE s_hrAdapterMux = portMUX_INITIALIZER_UNLOCKED;

class HeartRateClientCallbacks : public NimBLEClientCallbacks {
public:
    HeartRateClientCallbacks() = default;
    ~HeartRateClientCallbacks() override = default;

    bool bindOwner(HeartRateClient* newOwner) {
        portENTER_CRITICAL(&s_hrAdapterMux);
        if (owner_ != nullptr && owner_ != newOwner) {
            portEXIT_CRITICAL(&s_hrAdapterMux);
            return false;
        }
        owner_ = newOwner;
        portEXIT_CRITICAL(&s_hrAdapterMux);
        return true;
    }

    void clearOwner(HeartRateClient* caller) {
        portENTER_CRITICAL(&s_hrAdapterMux);
        if (owner_ == caller) {
            owner_ = nullptr;
        }
        portEXIT_CRITICAL(&s_hrAdapterMux);
    }

    HeartRateClient* getOwner() const {
        portENTER_CRITICAL(&s_hrAdapterMux);
        HeartRateClient* copy = owner_;
        portEXIT_CRITICAL(&s_hrAdapterMux);
        return copy;
    }

    void onConnect(NimBLEClient* pClient) override {
        (void)pClient;
        portENTER_CRITICAL(&s_hrAdapterMux);
        HeartRateClient* localOwner = owner_;
        portEXIT_CRITICAL(&s_hrAdapterMux);

        if (localOwner != nullptr) {
            localOwner->handleConnect();
        }
    }

    void onDisconnect(NimBLEClient* pClient) override {
        (void)pClient;
        portENTER_CRITICAL(&s_hrAdapterMux);
        HeartRateClient* localOwner = owner_;
        portEXIT_CRITICAL(&s_hrAdapterMux);

        if (localOwner != nullptr) {
            localOwner->handleDisconnect();
        }
    }

private:
    HeartRateClient* owner_{nullptr};
};

static HeartRateClientCallbacks s_hrClientCallbackAdapter;

void HeartRateClient::onScanResultTrampoline(const BleScanResult& result, void* context) {
    if (context != nullptr) {
        static_cast<HeartRateClient*>(context)->handleScanResult(result);
    }
}

void HeartRateClient::onHrNotifyTrampoline(::NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    (void)pChar;
    (void)isNotify;
    HeartRateClient* localOwner = s_hrClientCallbackAdapter.getOwner();
    if (localOwner != nullptr) {
        localOwner->handleNotification(pData, length);
    }
}

HeartRateClient::HeartRateClient() = default;

HeartRateClient::~HeartRateClient() {
    end();
}

bool HeartRateClient::begin(const BleConfig& config, BleManager* bleManager) {
    if (bleManager == nullptr) {
        return false;
    }

    portENTER_CRITICAL(&mux_);
    if (state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return true; // Idempotent success
    }
    if (isTransitioningLifecycle_ || isShuttingDown_) {
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    isTransitioningLifecycle_ = true;
    config_ = config;
    scanResultCount_ = 0;
    reconnectAttempts_ = 0;
    bleManager_ = bleManager;
    portEXIT_CRITICAL(&mux_);

    if (!s_hrClientCallbackAdapter.bindOwner(this)) {
        portENTER_CRITICAL(&mux_);
        isTransitioningLifecycle_ = false;
        bleManager_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    bool regOk = bleManager_->registerScanListener(onScanResultTrampoline, this);
    if (!regOk) {
        s_hrClientCallbackAdapter.clearOwner(this);
        portENTER_CRITICAL(&mux_);
        isTransitioningLifecycle_ = false;
        bleManager_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    const bool hasTargetDevice = (config.preferredHrMac[0] != '\0');
    const bool shouldAutoScan = (config.autoConnectHr && hasTargetDevice);

    bool scanningActive = false;
    if (shouldAutoScan && bleManager_ != nullptr) {
        scanningActive = bleManager_->getState().isScanning || bleManager_->startScan();
    }

    portENTER_CRITICAL(&mux_);
    state_.initialized = true;
    if (shouldAutoScan && scanningActive) {
        state_.connectionState = HeartRateConnectionState::Scanning;
        internalState_ = HeartRateInternalState::Scanning;
        stateEntryTimestampMs_ = millis();
    } else {
        state_.connectionState = HeartRateConnectionState::Disconnected;
        internalState_ = HeartRateInternalState::Idle;
        stateEntryTimestampMs_ = millis();
    }
    isTransitioningLifecycle_ = false;
    portEXIT_CRITICAL(&mux_);

    return true;
}

void HeartRateClient::end() {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || isShuttingDown_ || isTransitioningLifecycle_) {
        portEXIT_CRITICAL(&mux_);
        return; // Idempotent no-op
    }

    isShuttingDown_ = true;
    isTransitioningLifecycle_ = true;
    BleManager* localMgr = bleManager_;
    ::NimBLEClient* localClient = client_;
    portEXIT_CRITICAL(&mux_);

    if (localMgr != nullptr) {
        localMgr->unregisterScanListener(onScanResultTrampoline, this);
    }

    if (localClient != nullptr) {
        localClient->setClientCallbacks(nullptr, false);
        if (localClient->isConnected()) {
            localClient->disconnect();
        }
    }

    s_hrClientCallbackAdapter.clearOwner(this);

    portENTER_CRITICAL(&mux_);
    client_ = nullptr;
    hrChar_ = nullptr;
    bleManager_ = nullptr;
    state_ = HeartRateState{};
    metrics_ = HeartRateClientMetrics{};
    internalState_ = HeartRateInternalState::Idle;
    reconnectAttempts_ = 0;
    connParamsRequestedMs_ = 0;
    connParamsCheckPending_ = false;
    connectPending_ = false;
    samplePending_ = false;
    disconnectPending_ = false;
    connectEventPending_ = false;
    pendingHeartRateBpm_ = 0;
    pendingConnectAddressType_ = 0;
    isShuttingDown_ = false;
    isTransitioningLifecycle_ = false;
    portEXIT_CRITICAL(&mux_);
}

void HeartRateClient::updateConfig(const BleConfig& config) {
    BleManager* localMgr = nullptr;
    bool revertProfile = false;
    bool shouldDisconnect = false;
    portENTER_CRITICAL(&mux_);
    config_ = config;
    if (isDiscoveryScan_) {
        isDiscoveryScan_ = false;
        revertProfile = true;
        localMgr = bleManager_;
    }

    const bool isCurrentlyConnected = (state_.connectionState == HeartRateConnectionState::Connected ||
                                       internalState_ == HeartRateInternalState::ConnectedStreaming ||
                                       internalState_ == HeartRateInternalState::Connecting);

    if (isCurrentlyConnected) {
        const char* currentAddr = (state_.connectedAddress[0] != '\0') ? state_.connectedAddress : pendingConnectAddress_;
        if (config_.preferredHrMac[0] == '\0' || !config_.autoConnectHr ||
            (currentAddr[0] != '\0' && strcasecmp(config_.preferredHrMac, currentAddr) != 0)) {
            shouldDisconnect = true;
        }
    }

    if (internalState_ == HeartRateInternalState::Idle && config_.preferredHrMac[0] != '\0' && config_.autoConnectHr) {
        internalState_ = HeartRateInternalState::CooldownWait;
        reconnectAttempts_ = 0;
        metrics_.reconnectAttempts = 0;
        stateEntryTimestampMs_ = millis() - HeartRateClientTiming::RECONNECT_COOLDOWN_MS;
    }
    portEXIT_CRITICAL(&mux_);

    if (revertProfile && localMgr != nullptr) {
        localMgr->setScanProfile(BleScanProfile::Background);
    }

    if (shouldDisconnect) {
        disconnect();
    }
}

BleConfig HeartRateClient::getConfig() const {
    portENTER_CRITICAL(&mux_);
    BleConfig cfg = config_;
    portEXIT_CRITICAL(&mux_);
    return cfg;
}

size_t HeartRateClient::getScanResults(BleScanResult* outResults, size_t maxResults) const {
    if (outResults == nullptr || maxResults == 0) {
        return 0;
    }
    portENTER_CRITICAL(&mux_);
    size_t countToCopy = (scanResultCount_ < maxResults) ? scanResultCount_ : maxResults;
    for (size_t i = 0; i < countToCopy; ++i) {
        outResults[i] = scanResults_[i];
    }
    portEXIT_CRITICAL(&mux_);
    return countToCopy;
}

void HeartRateClient::startScan() {
    BleManager* localMgr = nullptr;
    portENTER_CRITICAL(&mux_);
    const bool init = state_.initialized;
    const bool shuttingDown = isShuttingDown_;
    const bool trans = isTransitioningLifecycle_;
    portEXIT_CRITICAL(&mux_);

    if (!init || shuttingDown || trans ||
        internalState_ == HeartRateInternalState::Connecting ||
        internalState_ == HeartRateInternalState::ConnectedStreaming) {
        return;
    }
    portENTER_CRITICAL(&mux_);
    scanResultCount_ = 0;
    isDiscoveryScan_ = true;
    localMgr = bleManager_;
    portEXIT_CRITICAL(&mux_);

    if (localMgr == nullptr) {
        return;
    }

    bool ok = localMgr->startScan(BleScanProfile::Discovery);

    portENTER_CRITICAL(&mux_);
    if (ok && !isShuttingDown_) {
        state_.connectionState = HeartRateConnectionState::Scanning;
        internalState_ = HeartRateInternalState::Scanning;
        stateEntryTimestampMs_ = millis();
    } else {
        isDiscoveryScan_ = false;
    }
    portEXIT_CRITICAL(&mux_);
}

void HeartRateClient::stopScan() {
    BleManager* localMgr = nullptr;
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || isShuttingDown_ || isTransitioningLifecycle_) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    const bool wasDiscovery = isDiscoveryScan_;
    isDiscoveryScan_ = false;
    localMgr = bleManager_;
    portEXIT_CRITICAL(&mux_);

    if (localMgr != nullptr) {
        if (wasDiscovery) {
            localMgr->setScanProfile(BleScanProfile::Background);
        }
        localMgr->stopScan();
    }

    portENTER_CRITICAL(&mux_);
    if (internalState_ == HeartRateInternalState::Scanning && !connectPending_) {
        state_.connectionState = HeartRateConnectionState::Disconnected;
        internalState_ = HeartRateInternalState::Idle;
    }
    portEXIT_CRITICAL(&mux_);
}

void HeartRateClient::disconnect() {
    ::NimBLEClient* localClient = nullptr;
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || isShuttingDown_ || isTransitioningLifecycle_) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    localClient = client_;
    internalState_ = HeartRateInternalState::Disconnecting;
    state_.heartRateValid = false;
    portEXIT_CRITICAL(&mux_);

    if (localClient != nullptr && localClient->isConnected()) {
        localClient->disconnect();
    } else {
        portENTER_CRITICAL(&mux_);
        state_.connectionState = HeartRateConnectionState::Disconnected;
        state_.heartRateValid = false;
        state_.connectedAddress[0] = '\0';
        state_.sensorAddress[0] = '\0';
        state_.sensorName[0] = '\0';
        internalState_ = HeartRateInternalState::Idle;
        hrChar_ = nullptr;
        portEXIT_CRITICAL(&mux_);
    }
}

void HeartRateClient::handleScanResult(const BleScanResult& result) {
    portENTER_CRITICAL(&mux_);
    if (isShuttingDown_ || !state_.initialized || internalState_ != HeartRateInternalState::Scanning) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    if (!isDiscoveryScan_ && connectPending_) {
        portEXIT_CRITICAL(&mux_);
        return;
    }

    if (result.advertisesHeartRateService && result.address[0] != '\0') {
        bool found = false;
        for (size_t i = 0; i < scanResultCount_; ++i) {
            if (strcasecmp(scanResults_[i].address, result.address) == 0) {
                scanResults_[i].rssiDbm = result.rssiDbm;
                if (result.name[0] != '\0') {
                    strncpy(scanResults_[i].name, result.name, sizeof(scanResults_[i].name) - 1);
                    scanResults_[i].name[sizeof(scanResults_[i].name) - 1] = '\0';
                }
                found = true;
                break;
            }
        }
        if (!found && scanResultCount_ < kMaxScanResults) {
            scanResults_[scanResultCount_++] = result;
        }
    }

    // In Discovery mode, never trigger auto-connect; solely collect nearby straps for pairing UI
    if (isDiscoveryScan_) {
        portEXIT_CRITICAL(&mux_);
        return;
    }

    // In Background mode, auto-connect strictly if matching saved preferredHrMac
    bool match = false;
    if (config_.preferredHrMac[0] != '\0') {
        match = (strcasecmp(result.address, config_.preferredHrMac) == 0);
    }

    if (match) {
        strncpy(pendingConnectAddress_, result.address, sizeof(pendingConnectAddress_) - 1);
        pendingConnectAddress_[sizeof(pendingConnectAddress_) - 1] = '\0';

        pendingConnectAddressType_ = result.addressType;

        strncpy(pendingConnectName_, result.name, sizeof(pendingConnectName_) - 1);
        pendingConnectName_[sizeof(pendingConnectName_) - 1] = '\0';

        pendingRssiDbm_ = result.rssiDbm;
        connectPending_ = true;
    }
    portEXIT_CRITICAL(&mux_);
}

void HeartRateClient::handleConnect() {
    portENTER_CRITICAL(&mux_);
    connectEventPending_ = true;
    portEXIT_CRITICAL(&mux_);
}

void HeartRateClient::handleDisconnect() {
    portENTER_CRITICAL(&mux_);
    if (state_.connectionState == HeartRateConnectionState::Connected ||
        internalState_ == HeartRateInternalState::ConnectedStreaming ||
        internalState_ == HeartRateInternalState::Connecting) {
        disconnectPending_ = true;
    }
    portEXIT_CRITICAL(&mux_);
}

void HeartRateClient::handleNotification(const uint8_t* pData, size_t length) {
    if (pData == nullptr || length < 2) {
        portENTER_CRITICAL(&mux_);
        metrics_.droppedOrInvalidPackets++;
        portEXIT_CRITICAL(&mux_);
        return;
    }

    uint8_t flags = pData[0];
    bool is16BitBpm = (flags & 0x01) != 0;
    bool hasEnergy = (flags & 0x08) != 0;
    bool hasRr = (flags & 0x10) != 0;

    size_t expectedLen = is16BitBpm ? 3 : 2;
    if (hasEnergy) {
        expectedLen += 2;
    }

    if (length < expectedLen) {
        portENTER_CRITICAL(&mux_);
        metrics_.droppedOrInvalidPackets++;
        portEXIT_CRITICAL(&mux_);
        return;
    }

    size_t remainingBytes = length - expectedLen;
    if (!hasRr && remainingBytes != 0) {
        portENTER_CRITICAL(&mux_);
        metrics_.droppedOrInvalidPackets++;
        portEXIT_CRITICAL(&mux_);
        return;
    }

    if (hasRr && (remainingBytes < 2 || (remainingBytes % 2) != 0)) {
        portENTER_CRITICAL(&mux_);
        metrics_.droppedOrInvalidPackets++;
        portEXIT_CRITICAL(&mux_);
        return;
    }

    uint16_t rawBpm = is16BitBpm ? (uint16_t)(pData[1] | (pData[2] << 8)) : (uint16_t)pData[1];

    if (rawBpm == 0 || rawBpm > 255) {
        portENTER_CRITICAL(&mux_);
        metrics_.droppedOrInvalidPackets++;
        portEXIT_CRITICAL(&mux_);
        return;
    }

    portENTER_CRITICAL(&mux_);
    pendingHeartRateBpm_ = rawBpm;
    samplePending_ = true;
    metrics_.totalSamplesReceived++;
    portEXIT_CRITICAL(&mux_);
}

void HeartRateClient::update(uint32_t nowMs) {
    // 1. Ingest Pending Disconnect Event
    portENTER_CRITICAL(&mux_);
    if (disconnectPending_) {
        disconnectPending_ = false;
        metrics_.disconnectEvents++;
        state_.connectionState = HeartRateConnectionState::Disconnected;
        state_.heartRateValid = false;
        state_.connectedAddress[0] = '\0';
        state_.sensorAddress[0] = '\0';
        state_.sensorName[0] = '\0';
        if (config_.preferredHrMac[0] != '\0' && config_.autoConnectHr) {
            internalState_ = HeartRateInternalState::CooldownWait;
        } else {
            internalState_ = HeartRateInternalState::Idle;
        }
        stateEntryTimestampMs_ = nowMs;
        hrChar_ = nullptr;
        connParamsCheckPending_ = false;
    }
    portEXIT_CRITICAL(&mux_);

    // 2. Ingest Pending Connect Event
    bool handleConn = false;
    ::NimBLEClient* localClientForRssi = nullptr;
    portENTER_CRITICAL(&mux_);
    if (connectEventPending_) {
        connectEventPending_ = false;
        handleConn = true;
        localClientForRssi = client_;
    }
    portEXIT_CRITICAL(&mux_);

    if (handleConn && localClientForRssi != nullptr) {
        int rssi = localClientForRssi->getRssi();
        portENTER_CRITICAL(&mux_);
        if (rssi != 0) {
            metrics_.lastRssiDbm = rssi;
        }
        portEXIT_CRITICAL(&mux_);
    }

    // 3. Ingest Pending Heart Rate Sample (Unified nowMs timebase)
    portENTER_CRITICAL(&mux_);
    if (samplePending_) {
        state_.heartRateBpm = (uint8_t)pendingHeartRateBpm_;
        state_.heartRateValid = (pendingHeartRateBpm_ > 0);
        state_.lastSampleTimestampMs = nowMs;
        state_.dataAgeMs = 0;
        lastDataReceivedMs_ = nowMs;
        samplePending_ = false;
    }
    portEXIT_CRITICAL(&mux_);

    // 3b. Scan Duration Watchdog (Enforce bounded scan window; never scan continuously)
    BleManager* scanStopMgr = nullptr;
    bool wasDiscoveryTimeout = false;
    portENTER_CRITICAL(&mux_);
    const uint32_t scanDuration = isDiscoveryScan_
        ? HeartRateClientTiming::DISCOVERY_SCAN_DURATION_MS
        : HeartRateClientTiming::SCAN_DURATION_MS;

    if (internalState_ == HeartRateInternalState::Scanning &&
        (nowMs - stateEntryTimestampMs_ >= scanDuration)) {
        scanStopMgr = bleManager_;
        wasDiscoveryTimeout = isDiscoveryScan_;
        isDiscoveryScan_ = false;

        if (config_.preferredHrMac[0] != '\0' && config_.autoConnectHr) {
            // Saved paired sensor: enter bounded cooldown before next targeted scan attempt
            internalState_ = HeartRateInternalState::CooldownWait;
            state_.connectionState = HeartRateConnectionState::Disconnected;
            stateEntryTimestampMs_ = nowMs;
            if (!wasDiscoveryTimeout) {
                reconnectAttempts_++;
                metrics_.reconnectAttempts = reconnectAttempts_;
                const uint32_t nextCooldownMs = calculateReconnectCooldownMs();
                stridecontrol::DiagnosticsLog::instance().addEntryf(
                    "[HRClient] Background scan timed out (failure %u), cooldown %u ms",
                    reconnectAttempts_, nextCooldownMs);
                Serial.printf("[HRClient] Background scan timed out (failure %u), cooldown %u ms\n",
                              reconnectAttempts_, nextCooldownMs);
            }
        } else {
            // Unpaired / discovery scan: return to completely Idle state (0% radio airtime)
            internalState_ = HeartRateInternalState::Idle;
            state_.connectionState = HeartRateConnectionState::Disconnected;
        }
    }
    portEXIT_CRITICAL(&mux_);

    if (scanStopMgr != nullptr) {
        if (wasDiscoveryTimeout) {
            scanStopMgr->setScanProfile(BleScanProfile::Background);
        }
        scanStopMgr->stopScan();
    }

    // 4. Connection Handshake (Triggered on Core 0)
    char localTargetAddress[18]{};
    uint8_t localTargetAddressType = 0;
    char localTargetName[32]{};
    int32_t localRssiDbm = 0;
    bool doConnect = false;
    ::NimBLEClient* localClient = nullptr;

    portENTER_CRITICAL(&mux_);
    if (connectPending_ && internalState_ == HeartRateInternalState::Scanning && !isShuttingDown_) {
        connectPending_ = false;
        doConnect = true;
        strncpy(localTargetAddress, pendingConnectAddress_, sizeof(localTargetAddress) - 1);
        localTargetAddressType = pendingConnectAddressType_;
        strncpy(localTargetName, pendingConnectName_, sizeof(localTargetName) - 1);
        localRssiDbm = pendingRssiDbm_;
        localClient = client_;
        internalState_ = HeartRateInternalState::Connecting;
        state_.connectionState = HeartRateConnectionState::Connecting;
        metrics_.connectionAttempts++;
    }
    portEXIT_CRITICAL(&mux_);

    if (doConnect) {
        // Stop active scanning immediately before establishing connection
        BleManager* localScanMgr = bleManager_;
        if (localScanMgr != nullptr) {
            localScanMgr->stopScan();
        }

        if (localClient == nullptr) {
            localClient = NimBLEDevice::createClient();
            portENTER_CRITICAL(&mux_);
            client_ = localClient;
            portEXIT_CRITICAL(&mux_);
        }

        if (localClient == nullptr) {
            portENTER_CRITICAL(&mux_);
            hrChar_ = nullptr;
            internalState_ = HeartRateInternalState::CooldownWait;
            state_.connectionState = HeartRateConnectionState::Failed;
            stateEntryTimestampMs_ = nowMs;
            reconnectAttempts_++;
            metrics_.reconnectAttempts = reconnectAttempts_;
            const uint32_t nextCooldownMs = calculateReconnectCooldownMs();
            stridecontrol::DiagnosticsLog::instance().addEntryf(
                "[HRClient] Client creation failed (failure %u), cooldown %u ms",
                reconnectAttempts_, nextCooldownMs);
            Serial.printf("[HRClient] Client creation failed (failure %u), cooldown %u ms\n",
                          reconnectAttempts_, nextCooldownMs);
            portEXIT_CRITICAL(&mux_);
            return;
        }

        localClient->setClientCallbacks(&s_hrClientCallbackAdapter, false);
        bool ok = localClient->connect(NimBLEAddress(std::string(localTargetAddress), localTargetAddressType), false);
        ::NimBLERemoteCharacteristic* localChar = nullptr;

        if (ok) {
            portENTER_CRITICAL(&mux_);
            if (isShuttingDown_ || !state_.initialized) {
                ok = false;
            }
            portEXIT_CRITICAL(&mux_);

            if (ok) {
                NimBLERemoteService* pSvc = localClient->getService(NimBLEUUID((uint16_t)0x180D));
                if (pSvc != nullptr) {
                    localChar = pSvc->getCharacteristic(NimBLEUUID((uint16_t)0x2A37));
                    if (localChar != nullptr && localChar->canNotify()) {
                        if (localChar->subscribe(true, onHrNotifyTrampoline)) {
                            portENTER_CRITICAL(&mux_);
                            if (!isShuttingDown_ && state_.initialized) {
                                hrChar_ = localChar;
                                internalState_ = HeartRateInternalState::ConnectedStreaming;
                                state_.connectionState = HeartRateConnectionState::Connected;
                                metrics_.successfulConnections++;
                                metrics_.lastRssiDbm = localRssiDbm;
                                strncpy(state_.sensorAddress, localTargetAddress, sizeof(state_.sensorAddress) - 1);
                                state_.sensorAddress[sizeof(state_.sensorAddress) - 1] = '\0';
                                strncpy(state_.sensorName, localTargetName, sizeof(state_.sensorName) - 1);
                                state_.sensorName[sizeof(state_.sensorName) - 1] = '\0';
                                strncpy(state_.connectedAddress, localTargetAddress, sizeof(state_.connectedAddress) - 1);
                                state_.connectedAddress[sizeof(state_.connectedAddress) - 1] = '\0';
                                state_.batteryPercent = 0;
                                state_.batteryPercentValid = false;
                                reconnectAttempts_ = 0;
                                metrics_.reconnectAttempts = 0;
                                stridecontrol::DiagnosticsLog::instance().addEntryf(
                                    "[HRClient] Connected to %s (%s), reset reconnect attempts to 0",
                                    localTargetName, localTargetAddress);
                                Serial.printf("[HRClient] Connected to %s (%s), reset reconnect attempts to 0\n",
                                              localTargetName, localTargetAddress);
                            }
                            portEXIT_CRITICAL(&mux_);

                            // Best-effort connection parameter negotiation for radio coexistence:
                            // HR strap notifies once per second - request longer interval (50-100ms)
                            // and slave latency to minimize BLE radio airtime competing with WiFi.
                            // Units: interval in 1.25ms steps (40=50ms, 80=100ms), timeout in 10ms steps (400=4000ms).
                            localClient->updateConnParams(40, 80, 4, 400);
                            portENTER_CRITICAL(&mux_);
                            connParamsRequestedMs_ = nowMs;
                            connParamsCheckPending_ = true;
                            portEXIT_CRITICAL(&mux_);
                            stridecontrol::DiagnosticsLog::instance().addEntry(
                                "[HRClient] Requested conn params: itvl=50-100ms, latency=4, timeout=4000ms");
                            Serial.println("[HRClient] Requested conn params: itvl=50-100ms, latency=4, timeout=4000ms");

                            return;
                        }
                    }
                }
            }
        }

        localClient->disconnect();
        portENTER_CRITICAL(&mux_);
        hrChar_ = nullptr;
        internalState_ = HeartRateInternalState::CooldownWait;
        state_.connectionState = HeartRateConnectionState::Failed;
        state_.connectedAddress[0] = '\0';
        state_.sensorAddress[0] = '\0';
        state_.sensorName[0] = '\0';
        stateEntryTimestampMs_ = nowMs;
        reconnectAttempts_++;
        metrics_.reconnectAttempts = reconnectAttempts_;
        const uint32_t nextCooldownMs = calculateReconnectCooldownMs();
        stridecontrol::DiagnosticsLog::instance().addEntryf(
            "[HRClient] Connection failed (failure %u), cooldown %u ms",
            reconnectAttempts_, nextCooldownMs);
        Serial.printf("[HRClient] Connection failed (failure %u), cooldown %u ms\n",
                      reconnectAttempts_, nextCooldownMs);
        portEXIT_CRITICAL(&mux_);
    }

    // 5. Cooldown / Retry (Only if a saved paired device is configured)
    BleManager* localMgr = nullptr;
    bool autoScan = false;
    portENTER_CRITICAL(&mux_);
    const uint32_t currentCooldownMs = calculateReconnectCooldownMs();
    if (internalState_ == HeartRateInternalState::CooldownWait &&
        (nowMs - stateEntryTimestampMs_ >= currentCooldownMs)) {
        if (config_.preferredHrMac[0] != '\0' && config_.autoConnectHr) {
            internalState_ = HeartRateInternalState::Scanning;
            state_.connectionState = HeartRateConnectionState::Scanning;
            stateEntryTimestampMs_ = nowMs;
            localMgr = bleManager_;
            autoScan = true;
        } else {
            // No saved target sensor: remain strictly Idle (0% radio airtime)
            internalState_ = HeartRateInternalState::Idle;
            state_.connectionState = HeartRateConnectionState::Disconnected;
            autoScan = false;
        }
    }
    portEXIT_CRITICAL(&mux_);

    if (localMgr != nullptr && autoScan) {
        localMgr->startScan(BleScanProfile::Background);
    }

    // 6. Data Freshness / Timeout
    portENTER_CRITICAL(&mux_);
    if (state_.connectionState == HeartRateConnectionState::Connected) {
        state_.dataAgeMs = (lastDataReceivedMs_ > 0 && nowMs >= lastDataReceivedMs_)
                               ? (nowMs - lastDataReceivedMs_)
                               : UINT32_MAX;
        if (state_.dataAgeMs > HeartRateClientTiming::DATA_TIMEOUT_MS) {
            state_.heartRateValid = false;
        }
    }
    portEXIT_CRITICAL(&mux_);

    // 7. Deferred Connection Parameter Verification (~2.5s post-negotiation)
    ::NimBLEClient* localClientForParams = nullptr;
    portENTER_CRITICAL(&mux_);
    if (connParamsCheckPending_ && (nowMs - connParamsRequestedMs_ >= 2500)) {
        connParamsCheckPending_ = false;
        if (internalState_ == HeartRateInternalState::ConnectedStreaming && !isShuttingDown_) {
            localClientForParams = client_;
        }
    }
    portEXIT_CRITICAL(&mux_);

    if (localClientForParams != nullptr && localClientForParams->isConnected()) {
        NimBLEConnInfo info = localClientForParams->getConnInfo();
        const uint16_t itvl = info.getConnInterval();
        const uint16_t latency = info.getConnLatency();
        const uint16_t timeout = info.getConnTimeout();
        stridecontrol::DiagnosticsLog::instance().addEntryf(
            "[HRClient] Verified conn params (delayed read): itvl=%u (%.2fms), latency=%u, timeout=%ums",
            itvl, itvl * 1.25f, latency, timeout * 10);
        Serial.printf(
            "[HRClient] Verified conn params (delayed read): itvl=%u (%.2fms), latency=%u, timeout=%ums\n",
            itvl, itvl * 1.25f, latency, timeout * 10);
    }
}

HeartRateState HeartRateClient::getState() const {
    portENTER_CRITICAL(&mux_);
    HeartRateState copy = state_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

HeartRateClientMetrics HeartRateClient::getMetrics() const {
    portENTER_CRITICAL(&mux_);
    HeartRateClientMetrics copy = metrics_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

HeartRateInternalState HeartRateClient::getInternalState() const {
    portENTER_CRITICAL(&mux_);
    HeartRateInternalState copy = internalState_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

uint32_t HeartRateClient::getReconnectAttempts() const {
    portENTER_CRITICAL(&mux_);
    uint32_t attempts = reconnectAttempts_;
    portEXIT_CRITICAL(&mux_);
    return attempts;
}

uint32_t HeartRateClient::calculateReconnectCooldownMs() const {
    if (reconnectAttempts_ > HeartRateClientTiming::RECONNECT_FAILURES_TIER2) {
        return HeartRateClientTiming::RECONNECT_COOLDOWN_TIER2_MS;
    }
    if (reconnectAttempts_ > HeartRateClientTiming::RECONNECT_FAILURES_TIER1) {
        return HeartRateClientTiming::RECONNECT_COOLDOWN_TIER1_MS;
    }
    return HeartRateClientTiming::RECONNECT_COOLDOWN_MS;
}

} // namespace stridecontrol
