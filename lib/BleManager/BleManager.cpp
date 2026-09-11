#include "BleManager.h"
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEServer.h>
#include <NimBLEAdvertising.h>
#include <NimBLEAdvertisedDevice.h>
#include <esp_timer.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include "../RscService/RscService.h"
#include "../FtmsService/FtmsService.h"

namespace stridecontrol {

static portMUX_TYPE s_adapterMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_serverAdapterMux = portMUX_INITIALIZER_UNLOCKED;

class BleScanCallbackAdapter : public NimBLEAdvertisedDeviceCallbacks {
public:
    BleScanCallbackAdapter() = default;
    ~BleScanCallbackAdapter() override = default;

    bool bindOwner(BleManager* newOwner) {
        portENTER_CRITICAL(&s_adapterMux);
        if (owner_ != nullptr && owner_ != newOwner) {
            portEXIT_CRITICAL(&s_adapterMux);
            return false;
        }
        owner_ = newOwner;
        portEXIT_CRITICAL(&s_adapterMux);
        return true;
    }

    void clearOwner(BleManager* caller) {
        portENTER_CRITICAL(&s_adapterMux);
        if (owner_ == caller) {
            owner_ = nullptr;
        }
        portEXIT_CRITICAL(&s_adapterMux);
    }

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
        if (advertisedDevice == nullptr) {
            return;
        }

        portENTER_CRITICAL(&s_adapterMux);
        BleManager* localOwner = owner_;
        portEXIT_CRITICAL(&s_adapterMux);

        if (localOwner != nullptr) {
            localOwner->handleAdvertisedDevice(advertisedDevice);
        }
    }

private:
    BleManager* owner_{nullptr};
};

class BleServerCallbackAdapter : public NimBLEServerCallbacks {
public:
    BleServerCallbackAdapter() = default;
    ~BleServerCallbackAdapter() override = default;

    bool bindOwner(BleManager* newOwner) {
        portENTER_CRITICAL(&s_serverAdapterMux);
        if (owner_ != nullptr && owner_ != newOwner) {
            portEXIT_CRITICAL(&s_serverAdapterMux);
            return false;
        }
        owner_ = newOwner;
        portEXIT_CRITICAL(&s_serverAdapterMux);
        return true;
    }

    void clearOwner(BleManager* caller) {
        portENTER_CRITICAL(&s_serverAdapterMux);
        if (owner_ == caller) {
            owner_ = nullptr;
        }
        portEXIT_CRITICAL(&s_serverAdapterMux);
    }

    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        (void)pServer;
        portENTER_CRITICAL(&s_serverAdapterMux);
        BleManager* localOwner = owner_;
        portEXIT_CRITICAL(&s_serverAdapterMux);

        if (localOwner != nullptr) {
            localOwner->handleServerConnect(desc);
        }
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        (void)pServer;
        portENTER_CRITICAL(&s_serverAdapterMux);
        BleManager* localOwner = owner_;
        portEXIT_CRITICAL(&s_serverAdapterMux);

        if (localOwner != nullptr) {
            localOwner->handleServerDisconnect(desc);
        }
    }

private:
    BleManager* owner_{nullptr};
};

// Single authoritative static adapter instances
static BleScanCallbackAdapter s_scanCallbackAdapter;
static BleServerCallbackAdapter s_serverCallbackAdapter;

BleManager::BleManager() = default;

BleManager::~BleManager() {
    end();
}

bool BleManager::attachServices(RscService* rsc, FtmsService* ftms) {
    portENTER_CRITICAL(&mux_);
    if (state_.initialized || isTransitioningLifecycle_ || isShuttingDown_) {
        portEXIT_CRITICAL(&mux_);
        return false;
    }
    if (rscService_ != nullptr || ftmsService_ != nullptr) {
        if (rscService_ == rsc && ftmsService_ == ftms) {
            portEXIT_CRITICAL(&mux_);
            return true; // Idempotent re-attachment
        }
        portEXIT_CRITICAL(&mux_);
        return false; // Cannot re-attach different services without resetting
    }
    rscService_ = rsc;
    ftmsService_ = ftms;
    portEXIT_CRITICAL(&mux_);
    return true;
}

bool BleManager::begin(const BleConfig& config) {
    if (memchr(config.advertisedDeviceName, '\0', sizeof(config.advertisedDeviceName)) == nullptr ||
        config.advertisedDeviceName[0] == '\0') {
        return false;
    }

    char localName[32]{};
    strncpy(localName, config.advertisedDeviceName, sizeof(localName) - 1);
    localName[sizeof(localName) - 1] = '\0';

    portENTER_CRITICAL(&mux_);
    if (state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return true; // Idempotent success; active config is preserved
    }
    if (isTransitioningLifecycle_ || isShuttingDown_) {
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    isTransitioningLifecycle_ = true;
    config_ = config;
    for (auto& h : activePeripheralHandles_) {
        h = kInvalidConnectionHandle;
    }
    pendingDisconnectCount_ = 0;
    disconnectResyncPending_ = false;
    advertisingRestartPending_ = false;
    portEXIT_CRITICAL(&mux_);

    // 1. Bind static adapter owners
    if (!s_scanCallbackAdapter.bindOwner(this) || !s_serverCallbackAdapter.bindOwner(this)) {
        s_scanCallbackAdapter.clearOwner(this);
        s_serverCallbackAdapter.clearOwner(this);
        portENTER_CRITICAL(&mux_);
        isTransitioningLifecycle_ = false;
        state_.state = BleManagerState::Uninitialized;
        internalState_ = BleCoordinatorInternalState::Uninitialized;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    // 2. Initialize NimBLE Host Stack
    NimBLEDevice::init(std::string(localName));

    // 3. Create Server for Peripheral GATT Services
    server_ = NimBLEDevice::createServer();
    if (server_ == nullptr) {
        NimBLEDevice::deinit(true);
        s_scanCallbackAdapter.clearOwner(this);
        s_serverCallbackAdapter.clearOwner(this);

        portENTER_CRITICAL(&mux_);
        server_ = nullptr;
        state_.initialized = false;
        state_.state = BleManagerState::Uninitialized;
        internalState_ = BleCoordinatorInternalState::Uninitialized;
        isTransitioningLifecycle_ = false;
        isShuttingDown_ = false;
        portEXIT_CRITICAL(&mux_);
        return false;
    }
    server_->setCallbacks(&s_serverCallbackAdapter, false);

    // 4. Initialize Attached Peripheral GATT Services
    if (rscService_ != nullptr) {
        if (!rscService_->begin(server_)) {
            NimBLEDevice::deinit(true);
            s_scanCallbackAdapter.clearOwner(this);
            s_serverCallbackAdapter.clearOwner(this);

            portENTER_CRITICAL(&mux_);
            server_ = nullptr;
            state_.initialized = false;
            state_.state = BleManagerState::Uninitialized;
            internalState_ = BleCoordinatorInternalState::Uninitialized;
            isTransitioningLifecycle_ = false;
            isShuttingDown_ = false;
            portEXIT_CRITICAL(&mux_);
            return false;
        }
    }

    if (ftmsService_ != nullptr) {
        if (!ftmsService_->begin(server_)) {
            if (rscService_ != nullptr) {
                rscService_->end();
            }
            NimBLEDevice::deinit(true);
            s_scanCallbackAdapter.clearOwner(this);
            s_serverCallbackAdapter.clearOwner(this);

            portENTER_CRITICAL(&mux_);
            server_ = nullptr;
            state_.initialized = false;
            state_.state = BleManagerState::Uninitialized;
            internalState_ = BleCoordinatorInternalState::Uninitialized;
            isTransitioningLifecycle_ = false;
            isShuttingDown_ = false;
            portEXIT_CRITICAL(&mux_);
            return false;
        }
    }

    // 5. Build and Start GAP Advertising with Error Propagation & Full Rollback
    if (!buildAndStartAdvertising()) {
        if (ftmsService_ != nullptr) {
            ftmsService_->end();
        }
        if (rscService_ != nullptr) {
            rscService_->end();
        }
        NimBLEDevice::deinit(true);
        s_scanCallbackAdapter.clearOwner(this);
        s_serverCallbackAdapter.clearOwner(this);

        portENTER_CRITICAL(&mux_);
        scan_ = nullptr;
        server_ = nullptr;
        advertising_ = nullptr;
        for (auto& h : activePeripheralHandles_) {
            h = kInvalidConnectionHandle;
        }
        pendingDisconnectCount_ = 0;
        disconnectResyncPending_ = false;
        advertisingRestartPending_ = false;
        state_ = BleState{};
        metrics_ = BleManagerMetrics{};
        internalState_ = BleCoordinatorInternalState::Uninitialized;
        isTransitioningLifecycle_ = false;
        isShuttingDown_ = false;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    // 6. Configure Central GAP Scanner
    scan_ = NimBLEDevice::getScan();
    if (scan_ == nullptr) {
        if (advertising_ != nullptr && ble_gap_adv_active()) {
            advertising_->stop();
        }
        if (ftmsService_ != nullptr) {
            ftmsService_->end();
        }
        if (rscService_ != nullptr) {
            rscService_->end();
        }
        NimBLEDevice::deinit(true);
        s_scanCallbackAdapter.clearOwner(this);
        s_serverCallbackAdapter.clearOwner(this);

        portENTER_CRITICAL(&mux_);
        scan_ = nullptr;
        server_ = nullptr;
        advertising_ = nullptr;
        for (auto& h : activePeripheralHandles_) {
            h = kInvalidConnectionHandle;
        }
        pendingDisconnectCount_ = 0;
        disconnectResyncPending_ = false;
        advertisingRestartPending_ = false;
        state_ = BleState{};
        metrics_ = BleManagerMetrics{};
        internalState_ = BleCoordinatorInternalState::Uninitialized;
        isTransitioningLifecycle_ = false;
        isShuttingDown_ = false;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    scan_->setActiveScan(true);
    scan_->setInterval(320); // 200 ms (320 * 0.625 ms) for low RF duty cycle
    scan_->setWindow(48);    // 30 ms (48 * 0.625 ms) -> ~15% duty cycle to protect Wi-Fi throughput
    scan_->setAdvertisedDeviceCallbacks(&s_scanCallbackAdapter, true);

    portENTER_CRITICAL(&mux_);
    state_.initialized = true;
    state_.state = BleManagerState::Ready;
    internalState_ = BleCoordinatorInternalState::DualRoleActive;
    metrics_.stackInitCount++;
    isTransitioningLifecycle_ = false;
    portEXIT_CRITICAL(&mux_);

    return true;
}

bool BleManager::buildAndStartAdvertising() {
    advertising_ = NimBLEDevice::getAdvertising();
    if (advertising_ == nullptr) {
        return false;
    }

    NimBLEAdvertisementData advData;
    advData.setFlags(0x06); // LE General Discoverable Mode + BR/EDR Not Supported
    advData.setAppearance(kBleAppearanceGenericRunningWalkingSensor);

    std::vector<NimBLEUUID> serviceUuids;
    if (rscService_ != nullptr) {
        serviceUuids.push_back(NimBLEUUID((uint16_t)0x1814));
    }
    if (ftmsService_ != nullptr) {
        serviceUuids.push_back(NimBLEUUID((uint16_t)0x1826));
    }
    if (!serviceUuids.empty()) {
        advData.setCompleteServices16(serviceUuids);
    }

    NimBLEAdvertisementData scanResponseData;
    size_t nameLen = strnlen(config_.advertisedDeviceName, sizeof(config_.advertisedDeviceName));
    if (nameLen > 0) {
        char boundedName[30]{};
        size_t copyLen = (nameLen > 29) ? 29 : nameLen;
        memcpy(boundedName, config_.advertisedDeviceName, copyLen);
        boundedName[copyLen] = '\0';
        if (nameLen > 29) {
            scanResponseData.setShortName(std::string(boundedName));
        } else {
            scanResponseData.setName(std::string(boundedName));
        }
    }

    advertising_->setAdvertisementData(advData);
    advertising_->setScanResponseData(scanResponseData);

    bool started = advertising_->start();
    if (started) {
        portENTER_CRITICAL(&mux_);
        state_.peripheralAdvertising = true;
        metrics_.advertisingStartCount++;
        portEXIT_CRITICAL(&mux_);
    }
    return started;
}

void BleManager::end() {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || isShuttingDown_ || isTransitioningLifecycle_) {
        portEXIT_CRITICAL(&mux_);
        return; // Idempotent no-op
    }

    isShuttingDown_ = true;
    isTransitioningLifecycle_ = true;
    state_.isScanning = false;
    state_.peripheralAdvertising = false;
    ::NimBLEScan* localScan = scan_;
    ::NimBLEAdvertising* localAdv = advertising_;
    RscService* localRsc = rscService_;
    FtmsService* localFtms = ftmsService_;
    portEXIT_CRITICAL(&mux_);

    // 1. Stop advertising and scanning
    if (localAdv != nullptr) {
        localAdv->stop();
    }
    if (localScan != nullptr) {
        localScan->stop();
        localScan->setAdvertisedDeviceCallbacks(nullptr);
    }

    // 2. Discard/drain pending disconnect mailbox and handle table
    portENTER_CRITICAL(&mux_);
    for (auto& h : activePeripheralHandles_) {
        h = kInvalidConnectionHandle;
    }
    pendingDisconnectCount_ = 0;
    disconnectResyncPending_ = false;
    advertisingRestartPending_ = false;
    portEXIT_CRITICAL(&mux_);

    // 3. Teardown attached services before stack deinit
    if (localRsc != nullptr) {
        localRsc->end();
    }
    if (localFtms != nullptr) {
        localFtms->end();
    }

    // 4. Deinit NimBLEDevice (host port stop + controller deinit barrier)
    NimBLEDevice::deinit(true);

    // 5. Unbind static adapter owners
    s_scanCallbackAdapter.clearOwner(this);
    s_serverCallbackAdapter.clearOwner(this);

    portENTER_CRITICAL(&mux_);
    scan_ = nullptr;
    server_ = nullptr;
    advertising_ = nullptr;
    state_ = BleState{};
    metrics_ = BleManagerMetrics{};
    internalState_ = BleCoordinatorInternalState::Uninitialized;
    isShuttingDown_ = false;
    isTransitioningLifecycle_ = false;
    for (auto& entry : scanListeners_) {
        entry = ScanListenerEntry{};
    }
    portEXIT_CRITICAL(&mux_);
}

void BleManager::handleServerConnect(const ::ble_gap_conn_desc* desc) {
    if (desc == nullptr) {
        return;
    }
    portENTER_CRITICAL(&mux_);
    bool alreadyPresent = false;
    for (auto h : activePeripheralHandles_) {
        if (h == desc->conn_handle) {
            alreadyPresent = true;
            break;
        }
    }
    if (!alreadyPresent) {
        for (auto& h : activePeripheralHandles_) {
            if (h == kInvalidConnectionHandle) {
                h = desc->conn_handle;
                break;
            }
        }
    }
    uint16_t occupied = 0;
    for (auto h : activePeripheralHandles_) {
        if (h != kInvalidConnectionHandle) {
            occupied++;
        }
    }
    metrics_.activePeripheralConnections = occupied;
    state_.peripheralAdvertising = false;
    state_.lastEventTimestampMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    portEXIT_CRITICAL(&mux_);
}

void BleManager::handleServerDisconnect(const ::ble_gap_conn_desc* desc) {
    if (desc == nullptr) {
        return;
    }
    portENTER_CRITICAL(&mux_);
    bool found = false;
    for (auto& h : activePeripheralHandles_) {
        if (h == desc->conn_handle) {
            h = kInvalidConnectionHandle;
            found = true;
            break;
        }
    }
    if (found) {
        uint16_t occupied = 0;
        for (auto h : activePeripheralHandles_) {
            if (h != kInvalidConnectionHandle) {
                occupied++;
            }
        }
        metrics_.activePeripheralConnections = occupied;
        if (pendingDisconnectCount_ < kMaxPendingServerDisconnects) {
            pendingDisconnects_[pendingDisconnectCount_++] = desc->conn_handle;
        } else {
            disconnectResyncPending_ = true;
        }
        advertisingRestartPending_ = true;
        state_.lastEventTimestampMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    }
    portEXIT_CRITICAL(&mux_);
}

void BleManager::update(uint32_t nowMs) {
    uint16_t localDisconnects[kMaxPendingServerDisconnects]{};
    size_t localCount = 0;
    bool restartAdv = false;

    portENTER_CRITICAL(&mux_);
    lastUpdateMs_ = nowMs;
    if (pendingDisconnectCount_ > 0) {
        localCount = pendingDisconnectCount_;
        for (size_t i = 0; i < localCount; ++i) {
            localDisconnects[i] = pendingDisconnects_[i];
        }
        pendingDisconnectCount_ = 0;
    }
    if (disconnectResyncPending_) {
        disconnectResyncPending_ = false;
        metrics_.stackErrorEvents++;
    }
    if (advertisingRestartPending_) {
        restartAdv = true;
        advertisingRestartPending_ = false;
    }
    portEXIT_CRITICAL(&mux_);

    // 1. Drain Disconnect Mailbox to attached services
    if (localCount > 0) {
        for (size_t i = 0; i < localCount; ++i) {
            if (rscService_ != nullptr) {
                rscService_->handleDisconnect(localDisconnects[i]);
            }
            if (ftmsService_ != nullptr) {
                ftmsService_->handleDisconnect(localDisconnects[i]);
            }
        }
    }

    // 2. Shared Capacity Guard & Deferred Advertising Restart on Core 0
    if (restartAdv) {
        portENTER_CRITICAL(&mux_);
        const uint16_t centralCount = state_.centralConnected ? 1U : 0U;
        const uint16_t usedConnections = centralCount + metrics_.activePeripheralConnections;
        const bool canAdvertise = state_.initialized && !isShuttingDown_ &&
                                  (metrics_.activePeripheralConnections < kMaxPeripheralConnections) &&
                                  (usedConnections < kTotalMaxConnections);
        portEXIT_CRITICAL(&mux_);

        if (canAdvertise && advertising_ != nullptr) {
            if (!ble_gap_adv_active()) {
                if (advertising_->start()) {
                    portENTER_CRITICAL(&mux_);
                    state_.peripheralAdvertising = true;
                    metrics_.advertisingStartCount++;
                    portEXIT_CRITICAL(&mux_);
                }
            }
        }
    }

    // 3. Core 0 Exclusive Advertising State Synchronization
    portENTER_CRITICAL(&mux_);
    const bool activeStack = state_.initialized && !isShuttingDown_;
    portEXIT_CRITICAL(&mux_);

    if (activeStack) {
        const bool isAdvActive = ble_gap_adv_active();
        portENTER_CRITICAL(&mux_);
        state_.peripheralAdvertising = isAdvActive;
        portEXIT_CRITICAL(&mux_);
    }
}

void BleManager::updateConfig(const BleConfig& config) {
    portENTER_CRITICAL(&mux_);
    config_ = config;
    portEXIT_CRITICAL(&mux_);
}

bool BleManager::startAdvertising() {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || isShuttingDown_ || isTransitioningLifecycle_) {
        portEXIT_CRITICAL(&mux_);
        return false;
    }
    portEXIT_CRITICAL(&mux_);

    if (advertising_ != nullptr && !ble_gap_adv_active()) {
        if (advertising_->start()) {
            portENTER_CRITICAL(&mux_);
            state_.peripheralAdvertising = true;
            metrics_.advertisingStartCount++;
            portEXIT_CRITICAL(&mux_);
            return true;
        }
    }
    return false;
}

void BleManager::stopAdvertising() {
    if (advertising_ != nullptr && ble_gap_adv_active()) {
        advertising_->stop();
    }
    portENTER_CRITICAL(&mux_);
    state_.peripheralAdvertising = false;
    portEXIT_CRITICAL(&mux_);
}

bool BleManager::registerScanListener(BleScanCallback callback, void* context) {
    if (callback == nullptr) {
        return false;
    }

    portENTER_CRITICAL(&mux_);

    // Check for duplicate callback/context pair
    for (const auto& entry : scanListeners_) {
        if (entry.callback == callback && entry.context == context) {
            portEXIT_CRITICAL(&mux_);
            return false;
        }
    }

    // Find first empty slot
    for (auto& entry : scanListeners_) {
        if (entry.callback == nullptr) {
            entry.callback = callback;
            entry.context = context;
            portEXIT_CRITICAL(&mux_);
            return true;
        }
    }

    // Table is full (max 4 entries)
    portEXIT_CRITICAL(&mux_);
    return false;
}

bool BleManager::unregisterScanListener(BleScanCallback callback, void* context) {
    if (callback == nullptr) {
        return false;
    }

    portENTER_CRITICAL(&mux_);

    // Find exact match
    for (auto& entry : scanListeners_) {
        if (entry.callback == callback && entry.context == context) {
            entry = ScanListenerEntry{};
            portEXIT_CRITICAL(&mux_);
            return true;
        }
    }

    // No exact registration found
    portEXIT_CRITICAL(&mux_);
    return false;
}

bool BleManager::startScan() {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || isShuttingDown_ || isTransitioningLifecycle_) {
        portEXIT_CRITICAL(&mux_);
        return false;
    }
    if (state_.isScanning) {
        portEXIT_CRITICAL(&mux_);
        return true;
    }
    portEXIT_CRITICAL(&mux_);

    if (scan_ == nullptr) {
        return false;
    }

    bool started = scan_->start(0, nullptr, false);
    if (started) {
        portENTER_CRITICAL(&mux_);
        state_.isScanning = true;
        metrics_.scanStartCount++;
        portEXIT_CRITICAL(&mux_);
    }
    return started;
}

void BleManager::stopScan() {
    portENTER_CRITICAL(&mux_);
    if (!state_.isScanning) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    state_.isScanning = false;
    ::NimBLEScan* localScan = scan_;
    portEXIT_CRITICAL(&mux_);

    if (localScan != nullptr) {
        localScan->stop();
    }
}

void BleManager::handleAdvertisedDevice(::NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice == nullptr) {
        return;
    }

    portENTER_CRITICAL(&mux_);
    if (isShuttingDown_ || !state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    portEXIT_CRITICAL(&mux_);

    BleScanResult result{};
    result.addressType = advertisedDevice->getAddressType();

    const NimBLEAddress& addr = advertisedDevice->getAddress();
    const uint8_t* rawAddr = addr.getNative();
    snprintf(result.address, sizeof(result.address), "%02X:%02X:%02X:%02X:%02X:%02X",
             rawAddr[5], rawAddr[4], rawAddr[3], rawAddr[2], rawAddr[1], rawAddr[0]);

    result.rssiDbm = advertisedDevice->getRSSI();

    const uint8_t* payload = advertisedDevice->getPayload();
    size_t payloadLen = advertisedDevice->getPayloadLength();

    if (payload != nullptr && payloadLen > 0) {
        size_t offset = 0;
        while (offset + 1 < payloadLen) {
            uint8_t adLen = payload[offset];
            if (adLen == 0) {
                break;
            }
            if (offset + 1 + adLen > payloadLen) {
                break; // Corrupt/truncated AD element; abort parsing
            }

            uint8_t adType = payload[offset + 1];
            const uint8_t* adData = &payload[offset + 2];
            size_t adDataLen = adLen - 1;

            if (adType == 0x08 || adType == 0x09) {
                size_t copyLen = (adDataLen < sizeof(result.name) - 1) ? adDataLen : (sizeof(result.name) - 1);
                memcpy(result.name, adData, copyLen);
                result.name[copyLen] = '\0';
            } else if (adType == 0x02 || adType == 0x03) {
                for (size_t i = 0; i + 1 < adDataLen; i += 2) {
                    uint16_t uuid16 = static_cast<uint16_t>(adData[i]) | (static_cast<uint16_t>(adData[i + 1]) << 8);
                    if (uuid16 == 0x180D) {
                        result.advertisesHeartRateService = true;
                    }
                }
            } else if (adType == 0x06 || adType == 0x07) {
                static const uint8_t kHrUuid128[16] = {
                    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                    0x00, 0x10, 0x00, 0x00, 0x0D, 0x18, 0x00, 0x00
                };
                for (size_t i = 0; i + 15 < adDataLen; i += 16) {
                    if (memcmp(&adData[i], kHrUuid128, 16) == 0) {
                        result.advertisesHeartRateService = true;
                    }
                }
            }

            offset += (1 + adLen);
        }
    }

    ScanListenerEntry localListeners[4]{};
    portENTER_CRITICAL(&mux_);
    for (size_t i = 0; i < 4; ++i) {
        localListeners[i] = scanListeners_[i];
    }
    portEXIT_CRITICAL(&mux_);

    for (const auto& entry : localListeners) {
        if (entry.callback != nullptr) {
            entry.callback(result, entry.context);
        }
    }
}

BleState BleManager::getState() const {
    portENTER_CRITICAL(&mux_);
    BleState copy = state_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

BleManagerMetrics BleManager::getMetrics() const {
    portENTER_CRITICAL(&mux_);
    BleManagerMetrics copy = metrics_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

BleCoordinatorInternalState BleManager::getInternalState() const {
    portENTER_CRITICAL(&mux_);
    BleCoordinatorInternalState copy = internalState_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

} // namespace stridecontrol
