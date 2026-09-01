#include "BleManager.h"
#include <NimBLEDevice.h>
#include <NimBLEScan.h>
#include <NimBLEAdvertisedDevice.h>
#include <cstdio>
#include <cstring>

namespace stridecontrol {

class BleScanCallbackAdapter;

static portMUX_TYPE s_adapterMux = portMUX_INITIALIZER_UNLOCKED;

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

// Single authoritative static adapter instance
static BleScanCallbackAdapter s_scanCallbackAdapter;

BleManager::BleManager() = default;

BleManager::~BleManager() {
    end();
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
    portEXIT_CRITICAL(&mux_);

    // Bind static adapter owner
    if (!s_scanCallbackAdapter.bindOwner(this)) {
        portENTER_CRITICAL(&mux_);
        isTransitioningLifecycle_ = false;
        state_.state = BleManagerState::Uninitialized;
        internalState_ = BleCoordinatorInternalState::Uninitialized;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    NimBLEDevice::init(std::string(localName));

    scan_ = NimBLEDevice::getScan();
    if (scan_ == nullptr) {
        NimBLEDevice::deinit(true);
        s_scanCallbackAdapter.clearOwner(this);

        portENTER_CRITICAL(&mux_);
        scan_ = nullptr;
        state_.initialized = false;
        state_.state = BleManagerState::Uninitialized;
        internalState_ = BleCoordinatorInternalState::Uninitialized;
        isTransitioningLifecycle_ = false;
        isShuttingDown_ = false;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    scan_->setActiveScan(true);
    scan_->setInterval(100);
    scan_->setWindow(60);
    scan_->setAdvertisedDeviceCallbacks(&s_scanCallbackAdapter, true);

    portENTER_CRITICAL(&mux_);
    state_.initialized = true;
    state_.state = BleManagerState::Ready;
    internalState_ = BleCoordinatorInternalState::Standby;
    metrics_.stackInitCount++;
    isTransitioningLifecycle_ = false;
    portEXIT_CRITICAL(&mux_);

    return true;
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
    ::NimBLEScan* localScan = scan_;
    portEXIT_CRITICAL(&mux_);

    if (localScan != nullptr) {
        localScan->stop();
        localScan->setAdvertisedDeviceCallbacks(nullptr);
    }

    NimBLEDevice::deinit(true);
    s_scanCallbackAdapter.clearOwner(this);

    portENTER_CRITICAL(&mux_);
    scan_ = nullptr;
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

void BleManager::update(uint32_t nowMs) {
    portENTER_CRITICAL(&mux_);
    lastUpdateMs_ = nowMs;
    portEXIT_CRITICAL(&mux_);
}

void BleManager::updateConfig(const BleConfig& config) {
    portENTER_CRITICAL(&mux_);
    config_ = config;
    portEXIT_CRITICAL(&mux_);
}

bool BleManager::startAdvertising() {
    return false; // Stub: Peripheral runtime deferred
}

void BleManager::stopAdvertising() {
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
    bool canStart = state_.initialized && !state_.isScanning && !isShuttingDown_ && (scan_ != nullptr);
    portEXIT_CRITICAL(&mux_);

    if (!canStart) {
        return false;
    }

    bool started = scan_->start(0, nullptr, false);
    if (started) {
        portENTER_CRITICAL(&mux_);
        state_.isScanning = true;
        internalState_ = BleCoordinatorInternalState::ScanningOnly;
        metrics_.scanStartCount++;
        portEXIT_CRITICAL(&mux_);
        return true;
    }

    return false;
}

void BleManager::stopScan() {
    if (scan_ != nullptr) {
        scan_->stop();
    }

    portENTER_CRITICAL(&mux_);
    state_.isScanning = false;
    if (internalState_ == BleCoordinatorInternalState::ScanningOnly) {
        internalState_ = BleCoordinatorInternalState::Standby;
    }
    portEXIT_CRITICAL(&mux_);
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

void BleManager::handleAdvertisedDevice(::NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice == nullptr) {
        return;
    }

    BleScanResult result{};

    // Address Type
    result.addressType = advertisedDevice->getAddressType();

    // MAC Address (Zero-Heap & Endian-Safe)
    const uint8_t* native = advertisedDevice->getAddress().getNative();
    if (native != nullptr) {
        snprintf(result.address, sizeof(result.address),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 native[5], native[4], native[3], native[2], native[1], native[0]);
    }

    // Device Name (Strict Bounds-Checked Raw AD Parsing, Zero std::string)
    result.name[0] = '\0';
    const uint8_t* payload = advertisedDevice->getPayload();
    size_t payloadLen = advertisedDevice->getPayloadLength();
    if (payload != nullptr && payloadLen > 0) {
        size_t index = 0;
        bool hasShortName = false;
        while (index < payloadLen) {
            uint8_t adLen = payload[index];
            if (adLen == 0 || (index + 1 + adLen) > payloadLen) {
                break; // Zero-length or malformed field bounds
            }
            uint8_t adType = payload[index + 1];
            size_t dataLen = adLen - 1;
            const uint8_t* dataPtr = &payload[index + 2];

            if (adType == 0x09) { // Complete Local Name
                size_t copyLen = dataLen < (sizeof(result.name) - 1) ? dataLen : (sizeof(result.name) - 1);
                memcpy(result.name, dataPtr, copyLen);
                result.name[copyLen] = '\0';
                break;
            } else if (adType == 0x08 && !hasShortName && result.name[0] == '\0') { // Shortened Local Name
                size_t copyLen = dataLen < (sizeof(result.name) - 1) ? dataLen : (sizeof(result.name) - 1);
                memcpy(result.name, dataPtr, copyLen);
                result.name[copyLen] = '\0';
                hasShortName = true;
            }

            index += (1 + adLen);
        }
    }

    result.rssiDbm = advertisedDevice->getRSSI();
    result.advertisesHeartRateService = advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0x180D));

    // Listener Dispatch Gate
    ScanListenerEntry localListeners[4]{};
    portENTER_CRITICAL(&mux_);
    if (isShuttingDown_ || !state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
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

} // namespace stridecontrol
