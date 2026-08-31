#include "BleManager.h"

namespace stridecontrol {

BleManager::BleManager() = default;
BleManager::~BleManager() = default;

bool BleManager::begin(const BleConfig& config) {
    portENTER_CRITICAL(&mux_);
    config_ = config;
    internalState_ = BleCoordinatorInternalState::Uninitialized;
    state_.initialized = false;
    portEXIT_CRITICAL(&mux_);
    return false; // Stub: Adapter not initialized yet
}

void BleManager::end() {
    portENTER_CRITICAL(&mux_);
    scan_ = nullptr;
    state_ = BleState{};
    metrics_ = BleManagerMetrics{};
    internalState_ = BleCoordinatorInternalState::Uninitialized;
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
    return false; // Stub: Scanning not active
}

void BleManager::stopScan() {
    portENTER_CRITICAL(&mux_);
    state_.isScanning = false;
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

} // namespace stridecontrol
