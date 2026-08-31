#include "HeartRateClient.h"

namespace stridecontrol {

HeartRateClient::HeartRateClient() = default;
HeartRateClient::~HeartRateClient() = default;

bool HeartRateClient::begin(const BleConfig& config) {
    config_ = config;
    return true;
}

void HeartRateClient::end() {
    // Stub: To be implemented with NimBLE lifecycle
}

void HeartRateClient::update(uint32_t nowMs) {
    lastUpdateMs_ = nowMs;
}

void HeartRateClient::updateConfig(const BleConfig& config) {
    config_ = config;
}

void HeartRateClient::startScan() {
    // Stub: To be implemented with NimBLE scanner
}

void HeartRateClient::stopScan() {
    // Stub: To be implemented with NimBLE scanner
}

void HeartRateClient::disconnect() {
    // Stub: To be implemented with NimBLE client
}

HeartRateState HeartRateClient::getState() const {
    return state_;
}

HeartRateClientMetrics HeartRateClient::getMetrics() const {
    return metrics_;
}

HeartRateInternalState HeartRateClient::getInternalState() const {
    return internalState_;
}

} // namespace stridecontrol
