#include "HeartRateService.h"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEService.h>
#include <NimBLECharacteristic.h>
#include <cmath>
#include <cstring>

namespace stridecontrol {

static portMUX_TYPE s_hrAdapterMux = portMUX_INITIALIZER_UNLOCKED;

class HeartRateMeasurementCallbackAdapter : public NimBLECharacteristicCallbacks {
public:
    HeartRateMeasurementCallbackAdapter() = default;
    ~HeartRateMeasurementCallbackAdapter() override = default;

    bool bindOwner(HeartRateService* newOwner) {
        portENTER_CRITICAL(&s_hrAdapterMux);
        if (owner_ != nullptr && owner_ != newOwner) {
            portEXIT_CRITICAL(&s_hrAdapterMux);
            return false;
        }
        owner_ = newOwner;
        portEXIT_CRITICAL(&s_hrAdapterMux);
        return true;
    }

    void clearOwner(HeartRateService* caller) {
        portENTER_CRITICAL(&s_hrAdapterMux);
        if (owner_ == caller) {
            owner_ = nullptr;
        }
        portEXIT_CRITICAL(&s_hrAdapterMux);
    }

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override {
        (void)pCharacteristic;
        portENTER_CRITICAL(&s_hrAdapterMux);
        HeartRateService* localOwner = owner_;
        portEXIT_CRITICAL(&s_hrAdapterMux);

        if (localOwner != nullptr && desc != nullptr) {
            localOwner->handleSubscribe(desc->conn_handle, subValue);
        }
    }

private:
    HeartRateService* owner_{nullptr};
};

static HeartRateMeasurementCallbackAdapter s_hrCallbackAdapter;

HeartRateService::HeartRateService() = default;

HeartRateService::~HeartRateService() {
    end();
}

bool HeartRateService::begin(::NimBLEServer* pServer) {
    if (pServer == nullptr) {
        return false;
    }

    portENTER_CRITICAL(&mux_);
    if (state_.initialized) {
        if (pServer_ == pServer) {
            portEXIT_CRITICAL(&mux_);
            return true; // Idempotent success
        }
        portEXIT_CRITICAL(&mux_);
        return false; // Cannot rebind active service to different server
    }
    pServer_ = pServer;
    portEXIT_CRITICAL(&mux_);

    if (!s_hrCallbackAdapter.bindOwner(this)) {
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    // 1. Create or retrieve primary Heart Rate Service (0x180D)
    pService_ = pServer_->createService(NimBLEUUID((uint16_t)0x180D));
    if (pService_ == nullptr) {
        pService_ = pServer_->getServiceByUUID(NimBLEUUID((uint16_t)0x180D));
    }
    if (pService_ == nullptr) {
        s_hrCallbackAdapter.clearOwner(this);
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    // 2. Create Heart Rate Measurement Characteristic (0x2A37, Notify only)
    pMeasurementChar_ = pService_->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A37),
        NIMBLE_PROPERTY::NOTIFY
    );
    if (pMeasurementChar_ == nullptr) {
        s_hrCallbackAdapter.clearOwner(this);
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        pService_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }
    pMeasurementChar_->setCallbacks(&s_hrCallbackAdapter);

    // 3. Start GATT Service
    pService_->start();

    portENTER_CRITICAL(&mux_);
    state_.initialized = true;
    state_.hasMeasurementSubscribers = false;
    state_.lastNotificationMs = 0;
    lastNotificationMs_ = 0;
    for (auto& sub : subscribers_) {
        sub = BleSubscriptionEntry{};
    }
    portEXIT_CRITICAL(&mux_);

    return true;
}

void HeartRateService::end() {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    state_.initialized = false;
    state_.hasMeasurementSubscribers = false;
    for (auto& sub : subscribers_) {
        sub = BleSubscriptionEntry{};
    }
    portEXIT_CRITICAL(&mux_);

    s_hrCallbackAdapter.clearOwner(this);

    portENTER_CRITICAL(&mux_);
    pServer_ = nullptr;
    pService_ = nullptr;
    pMeasurementChar_ = nullptr;
    lastNotificationMs_ = 0;
    portEXIT_CRITICAL(&mux_);
}

void HeartRateService::handleSubscribe(uint16_t connectionHandle, uint16_t subValue) {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return;
    }

    // Notification active if Bit 0 (0x01) is set
    bool active = (subValue & 0x0001) != 0;

    if (active) {
        bool found = false;
        for (auto& sub : subscribers_) {
            if (sub.active && sub.connectionHandle == connectionHandle) {
                found = true;
                break;
            }
        }
        if (!found) {
            for (auto& sub : subscribers_) {
                if (!sub.active) {
                    sub.connectionHandle = connectionHandle;
                    sub.active = true;
                    found = true;
                    break;
                }
            }
        }
    } else {
        for (auto& sub : subscribers_) {
            if (sub.connectionHandle == connectionHandle) {
                sub = BleSubscriptionEntry{};
            }
        }
    }

    bool hasSub = false;
    for (const auto& sub : subscribers_) {
        if (sub.active) {
            hasSub = true;
            break;
        }
    }
    state_.hasMeasurementSubscribers = hasSub;
    portEXIT_CRITICAL(&mux_);
}

void HeartRateService::handleDisconnect(uint16_t connectionHandle) {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return;
    }

    for (auto& sub : subscribers_) {
        if (sub.connectionHandle == connectionHandle) {
            sub = BleSubscriptionEntry{};
        }
    }

    bool hasSub = false;
    for (const auto& sub : subscribers_) {
        if (sub.active) {
            hasSub = true;
            break;
        }
    }
    state_.hasMeasurementSubscribers = hasSub;
    portEXIT_CRITICAL(&mux_);
}

size_t HeartRateService::packHeartRateMeasurement(const ApplicationSnapshot& snapshot, uint8_t* outPayload, size_t maxLen) const {
    if (outPayload == nullptr || maxLen < 2) {
        return 0;
    }
    if (!snapshot.heartRate.heartRateValid || snapshot.heartRate.heartRateBpm < 1 || snapshot.heartRate.heartRateBpm > 255) {
        return 0; // Do not notify/update when no valid reading exists
    }
    outPayload[0] = 0x00; // Flags: UINT8 format, no other fields
    outPayload[1] = static_cast<uint8_t>(snapshot.heartRate.heartRateBpm);
    return 2;
}

void HeartRateService::update(uint32_t nowMs, const ApplicationSnapshot& snapshot) {
    ::NimBLECharacteristic* localChar = nullptr;
    bool shouldNotify = false;

    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || !state_.hasMeasurementSubscribers) {
        portEXIT_CRITICAL(&mux_);
        return;
    }

    if (static_cast<uint32_t>(nowMs - lastNotificationMs_) >= kHrNotificationIntervalMs) {
        shouldNotify = true;
        localChar = pMeasurementChar_;
        lastNotificationMs_ = nowMs;
        state_.lastNotificationMs = nowMs;
    }
    portEXIT_CRITICAL(&mux_);

    if (shouldNotify && localChar != nullptr) {
        uint8_t payload[2]{};
        size_t len = packHeartRateMeasurement(snapshot, payload, sizeof(payload));
        if (len >= 2) {
            localChar->setValue(payload, len);
            localChar->notify(true);
        }
    }
}

HeartRateServerStateDto HeartRateService::getState() const {
    portENTER_CRITICAL(&mux_);
    HeartRateServerStateDto copy = state_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

} // namespace stridecontrol

