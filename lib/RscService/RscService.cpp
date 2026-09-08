#include "RscService.h"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEService.h>
#include <NimBLECharacteristic.h>
#include <cmath>
#include <cstring>

namespace stridecontrol {

static portMUX_TYPE s_rscAdapterMux = portMUX_INITIALIZER_UNLOCKED;

class RscMeasurementCallbackAdapter : public NimBLECharacteristicCallbacks {
public:
    RscMeasurementCallbackAdapter() = default;
    ~RscMeasurementCallbackAdapter() override = default;

    bool bindOwner(RscService* newOwner) {
        portENTER_CRITICAL(&s_rscAdapterMux);
        if (owner_ != nullptr && owner_ != newOwner) {
            portEXIT_CRITICAL(&s_rscAdapterMux);
            return false;
        }
        owner_ = newOwner;
        portEXIT_CRITICAL(&s_rscAdapterMux);
        return true;
    }

    void clearOwner(RscService* caller) {
        portENTER_CRITICAL(&s_rscAdapterMux);
        if (owner_ == caller) {
            owner_ = nullptr;
        }
        portEXIT_CRITICAL(&s_rscAdapterMux);
    }

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override {
        (void)pCharacteristic;
        portENTER_CRITICAL(&s_rscAdapterMux);
        RscService* localOwner = owner_;
        portEXIT_CRITICAL(&s_rscAdapterMux);

        if (localOwner != nullptr && desc != nullptr) {
            localOwner->handleSubscribe(desc->conn_handle, subValue);
        }
    }

private:
    RscService* owner_{nullptr};
};

static RscMeasurementCallbackAdapter s_rscCallbackAdapter;

static inline uint16_t saturatingRoundToUint16(float value, float minVal = 0.0f, float maxVal = 65535.0f) {
    if (!std::isfinite(value) || value <= minVal) return static_cast<uint16_t>(minVal);
    if (value >= maxVal) return static_cast<uint16_t>(maxVal);
    return static_cast<uint16_t>(lroundf(value));
}

static inline uint8_t saturatingRoundToUint8(float value, float minVal = 0.0f, float maxVal = 255.0f) {
    if (!std::isfinite(value) || value <= minVal) return static_cast<uint8_t>(minVal);
    if (value >= maxVal) return static_cast<uint8_t>(maxVal);
    return static_cast<uint8_t>(lroundf(value));
}

static inline uint32_t saturatingRoundToUint32(double value) {
    if (!std::isfinite(value) || value <= 0.0) return 0U;
    if (value >= 4294967295.0) return 4294967295U;
    return static_cast<uint32_t>(llround(value));
}

RscService::RscService() = default;

RscService::~RscService() {
    end();
}

bool RscService::begin(::NimBLEServer* pServer) {
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

    if (!s_rscCallbackAdapter.bindOwner(this)) {
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    // 1. Create or retrieve primary RSC Service (0x1814)
    pService_ = pServer_->createService(NimBLEUUID((uint16_t)0x1814));
    if (pService_ == nullptr) {
        pService_ = pServer_->getServiceByUUID(NimBLEUUID((uint16_t)0x1814));
    }
    if (pService_ == nullptr) {
        s_rscCallbackAdapter.clearOwner(this);
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    // 2. Create RSC Measurement Characteristic (0x2A53, Notify)
    pMeasurementChar_ = pService_->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A53),
        NIMBLE_PROPERTY::NOTIFY
    );
    if (pMeasurementChar_ == nullptr) {
        s_rscCallbackAdapter.clearOwner(this);
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        pService_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }
    pMeasurementChar_->setCallbacks(&s_rscCallbackAdapter);

    // 3. Create RSC Feature Characteristic (0x2A54, Read)
    pFeatureChar_ = pService_->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A54),
        NIMBLE_PROPERTY::READ
    );
    if (pFeatureChar_ != nullptr) {
        const uint8_t featureBytes[2] = {
            static_cast<uint8_t>(kRscFeatureTotalDistanceAndStatus & 0xFF),
            static_cast<uint8_t>((kRscFeatureTotalDistanceAndStatus >> 8) & 0xFF)
        };
        pFeatureChar_->setValue(featureBytes, sizeof(featureBytes));
    }

    // 4. Create Sensor Location Characteristic (0x2A5D, Read)
    pSensorLocationChar_ = pService_->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A5D),
        NIMBLE_PROPERTY::READ
    );
    if (pSensorLocationChar_ != nullptr) {
        const uint8_t locationByte = kRscSensorLocationOther;
        pSensorLocationChar_->setValue(&locationByte, sizeof(locationByte));
    }

    // 5. Start GATT Service
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

void RscService::end() {
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

    s_rscCallbackAdapter.clearOwner(this);

    portENTER_CRITICAL(&mux_);
    pServer_ = nullptr;
    pService_ = nullptr;
    pMeasurementChar_ = nullptr;
    pFeatureChar_ = nullptr;
    pSensorLocationChar_ = nullptr;
    lastNotificationMs_ = 0;
    portEXIT_CRITICAL(&mux_);
}

void RscService::handleSubscribe(uint16_t connectionHandle, uint16_t subValue) {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return;
    }

    // Notification active if Bit 0 (0x01) is set
    bool active = (subValue & 0x0001) != 0;

    if (active) {
        // Update existing entry or find first empty slot
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
        // Deactivate matching subscription
        for (auto& sub : subscribers_) {
            if (sub.connectionHandle == connectionHandle) {
                sub = BleSubscriptionEntry{};
            }
        }
    }

    // Recalculate hasMeasurementSubscribers
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

void RscService::handleDisconnect(uint16_t connectionHandle) {
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

size_t RscService::packRscMeasurement(const ApplicationSnapshot& snapshot, uint8_t* outPayload, size_t maxLen) const {
    if (outPayload == nullptr || maxLen < 4) {
        return 0;
    }

    uint8_t flags = 0;

    // Bit 2: Walking/Running status (0 = Walking/Inactive, 1 = Running)
    if (snapshot.runner.activity == RunnerActivity::Running) {
        flags |= (1U << 2);
    }

    // Instantaneous Speed: 1/256 m/s (Runner-Qualified)
    uint16_t rawSpeed = 0;
    const bool exportSpeedValid = snapshot.runner.runnerSpeedValid &&
                                  std::isfinite(snapshot.runner.runnerSpeedKmh) &&
                                  (snapshot.runner.runnerSpeedKmh >= 0.0f);
    const float exportSpeedKmh = exportSpeedValid ? static_cast<float>(snapshot.runner.runnerSpeedKmh) : 0.0f;
    const float speedMps = exportSpeedKmh / 3.6f;
    rawSpeed = saturatingRoundToUint16(speedMps * 256.0f);

    // Instantaneous Cadence: SPM
    uint8_t rawCadence = 0;
    if (snapshot.runner.cadenceValid && std::isfinite(snapshot.runner.smoothedCadenceSpm) && snapshot.runner.smoothedCadenceSpm >= 0.0f) {
        rawCadence = saturatingRoundToUint8(snapshot.runner.smoothedCadenceSpm);
    }

    // Dynamic Total Distance: 0.1 m
    bool hasDistance = false;
    uint32_t rawDistanceDm = 0;
    if (snapshot.runner.distanceValid && std::isfinite(snapshot.runner.validatedDistanceKm) && snapshot.runner.validatedDistanceKm >= 0.0) {
        if (maxLen >= 8) {
            hasDistance = true;
            flags |= (1U << 1);
            double distDm = snapshot.runner.validatedDistanceKm * 10000.0;
            rawDistanceDm = saturatingRoundToUint32(distDm);
        }
    }

    outPayload[0] = flags;
    outPayload[1] = static_cast<uint8_t>(rawSpeed & 0xFF);
    outPayload[2] = static_cast<uint8_t>((rawSpeed >> 8) & 0xFF);
    outPayload[3] = rawCadence;

    if (hasDistance) {
        outPayload[4] = static_cast<uint8_t>(rawDistanceDm & 0xFF);
        outPayload[5] = static_cast<uint8_t>((rawDistanceDm >> 8) & 0xFF);
        outPayload[6] = static_cast<uint8_t>((rawDistanceDm >> 16) & 0xFF);
        outPayload[7] = static_cast<uint8_t>((rawDistanceDm >> 24) & 0xFF);
        return 8;
    }

    return 4;
}

void RscService::update(uint32_t nowMs, const ApplicationSnapshot& snapshot) {
    ::NimBLECharacteristic* localChar = nullptr;
    bool shouldNotify = false;

    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || !state_.hasMeasurementSubscribers) {
        portEXIT_CRITICAL(&mux_);
        return;
    }

    if (static_cast<uint32_t>(nowMs - lastNotificationMs_) >= kRscNotificationIntervalMs) {
        shouldNotify = true;
        localChar = pMeasurementChar_;
        lastNotificationMs_ = nowMs;
        state_.lastNotificationMs = nowMs;
    }
    portEXIT_CRITICAL(&mux_);

    if (shouldNotify && localChar != nullptr) {
        uint8_t payload[10]{};
        size_t len = packRscMeasurement(snapshot, payload, sizeof(payload));
        if (len >= 4) {
            localChar->setValue(payload, len);
            localChar->notify(true);
        }
    }
}

RscServerStateDto RscService::getState() const {
    portENTER_CRITICAL(&mux_);
    RscServerStateDto copy = state_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

} // namespace stridecontrol

