#include "FtmsService.h"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEService.h>
#include <NimBLECharacteristic.h>
#include <cmath>
#include <cstring>

namespace stridecontrol {

static portMUX_TYPE s_ftmsAdapterMux = portMUX_INITIALIZER_UNLOCKED;

class FtmsTreadmillCallbackAdapter : public NimBLECharacteristicCallbacks {
public:
    FtmsTreadmillCallbackAdapter() = default;
    ~FtmsTreadmillCallbackAdapter() override = default;

    bool bindOwner(FtmsService* newOwner) {
        portENTER_CRITICAL(&s_ftmsAdapterMux);
        if (owner_ != nullptr && owner_ != newOwner) {
            portEXIT_CRITICAL(&s_ftmsAdapterMux);
            return false;
        }
        owner_ = newOwner;
        portEXIT_CRITICAL(&s_ftmsAdapterMux);
        return true;
    }

    void clearOwner(FtmsService* caller) {
        portENTER_CRITICAL(&s_ftmsAdapterMux);
        if (owner_ == caller) {
            owner_ = nullptr;
        }
        portEXIT_CRITICAL(&s_ftmsAdapterMux);
    }

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override {
        (void)pCharacteristic;
        portENTER_CRITICAL(&s_ftmsAdapterMux);
        FtmsService* localOwner = owner_;
        portEXIT_CRITICAL(&s_ftmsAdapterMux);

        if (localOwner != nullptr && desc != nullptr) {
            localOwner->handleSubscribe(desc->conn_handle, subValue);
        }
    }

private:
    FtmsService* owner_{nullptr};
};

static FtmsTreadmillCallbackAdapter s_ftmsCallbackAdapter;

static inline uint16_t saturatingRoundToUint16(float value, float minVal = 0.0f, float maxVal = 65535.0f) {
    if (!std::isfinite(value) || value <= minVal) return static_cast<uint16_t>(minVal);
    if (value >= maxVal) return static_cast<uint16_t>(maxVal);
    return static_cast<uint16_t>(lroundf(value));
}

static inline uint32_t saturatingRoundToUint24(double value) {
    if (!std::isfinite(value) || value <= 0.0) return 0U;
    if (value >= 16777215.0) return 16777215U;
    return static_cast<uint32_t>(llround(value));
}

static inline int16_t saturatingRoundToInt16(float value, float minVal = -32768.0f, float maxVal = 32767.0f) {
    if (!std::isfinite(value)) return 0;
    if (value <= minVal) return static_cast<int16_t>(minVal);
    if (value >= maxVal) return static_cast<int16_t>(maxVal);
    return static_cast<int16_t>(lroundf(value));
}

FtmsService::FtmsService() = default;

FtmsService::~FtmsService() {
    end();
}

bool FtmsService::begin(::NimBLEServer* pServer) {
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

    if (!s_ftmsCallbackAdapter.bindOwner(this)) {
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    // 1. Create or retrieve primary FTMS Service (0x1826)
    pService_ = pServer_->createService(NimBLEUUID((uint16_t)0x1826));
    if (pService_ == nullptr) {
        pService_ = pServer_->getServiceByUUID(NimBLEUUID((uint16_t)0x1826));
    }
    if (pService_ == nullptr) {
        s_ftmsCallbackAdapter.clearOwner(this);
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }

    // 2. Create Treadmill Data Characteristic (0x2ACD, Notify)
    pTreadmillDataChar_ = pService_->createCharacteristic(
        NimBLEUUID((uint16_t)0x2ACD),
        NIMBLE_PROPERTY::NOTIFY
    );
    if (pTreadmillDataChar_ == nullptr) {
        s_ftmsCallbackAdapter.clearOwner(this);
        portENTER_CRITICAL(&mux_);
        pServer_ = nullptr;
        pService_ = nullptr;
        portEXIT_CRITICAL(&mux_);
        return false;
    }
    pTreadmillDataChar_->setCallbacks(&s_ftmsCallbackAdapter);

    // 3. Create Fitness Machine Feature Characteristic (0x2ACC, Read)
    pFeatureChar_ = pService_->createCharacteristic(
        NimBLEUUID((uint16_t)0x2ACC),
        NIMBLE_PROPERTY::READ
    );
    if (pFeatureChar_ != nullptr) {
        const uint8_t featureBytes[8] = {
            static_cast<uint8_t>(kFtmsMachineFeatures & 0xFF),
            static_cast<uint8_t>((kFtmsMachineFeatures >> 8) & 0xFF),
            static_cast<uint8_t>((kFtmsMachineFeatures >> 16) & 0xFF),
            static_cast<uint8_t>((kFtmsMachineFeatures >> 24) & 0xFF),
            static_cast<uint8_t>(kFtmsTargetSettingFeatures & 0xFF),
            static_cast<uint8_t>((kFtmsTargetSettingFeatures >> 8) & 0xFF),
            static_cast<uint8_t>((kFtmsTargetSettingFeatures >> 16) & 0xFF),
            static_cast<uint8_t>((kFtmsTargetSettingFeatures >> 24) & 0xFF)
        };
        pFeatureChar_->setValue(featureBytes, sizeof(featureBytes));
    }

    // 4. Start GATT Service
    pService_->start();

    portENTER_CRITICAL(&mux_);
    state_.initialized = true;
    state_.hasTreadmillDataSubscribers = false;
    state_.lastNotificationMs = 0;
    lastNotificationMs_ = 0;
    for (auto& sub : subscribers_) {
        sub = BleSubscriptionEntry{};
    }
    portEXIT_CRITICAL(&mux_);

    return true;
}

void FtmsService::end() {
    portENTER_CRITICAL(&mux_);
    if (!state_.initialized) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    state_.initialized = false;
    state_.hasTreadmillDataSubscribers = false;
    for (auto& sub : subscribers_) {
        sub = BleSubscriptionEntry{};
    }
    portEXIT_CRITICAL(&mux_);

    s_ftmsCallbackAdapter.clearOwner(this);

    portENTER_CRITICAL(&mux_);
    pServer_ = nullptr;
    pService_ = nullptr;
    pTreadmillDataChar_ = nullptr;
    pFeatureChar_ = nullptr;
    lastNotificationMs_ = 0;
    portEXIT_CRITICAL(&mux_);
}

void FtmsService::handleSubscribe(uint16_t connectionHandle, uint16_t subValue) {
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

    // Recalculate hasTreadmillDataSubscribers
    bool hasSub = false;
    for (const auto& sub : subscribers_) {
        if (sub.active) {
            hasSub = true;
            break;
        }
    }
    state_.hasTreadmillDataSubscribers = hasSub;
    portEXIT_CRITICAL(&mux_);
}

void FtmsService::handleDisconnect(uint16_t connectionHandle) {
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
    state_.hasTreadmillDataSubscribers = hasSub;
    portEXIT_CRITICAL(&mux_);
}

size_t FtmsService::packTreadmillData(const ApplicationSnapshot& snapshot, uint8_t* outPayload, size_t maxLen) const {
    if (outPayload == nullptr || maxLen < 4) {
        return 0;
    }

    uint16_t flags = 0; // Bit 0 = 0 (Instantaneous Speed Present, Mandatory)

    // Instantaneous Speed (uint16_t, 0.01 km/h) (Runner-Qualified)
    uint16_t rawSpeed = 0;
    const bool exportSpeedValid = snapshot.runner.runnerSpeedValid &&
                                  std::isfinite(snapshot.runner.runnerSpeedKmh) &&
                                  (snapshot.runner.runnerSpeedKmh >= 0.0f);
    const float exportSpeedKmh = exportSpeedValid ? static_cast<float>(snapshot.runner.runnerSpeedKmh) : 0.0f;
    rawSpeed = saturatingRoundToUint16(exportSpeedKmh * 100.0f);

    // Bit 2: Total Distance (uint24_t, 1 meter)
    bool hasDistance = false;
    uint32_t rawDistanceM = 0;
    if (snapshot.runner.distanceValid && std::isfinite(snapshot.runner.validatedDistanceKm) && snapshot.runner.validatedDistanceKm >= 0.0) {
        hasDistance = true;
        flags |= (1U << 2);
        double distMeters = snapshot.runner.validatedDistanceKm * 1000.0;
        rawDistanceM = saturatingRoundToUint24(distMeters);
    }

    // Bit 3: Inclination & Ramp Angle
    bool hasIncline = false;
    int16_t rawIncline = 0;
    int16_t rawRampAngle = 0;
    if (snapshot.incline.positionTrusted && std::isfinite(snapshot.incline.estimatedInclinePct)) {
        hasIncline = true;
        flags |= (1U << 3);
        rawIncline = saturatingRoundToInt16(snapshot.incline.estimatedInclinePct * 10.0f, -750.0f, 750.0f);
        float grade = snapshot.incline.estimatedInclinePct;
        if (grade < -75.0f) grade = -75.0f;
        if (grade > 75.0f) grade = 75.0f;
        float angleDeg = atanf(grade / 100.0f) * (180.0f / 3.14159265358979323846f);
        rawRampAngle = saturatingRoundToInt16(angleDeg * 10.0f, -750.0f, 750.0f);
    }

    // Bit 8: Heart Rate (uint8_t, BPM)
    bool hasHeartRate = false;
    uint8_t rawHeartRate = 0;
    if (snapshot.heartRate.heartRateValid && snapshot.heartRate.heartRateBpm >= 1 && snapshot.heartRate.heartRateBpm <= 255) {
        hasHeartRate = true;
        flags |= (1U << 8);
        rawHeartRate = snapshot.heartRate.heartRateBpm;
    }

    size_t requiredLen = 4 + (hasDistance ? 3 : 0) + (hasIncline ? 4 : 0) + (hasHeartRate ? 1 : 0);
    if (maxLen < requiredLen) {
        return 0;
    }

    size_t cursor = 0;
    // Flags (2 bytes, little-endian)
    outPayload[cursor++] = static_cast<uint8_t>(flags & 0xFF);
    outPayload[cursor++] = static_cast<uint8_t>((flags >> 8) & 0xFF);

    // Speed (2 bytes, little-endian)
    outPayload[cursor++] = static_cast<uint8_t>(rawSpeed & 0xFF);
    outPayload[cursor++] = static_cast<uint8_t>((rawSpeed >> 8) & 0xFF);

    // Distance (3 bytes, uint24_t, little-endian)
    if (hasDistance) {
        outPayload[cursor++] = static_cast<uint8_t>(rawDistanceM & 0xFF);
        outPayload[cursor++] = static_cast<uint8_t>((rawDistanceM >> 8) & 0xFF);
        outPayload[cursor++] = static_cast<uint8_t>((rawDistanceM >> 16) & 0xFF);
    }

    // Inclination (2 bytes) & Ramp Angle (2 bytes)
    if (hasIncline) {
        outPayload[cursor++] = static_cast<uint8_t>(rawIncline & 0xFF);
        outPayload[cursor++] = static_cast<uint8_t>((rawIncline >> 8) & 0xFF);
        outPayload[cursor++] = static_cast<uint8_t>(rawRampAngle & 0xFF);
        outPayload[cursor++] = static_cast<uint8_t>((rawRampAngle >> 8) & 0xFF);
    }

    // Heart Rate (1 byte)
    if (hasHeartRate) {
        outPayload[cursor++] = rawHeartRate;
    }

    return cursor;
}

void FtmsService::update(uint32_t nowMs, const ApplicationSnapshot& snapshot) {
    ::NimBLECharacteristic* localChar = nullptr;
    bool shouldNotify = false;

    portENTER_CRITICAL(&mux_);
    if (!state_.initialized || !state_.hasTreadmillDataSubscribers) {
        portEXIT_CRITICAL(&mux_);
        return;
    }

    if (static_cast<uint32_t>(nowMs - lastNotificationMs_) >= kFtmsNotificationIntervalMs) {
        shouldNotify = true;
        localChar = pTreadmillDataChar_;
        lastNotificationMs_ = nowMs;
        state_.lastNotificationMs = nowMs;
    }
    portEXIT_CRITICAL(&mux_);

    if (shouldNotify && localChar != nullptr) {
        uint8_t payload[16]{};
        size_t len = packTreadmillData(snapshot, payload, sizeof(payload));
        if (len >= 4) {
            localChar->setValue(payload, len);
            localChar->notify(true);
        }
    }
}

FtmsServerStateDto FtmsService::getState() const {
    portENTER_CRITICAL(&mux_);
    FtmsServerStateDto copy = state_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

} // namespace stridecontrol

