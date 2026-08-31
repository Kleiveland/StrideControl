#include "MaintenanceService.h"
#include <cmath>

namespace stridecontrol {

MaintenanceService::MaintenanceService() = default;

void MaintenanceService::begin(SettingsService* settingsService) {
    settingsService_ = settingsService;

    uint64_t initialDistance = 0;
    uint64_t initialTime = 0;

    if (settingsService_ != nullptr) {
        MaintenanceConfig cfg = settingsService_->getMaintenanceConfig();
        initialDistance = cfg.totalDistanceMeters;
        initialTime = cfg.totalTimeSeconds;
    }

    portENTER_CRITICAL(&lock_);
    totalDistanceMeters_ = initialDistance;
    totalTimeSeconds_ = initialTime;
    lastSavedDistanceMeters_ = initialDistance;
    lastSavedTimeSeconds_ = initialTime;
    fractionalMetersAccumulator_ = 0.0f;
    fractionalTimeMsAccumulator_ = 0;
    initialized_ = (settingsService_ != nullptr);
    portEXIT_CRITICAL(&lock_);

    pendingSave_.store(false);
}

void MaintenanceService::update(float currentSpeedKmh, uint32_t deltaMs) {
    if (deltaMs == 0 || std::isnan(currentSpeedKmh) || std::isinf(currentSpeedKmh)) {
        return;
    }

    portENTER_CRITICAL(&lock_);
    if (!initialized_) {
        portEXIT_CRITICAL(&lock_);
        return;
    }

    if (currentSpeedKmh >= kMinMovingSpeedKmh) {
        // Delta distance in meters = (km/h / 3.6) * (deltaMs / 1000) = (speed * deltaMs) / 3600
        const float deltaMeters = (currentSpeedKmh * static_cast<float>(deltaMs)) / 3600.0f;
        fractionalMetersAccumulator_ += deltaMeters;

        if (fractionalMetersAccumulator_ >= 1.0f) {
            const uint64_t wholeMeters = static_cast<uint64_t>(fractionalMetersAccumulator_);
            totalDistanceMeters_ += wholeMeters;
            fractionalMetersAccumulator_ -= static_cast<float>(wholeMeters);
        }

        fractionalTimeMsAccumulator_ += deltaMs;
        if (fractionalTimeMsAccumulator_ >= 1000U) {
            const uint32_t wholeSeconds = fractionalTimeMsAccumulator_ / 1000U;
            totalTimeSeconds_ += static_cast<uint64_t>(wholeSeconds);
            fractionalTimeMsAccumulator_ %= 1000U;
        }

        // Wear-leveling condition: flag pending save without blocking the 50Hz loop
        if ((totalDistanceMeters_ - lastSavedDistanceMeters_ >= kSaveDistanceIntervalMeters) ||
            (totalTimeSeconds_ - lastSavedTimeSeconds_ >= kSaveTimeIntervalSeconds)) {
            pendingSave_.store(true);
        }
    }
    portEXIT_CRITICAL(&lock_);
}

bool MaintenanceService::isSavePending() const {
    return pendingSave_.load();
}

void MaintenanceService::processSave() {
    if (pendingSave_.exchange(false)) {
        flushToStorage();
    }
}

void MaintenanceService::forceSave() {
    pendingSave_.store(false);
    flushToStorage();
}

void MaintenanceService::flushToStorage() {
    MaintenanceConfig cfg{};
    bool readyToSave = false;

    portENTER_CRITICAL(&lock_);
    if (initialized_ && settingsService_ != nullptr) {
        cfg.totalDistanceMeters = totalDistanceMeters_;
        cfg.totalTimeSeconds = totalTimeSeconds_;
        lastSavedDistanceMeters_ = totalDistanceMeters_;
        lastSavedTimeSeconds_ = totalTimeSeconds_;
        readyToSave = true;
    }
    portEXIT_CRITICAL(&lock_);

    if (readyToSave && settingsService_ != nullptr) {
        settingsService_->saveMaintenanceConfig(cfg);
    }
}

uint64_t MaintenanceService::getTotalDistanceMeters() const {
    portENTER_CRITICAL(&lock_);
    const uint64_t distance = totalDistanceMeters_;
    portEXIT_CRITICAL(&lock_);
    return distance;
}

uint64_t MaintenanceService::getTotalTimeSeconds() const {
    portENTER_CRITICAL(&lock_);
    const uint64_t timeSec = totalTimeSeconds_;
    portEXIT_CRITICAL(&lock_);
    return timeSec;
}

const char* MaintenanceService::version() {
    return "1.1.0";
}

} // namespace stridecontrol
