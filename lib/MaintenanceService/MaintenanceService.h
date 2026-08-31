#pragma once

#include <cstdint>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include "../SettingsService/SettingsService.h"

namespace stridecontrol {

/**
 * @brief Flash-safe, wear-leveled hardware odometer and utilization tracker.
 *
 * Accumulates distance and active moving time in RAM at high loop rates (e.g. 50Hz)
 * without blocking on NVS flash writes. Flushes to SettingsService asynchronously
 * via processSave() or immediately via forceSave().
 */
class MaintenanceService {
public:
    MaintenanceService();
    ~MaintenanceService() = default;

    MaintenanceService(const MaintenanceService&) = delete;
    MaintenanceService& operator=(const MaintenanceService&) = delete;

    void begin(SettingsService* settingsService);
    void update(float currentSpeedKmh, uint32_t deltaMs);
    void forceSave();

    bool isSavePending() const;
    void processSave();

    uint64_t getTotalDistanceMeters() const;
    uint64_t getTotalTimeSeconds() const;

    static const char* version();

private:
    void flushToStorage();

    mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;

    SettingsService* settingsService_ = nullptr;
    bool initialized_ = false;
    std::atomic<bool> pendingSave_{false};

    uint64_t totalDistanceMeters_ = 0;
    uint64_t totalTimeSeconds_ = 0;

    // Fractional accumulators to prevent float precision degradation
    float fractionalMetersAccumulator_ = 0.0f;
    uint32_t fractionalTimeMsAccumulator_ = 0;

    // Wear-leveling thresholds tracking
    uint64_t lastSavedDistanceMeters_ = 0;
    uint64_t lastSavedTimeSeconds_ = 0;

    static constexpr uint64_t kSaveDistanceIntervalMeters = 500; // 500 meters
    static constexpr uint64_t kSaveTimeIntervalSeconds = 300;     // 5 minutes
    static constexpr float kMinMovingSpeedKmh = 0.1f;
};

} // namespace stridecontrol
