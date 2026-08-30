#pragma once

#include <cstddef>
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "SettingsServiceTypes.h"

namespace stridecontrol {

/**
 * @brief Thread-safe non-volatile storage (NVS) manager for StrideControl.
 *
 * Backed by ESP32 Preferences library with FreeRTOS Mutex synchronization.
 */
class SettingsService {
public:
    SettingsService();
    ~SettingsService();

    SettingsService(const SettingsService&) = delete;
    SettingsService& operator=(const SettingsService&) = delete;

    void begin();
    SystemConfig loadAll();

    InclineConfig getInclineConfig();
    SpeedConfig getSpeedConfig();
    MaintenanceConfig getMaintenanceConfig();

    bool saveInclineConfig(const InclineConfig& config);
    bool saveSpeedConfig(const SpeedConfig& config);
    bool saveMaintenanceConfig(const MaintenanceConfig& config);

    void factoryReset();

    static const char* version();

private:
    SemaphoreHandle_t mutex_ = nullptr;
    bool initialized_ = false;

    static constexpr const char* kNvsNamespace = "stride_cfg";
};

} // namespace stridecontrol
