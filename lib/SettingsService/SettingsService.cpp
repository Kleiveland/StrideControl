#include "SettingsService.h"
#include <Preferences.h>

namespace stridecontrol {

SettingsService::SettingsService() {
    mutex_ = xSemaphoreCreateMutex();
}

SettingsService::~SettingsService() {
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

void SettingsService::begin() {
    if (mutex_ == nullptr) {
        mutex_ = xSemaphoreCreateMutex();
    }
    if (mutex_ != nullptr && xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        initialized_ = true;
        xSemaphoreGive(mutex_);
    }
}

SystemConfig SettingsService::loadAll() {
    SystemConfig cfg{};
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return cfg;
    }

    Preferences prefs;
    if (prefs.begin(kNvsNamespace, true)) {
        InclineConfig inc{};
        size_t len = prefs.getBytes("incline", &inc, sizeof(InclineConfig));
        if (len == sizeof(InclineConfig)) {
            cfg.incline = inc;
        }

        SpeedConfig spd{};
        len = prefs.getBytes("speed", &spd, sizeof(SpeedConfig));
        if (len == sizeof(SpeedConfig) && spd.pointCount <= kMaxSpeedCalibrationPoints) {
            cfg.speed = spd;
        }

        MaintenanceConfig maint{};
        len = prefs.getBytes("maint", &maint, sizeof(MaintenanceConfig));
        if (len == sizeof(MaintenanceConfig)) {
            cfg.maintenance = maint;
        }

        prefs.end();
    }

    xSemaphoreGive(mutex_);
    return cfg;
}

InclineConfig SettingsService::getInclineConfig() {
    InclineConfig cfg{};
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return cfg;
    }

    Preferences prefs;
    if (prefs.begin(kNvsNamespace, true)) {
        InclineConfig inc{};
        size_t len = prefs.getBytes("incline", &inc, sizeof(InclineConfig));
        if (len == sizeof(InclineConfig)) {
            cfg = inc;
        }
        prefs.end();
    }

    xSemaphoreGive(mutex_);
    return cfg;
}

SpeedConfig SettingsService::getSpeedConfig() {
    SpeedConfig cfg{};
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return cfg;
    }

    Preferences prefs;
    if (prefs.begin(kNvsNamespace, true)) {
        SpeedConfig spd{};
        size_t len = prefs.getBytes("speed", &spd, sizeof(SpeedConfig));
        if (len == sizeof(SpeedConfig) && spd.pointCount <= kMaxSpeedCalibrationPoints) {
            cfg = spd;
        }
        prefs.end();
    }

    xSemaphoreGive(mutex_);
    return cfg;
}

MaintenanceConfig SettingsService::getMaintenanceConfig() {
    MaintenanceConfig cfg{};
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return cfg;
    }

    Preferences prefs;
    if (prefs.begin(kNvsNamespace, true)) {
        MaintenanceConfig maint{};
        size_t len = prefs.getBytes("maint", &maint, sizeof(MaintenanceConfig));
        if (len == sizeof(MaintenanceConfig)) {
            cfg = maint;
        }
        prefs.end();
    }

    xSemaphoreGive(mutex_);
    return cfg;
}

bool SettingsService::saveInclineConfig(const InclineConfig& config) {
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }

    bool success = false;
    Preferences prefs;
    if (prefs.begin(kNvsNamespace, false)) {
        size_t written = prefs.putBytes("incline", &config, sizeof(InclineConfig));
        success = (written == sizeof(InclineConfig));
        prefs.end();
    }

    xSemaphoreGive(mutex_);
    return success;
}

bool SettingsService::saveSpeedConfig(const SpeedConfig& config) {
    if (config.pointCount > kMaxSpeedCalibrationPoints) {
        return false;
    }

    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }

    bool success = false;
    Preferences prefs;
    if (prefs.begin(kNvsNamespace, false)) {
        size_t written = prefs.putBytes("speed", &config, sizeof(SpeedConfig));
        success = (written == sizeof(SpeedConfig));
        prefs.end();
    }

    xSemaphoreGive(mutex_);
    return success;
}

bool SettingsService::saveMaintenanceConfig(const MaintenanceConfig& config) {
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }

    bool success = false;
    Preferences prefs;
    if (prefs.begin(kNvsNamespace, false)) {
        size_t written = prefs.putBytes("maint", &config, sizeof(MaintenanceConfig));
        success = (written == sizeof(MaintenanceConfig));
        prefs.end();
    }

    xSemaphoreGive(mutex_);
    return success;
}

void SettingsService::factoryReset() {
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }

    Preferences prefs;
    if (prefs.begin(kNvsNamespace, false)) {
        prefs.clear();
        prefs.end();
    }

    xSemaphoreGive(mutex_);
}

const char* SettingsService::version() {
    return "1.0.0";
}

} // namespace stridecontrol
