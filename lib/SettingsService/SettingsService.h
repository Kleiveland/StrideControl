#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "SettingsServiceTypes.h"
#include "SettingsTypes.h"
#include <ArduinoJson.h>

namespace stridecontrol {

struct SystemSettingsDeleter {
    void operator()(SystemSettings* p) const;
};

using SystemSettingsPtr = std::unique_ptr<SystemSettings, SystemSettingsDeleter>;

SystemSettingsPtr makeSystemSettings();

class SettingsService {
public:
    static SettingsService& instance();

    SettingsService();
    ~SettingsService();

    SettingsService(const SettingsService&) = delete;
    SettingsService& operator=(const SettingsService&) = delete;

    void begin();

    // =======================================================================
    // HARDWARE NVS CONFIGURATION (Backwards-Compatible API)
    // =======================================================================
    SystemConfig loadAll();
    InclineConfig getInclineConfig();
    SpeedConfig getSpeedConfig();
    MaintenanceConfig getMaintenanceConfig();
    bool saveInclineConfig(const InclineConfig& config);
    bool saveSpeedConfig(const SpeedConfig& config);
    bool saveMaintenanceConfig(const MaintenanceConfig& config);
    void factoryReset();

    // =======================================================================
    // POWER-FAIL-SAFE ATOMIC USER & WORKOUT SETTINGS (Phase D API)
    // =======================================================================
    bool loadSystemSettings();
    const SystemSettings* getActiveSettings() const;
    const WorkoutDefinition* findWorkout(uint8_t userId, uint16_t workoutId) const;
    bool updateSystemSettings(const SystemSettings& candidate, char* errBuf = nullptr, size_t errBufLen = 0);

    static bool validateSystemSettings(const SystemSettings& candidate, char* errBuf = nullptr, size_t errBufLen = 0);
    static void populateFactoryDefaults(SystemSettings& target);

    static void serializeSettingsJson(const SystemSettings& settings, Print& output);
    static bool deserializeSettingsJson(const uint8_t* jsonBytes, size_t length, SystemSettings& outCandidate, char* errBuf = nullptr, size_t errBufLen = 0);

    static const char* version();

private:
    bool saveSystemSettingsAtomic(const SystemSettings& settings);
    bool recoverAndLoadSettings();
    bool loadFactorySeedFromUsersJson(SystemSettings& target);

    SemaphoreHandle_t mutex_ = nullptr;
    bool initialized_ = false;
    SystemSettingsPtr activeSettings_;
    SystemSettingsPtr candidateSettings_;

    static constexpr const char* kNvsNamespace = "stride_cfg";
    static constexpr const char* kConfigDir = "/config";
    static constexpr const char* kSettingsFile = "/config/settings.json";
    static constexpr const char* kSettingsTmpFile = "/config/settings.json.tmp";
    static constexpr const char* kSettingsBakFile = "/config/settings.json.bak";
    static constexpr const char* kUsersSeedFile = "/config/users.json";
};

} // namespace stridecontrol
