#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "NetworkManager.h"
#include "ITelemetryProvider.h"
#include "ControlCommand.h"
#include "SettingsService.h"

namespace stridecontrol {

#if defined(STRIDECONTROL_TESTBENCH)
class TestbenchControlRuntime;
#endif
class SystemManager;
class HeartRateClient;
class SpeedSensor;

class WebServerManager {
public:
    struct BackupSnapshot {
        bool valid{false};
        uint32_t capturedAtMs{0};
        SpeedConfig speed{};
        InclineConfig incline{};
        MaintenanceConfig maintenance{};
        RampCalibrationConfig ramp{};
        bool bleStackEnabled{true};
        SystemSettingsPtr settings{nullptr};
    };

    explicit WebServerManager(uint16_t port = 80);
    ~WebServerManager();

    // Non-copyable
    WebServerManager(const WebServerManager&) = delete;
    WebServerManager& operator=(const WebServerManager&) = delete;

    bool begin(const ITelemetryProvider* telemetryProvider = nullptr, IControlCommandStager* commandStager = nullptr);
    void attachCommandStager(IControlCommandStager* commandStager);
    void attachSystemManager(SystemManager* systemManager);
    void attachHeartRateClient(HeartRateClient* hrClient);
    void attachSpeedSensor(SpeedSensor* speedSensor);
#if defined(STRIDECONTROL_TESTBENCH)
    void attachSimulatorRuntime(TestbenchControlRuntime* simRuntime);
#endif
    void end();

    void captureLastKnownGoodSnapshot();
    bool rollbackToLastKnownGood(char* errBuf = nullptr, size_t errBufLen = 0);
    const BackupSnapshot& getLastKnownGoodSnapshot() const { return lastKnownGoodSnapshot_; }

private:
    void registerRoutes();

    AsyncWebServer server_;
    const ITelemetryProvider* telemetryProvider_{nullptr};
    IControlCommandStager* commandStager_{nullptr};
    SystemManager* systemManager_{nullptr};
    HeartRateClient* hrClient_{nullptr};
    SpeedSensor* speedSensor_{nullptr};
#if defined(STRIDECONTROL_TESTBENCH)
    TestbenchControlRuntime* simRuntime_{nullptr};
#endif
    BackupSnapshot lastKnownGoodSnapshot_{};
    bool running_{false};
};

} // namespace stridecontrol
