#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "NetworkManager.h"
#include "ITelemetryProvider.h"
#include "ControlCommand.h"

namespace stridecontrol {

#if defined(STRIDECONTROL_TESTBENCH)
class TestbenchControlRuntime;
#endif
class SystemManager;

class WebServerManager {
public:
    explicit WebServerManager(uint16_t port = 80);
    ~WebServerManager();

    // Non-copyable
    WebServerManager(const WebServerManager&) = delete;
    WebServerManager& operator=(const WebServerManager&) = delete;

    bool begin(const ITelemetryProvider* telemetryProvider = nullptr, IControlCommandStager* commandStager = nullptr);
    void attachCommandStager(IControlCommandStager* commandStager);
    void attachSystemManager(SystemManager* systemManager);
#if defined(STRIDECONTROL_TESTBENCH)
    void attachSimulatorRuntime(TestbenchControlRuntime* simRuntime);
#endif
    void end();

private:
    void registerRoutes();

    AsyncWebServer server_;
    const ITelemetryProvider* telemetryProvider_{nullptr};
    IControlCommandStager* commandStager_{nullptr};
    SystemManager* systemManager_{nullptr};
#if defined(STRIDECONTROL_TESTBENCH)
    TestbenchControlRuntime* simRuntime_{nullptr};
#endif
    bool running_{false};
};

} // namespace stridecontrol
