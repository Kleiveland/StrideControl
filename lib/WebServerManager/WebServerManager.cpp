#include "WebServerManager.h"
#include <WiFi.h>
#include <LittleFS.h>
#include "FileSystemManager.h"
#include "SettingsService.h"
#include <cstdlib>
#include <cstring>

#if defined(STRIDECONTROL_TESTBENCH)
#include "TestbenchControlRuntime.h"
#include "VirtualRunnerAdapter.h"
#include "SimulatorHtml.h"
#endif

namespace stridecontrol {

namespace {

struct HttpBodyBuffer {
    size_t capacity;
    size_t received;
    uint8_t* data() { return reinterpret_cast<uint8_t*>(this + 1); }
    const uint8_t* data() const { return reinterpret_cast<const uint8_t*>(this + 1); }
};

void handleRequestBodyChunk(AsyncWebServerRequest* request,
                            uint8_t* data,
                            size_t len,
                            size_t index,
                            size_t total,
                            size_t maxLimit,
                            const char* payloadTooLargeJson) {
    if (request == nullptr) {
        return;
    }

    // If an error response was already staged (e.g. 413 or 400 on an earlier chunk), ignore subsequent chunks
    if (request->getResponse() != nullptr) {
        return;
    }

    // Contract 2: Validate non-null data pointer for non-empty fragment
    if (len > 0 && data == nullptr) {
        if (request->_tempObject) {
            free(request->_tempObject);
            request->_tempObject = nullptr;
        }
        request->send(400, "application/json", "{\"error\":\"Null data fragment pointer\"}");
        return;
    }

    if (index == 0) {
        // Contract 3: Handle repeated index == 0 deterministically
        if (request->_tempObject != nullptr) {
            free(request->_tempObject);
            request->_tempObject = nullptr;
            request->send(400, "application/json", "{\"error\":\"Duplicate initial body fragment\"}");
            return;
        }

        // Contract 6 & 8: Enforce maximum body size & defensive integer overflow guard
        if (total > maxLimit || total > (SIZE_MAX - sizeof(HttpBodyBuffer) - 1)) {
            request->send(413, "application/json", payloadTooLargeJson);
            return;
        }

        // Zero-length bodies do not require buffer allocation
        if (total == 0) {
            return;
        }

        void* mem = malloc(sizeof(HttpBodyBuffer) + total + 1);
        if (!mem) {
            request->send(500, "application/json", "{\"error\":\"Memory allocation failed\"}");
            return;
        }

        HttpBodyBuffer* buf = static_cast<HttpBodyBuffer*>(mem);
        buf->capacity = total;
        buf->received = 0;
        buf->data()[total] = '\0';
        request->_tempObject = buf;
    }

    HttpBodyBuffer* buf = static_cast<HttpBodyBuffer*>(request->_tempObject);
    if (!buf) {
        // Allocation failed or was rejected on index == 0
        return;
    }

    // Contract 4: Enforce stable total length on subsequent chunks
    if (index > 0 && total != buf->capacity) {
        free(buf);
        request->_tempObject = nullptr;
        request->send(400, "application/json", "{\"error\":\"Inconsistent total body length\"}");
        return;
    }

    // Contract 1: Strictly contiguous fragment assembly
    if (index != buf->received) {
        free(buf);
        request->_tempObject = nullptr;
        request->send(400, "application/json", "{\"error\":\"Non-contiguous body fragment\"}");
        return;
    }

    // Contract 2: Validate remaining capacity using subtraction
    if (len > (buf->capacity - buf->received)) {
        free(buf);
        request->_tempObject = nullptr;
        request->send(400, "application/json", "{\"error\":\"Body fragment exceeds declared capacity\"}");
        return;
    }

    if (len > 0) {
        memcpy(buf->data() + index, data, len);
        buf->received += len;
    }
}

} // anonymous namespace

WebServerManager::WebServerManager(uint16_t port)
    : server_(port) {}

WebServerManager::~WebServerManager() {
    end();
}

void WebServerManager::attachCommandStager(IControlCommandStager* commandStager) {
    commandStager_ = commandStager;
}

#if defined(STRIDECONTROL_TESTBENCH)
void WebServerManager::attachSimulatorRuntime(TestbenchControlRuntime* simRuntime) {
    simRuntime_ = simRuntime;
    commandStager_ = simRuntime;
}
#endif

bool WebServerManager::begin(const ITelemetryProvider* telemetryProvider, IControlCommandStager* commandStager) {
    if (running_) {
        return true;
    }

    telemetryProvider_ = telemetryProvider;
    if (commandStager != nullptr) {
        commandStager_ = commandStager;
    }
    registerRoutes();
    server_.begin();
    running_ = true;
    Serial.println("[WebServerManager] HTTP server started on port 80.");
    return true;
}

void WebServerManager::end() {
    if (running_) {
        server_.end();
        running_ = false;
        Serial.println("[WebServerManager] HTTP server stopped.");
    }
}

void WebServerManager::registerRoutes() {
    // GET /api/v1/telemetry
    auto telemetryHandler = [this](AsyncWebServerRequest* request) {
        if (telemetryProvider_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"Telemetry provider not configured\"}");
            return;
        }

        TelemetryReport report{};
        if (!telemetryProvider_->getTelemetry(report)) {
            request->send(503, "application/json", "{\"error\":\"Failed to acquire telemetry\"}");
            return;
        }

        JsonDocument doc;
        doc["timestampMs"] = report.timestampMs;
        doc["authority"] = report.authority;

        JsonObject speed = doc["speed"].to<JsonObject>();
        speed["kmh"] = report.actualSpeedKmh;
        speed["runnerKmh"] = report.runnerSpeedKmh;
        speed["beltDistanceKm"] = report.beltDistanceKm;

        JsonObject incline = doc["incline"].to<JsonObject>();
        incline["pct"] = report.actualInclinePct;

        JsonObject runner = doc["runner"].to<JsonObject>();
        runner["distanceKm"] = report.runnerDistanceKm;
        runner["speedKmh"] = report.runnerSpeedKmh;
        runner["presence"] = report.runnerPresence;

        JsonObject session = doc["session"].to<JsonObject>();
        session["state"] = report.sessionState;
        session["stepIndex"] = report.stepIndex;
        session["stepRemainingMs"] = report.stepRemainingMs;
        session["elapsedTimeMs"] = report.totalElapsedTimeMs;

        JsonObject hr = doc["heartRate"].to<JsonObject>();
        hr["bpm"] = report.heartRateBpm;
        hr["valid"] = report.heartRateValid;

        doc["elapsedTimeMs"] = report.totalElapsedTimeMs;
        doc["targetSpeedKmh"] = report.targetSpeedKmh;
        doc["targetInclinePct"] = report.targetInclinePct;
        doc["droppedEvents"] = report.droppedEventsCount;

        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        stream->addHeader("Cache-Control", "no-cache");
        serializeJson(doc, *stream);
        request->send(stream);
    };

    server_.on("/api/v1/telemetry", HTTP_GET, telemetryHandler);

    // GET /api/health
    server_.on("/api/health", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;

        doc["status"] = "OK";

#if defined(STRIDECONTROL_TESTBENCH)
        doc["board"] = "XIAO_ESP32S3";
#else
        doc["board"] = "ESP32-S3-N16R8";
#endif

        doc["uptime_s"] = static_cast<uint32_t>(millis() / 1000UL);
        doc["free_heap"] = ESP.getFreeHeap();
        doc["min_free_heap"] = ESP.getMinFreeHeap();
        doc["free_psram"] = ESP.getFreePsram();
        doc["wifi_rssi"] = WiFi.RSSI();

        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        serializeJson(doc, *stream);
        request->send(stream);
    });

    // GET /api/settings: Stream current system settings JSON
    server_.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        stream->addHeader("Cache-Control", "no-cache");
        const SystemSettings* settings = SettingsService::instance().getActiveSettings();
        if (settings) {
            SettingsService::serializeSettingsJson(*settings, *stream);
        } else {
            stream->print("{}");
        }
        request->send(stream);
    });

    // POST /api/settings: Atomic validation and persistence
    server_.on(
        "/api/settings",
        HTTP_POST,
        [](AsyncWebServerRequest* request) {
            if (request->getResponse() != nullptr) {
                if (request->_tempObject) {
                    free(request->_tempObject);
                    request->_tempObject = nullptr;
                }
                return;
            }

            if (!request->_tempObject) {
                request->send(400, "application/json", "{\"error\":\"Missing request body\"}");
                return;
            }

            auto* buffer = static_cast<HttpBodyBuffer*>(request->_tempObject);
            if (buffer->received == 0) {
                free(buffer);
                request->_tempObject = nullptr;
                request->send(400, "application/json", "{\"error\":\"Empty payload\"}");
                return;
            }

            if (buffer->received != buffer->capacity) {
                free(buffer);
                request->_tempObject = nullptr;
                request->send(400, "application/json", "{\"error\":\"Incomplete request body\"}");
                return;
            }

            SystemSettingsPtr candidate = makeSystemSettings();
            if (!candidate) {
                free(buffer);
                request->_tempObject = nullptr;
                request->send(500, "application/json", "{\"error\":\"Memory allocation failed\"}");
                return;
            }

            char errBuf[128] = {};
            bool parseOk = SettingsService::deserializeSettingsJson(
                buffer->data(), buffer->received, *candidate, errBuf, sizeof(errBuf)
            );

            free(buffer);
            request->_tempObject = nullptr;

            if (!parseOk) {
                JsonDocument errDoc;
                errDoc["error"] = errBuf[0] != '\0' ? errBuf : "Invalid JSON payload or schema validation failed";
                AsyncResponseStream* stream = request->beginResponseStream("application/json");
                stream->setCode(400);
                stream->addHeader("Access-Control-Allow-Origin", "*");
                serializeJson(errDoc, *stream);
                request->send(stream);
                return;
            }

            bool saveOk = SettingsService::instance().updateSystemSettings(*candidate, errBuf, sizeof(errBuf));
            if (!saveOk) {
                JsonDocument errDoc;
                errDoc["error"] = errBuf[0] != '\0' ? errBuf : "Failed to persist settings";
                AsyncResponseStream* stream = request->beginResponseStream("application/json");
                stream->setCode(500);
                stream->addHeader("Access-Control-Allow-Origin", "*");
                serializeJson(errDoc, *stream);
                request->send(stream);
                return;
            }

            request->send(200, "application/json", "{\"status\":\"OK\"}");
        },
        nullptr,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            handleRequestBodyChunk(request, data, len, index, total, 32768, "{\"error\":\"Payload too large (max 32KB)\"}");
        }
    );

    // GET /: Serve index.html from LittleFS
    server_.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (LittleFS.exists("/index.html")) {
            AsyncWebServerResponse* response = request->beginResponse(LittleFS, "/index.html", "text/html");
            response->addHeader("Cache-Control", "no-cache");
            request->send(response);
        } else {
            request->send(404, "text/plain", "StrideControl: /index.html not found on LittleFS filesystem.");
        }
    });

    // Static font assets handler (1 year cache)
    server_.serveStatic("/fonts", LittleFS, "/fonts").setCacheControl("public, max-age=31536000, immutable");

    // Static asset handler from LittleFS root
    server_.serveStatic("/", LittleFS, "/").setCacheControl("public, max-age=3600");

    // -------------------------------------------------------------------------
    // Control Command API Routes (Producer interface into ControlTask queue)
    // -------------------------------------------------------------------------
    auto quickstartHandler = [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        ControlCommand cmd{};
        cmd.type = ControlCommandType::QuickStart;
        cmd.timestampMs = millis();
        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/quickstart", HTTP_POST, quickstartHandler);
    server_.on("/api/v1/control/quickstart", HTTP_POST, quickstartHandler);

    auto stopHandler = [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        ControlCommand cmd{};
        cmd.type = ControlCommandType::Stop;
        cmd.timestampMs = millis();
        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/stop", HTTP_POST, stopHandler);
    server_.on("/api/v1/control/stop", HTTP_POST, stopHandler);

    auto pauseHandler = [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        ControlCommand cmd{};
        cmd.type = ControlCommandType::Pause;
        cmd.timestampMs = millis();
        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/pause", HTTP_POST, pauseHandler);
    server_.on("/api/v1/control/pause", HTTP_POST, pauseHandler);

    auto resumeHandler = [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        ControlCommand cmd{};
        cmd.type = ControlCommandType::Resume;
        cmd.timestampMs = millis();
        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/resume", HTTP_POST, resumeHandler);
    server_.on("/api/v1/control/resume", HTTP_POST, resumeHandler);

    auto commandBodyBuffer = [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        handleRequestBodyChunk(request, data, len, index, total, 2048, "{\"error\":\"Payload too large\"}");
    };

    auto speedHandler = [this](AsyncWebServerRequest* request) {
        if (request->getResponse() != nullptr) {
            if (request->_tempObject) {
                free(request->_tempObject);
                request->_tempObject = nullptr;
            }
            return;
        }

        if (commandStager_ == nullptr) {
            if (request->_tempObject) {
                free(request->_tempObject);
                request->_tempObject = nullptr;
            }
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        bool hasTarget = false;
        float targetSpeed = 0.0f;
        bool hasDelta = false;
        float deltaSpeed = 0.0f;

        if (request->_tempObject) {
            auto* buffer = static_cast<HttpBodyBuffer*>(request->_tempObject);
            if (buffer->received != buffer->capacity) {
                free(buffer);
                request->_tempObject = nullptr;
                request->send(400, "application/json", "{\"error\":\"Incomplete request body\"}");
                return;
            }
            if (buffer->received > 0) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, buffer->data(), buffer->received);
                if (!err) {
                    if (doc.containsKey("speed")) {
                        hasTarget = true;
                        targetSpeed = doc["speed"].as<float>();
                    } else if (doc.containsKey("delta")) {
                        hasDelta = true;
                        deltaSpeed = doc["delta"].as<float>();
                    } else if (doc.containsKey("deltaSpeed")) {
                        hasDelta = true;
                        deltaSpeed = doc["deltaSpeed"].as<float>();
                    }
                }
            }
            free(buffer);
            request->_tempObject = nullptr;
        }
        if (!hasTarget && !hasDelta) {
            if (request->hasParam("speed")) {
                hasTarget = true;
                targetSpeed = request->getParam("speed")->value().toFloat();
            } else if (request->hasParam("delta")) {
                hasDelta = true;
                deltaSpeed = request->getParam("delta")->value().toFloat();
            } else if (request->hasParam("deltaSpeed")) {
                hasDelta = true;
                deltaSpeed = request->getParam("deltaSpeed")->value().toFloat();
            }
        }

        if (!hasTarget && !hasDelta) {
            request->send(400, "application/json", "{\"error\":\"Missing speed or delta parameter\"}");
            return;
        }

        ControlCommand cmd{};
        cmd.timestampMs = millis();
        if (hasTarget) {
            cmd.type = ControlCommandType::SetSpeed;
            cmd.data.target.speedKmh = targetSpeed;
        } else {
            cmd.type = ControlCommandType::StepSpeed;
            cmd.data.stepSpeed.deltaSpeedKmh = deltaSpeed;
        }

        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/speed", HTTP_POST, speedHandler, nullptr, commandBodyBuffer);
    server_.on("/api/v1/control/speed", HTTP_POST, speedHandler, nullptr, commandBodyBuffer);

    auto inclineHandler = [this](AsyncWebServerRequest* request) {
        if (request->getResponse() != nullptr) {
            if (request->_tempObject) {
                free(request->_tempObject);
                request->_tempObject = nullptr;
            }
            return;
        }

        if (commandStager_ == nullptr) {
            if (request->_tempObject) {
                free(request->_tempObject);
                request->_tempObject = nullptr;
            }
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        bool hasTarget = false;
        float targetIncline = 0.0f;
        bool hasDelta = false;
        float deltaIncline = 0.0f;

        if (request->_tempObject) {
            auto* buffer = static_cast<HttpBodyBuffer*>(request->_tempObject);
            if (buffer->received != buffer->capacity) {
                free(buffer);
                request->_tempObject = nullptr;
                request->send(400, "application/json", "{\"error\":\"Incomplete request body\"}");
                return;
            }
            if (buffer->received > 0) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, buffer->data(), buffer->received);
                if (!err) {
                    if (doc.containsKey("incline")) {
                        hasTarget = true;
                        targetIncline = doc["incline"].as<float>();
                    } else if (doc.containsKey("delta")) {
                        hasDelta = true;
                        deltaIncline = doc["delta"].as<float>();
                    } else if (doc.containsKey("deltaIncline")) {
                        hasDelta = true;
                        deltaIncline = doc["deltaIncline"].as<float>();
                    }
                }
            }
            free(buffer);
            request->_tempObject = nullptr;
        }
        if (!hasTarget && !hasDelta) {
            if (request->hasParam("incline")) {
                hasTarget = true;
                targetIncline = request->getParam("incline")->value().toFloat();
            } else if (request->hasParam("delta")) {
                hasDelta = true;
                deltaIncline = request->getParam("delta")->value().toFloat();
            } else if (request->hasParam("deltaIncline")) {
                hasDelta = true;
                deltaIncline = request->getParam("deltaIncline")->value().toFloat();
            }
        }

        if (!hasTarget && !hasDelta) {
            request->send(400, "application/json", "{\"error\":\"Missing incline or delta parameter\"}");
            return;
        }

        ControlCommand cmd{};
        cmd.timestampMs = millis();
        if (hasTarget) {
            cmd.type = ControlCommandType::SetIncline;
            cmd.data.target.inclinePct = targetIncline;
        } else {
            cmd.type = ControlCommandType::StepIncline;
            cmd.data.stepIncline.deltaInclinePct = deltaIncline;
        }

        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/incline", HTTP_POST, inclineHandler, nullptr, commandBodyBuffer);
    server_.on("/api/v1/control/incline", HTTP_POST, inclineHandler, nullptr, commandBodyBuffer);

#if defined(STRIDECONTROL_TESTBENCH)
    // GET /simulator.html (Testbench active universe UI)
    server_.on("/simulator.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send_P(200, "text/html", kSimulatorHtml);
    });

    // POST /api/v1/simulator/console
    server_.on(
        "/api/v1/simulator/console",
        HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            if (request->getResponse() != nullptr) {
                if (request->_tempObject) {
                    free(request->_tempObject);
                    request->_tempObject = nullptr;
                }
                return;
            }

            if (!request->_tempObject) {
                request->send(400, "application/json", "{\"error\":\"Missing body\"}");
                return;
            }
            auto* buffer = static_cast<HttpBodyBuffer*>(request->_tempObject);
            if (buffer->received != buffer->capacity) {
                free(buffer);
                request->_tempObject = nullptr;
                request->send(400, "application/json", "{\"error\":\"Incomplete request body\"}");
                return;
            }
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, buffer->data(), buffer->received);
            free(buffer);
            request->_tempObject = nullptr;

            if (err) {
                request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                return;
            }

            const char* btn = doc["button"] | "";
            float val = doc["value"] | 0.0f;
            bool ok = false;
            ControlCommand cmd{};
            cmd.timestampMs = millis();

            if (strcmp(btn, "QuickStart") == 0) {
                cmd.type = ControlCommandType::QuickStart;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "Stop") == 0) {
                cmd.type = ControlCommandType::Stop;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "EmergencyStop") == 0) {
                cmd.type = ControlCommandType::Stop;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "SpeedPlus") == 0) {
                cmd.type = ControlCommandType::StepSpeed;
                cmd.data.stepSpeed.deltaSpeedKmh = 0.5f;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "SpeedMinus") == 0) {
                cmd.type = ControlCommandType::StepSpeed;
                cmd.data.stepSpeed.deltaSpeedKmh = -0.5f;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "InclinePlus") == 0) {
                cmd.type = ControlCommandType::StepIncline;
                cmd.data.stepIncline.deltaInclinePct = 0.5f;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "InclineMinus") == 0) {
                cmd.type = ControlCommandType::StepIncline;
                cmd.data.stepIncline.deltaInclinePct = -0.5f;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "SetSpeed") == 0) {
                cmd.type = ControlCommandType::SetSpeed;
                cmd.data.target.speedKmh = val;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "SetIncline") == 0) {
                cmd.type = ControlCommandType::SetIncline;
                cmd.data.target.inclinePct = val;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else {
                request->send(400, "application/json", "{\"error\":\"Unknown button or command\"}");
                return;
            }

            request->send(ok ? 200 : 503, "application/json", ok ? "{\"status\":\"queued\"}" : "{\"error\":\"queue_full\"}");
        },
        nullptr,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            handleRequestBodyChunk(request, data, len, index, total, 2048, "{\"error\":\"Payload too large\"}");
        }
    );

    // POST /api/v1/simulator/runner
    server_.on(
        "/api/v1/simulator/runner",
        HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            if (request->getResponse() != nullptr) {
                if (request->_tempObject) {
                    free(request->_tempObject);
                    request->_tempObject = nullptr;
                }
                return;
            }

            if (simRuntime_ == nullptr) {
                if (request->_tempObject) {
                    free(request->_tempObject);
                    request->_tempObject = nullptr;
                }
                request->send(503, "application/json", "{\"error\":\"Simulator runtime not attached\"}");
                return;
            }
            if (!request->_tempObject) {
                request->send(400, "application/json", "{\"error\":\"Missing body\"}");
                return;
            }
            auto* buffer = static_cast<HttpBodyBuffer*>(request->_tempObject);
            if (buffer->received != buffer->capacity) {
                free(buffer);
                request->_tempObject = nullptr;
                request->send(400, "application/json", "{\"error\":\"Incomplete request body\"}");
                return;
            }
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, buffer->data(), buffer->received);
            free(buffer);
            request->_tempObject = nullptr;

            if (err) {
                request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                return;
            }

            const char* modeStr = doc["mode"] | "RunningOnBelt";
            uint16_t cadence = doc["cadence"] | 180;
            float mag = doc["impactMagnitudeG"] | 0.35f;
            bool valid = doc["signalValid"] | true;

            VirtualRunnerMode mode = VirtualRunnerMode::RunningOnBelt;
            if (strcmp(modeStr, "OnSideRails") == 0) {
                mode = VirtualRunnerMode::OnSideRails;
            } else if (strcmp(modeStr, "NotPresent") == 0) {
                mode = VirtualRunnerMode::NotPresent;
            }

            simRuntime_->setSimRunner(mode, cadence, mag, valid);
            request->send(200, "application/json", "{\"status\":\"staged\"}");
        },
        nullptr,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            handleRequestBodyChunk(request, data, len, index, total, 2048, "{\"error\":\"Payload too large\"}");
        }
    );
#endif
}

} // namespace stridecontrol
