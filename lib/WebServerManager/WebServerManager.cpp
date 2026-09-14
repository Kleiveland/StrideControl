#include "WebServerManager.h"
#include <WiFi.h>
#include <LittleFS.h>
#include "FileSystemManager.h"
#include "SettingsService.h"
#include "../DiagnosticsLog/DiagnosticsLog.h"
#include <cstdlib>
#include <cstring>
#include <algorithm>

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
        session["stepElapsedMs"] = report.stepElapsedMs;
        session["elapsedTimeMs"] = report.totalElapsedTimeMs;
        session["totalElevationMeters"] = report.totalElevationMeters;
        session["avgHeartRateBpm"] = report.avgHeartRateBpm;
        session["maxHeartRateBpm"] = report.maxHeartRateBpm;
        session["heartRateEverValid"] = report.heartRateEverValid;
        session["workoutId"] = report.workoutId;
        session["totalStepCount"] = report.totalStepCount;
        session["stepProgressFraction"] = report.stepProgressFraction;
        session["speedAdjustmentPromptActive"] = report.speedAdjustmentPromptActive;
        session["suggestedSpeedDeltaKmh"] = report.suggestedSpeedDeltaKmh;
        session["speedAdjustmentPromptExpiresMs"] = report.speedAdjustmentPromptExpiresMs;
        session["rampPreFireActive"] = report.rampPreFireActive;
        session["rampPreFireSpeedChanging"] = report.rampPreFireSpeedChanging;
        session["rampPreFireInclineChanging"] = report.rampPreFireInclineChanging;
        JsonArray actualDurations = session["actualStepDurationsMs"].to<JsonArray>();
        for (uint8_t i = 0; i < report.stepIndex && i < MAX_EXPANDED_WORKOUT_STEPS; ++i) {
            actualDurations.add(report.actualStepDurationsMs[i]);
        }

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

    // Captive portal / OS connectivity-check probes
    server_.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(204);
    });
    server_.on("/gen_204", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(204);
    });
    server_.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
    });
    server_.on("/library/test/success.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
    });
    server_.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/plain", "Microsoft Connect Test");
    });
    server_.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/plain", "Microsoft NCSI");
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

    auto finalizeHandler = [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        ControlCommand cmd{};
        cmd.type = ControlCommandType::FinalizeWorkout;
        cmd.timestampMs = millis();
        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/workout/finalize", HTTP_POST, finalizeHandler);
    server_.on("/api/v1/control/workout/finalize", HTTP_POST, finalizeHandler);

    server_.on(
        "/api/v1/control/workout/cutdrag",
        HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            if (commandStager_ == nullptr) {
                request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
                return;
            }
            ControlCommand cmd{};
            cmd.timestampMs = millis();
            cmd.type = ControlCommandType::CutDrag;
            if (commandStager_->stageCommand(cmd)) {
                request->send(200, "application/json", "{\"status\":\"queued\"}");
            } else {
                request->send(503, "application/json", "{\"error\":\"queue_full\"}");
            }
        }
    );

    server_.on(
        "/api/v1/control/workout/skiptodrag",
        HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            if (commandStager_ == nullptr) {
                request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
                return;
            }
            ControlCommand cmd{};
            cmd.timestampMs = millis();
            cmd.type = ControlCommandType::SkipToNextDrag;
            if (commandStager_->stageCommand(cmd)) {
                request->send(200, "application/json", "{\"status\":\"queued\"}");
            } else {
                request->send(503, "application/json", "{\"error\":\"queue_full\"}");
            }
        }
    );

    server_.on(
        "/api/v1/control/workout/extendrest",
        HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            if (commandStager_ == nullptr) {
                request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
                return;
            }
            ControlCommand cmd{};
            cmd.timestampMs = millis();
            cmd.type = ControlCommandType::ExtendRest;
            if (commandStager_->stageCommand(cmd)) {
                request->send(200, "application/json", "{\"status\":\"queued\"}");
            } else {
                request->send(503, "application/json", "{\"error\":\"queue_full\"}");
            }
        }
    );

    server_.on(
        "/api/v1/control/workout/acceptspeedshift",
        HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            if (commandStager_ == nullptr) {
                request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
                return;
            }
            ControlCommand cmd{};
            cmd.timestampMs = millis();
            cmd.type = ControlCommandType::AcceptSpeedShift;
            if (commandStager_->stageCommand(cmd)) {
                request->send(200, "application/json", "{\"status\":\"queued\"}");
            } else {
                request->send(503, "application/json", "{\"error\":\"queue_full\"}");
            }
        }
    );

    server_.on(
        "/api/v1/control/workout/rejectspeedshift",
        HTTP_POST,
        [this](AsyncWebServerRequest* request) {
            if (commandStager_ == nullptr) {
                request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
                return;
            }
            ControlCommand cmd{};
            cmd.timestampMs = millis();
            cmd.type = ControlCommandType::RejectSpeedShift;
            if (commandStager_->stageCommand(cmd)) {
                request->send(200, "application/json", "{\"status\":\"queued\"}");
            } else {
                request->send(503, "application/json", "{\"error\":\"queue_full\"}");
            }
        }
    );

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

    auto workoutSelectHandler = [this](AsyncWebServerRequest* request) {
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
        bool hasUserId = false;
        uint8_t userId = 0;
        bool hasWorkoutId = false;
        uint16_t workoutId = 0;

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
                    if (doc.containsKey("userId")) {
                        hasUserId = true;
                        userId = doc["userId"].as<uint8_t>();
                    }
                    if (doc.containsKey("workoutId")) {
                        hasWorkoutId = true;
                        workoutId = doc["workoutId"].as<uint16_t>();
                    }
                }
            }
            free(buffer);
            request->_tempObject = nullptr;
        }

        if (!hasUserId || !hasWorkoutId) {
            request->send(400, "application/json", "{\"error\":\"Missing userId or workoutId\"}");
            return;
        }

        ControlCommand cmd{};
        cmd.timestampMs = millis();
        cmd.type = ControlCommandType::ArmWorkout;
        cmd.data.arm.userId = userId;
        cmd.data.arm.workoutId = workoutId;

        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/workout/select", HTTP_POST, workoutSelectHandler, nullptr, commandBodyBuffer);
    server_.on("/api/v1/control/workout/select", HTTP_POST, workoutSelectHandler, nullptr, commandBodyBuffer);

    auto workoutSaveHandler = [this](AsyncWebServerRequest* request) {
            if (request->getResponse() != nullptr) {
                if (request->_tempObject) { free(request->_tempObject); request->_tempObject = nullptr; }
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

            if (err || !doc.containsKey("userId") || !doc.containsKey("workout")) {
                request->send(400, "application/json", "{\"error\":\"Invalid or missing userId/workout\"}");
                return;
            }

            const uint8_t userId = doc["userId"].as<uint8_t>();
            JsonObjectConst wObj = doc["workout"].as<JsonObjectConst>();

            const SystemSettings* current = SettingsService::instance().getActiveSettings();
            if (current == nullptr) {
                request->send(500, "application/json", "{\"error\":\"Settings unavailable\"}");
                return;
            }
            SystemSettingsPtr candidate = makeSystemSettings();
            if (!candidate) {
                request->send(500, "application/json", "{\"error\":\"Memory allocation failed\"}");
                return;
            }
            *candidate = *current;

            // Find the target user by id
            int userIdx = -1;
            for (size_t i = 0; i < MAX_USERS; ++i) {
                if (candidate->users[i].id == userId) { userIdx = static_cast<int>(i); break; }
            }
            if (userIdx < 0) {
                request->send(404, "application/json", "{\"error\":\"User not found\"}");
                return;
            }
            UserProfile& user = candidate->users[userIdx];

            // Parse the incoming workout JSON into a WorkoutDefinition
            WorkoutDefinition parsed{};
            const uint16_t incomingId = wObj["id"] | static_cast<uint16_t>(0);
            const char* wName = wObj["name"] | "";
            strncpy(parsed.name, wName, sizeof(parsed.name) - 1);
            parsed.name[sizeof(parsed.name) - 1] = '\0';
            parsed.lastUsedTimestamp = millis();

            JsonArrayConst segArr = wObj["segments"].as<JsonArrayConst>();
            parsed.segmentCount = std::min(segArr.size(), MAX_SEGMENTS_PER_WORKOUT);
            for (size_t sIdx = 0; sIdx < parsed.segmentCount; ++sIdx) {
                JsonObjectConst segObj = segArr[sIdx];
                WorkoutSegment& seg = parsed.segments[sIdx];
                seg.id = segObj["id"] | static_cast<uint16_t>(sIdx + 1);
                seg.type = parseSegmentType(segObj["type"] | "SINGLE_STEP");
                seg.repetitions = segObj["repetitions"] | 1;
                seg.startSpeedKmh = segObj["startSpeedKmh"] | 0.0f;
                seg.speedProgressionPerRepKmh = segObj["speedProgressionPerRepKmh"] | 0.0f;

                JsonArrayConst stArr = segObj["steps"].as<JsonArrayConst>();
                seg.stepCount = std::min(stArr.size(), MAX_STEPS_PER_GROUP);
                for (size_t stIdx = 0; stIdx < seg.stepCount; ++stIdx) {
                    JsonObjectConst stObj = stArr[stIdx];
                    WorkoutStep& st = seg.steps[stIdx];
                    st.id = stObj["id"] | static_cast<uint16_t>(stIdx + 1);
                    st.role = parseStepRole(stObj["role"] | "WORK");
                    st.durationType = parseDurationType(stObj["durationType"] | "TIME_SECONDS");
                    st.durationValue = stObj["durationValue"] | 0;
                    st.speedMode = parseSpeedMode(stObj["speedMode"] | "FIXED");
                    st.targetSpeedKmh = stObj["targetSpeedKmh"] | 0.0f;
                    st.targetInclinePct = stObj["targetInclinePct"] | 0;
                    st.setIncline = stObj["setIncline"] | false;
                }
            }

            // Find-or-append by id; assign a new id if this is a brand-new workout
            int targetSlot = -1;
            uint16_t maxExistingId = 0;
            for (size_t i = 0; i < user.workoutCount; ++i) {
                if (incomingId != 0 && user.workouts[i].id == incomingId) { targetSlot = static_cast<int>(i); }
                if (user.workouts[i].id > maxExistingId) { maxExistingId = user.workouts[i].id; }
            }
            if (targetSlot < 0) {
                if (user.workoutCount >= MAX_WORKOUTS_PER_USER) {
                    request->send(409, "application/json", "{\"error\":\"Workout limit reached\"}");
                    return;
                }
                targetSlot = static_cast<int>(user.workoutCount);
                user.workoutCount++;
                parsed.id = (incomingId != 0) ? incomingId : (maxExistingId + 1);
            } else {
                parsed.id = incomingId;
            }
            user.workouts[targetSlot] = parsed;

            char errBuf[128] = {};
            if (SettingsService::instance().updateSystemSettings(*candidate, errBuf, sizeof(errBuf))) {
                const bool saved = SettingsService::instance().commitUsersJson();
                if (saved) {
                    JsonDocument respDoc;
                    respDoc["status"] = "saved";
                    respDoc["workoutId"] = parsed.id;
                    String resp;
                    serializeJson(respDoc, resp);
                    request->send(200, "application/json", resp);
                } else {
                    request->send(500, "application/json", "{\"error\":\"failed_to_persist_json\"}");
                }
            } else {
                JsonDocument respDoc;
                respDoc["error"] = errBuf[0] ? errBuf : "Validation or persistence failed";
                String resp;
                serializeJson(respDoc, resp);
                request->send(400, "application/json", resp);
            }
    };
    auto workoutBodyBuffer = [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        handleRequestBodyChunk(request, data, len, index, total, 4096, "{\"error\":\"Payload too large\"}");
    };
    server_.on("/api/v1/settings/workout", HTTP_POST, workoutSaveHandler, nullptr, workoutBodyBuffer);
    server_.on("/api/v1/workouts/save", HTTP_POST, workoutSaveHandler, nullptr, workoutBodyBuffer);

    auto setGuiModeHandler = [this](AsyncWebServerRequest* request) {
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
        bool hasUserId = false;
        uint8_t userId = 0;
        bool isManual = false;

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
                    if (doc.containsKey("userId")) {
                        hasUserId = true;
                        userId = doc["userId"].as<uint8_t>();
                    }
                    if (doc.containsKey("isManual")) {
                        isManual = doc["isManual"].as<bool>();
                    }
                }
            }
            free(buffer);
            request->_tempObject = nullptr;
        }

        if (!hasUserId) {
            request->send(400, "application/json", "{\"error\":\"Missing userId\"}");
            return;
        }

        ControlCommand cmd{};
        cmd.timestampMs = millis();
        cmd.type = ControlCommandType::SetGuiMode;
        cmd.data.guiMode.userId = userId;
        cmd.data.guiMode.isManual = isManual;

        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/mode", HTTP_POST, setGuiModeHandler, nullptr, commandBodyBuffer);
    server_.on("/api/v1/control/mode", HTTP_POST, setGuiModeHandler, nullptr, commandBodyBuffer);

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

    server_.on("/commissioning.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (LittleFS.exists("/commissioning.html")) {
            AsyncWebServerResponse* response = request->beginResponse(LittleFS, "/commissioning.html", "text/html");
            response->addHeader("Cache-Control", "no-cache");
            request->send(response);
        } else {
            request->send(404, "text/plain", "StrideControl: /commissioning.html not found on LittleFS filesystem.");
        }
    });

    server_.on("/api/v1/diagnostics/log", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncResponseStream* stream = request->beginResponseStream("text/plain");
        DiagnosticsLog::instance().dumpTo(*stream);
        request->send(stream);
    });

    server_.on("/api/v1/config/ble", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        JsonDocument doc;
        doc["bleStackEnabled"] = SettingsService::instance().getBleStackEnabled();
        serializeJson(doc, *stream);
        request->send(stream);
    });

    auto bleConfigPostHandler = [this](AsyncWebServerRequest* request) {
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
        if (err || !doc.containsKey("bleStackEnabled")) {
            request->send(400, "application/json", "{\"error\":\"Invalid or missing bleStackEnabled\"}");
            return;
        }
        const bool enabled = doc["bleStackEnabled"].as<bool>();
        const bool ok = SettingsService::instance().saveBleStackEnabled(enabled);
        request->send(ok ? 200 : 500, "application/json", ok ? "{\"status\":\"saved\"}" : "{\"error\":\"save_failed\"}");
    };
    server_.on("/api/v1/config/ble", HTTP_POST, bleConfigPostHandler, nullptr, commandBodyBuffer);

    // POST /api/v1/commissioning/ramptest/start
    auto rampTestStartHandler = [this](AsyncWebServerRequest* request) {
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

        if (err || !doc.containsKey("startSpeedKmh") || !doc.containsKey("targetSpeedKmh")) {
            request->send(400, "application/json", "{\"error\":\"Invalid or missing startSpeedKmh or targetSpeedKmh\"}");
            return;
        }

        ControlCommand cmd{};
        cmd.type = ControlCommandType::StartRampCalibrationTest;
        cmd.timestampMs = millis();
        cmd.data.rampTest.startSpeedKmh = doc["startSpeedKmh"].as<float>();
        cmd.data.rampTest.targetSpeedKmh = doc["targetSpeedKmh"].as<float>();

        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/v1/commissioning/ramptest/start", HTTP_POST, rampTestStartHandler, nullptr, commandBodyBuffer);

    // GET /api/v1/commissioning/ramptest/status
    server_.on("/api/v1/commissioning/ramptest/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }

        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Cache-Control", "no-cache");
        JsonDocument doc;
        doc["active"] = commandStager_->isRampTestActive();
        doc["complete"] = commandStager_->isRampTestComplete();
        doc["timedOut"] = commandStager_->didRampTestTimeOut();
        doc["deadTimeMs"] = commandStager_->getRampTestDeadTimeMs();
        doc["totalMs"] = commandStager_->getRampTestTotalMs();
        serializeJson(doc, *stream);
        request->send(stream);
    });

    // POST /api/v1/commissioning/ramptest/save
    auto rampTestSaveHandler = [this](AsyncWebServerRequest* request) {
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

        if (err || !doc.containsKey("zone") || !doc.containsKey("direction") ||
            !doc.containsKey("rampMsPerKmh") || !doc.containsKey("deadTimeMs")) {
            request->send(400, "application/json", "{\"error\":\"Invalid or missing parameters\"}");
            return;
        }

        const int zone = doc["zone"].as<int>();
        const char* direction = doc["direction"] | "";
        const float rampMsPerKmh = doc["rampMsPerKmh"].as<float>();
        const uint32_t deadTimeMs = doc["deadTimeMs"].as<uint32_t>();

        if (zone < 0 || zone > 2 || (strcmp(direction, "accel") != 0 && strcmp(direction, "decel") != 0)) {
            request->send(400, "application/json", "{\"error\":\"Invalid zone or direction\"}");
            return;
        }

        RampCalibrationConfig cfg = SettingsService::instance().getRampCalibrationConfig();
        if (strcmp(direction, "accel") == 0) {
            cfg.accelMsPerKmh[zone] = rampMsPerKmh;
        } else {
            cfg.decelMsPerKmh[zone] = rampMsPerKmh;
        }
        cfg.deadTimeMs = deadTimeMs;
        cfg.calibrated = true;

        const bool ok = SettingsService::instance().saveRampCalibrationConfig(cfg);
        request->send(ok ? 200 : 500, "application/json", ok ? "{\"status\":\"saved\"}" : "{\"error\":\"save_failed\"}");
    };
    server_.on("/api/v1/commissioning/ramptest/save", HTTP_POST, rampTestSaveHandler, nullptr, commandBodyBuffer);

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

    // POST /api/v1/simulator/heartrate
    server_.on(
        "/api/v1/simulator/heartrate",
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

            const bool simulate = doc["simulateFromSpeed"] | false;
            simRuntime_->setSimHeartRateFromSpeed(simulate);
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
