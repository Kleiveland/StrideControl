#include "WebServerManager.h"
#include <WiFi.h>
#include <LittleFS.h>
#include "FileSystemManager.h"
#include "SettingsService.h"
#include "../DiagnosticsLog/DiagnosticsLog.h"
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include "../CsafeInterface/CsafeTypes.h"
#include "SystemManager.h"
#include "../HeartRateClient/HeartRateClient.h"
#include "SpeedSensor.h"
#include "SpeedCalibration.h"

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

const char* inclineCommissioningPhaseToString(InclineCommissioningPhase phase) {
    switch (phase) {
        case InclineCommissioningPhase::Idle: return "Idle";
        case InclineCommissioningPhase::Homing: return "Homing";
        case InclineCommissioningPhase::HomedSettled: return "HomedSettled";
        case InclineCommissioningPhase::MeasuringPoint: return "MeasuringPoint";
        case InclineCommissioningPhase::Complete: return "Complete";
        case InclineCommissioningPhase::Failed: return "Failed";
        case InclineCommissioningPhase::TimedOut: return "TimedOut";
        case InclineCommissioningPhase::Aborted: return "Aborted";
        default: return "Unknown";
    }
}

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

void WebServerManager::attachSystemManager(SystemManager* systemManager) {
    systemManager_ = systemManager;
}

void WebServerManager::attachHeartRateClient(HeartRateClient* hrClient) {
    hrClient_ = hrClient;
}

void WebServerManager::attachSpeedSensor(SpeedSensor* speedSensor) {
    speedSensor_ = speedSensor;
}

#if defined(STRIDECONTROL_TESTBENCH)
void WebServerManager::attachSimulatorRuntime(TestbenchControlRuntime* simRuntime) {
    simRuntime_ = simRuntime;
    commandStager_ = simRuntime;
    if (simRuntime != nullptr && speedSensor_ == nullptr) {
        speedSensor_ = &simRuntime->getSpeedSensor();
    }
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
        doc["connectionWarningActive"] = report.connectionWarningActive;
        doc["estopActive"] = report.estopActive;

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
        session["isEmergencyStopped"] = report.isEmergencyStopped;
        session["stepIndex"] = report.stepIndex;
        session["stepRemainingMs"] = report.stepRemainingMs;
        session["stepElapsedMs"] = report.stepElapsedMs;
        session["elapsedTimeMs"] = report.totalElapsedTimeMs;
        session["totalElevationMeters"] = report.totalElevationMeters;
        session["totalValidatedDistanceKm"] = report.totalValidatedDistanceKm;
        session["activeUserId"] = report.activeUserId;
        session["hasActiveUser"] = report.hasActiveUser;
        session["currentRole"] = stepRoleName(report.currentRole);
        session["hasUpcomingDragStep"] = report.hasUpcomingDragStep;
        session["avgHeartRateBpm"] = report.avgHeartRateBpm;
        session["maxHeartRateBpm"] = report.maxHeartRateBpm;
        session["maxSpeedKmh"] = report.maxSpeedKmh;
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
        session["continuationWindowActive"] = report.continuationWindowActive;
        session["continuationWindowRemainingMs"] = report.continuationWindowRemainingMs;
        JsonArray actualDurations = session["actualStepDurationsMs"].to<JsonArray>();
        for (uint8_t i = 0; i < report.stepIndex && i < MAX_EXPANDED_WORKOUT_STEPS; ++i) {
            actualDurations.add(report.actualStepDurationsMs[i]);
        }

        JsonObject hr = doc["heartRate"].to<JsonObject>();
        hr["bpm"] = report.heartRateBpm;
        hr["valid"] = report.heartRateValid;

        JsonObject csafe = doc["csafe"].to<JsonObject>();
        csafe["machineState"] = csafeMachineStateName(report.csafe.qualifiedState);
        csafe["rawStateByte"] = report.csafe.rawStateByte;
        csafe["online"] = report.csafe.online;
        csafe["linkStatus"] = csafeLinkStatusName(report.csafe.linkStatus);
        csafe["fresh"] = report.csafe.machineStateFresh;

        JsonObject maintenance = doc["maintenance"].to<JsonObject>();
        maintenance["distanceMeters"] = report.maintenanceDistanceMeters;
        maintenance["timeSeconds"] = report.maintenanceTimeSeconds;
        maintenance["savePending"] = report.maintenanceSavePending;

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

            // Preserve preferredHrMac server-side if incoming candidate has it empty,
            // preventing general settings saves (speed/incline/workouts) from clobbering HR pairing.
            const SystemSettings* active = SettingsService::instance().getActiveSettings();
            if (active != nullptr) {
                for (size_t u = 0; u < MAX_USERS; ++u) {
                    if (candidate->users[u].preferredHrMac[0] == '\0' && active->users[u].preferredHrMac[0] != '\0') {
                        strncpy(candidate->users[u].preferredHrMac, active->users[u].preferredHrMac, sizeof(candidate->users[u].preferredHrMac) - 1);
                        candidate->users[u].preferredHrMac[sizeof(candidate->users[u].preferredHrMac) - 1] = '\0';
                    }
                }
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

    server_.on("/service.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (LittleFS.exists("/service.html")) {
            AsyncWebServerResponse* response = request->beginResponse(LittleFS, "/service.html", "text/html");
            response->addHeader("Cache-Control", "no-cache");
            request->send(response);
        } else {
            request->send(404, "text/plain", "StrideControl: /service.html not found on LittleFS filesystem.");
        }
    });

    server_.on("/commissioning.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("/service.html");
    });

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

    auto selectUserHandler = [this](AsyncWebServerRequest* request) {
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
        cmd.type = ControlCommandType::SelectUser;
        cmd.data.selectUser.userId = userId;

        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/control/selectuser", HTTP_POST, selectUserHandler, nullptr, commandBodyBuffer);
    server_.on("/api/v1/control/selectuser", HTTP_POST, selectUserHandler, nullptr, commandBodyBuffer);

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

    auto capabilitiesSpeedHandler = [this](AsyncWebServerRequest* request) {
        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        stream->addHeader("Cache-Control", "no-cache");
        float maxAchievableSpeedKmh = 25.0f;
        bool verified = false;
        if (systemManager_ != nullptr) {
            maxAchievableSpeedKmh = systemManager_->getMaxAchievableSpeedKmh();
            verified = systemManager_->isMaxAchievableSpeedVerified();
        } else if (commandStager_ != nullptr) {
            maxAchievableSpeedKmh = commandStager_->getMaxAchievableSpeedKmh();
            verified = commandStager_->isMaxAchievableSpeedVerified();
        } else {
            SpeedConfig cfg = SettingsService::instance().getSpeedConfig();
            maxAchievableSpeedKmh = cfg.maxAchievableSpeedKmh;
            verified = cfg.maxAchievableSpeedVerified;
        }
        JsonDocument doc;
        doc["maxAchievableSpeedKmh"] = maxAchievableSpeedKmh;
        doc["verified"] = verified;
        serializeJson(doc, *stream);
        request->send(stream);
    };
    server_.on("/api/capabilities/speed", HTTP_GET, capabilitiesSpeedHandler);
    server_.on("/api/v1/capabilities/speed", HTTP_GET, capabilitiesSpeedHandler);

    // GET /api/v1/calibration/speed: Return SpeedConfig (sensor factor, max speed, command points)
    auto speedCalibrationGetHandler = [this](AsyncWebServerRequest* request) {
        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        stream->addHeader("Cache-Control", "no-cache");
        SpeedConfig cfg = SettingsService::instance().getSpeedConfig();
        float sensorFactor = cfg.sensorCalibrationFactor;
        if (speedSensor_ != nullptr) {
            sensorFactor = speedSensor_->getCalibrationFactor();
        }
        JsonDocument doc;
        doc["sensorCalibrationFactor"] = sensorFactor;
        doc["maxAchievableSpeedKmh"] = cfg.maxAchievableSpeedKmh;
        doc["maxAchievableSpeedVerified"] = cfg.maxAchievableSpeedVerified;
        doc["commandMapValid"] = cfg.commandMapValid;
        doc["pointCount"] = cfg.pointCount;
        JsonArray pts = doc["points"].to<JsonArray>();
        for (size_t i = 0; i < cfg.pointCount; ++i) {
            JsonObject p = pts.add<JsonObject>();
            p["measuredPhysicalSpeedKmh"] = cfg.points[i].measuredPhysicalSpeedKmh;
            p["treadmillCommandKmh"] = cfg.points[i].treadmillCommandKmh;
        }
        serializeJson(doc, *stream);
        request->send(stream);
    };
    server_.on("/api/v1/calibration/speed", HTTP_GET, speedCalibrationGetHandler);

    // GET /api/v1/calibration/speed/calculate?speed=X: Query calculateCommand result
    server_.on("/api/v1/calibration/speed/calculate", HTTP_GET, [this](AsyncWebServerRequest* request) {
        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        stream->addHeader("Cache-Control", "no-cache");
        float targetSpd = 0.0f;
        if (request->hasParam("speed")) {
            targetSpd = request->getParam("speed")->value().toFloat();
        }
        SpeedCalibrationResult res{};
        if (commandStager_ != nullptr) {
            res = commandStager_->calculateSpeedCommand(targetSpd);
        } else {
            SpeedCalibration tempCal;
            tempCal.setConfiguration(SettingsService::instance().getSpeedConfig());
            res = tempCal.calculateCommand(targetSpd);
        }
        JsonDocument doc;
        doc["desiredPhysicalSpeedKmh"] = res.desiredPhysicalSpeedKmh;
        doc["treadmillCommandKmh"] = res.treadmillCommandKmh;
        doc["status"] = static_cast<uint8_t>(res.status);
        serializeJson(doc, *stream);
        request->send(stream);
    });

    // POST /api/v1/calibration/speed/sensor: Save one-point sensor calibration factor
    auto speedSensorCalHandler = [this](AsyncWebServerRequest* request) {
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

        float factor = 1.0f;
        if (doc.containsKey("factor")) {
            factor = doc["factor"].as<float>();
        } else if (doc.containsKey("systemSpeedKmh") && doc.containsKey("externalSpeedKmh")) {
            float sysSpd = doc["systemSpeedKmh"].as<float>();
            float extSpd = doc["externalSpeedKmh"].as<float>();
            if (!std::isfinite(sysSpd) || sysSpd <= 0.0f || !std::isfinite(extSpd) || extSpd <= 0.0f) {
                request->send(400, "application/json", "{\"error\":\"Speeds must be positive finite numbers\"}");
                return;
            }
            factor = extSpd / sysSpd;
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing factor or speed pair\"}");
            return;
        }

        if (!std::isfinite(factor) || factor < 0.5f || factor > 2.0f) {
            request->send(400, "application/json", "{\"error\":\"Calibration factor must be between 0.5 and 2.0\"}");
            return;
        }

        const bool saved = SettingsService::instance().saveSpeedSensorCalibrationFactor(factor);
        if (!saved) {
            request->send(500, "application/json", "{\"error\":\"Failed to persist sensor calibration factor\"}");
            return;
        }

        if (speedSensor_ != nullptr) {
            speedSensor_->setCalibrationFactor(factor);
        }

        JsonDocument resp;
        resp["status"] = "saved";
        resp["factor"] = factor;
        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        serializeJson(resp, *stream);
        request->send(stream);
    };
    server_.on("/api/v1/calibration/speed/sensor", HTTP_POST, speedSensorCalHandler, nullptr, commandBodyBuffer);

    // POST /api/v1/calibration/speed/table: Save full SpeedConfig points table
    auto speedTableSaveHandler = [this](AsyncWebServerRequest* request) {
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

        SpeedConfig candidate = SettingsService::instance().getSpeedConfig();
        if (doc.containsKey("maxAchievableSpeedKmh")) {
            candidate.maxAchievableSpeedKmh = doc["maxAchievableSpeedKmh"].as<float>();
        }
        if (doc.containsKey("maxAchievableSpeedVerified")) {
            candidate.maxAchievableSpeedVerified = doc["maxAchievableSpeedVerified"].as<bool>();
        }
        if (doc.containsKey("commandMapValid")) {
            candidate.commandMapValid = doc["commandMapValid"].as<bool>();
        }
        if (doc["points"].is<JsonArray>()) {
            JsonArray arr = doc["points"].as<JsonArray>();
            candidate.pointCount = std::min(arr.size(), kMaxSpeedCalibrationPoints);
            for (size_t i = 0; i < candidate.pointCount; ++i) {
                candidate.points[i].measuredPhysicalSpeedKmh = arr[i]["measuredPhysicalSpeedKmh"] | 0.0f;
                candidate.points[i].treadmillCommandKmh = arr[i]["treadmillCommandKmh"] | 0.0f;
            }
        } else if (doc.containsKey("points") && doc["points"].isNull()) {
            candidate.pointCount = 0;
            candidate.commandMapValid = false;
        }

        char errBuf[128] = {};
        if (!SpeedCalibration::validateCandidate(candidate, errBuf, sizeof(errBuf))) {
            JsonDocument errDoc;
            errDoc["error"] = errBuf[0] != '\0' ? errBuf : "Validation failed";
            AsyncResponseStream* stream = request->beginResponseStream("application/json");
            stream->setCode(400);
            serializeJson(errDoc, *stream);
            request->send(stream);
            return;
        }

        const bool saved = SettingsService::instance().saveSpeedConfig(candidate);
        if (!saved) {
            request->send(500, "application/json", "{\"error\":\"Failed to save speed configuration\"}");
            return;
        }

        if (commandStager_ != nullptr) {
            commandStager_->onSpeedConfigUpdated(candidate);
        }
        if (systemManager_ != nullptr) {
            systemManager_->onSpeedConfigUpdated(candidate);
        }

        request->send(200, "application/json", "{\"status\":\"saved\"}");
    };
    server_.on("/api/v1/calibration/speed/table", HTTP_POST, speedTableSaveHandler, nullptr, commandBodyBuffer);

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

    server_.on("/api/v1/heartrate/scan/start", HTTP_POST, [this](AsyncWebServerRequest* request) {
        if (hrClient_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"heart_rate_client_unavailable\"}");
            return;
        }
        hrClient_->startScan();
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });

    server_.on("/api/v1/heartrate/scan/results", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (hrClient_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"heart_rate_client_unavailable\"}");
            return;
        }
        BleScanResult results[HeartRateClient::kMaxScanResults];
        size_t count = hrClient_->getScanResults(results, HeartRateClient::kMaxScanResults);

        char connectedMac[18] = {};
        HeartRateState hrState = hrClient_->getState();
        if (hrState.connectionState == HeartRateConnectionState::Connected) {
            const char* liveAddr = (hrState.connectedAddress[0] != '\0')
                ? hrState.connectedAddress
                : hrState.sensorAddress;
            if (liveAddr[0] == '\0') {
                const auto* settings = SettingsService::instance().getActiveSettings();
                if (settings != nullptr) {
                    uint8_t targetUserId = 1;
                    if (telemetryProvider_ != nullptr) {
                        TelemetryReport r{};
                        if (telemetryProvider_->getTelemetry(r) && r.hasActiveUser && r.activeUserId != 0) {
                            targetUserId = r.activeUserId;
                        }
                    }
                    for (size_t u = 0; u < MAX_USERS; ++u) {
                        if (settings->users[u].id == targetUserId) {
                            liveAddr = settings->users[u].preferredHrMac;
                            break;
                        }
                    }
                }
            }
            strncpy(connectedMac, liveAddr, sizeof(connectedMac) - 1);
            connectedMac[sizeof(connectedMac) - 1] = '\0';
        }

        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (size_t i = 0; i < count; ++i) {
            if (results[i].advertisesHeartRateService && results[i].address[0] != '\0') {
                if (connectedMac[0] != '\0' && strcasecmp(results[i].address, connectedMac) == 0) {
                    continue;
                }
                JsonObject obj = arr.add<JsonObject>();
                obj["address"] = results[i].address;
                obj["name"] = results[i].name;
                obj["rssiDbm"] = results[i].rssiDbm;
            }
        }

        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        stream->addHeader("Cache-Control", "no-cache");
        serializeJson(doc, *stream);
        request->send(stream);
    });

    auto hrSaveHandler = [this](AsyncWebServerRequest* request) {
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

        if (err || !doc.containsKey("address")) {
            request->send(400, "application/json", "{\"error\":\"Invalid JSON or missing address\"}");
            return;
        }

        const char* address = doc["address"] | "";
        if (address[0] == '\0') {
            request->send(400, "application/json", "{\"error\":\"Empty address\"}");
            return;
        }

        uint8_t targetUserId = 0;
        if (doc.containsKey("userId")) {
            targetUserId = doc["userId"].as<uint8_t>();
        } else if (telemetryProvider_ != nullptr) {
            TelemetryReport r{};
            if (telemetryProvider_->getTelemetry(r) && r.hasActiveUser && r.activeUserId != 0) {
                targetUserId = r.activeUserId;
            }
        }
        if (targetUserId == 0) {
            const auto* settings = SettingsService::instance().getActiveSettings();
            if (settings != nullptr) {
                targetUserId = settings->users[0].id;
            } else {
                targetUserId = 1;
            }
        }

        SystemSettingsPtr candidate = makeSystemSettings();
        const SystemSettings* active = SettingsService::instance().getActiveSettings();
        if (active != nullptr) {
            *candidate = *active;
        }

        bool userFound = false;
        for (size_t i = 0; i < MAX_USERS; ++i) {
            if (candidate->users[i].id == targetUserId) {
                strncpy(candidate->users[i].preferredHrMac, address, sizeof(candidate->users[i].preferredHrMac) - 1);
                candidate->users[i].preferredHrMac[sizeof(candidate->users[i].preferredHrMac) - 1] = '\0';
                userFound = true;
                break;
            }
        }

        if (!userFound) {
            request->send(404, "application/json", "{\"error\":\"User not found\"}");
            return;
        }

        char errBuf[128] = {};
        if (!SettingsService::instance().updateSystemSettings(*candidate, errBuf, sizeof(errBuf))) {
            JsonDocument errDoc;
            errDoc["error"] = errBuf[0] ? errBuf : "Failed to persist settings";
            String resp;
            serializeJson(errDoc, resp);
            request->send(500, "application/json", resp);
            return;
        }
        SettingsService::instance().commitUsersJson();

        if (hrClient_ != nullptr) {
            hrClient_->stopScan();
            BleConfig bleCfg = hrClient_->getConfig();
            strncpy(bleCfg.preferredHrMac, address, sizeof(bleCfg.preferredHrMac) - 1);
            bleCfg.preferredHrMac[sizeof(bleCfg.preferredHrMac) - 1] = '\0';
            bleCfg.autoConnectHr = true;
            hrClient_->updateConfig(bleCfg);
        }

        request->send(200, "application/json", "{\"status\":\"ok\"}");
    };
    server_.on("/api/v1/heartrate/save", HTTP_POST, hrSaveHandler, nullptr, commandBodyBuffer);

    auto hrForgetHandler = [this](AsyncWebServerRequest* request) {
        if (request->getResponse() != nullptr) {
            if (request->_tempObject) {
                free(request->_tempObject);
                request->_tempObject = nullptr;
            }
            return;
        }

        uint8_t targetUserId = 0;
        if (request->_tempObject) {
            auto* buffer = static_cast<HttpBodyBuffer*>(request->_tempObject);
            if (buffer->received > 0 && buffer->received <= buffer->capacity) {
                JsonDocument doc;
                if (!deserializeJson(doc, buffer->data(), buffer->received)) {
                    if (doc.containsKey("userId")) {
                        targetUserId = doc["userId"].as<uint8_t>();
                    }
                }
            }
            free(buffer);
            request->_tempObject = nullptr;
        }

        if (targetUserId == 0 && telemetryProvider_ != nullptr) {
            TelemetryReport r{};
            if (telemetryProvider_->getTelemetry(r) && r.hasActiveUser && r.activeUserId != 0) {
                targetUserId = r.activeUserId;
            }
        }
        if (targetUserId == 0) {
            const auto* settings = SettingsService::instance().getActiveSettings();
            if (settings != nullptr) {
                targetUserId = settings->users[0].id;
            } else {
                targetUserId = 1;
            }
        }

        SystemSettingsPtr candidate = makeSystemSettings();
        const SystemSettings* active = SettingsService::instance().getActiveSettings();
        if (active != nullptr) {
            *candidate = *active;
        }

        bool userFound = false;
        for (size_t i = 0; i < MAX_USERS; ++i) {
            if (candidate->users[i].id == targetUserId) {
                candidate->users[i].preferredHrMac[0] = '\0';
                userFound = true;
                break;
            }
        }

        if (!userFound) {
            request->send(404, "application/json", "{\"error\":\"User not found\"}");
            return;
        }

        char errBuf[128] = {};
        if (!SettingsService::instance().updateSystemSettings(*candidate, errBuf, sizeof(errBuf))) {
            JsonDocument errDoc;
            errDoc["error"] = errBuf[0] ? errBuf : "Failed to persist settings";
            String resp;
            serializeJson(errDoc, resp);
            request->send(500, "application/json", resp);
            return;
        }
        SettingsService::instance().commitUsersJson();

        if (hrClient_ != nullptr) {
            BleConfig bleCfg = hrClient_->getConfig();
            bleCfg.preferredHrMac[0] = '\0';
            bleCfg.autoConnectHr = false;
            hrClient_->updateConfig(bleCfg);
            hrClient_->disconnect();
        }

        request->send(200, "application/json", "{\"status\":\"ok\"}");
    };
    server_.on("/api/v1/heartrate/forget", HTTP_POST, hrForgetHandler, nullptr, commandBodyBuffer);

    // GET /api/v1/heartrate/status
    auto hrStatusHandler = [this](AsyncWebServerRequest* request) {
        if (request->getResponse() != nullptr) {
            return;
        }

        uint8_t targetUserId = 0;
        if (telemetryProvider_ != nullptr) {
            TelemetryReport r{};
            if (telemetryProvider_->getTelemetry(r) && r.hasActiveUser && r.activeUserId != 0) {
                targetUserId = r.activeUserId;
            }
        }
        if (targetUserId == 0) {
            const auto* settings = SettingsService::instance().getActiveSettings();
            if (settings != nullptr) {
                targetUserId = settings->users[0].id;
            } else {
                targetUserId = 1;
            }
        }

        const char* savedAddress = "";
        const SystemSettings* active = SettingsService::instance().getActiveSettings();
        if (active != nullptr) {
            for (size_t i = 0; i < MAX_USERS; ++i) {
                if (active->users[i].id == targetUserId) {
                    savedAddress = active->users[i].preferredHrMac;
                    break;
                }
            }
        }

        bool connected = false;
        uint8_t batteryPercent = 0;
        bool batteryValid = false;
        String sensorName = "";
        String liveAddress = "";
        if (hrClient_ != nullptr) {
            HeartRateState hrState = hrClient_->getState();
            connected = (hrState.connectionState == HeartRateConnectionState::Connected);
            batteryPercent = hrState.batteryPercent;
            batteryValid = hrState.batteryPercentValid;
            if (hrState.sensorName[0] != '\0') {
                sensorName = hrState.sensorName;
            }
            if (hrState.connectedAddress[0] != '\0') {
                liveAddress = hrState.connectedAddress;
            } else if (hrState.sensorAddress[0] != '\0') {
                liveAddress = hrState.sensorAddress;
            }
        }

        const char* finalAddress = savedAddress;
        if (connected && liveAddress.length() > 0) {
            finalAddress = liveAddress.c_str();
        }

        JsonDocument doc;
        doc["savedAddress"] = finalAddress;
        doc["connected"] = connected;
        doc["batteryPercent"] = batteryPercent;
        doc["batteryValid"] = batteryValid;
        if (sensorName.length() > 0) {
            doc["name"] = sensorName;
        }
        String resp;
        serializeJson(doc, resp);
        request->send(200, "application/json", resp);
    };
    server_.on("/api/v1/heartrate/status", HTTP_GET, hrStatusHandler);

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
        cfg.calibratedAtMs = millis();
#if defined(STRIDECONTROL_TESTBENCH)
        cfg.source = CalibrationSource::Simulated;
#else
        cfg.source = CalibrationSource::PhysicalCommissioning;
#endif

        const bool ok = SettingsService::instance().saveRampCalibrationConfig(cfg);
        request->send(ok ? 200 : 500, "application/json",
            ok ? String("{\"status\":\"saved\",\"source\":\"") + (cfg.source == CalibrationSource::Simulated ? "Simulated" : "PhysicalCommissioning") + "\"}"
               : "{\"error\":\"save_failed\"}");
    };
    server_.on("/api/v1/commissioning/ramptest/save", HTTP_POST, rampTestSaveHandler, nullptr, commandBodyBuffer);

    // POST /api/v1/commissioning/incline/homing/start
    auto inclineHomingStartHandler = [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        ControlCommand cmd{};
        cmd.type = ControlCommandType::StartInclineHoming;
        cmd.timestampMs = millis();
        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/v1/commissioning/incline/homing/start", HTTP_POST, inclineHomingStartHandler);

    // POST /api/v1/commissioning/incline/measure/start
    auto inclineMeasureStartHandler = [this](AsyncWebServerRequest* request) {
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

        if (err || !doc["commandedPct"].is<float>() || !doc["direction"].is<const char*>()) {
            request->send(400, "application/json", "{\"error\":\"Invalid or missing commandedPct or direction\"}");
            return;
        }

        const char* dirStr = doc["direction"] | "";
        InclineDirection dir = InclineDirection::Unknown;
        if (strcmp(dirStr, "up") == 0 || strcmp(dirStr, "Up") == 0) {
            dir = InclineDirection::Up;
        } else if (strcmp(dirStr, "down") == 0 || strcmp(dirStr, "Down") == 0) {
            dir = InclineDirection::Down;
        } else {
            request->send(400, "application/json", "{\"error\":\"Invalid direction, must be up or down\"}");
            return;
        }

        ControlCommand cmd{};
        cmd.type = ControlCommandType::StartInclineMeasurePoint;
        cmd.timestampMs = millis();
        cmd.data.inclineMeasurePoint.commandedPct = doc["commandedPct"].as<float>();
        cmd.data.inclineMeasurePoint.expectedDirection = static_cast<uint8_t>(dir);

        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/v1/commissioning/incline/measure/start", HTTP_POST, inclineMeasureStartHandler, nullptr, commandBodyBuffer);

    // GET /api/v1/commissioning/incline/status
    server_.on("/api/v1/commissioning/incline/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }

        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Cache-Control", "no-cache");
        JsonDocument doc;
        doc["phase"] = inclineCommissioningPhaseToString(commandStager_->getInclineCommissioningPhase());
        doc["pointCount"] = commandStager_->getInclineCommissioningPointCount();
        doc["timedOut"] = commandStager_->didInclineCommissioningTimeOut();
        serializeJson(doc, *stream);
        request->send(stream);
    });

    // POST /api/v1/commissioning/incline/save
    auto inclineSaveHandler = [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        ControlCommand cmd{};
        cmd.type = ControlCommandType::SaveInclineCalibration;
        cmd.timestampMs = millis();
        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/v1/commissioning/incline/save", HTTP_POST, inclineSaveHandler);

    // POST /api/v1/commissioning/incline/abort
    auto inclineAbortHandler = [this](AsyncWebServerRequest* request) {
        if (commandStager_ == nullptr) {
            request->send(503, "application/json", "{\"error\":\"control_runtime_unavailable\"}");
            return;
        }
        ControlCommand cmd{};
        cmd.type = ControlCommandType::AbortInclineCommissioning;
        cmd.timestampMs = millis();
        if (commandStager_->stageCommand(cmd)) {
            request->send(200, "application/json", "{\"status\":\"queued\"}");
        } else {
            request->send(503, "application/json", "{\"error\":\"queue_full\"}");
        }
    };
    server_.on("/api/v1/commissioning/incline/abort", HTTP_POST, inclineAbortHandler);

    // =======================================================================
    // PHASE 5: REAL MAINTENANCE & BACKUP/RESTORE ENDPOINTS
    // =======================================================================

    // GET /api/v1/maintenance
    server_.on("/api/v1/maintenance", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        stream->addHeader("Cache-Control", "no-cache");
        MaintenanceConfig m = SettingsService::instance().getMaintenanceConfig();
        JsonDocument doc;
        doc["totalDistanceMeters"] = m.totalDistanceMeters;
        doc["totalTimeSeconds"] = m.totalTimeSeconds;
        doc["lastLubricationDate"] = m.lastLubricationDate;
        doc["lastLubricationTimeSeconds"] = m.lastLubricationTimeSeconds;
        doc["lubricationIntervalHours"] = m.lubricationIntervalHours;
        doc["lubricationIntervalDays"] = m.lubricationIntervalDays;
        serializeJson(doc, *stream);
        request->send(stream);
    });

    // POST /api/v1/maintenance
    auto maintenancePostHandler = [](AsyncWebServerRequest* request) {
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
        if (buffer->received == 0 || buffer->received != buffer->capacity) {
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

        MaintenanceConfig candidate = SettingsService::instance().getMaintenanceConfig();
        if (doc["totalDistanceMeters"].is<uint64_t>()) {
            candidate.totalDistanceMeters = doc["totalDistanceMeters"].as<uint64_t>();
        }
        if (doc["totalTimeSeconds"].is<uint64_t>()) {
            candidate.totalTimeSeconds = doc["totalTimeSeconds"].as<uint64_t>();
        }
        if (doc["lastLubricationDate"].is<const char*>()) {
            const char* dStr = doc["lastLubricationDate"].as<const char*>();
            strncpy(candidate.lastLubricationDate, dStr ? dStr : "", sizeof(candidate.lastLubricationDate) - 1);
            candidate.lastLubricationDate[sizeof(candidate.lastLubricationDate) - 1] = '\0';
        }
        if (doc["lastLubricationTimeSeconds"].is<uint64_t>()) {
            candidate.lastLubricationTimeSeconds = doc["lastLubricationTimeSeconds"].as<uint64_t>();
        }
        if (doc["lubricationIntervalHours"].is<uint32_t>()) {
            candidate.lubricationIntervalHours = doc["lubricationIntervalHours"].as<uint32_t>();
        }
        if (doc["lubricationIntervalDays"].is<uint32_t>()) {
            candidate.lubricationIntervalDays = doc["lubricationIntervalDays"].as<uint32_t>();
        }

        char errBuf[128]{};
        if (!SettingsService::validateMaintenanceConfig(candidate, errBuf, sizeof(errBuf))) {
            JsonDocument errDoc;
            errDoc["error"] = errBuf[0] != '\0' ? errBuf : "Validation failed";
            AsyncResponseStream* stream = request->beginResponseStream("application/json");
            stream->setCode(400);
            serializeJson(errDoc, *stream);
            request->send(stream);
            return;
        }

        bool ok = SettingsService::instance().saveMaintenanceConfig(candidate);
        if (!ok) {
            request->send(500, "application/json", "{\"error\":\"Failed to save maintenance config\"}");
            return;
        }

        request->send(200, "application/json", "{\"status\":\"saved\"}");
    };
    server_.on("/api/v1/maintenance", HTTP_POST, maintenancePostHandler, nullptr, commandBodyBuffer);

    // GET /api/v1/backup/export
    server_.on("/api/v1/backup/export", HTTP_GET, [](AsyncWebServerRequest* request) {
        AsyncResponseStream* stream = request->beginResponseStream("application/json");
        stream->addHeader("Access-Control-Allow-Origin", "*");
        stream->addHeader("Cache-Control", "no-cache");

        String scope = "complete";
        if (request->hasParam("scope") && request->getParam("scope")->value() == "machine") {
            scope = "machine";
        }

        char timeBuf[32];
        time_t now = time(nullptr);
        if (now > 1700000000) {
            struct tm tmInfo;
            gmtime_r(&now, &tmInfo);
            strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%dT%H:%M:%SZ", &tmInfo);
        } else {
            snprintf(timeBuf, sizeof(timeBuf), "millis_%lu", static_cast<unsigned long>(millis()));
        }

        JsonDocument doc;
        doc["schema"] = "stridecontrol-backup";
        doc["schemaVersion"] = 1;
        doc["scope"] = scope;
        doc["createdAt"] = timeBuf;
        doc["createdAtMs"] = millis();

        JsonObject manifest = doc["manifest"].to<JsonObject>();
        manifest["product"] = "StrideControl";
        manifest["firmwareVersion"] = "1.0.0";
        manifest["schemaVersion"] = 1;
        manifest["scope"] = scope;
        manifest["generatedAt"] = timeBuf;
        manifest["generatedAtMs"] = millis();

        // SpeedConfig
        SpeedConfig spd = SettingsService::instance().getSpeedConfig();
        JsonObject spdObj = doc["speed"].to<JsonObject>();
        spdObj["sensorCalibrationFactor"] = spd.sensorCalibrationFactor;
        spdObj["maxAchievableSpeedKmh"] = spd.maxAchievableSpeedKmh;
        spdObj["maxAchievableSpeedVerified"] = spd.maxAchievableSpeedVerified;
        spdObj["commandMapValid"] = spd.commandMapValid;
        spdObj["pointCount"] = spd.pointCount;
        JsonArray spdPts = spdObj["points"].to<JsonArray>();
        for (size_t i = 0; i < spd.pointCount; ++i) {
            JsonObject p = spdPts.add<JsonObject>();
            p["measuredPhysicalSpeedKmh"] = spd.points[i].measuredPhysicalSpeedKmh;
            p["treadmillCommandKmh"] = spd.points[i].treadmillCommandKmh;
        }

        // InclineConfig
        InclineConfig inc = SettingsService::instance().getInclineConfig();
        JsonObject incObj = doc["incline"].to<JsonObject>();
        incObj["maxAchievableInclinePct"] = inc.maxAchievableInclinePct;
        incObj["maxAchievableInclineVerified"] = inc.maxAchievableInclineVerified;
        incObj["commandMapValid"] = inc.commandMapValid;
        incObj["source"] = static_cast<uint8_t>(inc.source);
        incObj["calibratedAtMs"] = inc.calibratedAtMs;
        incObj["pointCount"] = inc.pointCount;
        JsonArray incPts = incObj["points"].to<JsonArray>();
        for (size_t i = 0; i < inc.pointCount; ++i) {
            JsonObject p = incPts.add<JsonObject>();
            p["measuredActualInclinePct"] = inc.points[i].measuredActualInclinePct;
            p["treadmillCommandPct"] = inc.points[i].treadmillCommandPct;
        }

        // MaintenanceConfig
        MaintenanceConfig m = SettingsService::instance().getMaintenanceConfig();
        JsonObject mObj = doc["maintenance"].to<JsonObject>();
        mObj["totalDistanceMeters"] = m.totalDistanceMeters;
        mObj["totalTimeSeconds"] = m.totalTimeSeconds;
        mObj["lastLubricationDate"] = m.lastLubricationDate;
        mObj["lastLubricationTimeSeconds"] = m.lastLubricationTimeSeconds;
        mObj["lubricationIntervalHours"] = m.lubricationIntervalHours;
        mObj["lubricationIntervalDays"] = m.lubricationIntervalDays;

        // RampCalibrationConfig
        RampCalibrationConfig r = SettingsService::instance().getRampCalibrationConfig();
        JsonObject rObj = doc["ramp"].to<JsonObject>();
        rObj["deadTimeMs"] = r.deadTimeMs;
        JsonArray aArr = rObj["accelMsPerKmh"].to<JsonArray>();
        JsonArray dArr = rObj["decelMsPerKmh"].to<JsonArray>();
        for (int i = 0; i < 3; ++i) {
            aArr.add(r.accelMsPerKmh[i]);
            dArr.add(r.decelMsPerKmh[i]);
        }
        rObj["loadMultiplier"] = r.loadMultiplier;
        rObj["calibrated"] = r.calibrated;
        rObj["source"] = static_cast<uint8_t>(r.source);
        rObj["calibratedAtMs"] = r.calibratedAtMs;

        // BleConfig
        JsonObject bleObj = doc["ble"].to<JsonObject>();
        bleObj["bleStackEnabled"] = SettingsService::instance().getBleStackEnabled();

        // SystemSettings (Users)
        JsonArray usersArr = doc["users"].to<JsonArray>();
        if (scope == "complete") {
            const SystemSettings* settings = SettingsService::instance().getActiveSettings();
            if (settings != nullptr) {
                for (size_t uIdx = 0; uIdx < MAX_USERS; ++uIdx) {
                    const UserProfile& u = settings->users[uIdx];
                    JsonObject uObj = usersArr.add<JsonObject>();
                    uObj["id"] = u.id;
                    uObj["name"] = u.name;
                    uObj["hvileSpeedKmh"] = u.hvileSpeedKmh;
                    uObj["dragSpeedKmh"] = u.dragSpeedKmh;
                    uObj["preferredHrMac"] = u.preferredHrMac;

                    JsonArray spdKeys = uObj["speedQuickKeys"].to<JsonArray>();
                    for (float k : u.speedQuickKeys) spdKeys.add(k);

                    JsonArray incKeys = uObj["inclineQuickKeys"].to<JsonArray>();
                    for (uint8_t k : u.inclineQuickKeys) incKeys.add(k);

                    uObj["selectedWorkoutId"] = u.selectedWorkoutId;

                    JsonArray recArr = uObj["recentWorkoutIds"].to<JsonArray>();
                    for (uint16_t rId : u.recentWorkoutIds) recArr.add(rId);

                    JsonArray wArr = uObj["workouts"].to<JsonArray>();
                    for (size_t wIdx = 0; wIdx < u.workoutCount; ++wIdx) {
                        const WorkoutDefinition& w = u.workouts[wIdx];
                        JsonObject wObj = wArr.add<JsonObject>();
                        wObj["id"] = w.id;
                        wObj["name"] = w.name;
                        wObj["lastUsedTimestamp"] = w.lastUsedTimestamp;

                        JsonArray segArr = wObj["segments"].to<JsonArray>();
                        for (size_t sIdx = 0; sIdx < w.segmentCount; ++sIdx) {
                            const WorkoutSegment& seg = w.segments[sIdx];
                            JsonObject segObj = segArr.add<JsonObject>();
                            segObj["id"] = seg.id;
                            segObj["type"] = segmentTypeName(seg.type);
                            segObj["repetitions"] = seg.repetitions;
                            segObj["startSpeedKmh"] = seg.startSpeedKmh;
                            segObj["speedProgressionPerRepKmh"] = seg.speedProgressionPerRepKmh;

                            JsonArray stArr = segObj["steps"].to<JsonArray>();
                            for (size_t stepIdx = 0; stepIdx < seg.stepCount; ++stepIdx) {
                                const WorkoutStep& st = seg.steps[stepIdx];
                                JsonObject stObj = stArr.add<JsonObject>();
                                stObj["id"] = st.id;
                                stObj["role"] = stepRoleName(st.role);
                                stObj["durationType"] = durationTypeName(st.durationType);
                                stObj["durationValue"] = st.durationValue;
                                stObj["speedMode"] = speedModeName(st.speedMode);
                                stObj["targetSpeedKmh"] = st.targetSpeedKmh;
                                stObj["targetInclinePct"] = st.targetInclinePct;
                                stObj["setIncline"] = st.setIncline;
                            }
                        }
                    }
                }
            }
        }

        serializeJson(doc, *stream);
        request->send(stream);
    });

    // Body buffer for backup restore (up to 64KB)
    auto backupBodyBuffer = [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        handleRequestBodyChunk(request, data, len, index, total, 65536, "{\"error\":\"Payload too large (max 64KB)\"}");
    };

    // POST /api/v1/backup/restore
    auto backupRestoreHandler = [this](AsyncWebServerRequest* request) {
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
        if (buffer->received == 0 || buffer->received != buffer->capacity) {
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

        // Schema validation
        int schemaVer = doc["schemaVersion"] | doc["manifest"]["schemaVersion"] | 0;
        if (schemaVer != 1) {
            request->send(400, "application/json", "{\"error\":\"Unsupported schema version (expected 1)\"}");
            return;
        }

        char errBuf[128]{};

        // 1. Pre-flight parse and validate SpeedConfig
        bool hasSpeed = doc["speed"].is<JsonObjectConst>();
        SpeedConfig candSpeed = SettingsService::instance().getSpeedConfig();
        if (hasSpeed) {
            JsonObjectConst sObj = doc["speed"].as<JsonObjectConst>();
            if (sObj["sensorCalibrationFactor"].is<float>()) {
                candSpeed.sensorCalibrationFactor = sObj["sensorCalibrationFactor"].as<float>();
            }
            if (sObj["maxAchievableSpeedKmh"].is<float>()) {
                candSpeed.maxAchievableSpeedKmh = sObj["maxAchievableSpeedKmh"].as<float>();
            }
            if (sObj["maxAchievableSpeedVerified"].is<bool>()) {
                candSpeed.maxAchievableSpeedVerified = sObj["maxAchievableSpeedVerified"].as<bool>();
            }
            if (sObj["commandMapValid"].is<bool>()) {
                candSpeed.commandMapValid = sObj["commandMapValid"].as<bool>();
            }
            if (sObj["points"].is<JsonArrayConst>()) {
                JsonArrayConst pts = sObj["points"].as<JsonArrayConst>();
                candSpeed.pointCount = std::min(pts.size(), kMaxSpeedCalibrationPoints);
                for (size_t i = 0; i < candSpeed.pointCount; ++i) {
                    candSpeed.points[i].measuredPhysicalSpeedKmh = pts[i]["measuredPhysicalSpeedKmh"] | 0.0f;
                    candSpeed.points[i].treadmillCommandKmh = pts[i]["treadmillCommandKmh"] | 0.0f;
                }
            }
            if (!SpeedCalibration::validateCandidate(candSpeed, errBuf, sizeof(errBuf))) {
                JsonDocument errDoc;
                errDoc["error"] = String("SpeedConfig validation failed: ") + (errBuf[0] ? errBuf : "invalid bounds");
                AsyncResponseStream* stream = request->beginResponseStream("application/json");
                stream->setCode(400);
                serializeJson(errDoc, *stream);
                request->send(stream);
                return;
            }
        }

        // 2. Pre-flight parse and validate InclineConfig
        bool hasIncline = doc["incline"].is<JsonObjectConst>();
        InclineConfig candIncline = SettingsService::instance().getInclineConfig();
        if (hasIncline) {
            JsonObjectConst iObj = doc["incline"].as<JsonObjectConst>();
            if (iObj["maxAchievableInclinePct"].is<float>()) {
                candIncline.maxAchievableInclinePct = iObj["maxAchievableInclinePct"].as<float>();
            }
            if (iObj["maxAchievableInclineVerified"].is<bool>()) {
                candIncline.maxAchievableInclineVerified = iObj["maxAchievableInclineVerified"].as<bool>();
            }
            if (iObj["commandMapValid"].is<bool>()) {
                candIncline.commandMapValid = iObj["commandMapValid"].as<bool>();
            }
            if (iObj["source"].is<uint8_t>()) {
                candIncline.source = static_cast<CalibrationSource>(iObj["source"].as<uint8_t>());
            }
            if (iObj["calibratedAtMs"].is<uint32_t>()) {
                candIncline.calibratedAtMs = iObj["calibratedAtMs"].as<uint32_t>();
            }
            if (iObj["points"].is<JsonArrayConst>()) {
                JsonArrayConst pts = iObj["points"].as<JsonArrayConst>();
                candIncline.pointCount = std::min(pts.size(), static_cast<size_t>(kMaxInclineCalibrationPoints));
                for (size_t i = 0; i < candIncline.pointCount; ++i) {
                    candIncline.points[i].measuredActualInclinePct = pts[i]["measuredActualInclinePct"] | 0.0f;
                    candIncline.points[i].treadmillCommandPct = pts[i]["treadmillCommandPct"] | 0.0f;
                }
            }
            if (!SettingsService::validateInclineConfig(candIncline, errBuf, sizeof(errBuf))) {
                JsonDocument errDoc;
                errDoc["error"] = String("InclineConfig validation failed: ") + (errBuf[0] ? errBuf : "invalid bounds");
                AsyncResponseStream* stream = request->beginResponseStream("application/json");
                stream->setCode(400);
                serializeJson(errDoc, *stream);
                request->send(stream);
                return;
            }
        }

        // 3. Pre-flight parse and validate MaintenanceConfig
        bool hasMaint = doc["maintenance"].is<JsonObjectConst>();
        MaintenanceConfig candMaint = SettingsService::instance().getMaintenanceConfig();
        if (hasMaint) {
            JsonObjectConst mObj = doc["maintenance"].as<JsonObjectConst>();
            if (mObj["totalDistanceMeters"].is<uint64_t>()) {
                candMaint.totalDistanceMeters = mObj["totalDistanceMeters"].as<uint64_t>();
            }
            if (mObj["totalTimeSeconds"].is<uint64_t>()) {
                candMaint.totalTimeSeconds = mObj["totalTimeSeconds"].as<uint64_t>();
            }
            if (mObj["lastLubricationDate"].is<const char*>()) {
                const char* dStr = mObj["lastLubricationDate"].as<const char*>();
                strncpy(candMaint.lastLubricationDate, dStr ? dStr : "", sizeof(candMaint.lastLubricationDate) - 1);
                candMaint.lastLubricationDate[sizeof(candMaint.lastLubricationDate) - 1] = '\0';
            }
            if (mObj["lastLubricationTimeSeconds"].is<uint64_t>()) {
                candMaint.lastLubricationTimeSeconds = mObj["lastLubricationTimeSeconds"].as<uint64_t>();
            }
            if (mObj["lubricationIntervalHours"].is<uint32_t>()) {
                candMaint.lubricationIntervalHours = mObj["lubricationIntervalHours"].as<uint32_t>();
            }
            if (mObj["lubricationIntervalDays"].is<uint32_t>()) {
                candMaint.lubricationIntervalDays = mObj["lubricationIntervalDays"].as<uint32_t>();
            }
            if (!SettingsService::validateMaintenanceConfig(candMaint, errBuf, sizeof(errBuf))) {
                JsonDocument errDoc;
                errDoc["error"] = String("MaintenanceConfig validation failed: ") + (errBuf[0] ? errBuf : "invalid bounds");
                AsyncResponseStream* stream = request->beginResponseStream("application/json");
                stream->setCode(400);
                serializeJson(errDoc, *stream);
                request->send(stream);
                return;
            }
        }

        // 4. Pre-flight parse and validate RampCalibrationConfig
        bool hasRamp = doc["ramp"].is<JsonObjectConst>();
        RampCalibrationConfig candRamp = SettingsService::instance().getRampCalibrationConfig();
        if (hasRamp) {
            JsonObjectConst rObj = doc["ramp"].as<JsonObjectConst>();
            if (rObj["deadTimeMs"].is<uint32_t>()) {
                candRamp.deadTimeMs = rObj["deadTimeMs"].as<uint32_t>();
            }
            if (rObj["accelMsPerKmh"].is<JsonArrayConst>()) {
                JsonArrayConst a = rObj["accelMsPerKmh"].as<JsonArrayConst>();
                for (size_t i = 0; i < 3 && i < a.size(); ++i) candRamp.accelMsPerKmh[i] = a[i] | 1000.0f;
            }
            if (rObj["decelMsPerKmh"].is<JsonArrayConst>()) {
                JsonArrayConst d = rObj["decelMsPerKmh"].as<JsonArrayConst>();
                for (size_t i = 0; i < 3 && i < d.size(); ++i) candRamp.decelMsPerKmh[i] = d[i] | 1000.0f;
            }
            if (rObj["loadMultiplier"].is<float>()) {
                candRamp.loadMultiplier = rObj["loadMultiplier"].as<float>();
            }
            if (rObj["calibrated"].is<bool>()) {
                candRamp.calibrated = rObj["calibrated"].as<bool>();
            }
            if (rObj["source"].is<uint8_t>()) {
                candRamp.source = static_cast<CalibrationSource>(rObj["source"].as<uint8_t>());
            }
            if (rObj["calibratedAtMs"].is<uint32_t>()) {
                candRamp.calibratedAtMs = rObj["calibratedAtMs"].as<uint32_t>();
            }
            if (!SettingsService::validateRampCalibrationConfig(candRamp, errBuf, sizeof(errBuf))) {
                JsonDocument errDoc;
                errDoc["error"] = String("RampCalibrationConfig validation failed: ") + (errBuf[0] ? errBuf : "invalid bounds");
                AsyncResponseStream* stream = request->beginResponseStream("application/json");
                stream->setCode(400);
                serializeJson(errDoc, *stream);
                request->send(stream);
                return;
            }
        }

        // 5. Pre-flight parse and validate BleConfig
        bool hasBle = doc["ble"].is<JsonObjectConst>();
        bool candBleEnabled = SettingsService::instance().getBleStackEnabled();
        if (hasBle) {
            JsonObjectConst bObj = doc["ble"].as<JsonObjectConst>();
            if (bObj["bleStackEnabled"].is<bool>()) {
                candBleEnabled = bObj["bleStackEnabled"].as<bool>();
            }
        }

        // 6. Pre-flight parse and validate SystemSettings (Users)
        bool hasUsers = doc["users"].is<JsonArrayConst>() && doc["users"].as<JsonArrayConst>().size() > 0;
        SystemSettingsPtr candUsers = makeSystemSettings();
        if (hasUsers && candUsers) {
            String usersJsonStr;
            JsonDocument uDoc;
            uDoc["schemaVersion"] = 1;
            uDoc["users"] = doc["users"];
            serializeJson(uDoc, usersJsonStr);

            if (!SettingsService::deserializeSettingsJson(
                reinterpret_cast<const uint8_t*>(usersJsonStr.c_str()), usersJsonStr.length(), *candUsers, errBuf, sizeof(errBuf))) {
                JsonDocument errDoc;
                errDoc["error"] = String("Users validation failed: ") + (errBuf[0] ? errBuf : "invalid structure");
                AsyncResponseStream* stream = request->beginResponseStream("application/json");
                stream->setCode(400);
                serializeJson(errDoc, *stream);
                request->send(stream);
                return;
            }
        }

        // --- All validations passed! Snapshot active configuration before commit ---
        captureLastKnownGoodSnapshot();

        // --- Sequential apply with automatic rollback on error ---
        bool ok = true;
        const char* failedSection = "";

        if (hasSpeed) {
            if (!SettingsService::instance().saveSpeedConfig(candSpeed)) {
                ok = false;
                failedSection = "speed";
            }
        }
        if (ok && hasIncline) {
            if (!SettingsService::instance().saveInclineConfig(candIncline)) {
                ok = false;
                failedSection = "incline";
            }
        }
        if (ok && hasMaint) {
            if (!SettingsService::instance().saveMaintenanceConfig(candMaint)) {
                ok = false;
                failedSection = "maintenance";
            }
        }
        if (ok && hasRamp) {
            if (!SettingsService::instance().saveRampCalibrationConfig(candRamp)) {
                ok = false;
                failedSection = "ramp";
            }
        }
        if (ok && hasBle) {
            if (!SettingsService::instance().saveBleStackEnabled(candBleEnabled)) {
                ok = false;
                failedSection = "ble";
            }
        }
        if (ok && hasUsers && candUsers) {
            char uErr[128]{};
            if (!SettingsService::instance().updateSystemSettings(*candUsers, uErr, sizeof(uErr))) {
                ok = false;
                failedSection = "users";
            } else {
                SettingsService::instance().commitUsersJson();
            }
        }

        if (!ok) {
            // Automatic rollback
            char rErr[128]{};
            rollbackToLastKnownGood(rErr, sizeof(rErr));
            JsonDocument errDoc;
            errDoc["error"] = String("Commit failed on section '") + failedSection + "'. Configuration rolled back to previous snapshot.";
            AsyncResponseStream* stream = request->beginResponseStream("application/json");
            stream->setCode(500);
            serializeJson(errDoc, *stream);
            request->send(stream);
            return;
        }

        if (hasSpeed) {
            if (commandStager_ != nullptr) {
                commandStager_->onSpeedConfigUpdated(candSpeed);
            }
            if (systemManager_ != nullptr) {
                systemManager_->onSpeedConfigUpdated(candSpeed);
            }
        }

        request->send(200, "application/json", "{\"status\":\"restored\"}");
    };
    server_.on("/api/v1/backup/restore", HTTP_POST, backupRestoreHandler, nullptr, backupBodyBuffer);

    // POST /api/v1/backup/rollback
    auto backupRollbackHandler = [this](AsyncWebServerRequest* request) {
        char errBuf[128]{};
        if (!rollbackToLastKnownGood(errBuf, sizeof(errBuf))) {
            JsonDocument errDoc;
            errDoc["error"] = errBuf[0] ? errBuf : "Rollback failed";
            AsyncResponseStream* stream = request->beginResponseStream("application/json");
            stream->setCode(400);
            serializeJson(errDoc, *stream);
            request->send(stream);
            return;
        }
        request->send(200, "application/json", "{\"status\":\"rolled_back\"}");
    };
    server_.on("/api/v1/backup/rollback", HTTP_POST, backupRollbackHandler);

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
#if defined(STRIDECONTROL_TESTBENCH)
                if (simRuntime_ != nullptr) {
                    bool targetState = true;
                    if (doc.containsKey("active")) {
                        targetState = doc["active"].as<bool>();
                    } else {
                        // Toggle if no explicit active flag provided (e.g. clicking Emergency Stop in simulator UI)
                        targetState = !simRuntime_->isEmergencyStopActive();
                    }
                    ok = simRuntime_->triggerEmergencyStop(targetState, millis());
                    request->send(ok ? 200 : 503, "application/json",
                        ok ? (targetState ? "{\"status\":\"estop_triggered\",\"active\":true}" : "{\"status\":\"estop_released\",\"active\":false}")
                           : "{\"error\":\"failed\"}");
                    return;
                }
#endif
                cmd.type = ControlCommandType::Stop;
                ok = commandStager_ ? commandStager_->stageCommand(cmd) : false;
            } else if (strcmp(btn, "EmergencyStopRelease") == 0 || strcmp(btn, "ResetEmergencyStop") == 0) {
#if defined(STRIDECONTROL_TESTBENCH)
                if (simRuntime_ != nullptr) {
                    ok = simRuntime_->triggerEmergencyStop(false, millis());
                    request->send(ok ? 200 : 503, "application/json", ok ? "{\"status\":\"estop_released\"}" : "{\"error\":\"failed\"}");
                    return;
                }
#endif
                request->send(200, "application/json", "{\"status\":\"ok\"}");
                return;
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
            } else if (strcmp(btn, "SetBeltSpeedDirect") == 0 || strcmp(btn, "SpinBeltDirect") == 0) {
#if defined(STRIDECONTROL_TESTBENCH)
                if (simRuntime_ != nullptr) {
                    simRuntime_->setSimBeltSpeedDirect(val);
                    request->send(200, "application/json", "{\"status\":\"belt_spun\"}");
                    return;
                }
#endif
                request->send(503, "application/json", "{\"error\":\"testbench_only\"}");
                return;
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
#endif // STRIDECONTROL_TESTBENCH
}

void WebServerManager::captureLastKnownGoodSnapshot() {
    lastKnownGoodSnapshot_.speed = SettingsService::instance().getSpeedConfig();
    lastKnownGoodSnapshot_.incline = SettingsService::instance().getInclineConfig();
    lastKnownGoodSnapshot_.maintenance = SettingsService::instance().getMaintenanceConfig();
    lastKnownGoodSnapshot_.ramp = SettingsService::instance().getRampCalibrationConfig();
    lastKnownGoodSnapshot_.bleStackEnabled = SettingsService::instance().getBleStackEnabled();
    const SystemSettings* cur = SettingsService::instance().getActiveSettings();
    if (cur != nullptr) {
        if (!lastKnownGoodSnapshot_.settings) {
            lastKnownGoodSnapshot_.settings = makeSystemSettings();
        }
        if (lastKnownGoodSnapshot_.settings) {
            *lastKnownGoodSnapshot_.settings = *cur;
        }
    }
    lastKnownGoodSnapshot_.capturedAtMs = millis();
    lastKnownGoodSnapshot_.valid = true;
}

bool WebServerManager::rollbackToLastKnownGood(char* errBuf, size_t errBufLen) {
    auto setErr = [&](const char* msg) {
        if (errBuf && errBufLen > 0) {
            strncpy(errBuf, msg, errBufLen - 1);
            errBuf[errBufLen - 1] = '\0';
        }
    };

    if (!lastKnownGoodSnapshot_.valid) {
        setErr("No valid snapshot available");
        return false;
    }

    bool allOk = true;
    if (!SettingsService::instance().saveSpeedConfig(lastKnownGoodSnapshot_.speed)) {
        allOk = false;
        setErr("Failed to rollback SpeedConfig");
    }
    if (!SettingsService::instance().saveInclineConfig(lastKnownGoodSnapshot_.incline)) {
        allOk = false;
        setErr("Failed to rollback InclineConfig");
    }
    if (!SettingsService::instance().saveMaintenanceConfig(lastKnownGoodSnapshot_.maintenance)) {
        allOk = false;
        setErr("Failed to rollback MaintenanceConfig");
    }
    if (!SettingsService::instance().saveRampCalibrationConfig(lastKnownGoodSnapshot_.ramp)) {
        allOk = false;
        setErr("Failed to rollback RampCalibrationConfig");
    }
    if (!SettingsService::instance().saveBleStackEnabled(lastKnownGoodSnapshot_.bleStackEnabled)) {
        allOk = false;
        setErr("Failed to rollback BleConfig");
    }
    if (lastKnownGoodSnapshot_.settings) {
        char sErr[128]{};
        if (!SettingsService::instance().updateSystemSettings(*lastKnownGoodSnapshot_.settings, sErr, sizeof(sErr))) {
            allOk = false;
            setErr(sErr[0] ? sErr : "Failed to rollback SystemSettings");
        } else {
            SettingsService::instance().commitUsersJson();
        }
    }

    if (commandStager_ != nullptr) {
        commandStager_->onSpeedConfigUpdated(lastKnownGoodSnapshot_.speed);
    }
    if (systemManager_ != nullptr) {
        systemManager_->onSpeedConfigUpdated(lastKnownGoodSnapshot_.speed);
    }

    return allOk;
}

} // namespace stridecontrol
