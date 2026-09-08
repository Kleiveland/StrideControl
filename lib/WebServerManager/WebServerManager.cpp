#include "WebServerManager.h"
#include <WiFi.h>
#include <LittleFS.h>
#include "FileSystemManager.h"
#include "SettingsService.h"
#include <vector>

namespace stridecontrol {

WebServerManager::WebServerManager(uint16_t port)
    : server_(port) {}

WebServerManager::~WebServerManager() {
    end();
}

bool WebServerManager::begin(const ITelemetryProvider* telemetryProvider) {
    if (running_) {
        return true;
    }

    telemetryProvider_ = telemetryProvider;
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

        JsonObject incline = doc["incline"].to<JsonObject>();
        incline["pct"] = report.actualInclinePct;

        JsonObject runner = doc["runner"].to<JsonObject>();
        runner["distanceKm"] = report.runnerDistanceKm;

        JsonObject session = doc["session"].to<JsonObject>();
        session["state"] = report.sessionState;
        session["stepIndex"] = report.stepIndex;
        session["stepRemainingMs"] = report.stepRemainingMs;

        doc["targetSpeedKmh"] = report.targetSpeedKmh;
        doc["targetInclinePct"] = report.targetInclinePct;

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
            if (!request->_tempObject) {
                request->send(400, "application/json", "{\"error\":\"Missing request body\"}");
                return;
            }

            auto* buffer = static_cast<std::vector<uint8_t>*>(request->_tempObject);
            if (buffer->empty()) {
                delete buffer;
                request->_tempObject = nullptr;
                request->send(400, "application/json", "{\"error\":\"Empty payload\"}");
                return;
            }

            SystemSettingsPtr candidate = makeSystemSettings();
            if (!candidate) {
                delete buffer;
                request->_tempObject = nullptr;
                request->send(500, "application/json", "{\"error\":\"Memory allocation failed\"}");
                return;
            }

            char errBuf[128] = {};
            bool parseOk = SettingsService::deserializeSettingsJson(
                buffer->data(), buffer->size(), *candidate, errBuf, sizeof(errBuf)
            );

            delete buffer;
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
            if (total > 32768) {
                request->send(413, "application/json", "{\"error\":\"Payload too large (max 32KB)\"}");
                return;
            }

            std::vector<uint8_t>* buffer = nullptr;
            if (index == 0) {
                buffer = new std::vector<uint8_t>();
                buffer->reserve(total);
                request->_tempObject = buffer;
            } else {
                buffer = static_cast<std::vector<uint8_t>*>(request->_tempObject);
            }

            if (buffer != nullptr && data != nullptr && len > 0) {
                buffer->insert(buffer->end(), data, data + len);
            }
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
}

} // namespace stridecontrol
