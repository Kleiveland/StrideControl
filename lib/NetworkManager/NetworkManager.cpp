#include "NetworkManager.h"
#include "../DiagnosticsLog/DiagnosticsLog.h"

namespace stridecontrol {

static const char* getMdnsHostname() {
    static char s_hostname[64]{};
    static bool s_initialized = false;
    if (!s_initialized) {
#if defined(STRIDECONTROL_TESTBENCH)
        const char* base = "stridecontrol";
#else
        const char* base = "sportsmaster";
#endif
        snprintf(s_hostname, sizeof(s_hostname), "%s", base);
        size_t len = strlen(s_hostname);
        if (len > 63) len = 63;
        size_t outIdx = 0;
        for (size_t i = 0; i < len; ++i) {
            char c = s_hostname[i];
            if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-') {
                s_hostname[outIdx++] = c;
            } else if (c >= 'A' && c <= 'Z') {
                s_hostname[outIdx++] = static_cast<char>(c + ('a' - 'A'));
            }
        }
        s_hostname[outIdx] = '\0';
        if (outIdx == 0) {
#if defined(STRIDECONTROL_TESTBENCH)
            strncpy(s_hostname, "stridecontrol", sizeof(s_hostname) - 1);
#else
            strncpy(s_hostname, "sportsmaster", sizeof(s_hostname) - 1);
#endif
            s_hostname[sizeof(s_hostname) - 1] = '\0';
        }
        s_initialized = true;
    }
    return s_hostname;
}

NetworkManager::NetworkManager() = default;

bool NetworkManager::begin() {
    Serial.println("[NetworkManager] Initializing network subsystem...");
    startSTA();
    return true;
}

void NetworkManager::startSTA() {
    dnsServer_.stop();
    state_ = NetworkState::ConnectingSTA;
    staStartTimeMs_ = millis();

    if (strlen(Secrets::WIFI_SSID) == 0 || strcmp(Secrets::WIFI_SSID, "Your_WiFi_SSID") == 0) {
        Serial.println("[NetworkManager] No valid Wi-Fi credentials configured. Falling back to AP mode immediately.");
        startAP();
        return;
    }

    Serial.printf("[NetworkManager] Connecting to Wi-Fi SSID: %s (Timeout: %u ms)...\n",
                  Secrets::WIFI_SSID, static_cast<unsigned int>(kStaTimeoutMs));

    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    WiFi.setHostname(getMdnsHostname());
    WiFi.begin(Secrets::WIFI_SSID, Secrets::WIFI_PASS);
}

void NetworkManager::startAP() {
    state_ = NetworkState::APFallback;
    lastApRetryTimeMs_ = millis();

    Serial.printf("[NetworkManager] Starting AP Fallback (SSID: %s, Pass: %s)...\n", kApSsid, kApPass);
    DiagnosticsLog::instance().addEntryf("[NetworkManager] Starting AP Fallback (SSID: %s, Pass: %s)...", kApSsid, kApPass);

    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    WiFi.softAP(kApSsid, kApPass);
    dnsServer_.start(kDnsPort, "*", WiFi.softAPIP());
    Serial.println("[NetworkManager] Captive portal DNS started (wildcard -> AP IP).");

    Serial.print("[NetworkManager] AP IP Address: ");
    Serial.println(WiFi.softAPIP());

    initMDNS();
}

void NetworkManager::initMDNS() {
    if (!mdnsStarted_) {
        const char* hostname = getMdnsHostname();
        if (MDNS.begin(hostname)) {
            mdnsStarted_ = true;
            MDNS.addService("http", "tcp", 80);
            Serial.printf("[NetworkManager] mDNS responder started: http://%s.local\n", hostname);
        } else {
            Serial.println("[NetworkManager] Failed to start mDNS responder.");
        }
    }
}

void NetworkManager::update() {
    const uint32_t nowMs = millis();

    switch (state_) {
        case NetworkState::ConnectingSTA: {
            if (WiFi.status() == WL_CONNECTED) {
                state_ = NetworkState::ConnectedSTA;
                lastGoodStatusMs_ = nowMs;
                Serial.printf("[NetworkManager] Wi-Fi Connected! IP: %s, RSSI: %d dBm\n",
                              WiFi.localIP().toString().c_str(), WiFi.RSSI());
                DiagnosticsLog::instance().addEntryf("[NetworkManager] Wi-Fi Connected! IP: %s, RSSI: %d dBm",
                                                     WiFi.localIP().toString().c_str(), WiFi.RSSI());
                initMDNS();
            } else if (nowMs - staStartTimeMs_ >= kStaTimeoutMs) {
                Serial.println("[NetworkManager] Wi-Fi connection timed out. Transitioning to AP mode.");
                startAP();
            }
            break;
        }

        case NetworkState::ConnectedSTA: {
            if (WiFi.status() == WL_CONNECTED) {
                lastGoodStatusMs_ = nowMs;
            } else if (nowMs - lastGoodStatusMs_ >= kDisconnectGraceMs) {
                Serial.println("[NetworkManager] Wi-Fi connection lost (sustained). Attempting reconnection...");
                DiagnosticsLog::instance().addEntry("[NetworkManager] Wi-Fi connection lost (sustained). Attempting reconnection...");
                startSTA();
            }
            break;
        }

        case NetworkState::APFallback: {
            dnsServer_.processNextRequest();
            if (nowMs - lastApRetryTimeMs_ >= kApRetryIntervalMs) {
                Serial.println("[NetworkManager] Periodic check: retrying STA connection from AP mode...");
                startSTA();
            }
            break;
        }

        case NetworkState::Uninitialized:
        default:
            break;
    }
}

bool NetworkManager::isConnected() const {
    return state_ == NetworkState::ConnectedSTA && (WiFi.status() == WL_CONNECTED);
}

bool NetworkManager::isAP() const {
    return state_ == NetworkState::APFallback;
}

IPAddress NetworkManager::getIP() const {
    if (state_ == NetworkState::ConnectedSTA) {
        return WiFi.localIP();
    } else if (state_ == NetworkState::APFallback) {
        return WiFi.softAPIP();
    }
    return IPAddress(0, 0, 0, 0);
}

int8_t NetworkManager::getRSSI() const {
    if (state_ == NetworkState::ConnectedSTA) {
        return static_cast<int8_t>(WiFi.RSSI());
    }
    return 0;
}

NetworkState NetworkManager::getState() const {
    return state_;
}

const char* NetworkManager::getStateString() const {
    switch (state_) {
        case NetworkState::Uninitialized: return "UNINITIALIZED";
        case NetworkState::ConnectingSTA: return "CONNECTING_STA";
        case NetworkState::ConnectedSTA:  return "CONNECTED_STA";
        case NetworkState::APFallback:   return "AP_FALLBACK";
        default:                         return "UNKNOWN";
    }
}

} // namespace stridecontrol

