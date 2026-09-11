#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>

#if __has_include("secrets.h")
#include "secrets.h"
#elif __has_include("../../include/secrets.h")
#include "../../include/secrets.h"
#endif

namespace stridecontrol {

enum class NetworkState : uint8_t {
    Uninitialized,
    ConnectingSTA,
    ConnectedSTA,
    APFallback
};

class NetworkManager {
public:
    NetworkManager();
    ~NetworkManager() = default;

    // Non-copyable
    NetworkManager(const NetworkManager&) = delete;
    NetworkManager& operator=(const NetworkManager&) = delete;

    bool begin();
    void update();

    bool isConnected() const;
    bool isAP() const;
    IPAddress getIP() const;
    int8_t getRSSI() const;
    NetworkState getState() const;
    const char* getStateString() const;

private:
    void startSTA();
    void startAP();
    void initMDNS();

    NetworkState state_{NetworkState::Uninitialized};
    uint32_t staStartTimeMs_{0};
    uint32_t lastApRetryTimeMs_{0};
    uint32_t lastGoodStatusMs_{0};
    bool mdnsStarted_{false};
    DNSServer dnsServer_;

    static constexpr uint8_t kDnsPort = 53;
    static constexpr uint32_t kStaTimeoutMs = 15000;          // 15 seconds before AP fallback
    static constexpr uint32_t kApRetryIntervalMs = 60000;     // Retry STA every 60 seconds when in AP mode
    static constexpr uint32_t kDisconnectGraceMs = 3000;      // Require 3s sustained loss before reconnecting
    static constexpr const char* kApSsid = "StrideControl-Setup";
    static constexpr const char* kApPass = "stride1234";
};

} // namespace stridecontrol

