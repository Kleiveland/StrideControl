#pragma once

#include <cstddef>
#include <cstdint>

#include "InclineVerifierConfig.h"
#include "InclineVerifierTypes.h"
#include "../InclineSensor/InclineSensorTypes.h"
#include "../ImuInterface/ImuTypes.h"

namespace stridecontrol {

/**
 * @brief Thread-safe observer and diagnostic verification engine for treadmill incline.
 *
 * Compares requested command targets, pulse-integrated tracking, and physical IMU pitch.
 */
class InclineVerifier {
public:
    InclineVerifier();
    ~InclineVerifier();

    InclineVerifier(const InclineVerifier&) = delete;
    InclineVerifier& operator=(const InclineVerifier&) = delete;

    bool begin(const InclineVerifierConfig& config = InclineVerifierConfig{});
    void end();

    void update(
        const InclineVerificationCommandInput& commandInput,
        const InclineState& inclineState,
        const ImuState& imuState,
        uint32_t nowMs
    );

    InclineVerifierState getState() const;
    InclineVerifierConfig configSnapshot() const;
    bool isReady() const;

    static const char* version();

private:
    struct Impl;
    Impl* impl_;
};

} // namespace stridecontrol

