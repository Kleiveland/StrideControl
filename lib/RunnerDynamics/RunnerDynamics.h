#pragma once

#include <Arduino.h>
#include "RunnerDynamicsConfig.h"
#include "RunnerDynamicsTypes.h"
#include "../ImuInterface/ImuTypes.h"
#include "../SpeedSensor/SpeedSensorTypes.h"
#include "../CsafeInterface/CsafeTypes.h"

namespace stridecontrol {

/**
 * @brief Thread-safe signal processing engine for runner dynamics, step detection,
 * presence qualification, candidate tracking, side-rail monitoring, and validated distance integration.
 *
 * Concurrency & Thread-Safety:
 * - Private working state and internal algorithms are owned and serialized by a FreeRTOS mutex (opMutex).
 * - Public state snapshots and config snapshots are protected by a FreeRTOS spinlock (stateMux).
 * - update() acquires opMutex in a non-blocking manner.
 * - Callers must ensure all concurrent operations have ceased before destroying this object.
 */
class RunnerDynamics {
public:
    RunnerDynamics();
    ~RunnerDynamics();

    RunnerDynamics(const RunnerDynamics&) = delete;
    RunnerDynamics& operator=(const RunnerDynamics&) = delete;

    bool begin(const RunnerDynamicsConfig& config = RunnerDynamicsConfig{});
    void end();
    void update(const ImuSample* samples,
                size_t sampleCount,
                const SpeedSensorState& speedState,
                const CsafeState& csafeState,
                uint32_t nowMs);

    RunnerDynamicsState getState() const;
    RunnerDynamicsConfig configSnapshot() const;
    void resetStepCounter();
    void resetValidatedDistance();
    void resetSession();
    bool isReady() const;

    static const char* version();

private:
    struct Impl;
    Impl* impl_;
};

} // namespace stridecontrol