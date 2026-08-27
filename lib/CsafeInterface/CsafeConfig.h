#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

namespace stridecontrol {

/**
 * @brief Configuration for CsafeInterface module.
 *
 * NOTE ON DEDICATED UART OWNERSHIP:
 * While CsafeInterface is initialized (from begin() until end()), the configured
 * HardwareSerial instance is owned exclusively by CsafeInterface.
 * Application code must not read, write, or reconfigure the owned UART directly.
 */
struct CsafeConfig {
  HardwareSerial* serial = &Serial1;
  int txPin = 17;
  int rxPin = 40;
  uint32_t baudRate = 9600;
  uint32_t serialConfig = SERIAL_8N1;

  // Polling and Timing Defaults
  uint32_t pollIntervalMs = 200;           // Polling interval: 5 Hz
  uint32_t responseTimeoutMs = 150;        // Timeout per single-flight request
  uint32_t linkTimeoutMs = 1500;           // Physical link watchdog timeout
  uint32_t incompleteFrameTimeoutMs = 150; // Parser inter-byte timeout
  uint32_t errorConfirmationMs = 2500;     // Defensive State 15 persistence qualification

  // Execution Budget & Buffers
  uint16_t maximumRxBytesPerUpdate = 64;   // Maximum bytes read per update() cycle
};

}  // namespace stridecontrol

