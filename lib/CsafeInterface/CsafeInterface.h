#pragma once
#include <Arduino.h>
#include "CsafeConfig.h"
#include "CsafeTypes.h"

namespace stridecontrol {

/**
 * @brief Thread-safe, non-blocking CSAFE monitoring interface for DK-City APM900T.
 *
 * Implements dedicated UART ownership, periodic GetStatus polling (F1 80 80 F2),
 * frame parsing, XOR checksum validation, F3 byte unstuffing, lower-nibble state decoding,
 * State 15 qualification, link watchdogs, and comprehensive communication diagnostics.
 */
class CsafeInterface {
 public:
  CsafeInterface();

  /**
   * @brief Destructor for CsafeInterface.
   *
   * Safely performs UART teardown and releases synchronization primitives.
   * Note: Callers must ensure all external tasks have stopped invoking methods
   * on this instance prior to destroying it.
   */
  ~CsafeInterface();

  CsafeInterface(const CsafeInterface&) = delete;
  CsafeInterface& operator=(const CsafeInterface&) = delete;

  bool begin(const CsafeConfig& config = CsafeConfig{});
  void end();
  void update();

  CsafeState getState() const;
  CsafeConfig configSnapshot() const;
  bool isReady() const;

  static const char* version();

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace stridecontrol

