#pragma once
#include <Arduino.h>
#include "ConsoleConfig.h"
#include "ConsoleTypes.h"

namespace stridecontrol {

class ConsoleInterface {
 public:
  ConsoleInterface();
  ~ConsoleInterface();

  ConsoleInterface(const ConsoleInterface&) = delete;
  ConsoleInterface& operator=(const ConsoleInterface&) = delete;

  bool begin(const ConsoleConfig& config = ConsoleConfig{});
  void end();

  bool submit(const TreadmillCommand& command, TickType_t waitTicks = 0);
  bool receiveCommandEvent(CommandEvent& event, TickType_t waitTicks = 0);
  bool receivePhysicalButtonEvent(PhysicalButtonEvent& event, TickType_t waitTicks = 0);

  bool isActive() const;
  bool isReady() const;
  void abortActiveCommand();

  ConsoleConfig configSnapshot() const;
  bool setClearMapping(const ButtonMapping& verifiedMapping);
  bool clearMappingVerified() const;

  ConsoleOutcome pressButton(ButtonId button, uint16_t holdMs, AckMetrics& metrics);

  static const char* version();

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace stridecontrol