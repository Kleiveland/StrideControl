#pragma once
#include <Arduino.h>
#include "ConsoleTypes.h"

namespace stridecontrol {

struct ButtonMapping {
  ButtonId button = ButtonId::Unknown;
  uint8_t targetPhase = 0xFF;
  uint8_t activeO2 = 0xFF;
  uint8_t preloadPhase = 0xFF;
  uint8_t preloadO2 = 0xFF;
  bool verified = false;

  ButtonMapping() = default;
  ButtonMapping(ButtonId b, uint8_t tp, uint8_t ao, uint8_t pp, uint8_t po, bool v)
      : button(b), targetPhase(tp), activeO2(ao), preloadPhase(pp), preloadO2(po), verified(v) {}
};

struct ConsolePins {
  gpio_num_t txsOe = GPIO_NUM_2;
  gpio_num_t muteAll = GPIO_NUM_21;
  gpio_num_t estopIn = GPIO_NUM_18;
  gpio_num_t buzzerIn = GPIO_NUM_16;
  gpio_num_t speedPlusRelay = GPIO_NUM_39;
  gpio_num_t speedMinusRelay = GPIO_NUM_38;
  int o1[5] = {4, 5, 6, 7, 15};
  int o2[8] = {41, 42, 8, 9, 10, 11, 12, 13};
};

struct ConsoleTiming {
  // Siffer timing basert på V5.6.2 analysen
  uint16_t digitHoldMs = 82;
  uint16_t firstDigitHoldMs = 82;
  uint16_t digitPauseMs = 150;
  uint16_t firstDigitPauseMs = 150;
  uint16_t instantHoldMs = 200;
  uint16_t postInstantPauseMs = 250;
  uint16_t preEnterPauseMs = 200;
  uint16_t enterHoldMs = 82;
  uint16_t enterPauseMs = 50;
  uint16_t clearHoldMs = 82;
  uint16_t postClearPauseMs = 300;
  uint16_t relayHoldMs = 80;
  uint16_t relayPauseMs = 80;
  uint16_t busToRelayDelayMs = 150;
  
  // ACK og Recovery timing
  uint16_t ackTimeoutMs = 450;
  uint16_t retryCooldownMs = 300;
  uint8_t maxLocalRetries = 1;
  uint8_t maxSequenceRestarts = 2;
  
  // Sikkerhet og poll-grenser
  uint32_t muteSwitchDelayUs = 50;
  uint32_t syncTimeoutMs = 30;
  uint32_t idleWaitTimeoutMs = 30;
  uint32_t phaseWatchdogMs = 35;
  uint32_t endSyncTimeoutMs = 35;
  uint32_t idleSwitchUs = 1200;
  uint32_t eventEndGapUs = 40000;
  uint32_t minBuzzerSegmentUs = 1000;
  uint32_t normalEnvelopeMaxUs = 110000;
  uint32_t ackGuardMs = 80;
  uint32_t passiveStableMs = 30;
  uint32_t passivePollMs = 5;
};

struct ConsoleConfig {
  ConsolePins pins{};
  ConsoleTiming timing{};
  uint8_t requestQueueDepth = 16;
  uint8_t commandEventQueueDepth = 48;
  uint8_t physicalEventQueueDepth = 24;
  
  ButtonMapping clearMapping{ButtonId::Clear, 0x0F, 0x81, 0xFF, 0xFF, true};
};

}  // namespace stridecontrol