#pragma once
#include <Arduino.h>

namespace stridecontrol {

enum class ButtonId : uint8_t {
  InstantSpeed, InstantIncline, Enter, Clear,
  Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
  SpeedPlus, SpeedMinus, InclinePlus, InclineMinus,
  QuickStart, Stop, Unknown
};

enum class CommandType : uint8_t {
  SetSpeed, SetIncline, PressSpeedPlus, PressSpeedMinus, PressButton
};

enum class CommandStatus : uint8_t {
  Queued, Started, StepStarted, StepConfirmed, Retrying, 
  RecoveryRequired, Completed, Rejected, Failed, Aborted
};

enum class ConsoleOutcome : uint8_t {
  NormalSingle, NoResponse, NormalLong, MergedMulti, DistinctMulti, 
  EdgeLoss, Invalid, SyncFailure, IdleTimeout, PhaseTimeout, 
  WatchdogTimeout, ClearMappingUnverified, ClearFailed, 
  RecoveryUnavailable, Aborted, HardwareError
};

enum class PhysicalButtonAction : uint8_t {
  Pressed, Released
};

struct TreadmillCommand {
  uint32_t requestId = 0;
  CommandType type = CommandType::PressButton;
  float value = 0.0f;
  ButtonId button = ButtonId::Unknown;
};

struct AckMetrics {
  uint32_t startLatencyUs = 0;
  uint32_t completionLatencyUs = 0;
  uint32_t envelopeUs = 0;
  uint32_t activeUs = 0;
  uint32_t longestInternalGapUs = 0;
  uint16_t rawEdgeCount = 0;
  uint8_t segmentCount = 0;
  uint8_t eventCount = 0;
  bool noisy = false;
  bool late = false;
};

struct CommandEvent {
  uint32_t requestId = 0;
  CommandStatus status = CommandStatus::Failed;
  ConsoleOutcome outcome = ConsoleOutcome::HardwareError;
  ButtonId button = ButtonId::Unknown;
  float requestedValue = 0.0f;
  float normalizedValue = 0.0f;
  uint8_t step = 0;
  uint8_t stepCount = 0;
  uint8_t localAttempt = 0;
  uint8_t sequenceRestart = 0;
  uint32_t timestampMs = 0;
  AckMetrics ack{};
  char detail[160] = {};
};

struct PhysicalButtonEvent {
  ButtonId button = ButtonId::Unknown;
  PhysicalButtonAction action = PhysicalButtonAction::Pressed;
  uint32_t timestampMs = 0;
  uint32_t durationMs = 0;
  uint8_t observedO1 = 0xFF;
  uint8_t observedO2 = 0xFF;

  PhysicalButtonEvent() = default;
  PhysicalButtonEvent(ButtonId b, PhysicalButtonAction a, uint32_t ts, uint32_t d, uint8_t o1, uint8_t o2)
      : button(b), action(a), timestampMs(ts), durationMs(d), observedO1(o1), observedO2(o2) {}
};

const char* buttonName(ButtonId button);
const char* outcomeName(ConsoleOutcome outcome);
const char* commandStatusName(CommandStatus status);

}  // namespace stridecontrol