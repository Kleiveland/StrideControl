#pragma once
#include <stdint.h>
#include <stddef.h>

namespace stridecontrol {

enum class CsafeMachineState : uint8_t {
  Unknown = 0,
  Idle = 1,
  Ready = 2,
  Manual = 3,
  Offline = 4,
  Paused = 5,
  InUse = 6,
  Finished = 7,
  Upload = 8,
  Starting = 9,
  Error = 10
};

enum class CsafeLinkStatus : uint8_t {
  Uninitialized = 0,
  WaitingForFirstResponse = 1,
  Online = 2,
  TimedOut = 3,
  UartError = 4,
  HardwareError = 5
};

enum class CsafeParserState : uint8_t {
  Idle = 0,
  InFrame = 1,
  Escaped = 2,
  Discarding = 3
};

constexpr size_t CSAFE_MAX_RAW_FRAME_SIZE = 40;
constexpr size_t CSAFE_MAX_DECODED_FRAME_SIZE = 32;

struct CsafeFrame {
  uint8_t rawData[CSAFE_MAX_RAW_FRAME_SIZE] = {0};
  uint8_t rawLength = 0;

  uint8_t decodedData[CSAFE_MAX_DECODED_FRAME_SIZE] = {0};
  uint8_t decodedLength = 0;

  uint32_t timestampMs = 0;
  uint32_t sequence = 0;
  bool checksumValid = false;
};

struct CsafeState {
  // Lifecycle & Connection Flags
  bool initialized = false;
  bool online = false;
  bool awaitingResponse = false;
  bool machineStateFresh = false;

  // Central Link and Parser Status
  CsafeLinkStatus linkStatus = CsafeLinkStatus::Uninitialized;
  CsafeParserState parserState = CsafeParserState::Idle;

  // Decoded State Telemetry
  uint8_t rawStateByte = 0x00;
  uint8_t stateNibble = 0x00;
  uint8_t upperNibble = 0x00;

  CsafeMachineState reportedState = CsafeMachineState::Unknown;
  CsafeMachineState qualifiedState = CsafeMachineState::Unknown;

  // State 15 (Error) Qualification
  bool errorCandidateActive = false;
  bool errorConfirmed = false;
  uint32_t errorCandidateDurationMs = 0;
  uint32_t consecutiveErrorStateCount = 0;

  // Raw and Decoded Frame Snapshots
  CsafeFrame lastCompleteFrame{};
  CsafeFrame lastValidStateFrame{};

  // Watchdogs & Timestamps
  uint32_t lastReceivedByteMs = 0;
  uint32_t lastCompleteFrameMs = 0;
  uint32_t lastChecksumValidFrameMs = 0;
  uint32_t lastValidStateMs = 0;
  uint32_t lastRequestMs = 0;
  uint32_t validStateAgeMs = 0;
  uint32_t snapshotTimestampMs = 0;

  // Diagnostic Counters
  uint32_t transmittedStatusRequestCount = 0;
  uint32_t transmittedByteCount = 0;
  uint32_t receivedByteCount = 0;
  uint32_t completeFrameCount = 0;
  uint32_t checksumValidFrameCount = 0;
  uint32_t validStateFrameCount = 0;
  uint32_t checksumErrorCount = 0;
  uint32_t tooShortFrameCount = 0;
  uint32_t invalidEscapeCount = 0;
  uint32_t frameOverflowCount = 0;
  uint32_t parserResyncCount = 0;
  uint32_t incompleteFrameTimeoutCount = 0;
  uint32_t responseTimeoutCount = 0;
  uint32_t linkTimeoutCount = 0;
  uint32_t unknownPayloadCount = 0;
  uint32_t unknownStateCount = 0;
  uint32_t rejectedTransmitCount = 0;
  uint32_t uartWriteErrorCount = 0;
};

const char* csafeMachineStateName(CsafeMachineState state);
const char* csafeLinkStatusName(CsafeLinkStatus status);
const char* csafeParserStateName(CsafeParserState state);

}  // namespace stridecontrol

