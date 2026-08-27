#include "CsafeInterface.h"
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/semphr.h>
#include <string.h>

namespace stridecontrol {

namespace {

constexpr const char* kCsafeInterfaceVersion = "1.0.0";
constexpr TickType_t kLifecycleMutexTimeoutTicks = pdMS_TO_TICKS(1000);
constexpr TickType_t kUpdateMutexTimeoutTicks = 0;

// CSAFE Protocol Framing Delimiters & Escape Codes
constexpr uint8_t kFrameStart = 0xF1;
constexpr uint8_t kFrameEnd   = 0xF2;
constexpr uint8_t kByteEscape = 0xF3;
constexpr uint8_t kEscapedF1  = 0x01;
constexpr uint8_t kEscapedF2  = 0x02;
constexpr uint8_t kEscapedF3  = 0x03;

// Verified Polling Command Opcode
constexpr uint8_t kGetStatusCmd = 0x80;

// Forbidden Opcodes (Must never be transmitted)
constexpr uint8_t kForbiddenAA = 0xAA;
constexpr uint8_t kForbiddenA5 = 0xA5;
constexpr uint8_t kForbidden9C = 0x9C;

class MutexLock {
 public:
  explicit MutexLock(SemaphoreHandle_t mutex, TickType_t waitTicks)
      : mutex_(mutex), locked_(false) {
    if (mutex_ != nullptr && xSemaphoreTake(mutex_, waitTicks) == pdTRUE) {
      locked_ = true;
    }
  }

  ~MutexLock() {
    if (locked_ && mutex_ != nullptr) {
      xSemaphoreGive(mutex_);
    }
  }

  bool isLocked() const {
    return locked_;
  }

 private:
  SemaphoreHandle_t mutex_;
  bool locked_;
};

bool validateConfiguration(const CsafeConfig& config) {
  if (config.serial == nullptr) return false;
  if (config.txPin < 0 || config.rxPin < 0 || config.txPin == config.rxPin) return false;
  if (config.baudRate != 9600) return false;
  if (config.serialConfig != SERIAL_8N1) return false;
  if (config.pollIntervalMs <= config.responseTimeoutMs) return false;
  if (config.pollIntervalMs == 0 || config.responseTimeoutMs == 0 ||
      config.incompleteFrameTimeoutMs == 0 || config.errorConfirmationMs == 0 ||
      config.maximumRxBytesPerUpdate == 0) {
    return false;
  }
  if (config.linkTimeoutMs <= config.pollIntervalMs || config.linkTimeoutMs <= config.responseTimeoutMs) {
    return false;
  }
  return true;
}

CsafeMachineState decodeMachineState(uint8_t stateNibble) {
  switch (stateNibble) {
    case 0x00: return CsafeMachineState::Idle;
    case 0x01: return CsafeMachineState::Ready;
    case 0x02: return CsafeMachineState::Manual;
    case 0x03: return CsafeMachineState::Offline;
    case 0x04: return CsafeMachineState::Paused;
    case 0x05: return CsafeMachineState::InUse;
    case 0x06: return CsafeMachineState::Finished;
    case 0x07: return CsafeMachineState::Upload;
    case 0x08: return CsafeMachineState::Starting;
    case 0x0F: return CsafeMachineState::Error;
    default:   return CsafeMachineState::Unknown;
  }
}

}  // namespace

const char* csafeMachineStateName(CsafeMachineState state) {
  switch (state) {
    case CsafeMachineState::Unknown:  return "UNKNOWN";
    case CsafeMachineState::Idle:     return "IDLE";
    case CsafeMachineState::Ready:    return "READY";
    case CsafeMachineState::Manual:   return "MANUAL";
    case CsafeMachineState::Offline:  return "OFFLINE";
    case CsafeMachineState::Paused:   return "PAUSED";
    case CsafeMachineState::InUse:    return "IN_USE";
    case CsafeMachineState::Finished: return "FINISHED";
    case CsafeMachineState::Upload:   return "UPLOAD";
    case CsafeMachineState::Starting: return "STARTING";
    case CsafeMachineState::Error:    return "ERROR";
    default:                          return "UNKNOWN";
  }
}

const char* csafeLinkStatusName(CsafeLinkStatus status) {
  switch (status) {
    case CsafeLinkStatus::Uninitialized:            return "UNINITIALIZED";
    case CsafeLinkStatus::WaitingForFirstResponse:  return "WAITING_FOR_FIRST_RESPONSE";
    case CsafeLinkStatus::Online:                   return "ONLINE";
    case CsafeLinkStatus::TimedOut:                 return "TIMED_OUT";
    case CsafeLinkStatus::UartError:                return "UART_ERROR";
    case CsafeLinkStatus::HardwareError:            return "HARDWARE_ERROR";
    default:                                        return "UNKNOWN";
  }
}

const char* csafeParserStateName(CsafeParserState state) {
  switch (state) {
    case CsafeParserState::Idle:       return "IDLE";
    case CsafeParserState::InFrame:    return "IN_FRAME";
    case CsafeParserState::Escaped:    return "ESCAPED";
    case CsafeParserState::Discarding: return "DISCARDING";
    default:                           return "UNKNOWN";
  }
}

struct CsafeInterface::Impl {
  // FreeRTOS Task Mutex serializing all hardware operations and model mutations
  SemaphoreHandle_t opMutex = nullptr;

  // Spinlock for atomic read snapshot
  mutable portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

  // Private Configuration (owned by opMutex)
  CsafeConfig config{};

  // Public Configuration Snapshot (protected by stateMux)
  CsafeConfig publicConfigSnapshot{};

  // Lifecycle & Connection Flags (owned by opMutex)
  bool initialized = false;
  bool linkOnline = false;
  bool awaitingResponse = false;
  bool uartError = false;
  bool initialTimeoutLatched = false;
  uint32_t beginTimeMs = 0;

  // Parser Working Storage (owned by opMutex)
  CsafeParserState parserState = CsafeParserState::Idle;
  uint8_t rawParserBuf[CSAFE_MAX_RAW_FRAME_SIZE] = {0};
  uint8_t rawParserLen = 0;
  uint8_t decodedParserBuf[CSAFE_MAX_DECODED_FRAME_SIZE] = {0};
  uint8_t decodedParserLen = 0;
  uint32_t frameStartTimeMs = 0;

  // Decoded Telemetry Working State (owned by opMutex)
  uint8_t rawStateByte = 0x00;
  uint8_t stateNibble = 0x00;
  uint8_t upperNibble = 0x00;
  CsafeMachineState reportedState = CsafeMachineState::Unknown;
  CsafeMachineState qualifiedState = CsafeMachineState::Unknown;

  // State 15 (Error) Qualification State (owned by opMutex)
  bool errorCandidateActive = false;
  bool errorConfirmed = false;
  uint32_t errorCandidateStartMs = 0;
  uint32_t errorCandidateDurationMs = 0;
  uint32_t consecutiveErrorStateCount = 0;

  // Frame Snapshots (owned by opMutex)
  CsafeFrame lastCompleteFrame{};
  CsafeFrame lastValidStateFrame{};
  uint32_t frameSequence = 0;

  // Timestamps & Watchdogs (owned by opMutex)
  uint32_t lastReceivedByteMs = 0;
  uint32_t lastCompleteFrameMs = 0;
  uint32_t lastChecksumValidFrameMs = 0;
  uint32_t lastValidStateMs = 0;
  uint32_t lastRequestMs = 0;

  // Diagnostic Counters (owned by opMutex)
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

  // Public State Snapshot (protected by stateMux)
  CsafeState state{};

  Impl() {
    opMutex = xSemaphoreCreateMutex();
  }

  ~Impl() {
    if (opMutex != nullptr) {
      vSemaphoreDelete(opMutex);
      opMutex = nullptr;
    }
  }

  void resetRuntimeStateLocked() {
    linkOnline = false;
    awaitingResponse = false;
    uartError = false;
    initialTimeoutLatched = false;
    beginTimeMs = 0;

    parserState = CsafeParserState::Idle;
    rawParserLen = 0;
    decodedParserLen = 0;
    frameStartTimeMs = 0;
    memset(rawParserBuf, 0, sizeof(rawParserBuf));
    memset(decodedParserBuf, 0, sizeof(decodedParserBuf));

    rawStateByte = 0x00;
    stateNibble = 0x00;
    upperNibble = 0x00;
    reportedState = CsafeMachineState::Unknown;
    qualifiedState = CsafeMachineState::Unknown;

    errorCandidateActive = false;
    errorConfirmed = false;
    errorCandidateStartMs = 0;
    errorCandidateDurationMs = 0;
    consecutiveErrorStateCount = 0;

    lastCompleteFrame = CsafeFrame{};
    lastValidStateFrame = CsafeFrame{};
    frameSequence = 0;

    lastReceivedByteMs = 0;
    lastCompleteFrameMs = 0;
    lastChecksumValidFrameMs = 0;
    lastValidStateMs = 0;
    lastRequestMs = 0;

    transmittedStatusRequestCount = 0;
    transmittedByteCount = 0;
    receivedByteCount = 0;
    completeFrameCount = 0;
    checksumValidFrameCount = 0;
    validStateFrameCount = 0;
    checksumErrorCount = 0;
    tooShortFrameCount = 0;
    invalidEscapeCount = 0;
    frameOverflowCount = 0;
    parserResyncCount = 0;
    incompleteFrameTimeoutCount = 0;
    responseTimeoutCount = 0;
    linkTimeoutCount = 0;
    unknownPayloadCount = 0;
    unknownStateCount = 0;
    rejectedTransmitCount = 0;
    uartWriteErrorCount = 0;
  }

  void endLocked() {
    if (initialized) {
      if (config.serial != nullptr) {
        config.serial->end();
      }
    }

    initialized = false;
    resetRuntimeStateLocked();

    portENTER_CRITICAL(&stateMux);
    state = CsafeState{};
    state.linkStatus = CsafeLinkStatus::Uninitialized;
    publicConfigSnapshot = CsafeConfig{};
    portEXIT_CRITICAL(&stateMux);
  }

  void processByte(uint8_t byte, uint32_t nowMs) {
    receivedByteCount++;
    lastReceivedByteMs = nowMs;

    // Check incomplete frame timeout
    if ((parserState == CsafeParserState::InFrame || parserState == CsafeParserState::Escaped) &&
        (nowMs - frameStartTimeMs) > config.incompleteFrameTimeoutMs) {
      incompleteFrameTimeoutCount++;
      parserState = CsafeParserState::Idle;
      rawParserLen = 0;
      decodedParserLen = 0;
    }

    switch (parserState) {
      case CsafeParserState::Idle: {
        if (byte == kFrameStart) {
          rawParserBuf[0] = kFrameStart;
          rawParserLen = 1;
          decodedParserLen = 0;
          frameStartTimeMs = nowMs;
          parserState = CsafeParserState::InFrame;
        }
        break;
      }

      case CsafeParserState::InFrame: {
        if (byte == kFrameStart) {
          // Unexpected Start Delimiter -> Resynchronize
          parserResyncCount++;
          rawParserBuf[0] = kFrameStart;
          rawParserLen = 1;
          decodedParserLen = 0;
          frameStartTimeMs = nowMs;
          parserState = CsafeParserState::InFrame;
        } else if (byte == kFrameEnd) {
          // End of Frame: verify space for final F2 in raw buffer
          if (rawParserLen < CSAFE_MAX_RAW_FRAME_SIZE) {
            rawParserBuf[rawParserLen++] = kFrameEnd;
            dispatchCompleteFrame(nowMs);
          } else {
            frameOverflowCount++;
          }
          parserState = CsafeParserState::Idle;
          rawParserLen = 0;
          decodedParserLen = 0;
        } else if (byte == kByteEscape) {
          if (rawParserLen < CSAFE_MAX_RAW_FRAME_SIZE) {
            rawParserBuf[rawParserLen++] = kByteEscape;
            parserState = CsafeParserState::Escaped;
          } else {
            frameOverflowCount++;
            parserState = CsafeParserState::Discarding;
          }
        } else {
          // Normal payload/checksum byte
          if (rawParserLen < CSAFE_MAX_RAW_FRAME_SIZE && decodedParserLen < CSAFE_MAX_DECODED_FRAME_SIZE) {
            rawParserBuf[rawParserLen++] = byte;
            decodedParserBuf[decodedParserLen++] = byte;
          } else {
            frameOverflowCount++;
            parserState = CsafeParserState::Discarding;
          }
        }
        break;
      }

      case CsafeParserState::Escaped: {
        if (rawParserLen < CSAFE_MAX_RAW_FRAME_SIZE) {
          rawParserBuf[rawParserLen++] = byte;
        } else {
          frameOverflowCount++;
          parserState = CsafeParserState::Discarding;
          break;
        }

        uint8_t unescapedByte = 0;
        bool validEscape = true;
        if (byte == kEscapedF1) {
          unescapedByte = kFrameStart;
        } else if (byte == kEscapedF2) {
          unescapedByte = kFrameEnd;
        } else if (byte == kEscapedF3) {
          unescapedByte = kByteEscape;
        } else {
          validEscape = false;
        }

        if (validEscape) {
          if (decodedParserLen < CSAFE_MAX_DECODED_FRAME_SIZE) {
            decodedParserBuf[decodedParserLen++] = unescapedByte;
            parserState = CsafeParserState::InFrame;
          } else {
            frameOverflowCount++;
            parserState = CsafeParserState::Discarding;
          }
        } else {
          invalidEscapeCount++;
          parserState = CsafeParserState::Idle;
          rawParserLen = 0;
          decodedParserLen = 0;
        }
        break;
      }

      case CsafeParserState::Discarding: {
        if (byte == kFrameEnd) {
          parserState = CsafeParserState::Idle;
          rawParserLen = 0;
          decodedParserLen = 0;
        } else if (byte == kFrameStart) {
          rawParserBuf[0] = kFrameStart;
          rawParserLen = 1;
          decodedParserLen = 0;
          frameStartTimeMs = nowMs;
          parserState = CsafeParserState::InFrame;
        }
        break;
      }
    }
  }

  void dispatchCompleteFrame(uint32_t nowMs) {
    completeFrameCount++;
    lastCompleteFrameMs = nowMs;

    // Record lastCompleteFrame snapshot
    lastCompleteFrame = CsafeFrame{};
    memcpy(lastCompleteFrame.rawData, rawParserBuf, rawParserLen);
    lastCompleteFrame.rawLength = rawParserLen;
    memcpy(lastCompleteFrame.decodedData, decodedParserBuf, decodedParserLen);
    lastCompleteFrame.decodedLength = decodedParserLen;
    lastCompleteFrame.timestampMs = nowMs;
    lastCompleteFrame.sequence = ++frameSequence;

    // Check minimum unstuffed length: at least 1 payload byte + 1 checksum byte
    if (decodedParserLen < 2) {
      tooShortFrameCount++;
      lastCompleteFrame.checksumValid = false;
      return;
    }

    // Validate XOR checksum
    uint8_t calcChecksum = 0x00;
    for (size_t i = 0; i < static_cast<size_t>(decodedParserLen - 1); ++i) {
      calcChecksum ^= decodedParserBuf[i];
    }
    const uint8_t rxChecksum = decodedParserBuf[decodedParserLen - 1];

    if (calcChecksum != rxChecksum) {
      checksumErrorCount++;
      lastCompleteFrame.checksumValid = false;
      return;
    }

    // Checksum valid
    checksumValidFrameCount++;
    lastChecksumValidFrameMs = nowMs;
    lastCompleteFrame.checksumValid = true;

    // Check if frame is a State-Only response (payload length exactly 1 byte)
    const uint8_t payloadLen = decodedParserLen - 1;
    if (payloadLen == 1) {
      validStateFrameCount++;
      lastValidStateMs = nowMs;
      awaitingResponse = false;
      linkOnline = true;
      initialTimeoutLatched = false;

      lastValidStateFrame = lastCompleteFrame;

      rawStateByte = decodedParserBuf[0];
      stateNibble = rawStateByte & 0x0F;
      upperNibble = rawStateByte & 0xF0;

      const CsafeMachineState decodedState = decodeMachineState(stateNibble);
      reportedState = decodedState;

      if (decodedState == CsafeMachineState::Error) {
        // State 15 (Error candidate)
        if (!errorCandidateActive) {
          errorCandidateActive = true;
          errorConfirmed = false;
          errorCandidateStartMs = nowMs;
          errorCandidateDurationMs = 0;
          consecutiveErrorStateCount = 1;
        } else {
          consecutiveErrorStateCount++;
          errorCandidateDurationMs = nowMs - errorCandidateStartMs;
          if (errorCandidateDurationMs >= config.errorConfirmationMs) {
            errorConfirmed = true;
            qualifiedState = CsafeMachineState::Error;
          }
        }
      } else if (decodedState != CsafeMachineState::Unknown) {
        // Checksum-valid known non-error state (0x00 - 0x08)
        errorCandidateActive = false;
        errorConfirmed = false;
        errorCandidateStartMs = 0;
        errorCandidateDurationMs = 0;
        consecutiveErrorStateCount = 0;
        qualifiedState = decodedState;
      } else {
        // Unknown lower nibble (0x09 - 0x0E)
        unknownStateCount++;
        errorCandidateActive = false;
        errorConfirmed = false;
        errorCandidateStartMs = 0;
        errorCandidateDurationMs = 0;
        consecutiveErrorStateCount = 0;
        // Preserves previous qualifiedState as last known state
      }
    } else {
      // Checksum-valid but multi-byte / unsupported payload
      unknownPayloadCount++;
    }
  }
};

CsafeInterface::CsafeInterface()
    : impl_(new Impl()) {}

CsafeInterface::~CsafeInterface() {
  if (impl_ != nullptr) {
    if (impl_->opMutex != nullptr) {
      if (xSemaphoreTake(impl_->opMutex, portMAX_DELAY) == pdTRUE) {
        impl_->endLocked();
        xSemaphoreGive(impl_->opMutex);
      }
    }
    delete impl_;
    impl_ = nullptr;
  }
}

bool CsafeInterface::begin(const CsafeConfig& config) {
  if (impl_ == nullptr || impl_->opMutex == nullptr) {
    return false;
  }

  MutexLock lock(impl_->opMutex, kLifecycleMutexTimeoutTicks);
  if (!lock.isLocked()) {
    return false;
  }

  // Idempotent guard
  if (impl_->initialized) {
    return true;
  }

  // Validate configuration before touching UART
  if (!validateConfiguration(config)) {
    portENTER_CRITICAL(&impl_->stateMux);
    impl_->state = CsafeState{};
    impl_->state.linkStatus = CsafeLinkStatus::HardwareError;
    impl_->publicConfigSnapshot = CsafeConfig{};
    portEXIT_CRITICAL(&impl_->stateMux);
    return false;
  }

  impl_->resetRuntimeStateLocked();
  impl_->config = config;
  impl_->beginTimeMs = millis();

  // Initialize UART
  config.serial->begin(config.baudRate, config.serialConfig, config.rxPin, config.txPin);

  // Drain stale RX bytes within a bounded limit
  for (int i = 0; i < 64 && config.serial->available() > 0; ++i) {
    config.serial->read();
  }

  impl_->initialized = true;
  // Initialize lastRequestMs so the first GetStatus poll can transmit immediately on the first update()
  impl_->lastRequestMs = millis() - config.pollIntervalMs;

  portENTER_CRITICAL(&impl_->stateMux);
  impl_->state = CsafeState{};
  impl_->state.initialized = true;
  impl_->state.linkStatus = CsafeLinkStatus::WaitingForFirstResponse;
  impl_->publicConfigSnapshot = config;
  portEXIT_CRITICAL(&impl_->stateMux);

  return true;
}

void CsafeInterface::end() {
  if (impl_ == nullptr || impl_->opMutex == nullptr) {
    return;
  }

  MutexLock lock(impl_->opMutex, kLifecycleMutexTimeoutTicks);
  if (!lock.isLocked()) {
    return;
  }

  impl_->endLocked();
}

void CsafeInterface::update() {
  if (impl_ == nullptr || impl_->opMutex == nullptr) {
    return;
  }

  // Non-blocking update: if operation mutex is busy, skip this cycle promptly
  MutexLock lock(impl_->opMutex, kUpdateMutexTimeoutTicks);
  if (!lock.isLocked()) {
    return;
  }

  if (!impl_->initialized) {
    return;
  }

  const uint32_t nowMs = millis();

  // 1. Drain incoming UART bytes up to maximumRxBytesPerUpdate
  uint16_t bytesRead = 0;
  while (impl_->config.serial->available() > 0 && bytesRead < impl_->config.maximumRxBytesPerUpdate) {
    const int byteVal = impl_->config.serial->read();
    if (byteVal < 0) {
      break;
    }
    impl_->processByte(static_cast<uint8_t>(byteVal), nowMs);
    bytesRead++;
  }

  // 2. Manage Response Timeout
  if (impl_->awaitingResponse && (nowMs - impl_->lastRequestMs) > impl_->config.responseTimeoutMs) {
    impl_->awaitingResponse = false;
    impl_->responseTimeoutCount++;
  }

  // 3. Transmit Periodic GetStatus (F1 80 80 F2)
  if (!impl_->awaitingResponse && (nowMs - impl_->lastRequestMs) >= impl_->config.pollIntervalMs) {
    const uint8_t frame[4] = {kFrameStart, kGetStatusCmd, kGetStatusCmd, kFrameEnd};

    // Outgoing safety check
    bool forbiddenFound = false;
    for (size_t i = 0; i < 4; ++i) {
      if (frame[i] == kForbiddenAA || frame[i] == kForbiddenA5 || frame[i] == kForbidden9C) {
        forbiddenFound = true;
        break;
      }
    }

    if (forbiddenFound) {
      impl_->rejectedTransmitCount++;
    } else {
      const size_t written = impl_->config.serial->write(frame, 4);
      if (written != 4) {
        impl_->uartWriteErrorCount++;
        impl_->uartError = true;
      } else {
        impl_->uartError = false;
        impl_->transmittedStatusRequestCount++;
        impl_->transmittedByteCount += 4;
        impl_->lastRequestMs = nowMs;
        impl_->awaitingResponse = true;
      }
    }
  }

  // 4. Manage Link Watchdog & Timeout (with rollover-safe unsigned comparison)
  const uint32_t watchdogRefMs = (impl_->lastValidStateMs != 0) ? impl_->lastValidStateMs : impl_->beginTimeMs;
  if ((nowMs - watchdogRefMs) > impl_->config.linkTimeoutMs) {
    if (impl_->linkOnline || !impl_->initialTimeoutLatched) {
      impl_->linkOnline = false;
      impl_->linkTimeoutCount++;
      impl_->initialTimeoutLatched = true;
    }
  }

  // 5. Determine Central Link Status
  CsafeLinkStatus currentLinkStatus = CsafeLinkStatus::Uninitialized;
  if (!impl_->initialized) {
    currentLinkStatus = CsafeLinkStatus::Uninitialized;
  } else if (impl_->uartError) {
    currentLinkStatus = CsafeLinkStatus::UartError;
  } else if (impl_->linkOnline) {
    currentLinkStatus = CsafeLinkStatus::Online;
  } else {
    if (impl_->lastValidStateMs == 0 && !impl_->initialTimeoutLatched) {
      currentLinkStatus = CsafeLinkStatus::WaitingForFirstResponse;
    } else {
      currentLinkStatus = CsafeLinkStatus::TimedOut;
    }
  }

  const bool machineStateFresh = impl_->linkOnline && (impl_->lastValidStateMs != 0) &&
                                 ((nowMs - impl_->lastValidStateMs) <= impl_->config.linkTimeoutMs);
  const uint32_t validStateAgeMs = (impl_->lastValidStateMs != 0) ? (nowMs - impl_->lastValidStateMs) : 0;

  // 6. Commit Public State Snapshot under stateMux
  portENTER_CRITICAL(&impl_->stateMux);

  impl_->state.initialized = impl_->initialized;
  impl_->state.online = impl_->linkOnline;
  impl_->state.awaitingResponse = impl_->awaitingResponse;
  impl_->state.machineStateFresh = machineStateFresh;
  impl_->state.linkStatus = currentLinkStatus;
  impl_->state.parserState = impl_->parserState;

  impl_->state.rawStateByte = impl_->rawStateByte;
  impl_->state.stateNibble = impl_->stateNibble;
  impl_->state.upperNibble = impl_->upperNibble;
  impl_->state.reportedState = impl_->reportedState;
  impl_->state.qualifiedState = impl_->qualifiedState;

  impl_->state.errorCandidateActive = impl_->errorCandidateActive;
  impl_->state.errorConfirmed = impl_->errorConfirmed;
  impl_->state.errorCandidateDurationMs = impl_->errorCandidateDurationMs;
  impl_->state.consecutiveErrorStateCount = impl_->consecutiveErrorStateCount;

  impl_->state.lastCompleteFrame = impl_->lastCompleteFrame;
  impl_->state.lastValidStateFrame = impl_->lastValidStateFrame;

  impl_->state.lastReceivedByteMs = impl_->lastReceivedByteMs;
  impl_->state.lastCompleteFrameMs = impl_->lastCompleteFrameMs;
  impl_->state.lastChecksumValidFrameMs = impl_->lastChecksumValidFrameMs;
  impl_->state.lastValidStateMs = impl_->lastValidStateMs;
  impl_->state.lastRequestMs = impl_->lastRequestMs;
  impl_->state.validStateAgeMs = validStateAgeMs;
  impl_->state.snapshotTimestampMs = nowMs;

  impl_->state.transmittedStatusRequestCount = impl_->transmittedStatusRequestCount;
  impl_->state.transmittedByteCount = impl_->transmittedByteCount;
  impl_->state.receivedByteCount = impl_->receivedByteCount;
  impl_->state.completeFrameCount = impl_->completeFrameCount;
  impl_->state.checksumValidFrameCount = impl_->checksumValidFrameCount;
  impl_->state.validStateFrameCount = impl_->validStateFrameCount;
  impl_->state.checksumErrorCount = impl_->checksumErrorCount;
  impl_->state.tooShortFrameCount = impl_->tooShortFrameCount;
  impl_->state.invalidEscapeCount = impl_->invalidEscapeCount;
  impl_->state.frameOverflowCount = impl_->frameOverflowCount;
  impl_->state.parserResyncCount = impl_->parserResyncCount;
  impl_->state.incompleteFrameTimeoutCount = impl_->incompleteFrameTimeoutCount;
  impl_->state.responseTimeoutCount = impl_->responseTimeoutCount;
  impl_->state.linkTimeoutCount = impl_->linkTimeoutCount;
  impl_->state.unknownPayloadCount = impl_->unknownPayloadCount;
  impl_->state.unknownStateCount = impl_->unknownStateCount;
  impl_->state.rejectedTransmitCount = impl_->rejectedTransmitCount;
  impl_->state.uartWriteErrorCount = impl_->uartWriteErrorCount;

  portEXIT_CRITICAL(&impl_->stateMux);
}

CsafeState CsafeInterface::getState() const {
  CsafeState snapshot{};
  if (impl_ == nullptr) {
    return snapshot;
  }
  portENTER_CRITICAL(&impl_->stateMux);
  snapshot = impl_->state;
  portEXIT_CRITICAL(&impl_->stateMux);
  return snapshot;
}

CsafeConfig CsafeInterface::configSnapshot() const {
  if (impl_ == nullptr) {
    return CsafeConfig{};
  }
  portENTER_CRITICAL(&impl_->stateMux);
  const CsafeConfig cfg = impl_->publicConfigSnapshot;
  portEXIT_CRITICAL(&impl_->stateMux);
  return cfg;
}

bool CsafeInterface::isReady() const {
  if (impl_ == nullptr) {
    return false;
  }
  portENTER_CRITICAL(&impl_->stateMux);
  const bool ready = impl_->state.initialized &&
                     (impl_->state.linkStatus != CsafeLinkStatus::Uninitialized) &&
                     (impl_->state.linkStatus != CsafeLinkStatus::HardwareError);
  portEXIT_CRITICAL(&impl_->stateMux);
  return ready;
}

const char* CsafeInterface::version() {
  return kCsafeInterfaceVersion;
}

}  // namespace stridecontrol
