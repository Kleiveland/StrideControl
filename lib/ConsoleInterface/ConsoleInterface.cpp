#include "ConsoleInterface.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include "soc/gpio_struct.h"

namespace stridecontrol {
namespace {

constexpr uint8_t PHASE_A = 0x0F;
constexpr uint8_t PHASE_B = 0x17;
constexpr uint8_t PHASE_C = 0x1B;
constexpr uint8_t PHASE_D = 0x1D;
constexpr uint8_t IDLE = 0x1F;
constexpr uint8_t BASE_A = 0x80;
constexpr uint8_t BASE_B = 0x80;
constexpr uint8_t BASE_C = 0x80;
constexpr uint8_t BASE_D = 0xC0;
constexpr uint8_t IDLE_FIRST = 0x00;
constexpr uint8_t IDLE_SECOND = 0xFF;

constexpr uint16_t BUZZER_EDGE_CAPACITY = 192;
constexpr uint16_t BUZZER_EVENT_CAPACITY = 64;
constexpr uint8_t PLAN_CAPACITY = 40;

struct BuzzerEdge { uint32_t us; uint8_t active; };
struct BuzzerEvent {
  uint32_t sequence = 0; uint32_t startUs = 0; uint32_t endUs = 0;
  uint32_t envelopeUs = 0; uint32_t activeUs = 0; uint32_t longestInternalGapUs = 0;
  uint16_t rawEdgeCount = 0; uint8_t segmentCount = 0;
};
struct PhaseTracker {
  uint8_t lastRaw = 0xFF; uint8_t valid = 0xFF;
  uint32_t validStartUs = 0; uint32_t lastValidChangeMs = 0;
};
struct PlanStep {
  ButtonId button = ButtonId::Unknown; uint16_t holdMs = 82; uint16_t pauseMs = 150;
  PlanStep() = default;
  PlanStep(ButtonId b, uint16_t h, uint16_t p) : button(b), holdMs(h), pauseMs(p) {}
};

bool isValidPhase(uint8_t value) {
  return value == PHASE_A || value == PHASE_B || value == PHASE_C || value == PHASE_D || value == IDLE;
}

bool isDigit(ButtonId button) {
  return button >= ButtonId::Num0 && button <= ButtonId::Num9;
}

ButtonId digitButton(char digit) {
  if (digit < '0' || digit > '9') return ButtonId::Unknown;
  return static_cast<ButtonId>(static_cast<uint8_t>(ButtonId::Num0) + (digit - '0'));
}

bool requiresNumericRecovery(ConsoleOutcome outcome) {
  switch (outcome) {
    case ConsoleOutcome::NoResponse:
    case ConsoleOutcome::NormalLong:
    case ConsoleOutcome::MergedMulti:
    case ConsoleOutcome::DistinctMulti:
    case ConsoleOutcome::EdgeLoss:
    case ConsoleOutcome::Invalid:
      return true;
    default:
      return false;
  }
}

} // namespace

struct ConsoleInterface::Impl {
  ConsoleConfig config{};
  ConsoleExecutionMode executionMode{ConsoleExecutionMode::HardwareMatrix};
  ICommandIntentSink* commandSink{nullptr};
  portMUX_TYPE sinkMux = portMUX_INITIALIZER_UNLOCKED;
  QueueHandle_t requestQueue = nullptr;
  QueueHandle_t commandEventQueue = nullptr;
  QueueHandle_t physicalEventQueue = nullptr;
  TaskHandle_t commandTaskHandle = nullptr;
  TaskHandle_t buzzerTaskHandle = nullptr;
  TaskHandle_t panelTaskHandle = nullptr;
  std::atomic<bool> ready{false};
  std::atomic<bool> active{false};
  std::atomic<bool> abortRequested{false};

  volatile BuzzerEdge edgeRing[BUZZER_EDGE_CAPACITY]{};
  volatile uint16_t edgeHead = 0, edgeTail = 0;
  volatile uint32_t droppedEdgesIsr = 0;
  portMUX_TYPE edgeMux = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE eventMux = portMUX_INITIALIZER_UNLOCKED;
  
  BuzzerEvent eventRing[BUZZER_EVENT_CAPACITY]{};
  uint16_t eventHead = 0;
  std::atomic<uint32_t> eventSequence{0};
  std::atomic<uint32_t> droppedEdges{0};
  static Impl* isrOwner;

  static void IRAM_ATTR buzzerIsrRouter() { if (isrOwner) isrOwner->buzzerIsr(); }
  void IRAM_ATTR buzzerIsr() {
    const uint32_t now = micros();
    const uint32_t pin = static_cast<uint32_t>(config.pins.buzzerIn);
    const bool activeLow = (pin < 32 ? ((GPIO.in >> pin) & 1U) : ((GPIO.in1.val >> (pin - 32)) & 1U)) == 0;
    portENTER_CRITICAL_ISR(&edgeMux);
    const uint16_t next = (edgeHead + 1) % BUZZER_EDGE_CAPACITY;
    if (next == edgeTail) {
      droppedEdgesIsr++;
    } else {
      edgeRing[edgeHead].us = now;
      edgeRing[edgeHead].active = activeLow ? 1 : 0;
      edgeHead = next;
    }
    portEXIT_CRITICAL_ISR(&edgeMux);
  }

  bool popEdge(BuzzerEdge& edge) {
    bool available = false; uint32_t lost = 0;
    portENTER_CRITICAL(&edgeMux);
    if (edgeTail != edgeHead) {
      edge.us = edgeRing[edgeTail].us; edge.active = edgeRing[edgeTail].active;
      edgeTail = (edgeTail + 1) % BUZZER_EDGE_CAPACITY; available = true;
    }
    lost = droppedEdgesIsr; droppedEdgesIsr = 0;
    portEXIT_CRITICAL(&edgeMux);
    if (lost) droppedEdges.fetch_add(lost);
    return available;
  }

  void publishEvent(const BuzzerEvent& event) {
    portENTER_CRITICAL(&eventMux);
    eventRing[eventHead] = event;
    eventHead = (eventHead + 1) % BUZZER_EVENT_CAPACITY;
    portEXIT_CRITICAL(&eventMux);
    eventSequence.store(event.sequence);
  }

  bool getEventAfter(uint32_t sequence, BuzzerEvent& out) {
    if (eventSequence.load() <= sequence) return false;
    bool found = false; uint32_t bestSequence = UINT32_MAX;
    portENTER_CRITICAL(&eventMux);
    for (const auto& ev : eventRing) {
      if (ev.sequence > sequence && ev.sequence < bestSequence) {
        bestSequence = ev.sequence; out = ev; found = true;
      }
    }
    portEXIT_CRITICAL(&eventMux);
    return found;
  }

  static void buzzerTaskRouter(void* arg) { static_cast<Impl*>(arg)->buzzerTask(); }
  void buzzerTask() {
    bool envelopeOpen = false, segmentOpen = false;
    uint32_t envelopeStart = 0, segmentStart = 0, lastQualifiedEnd = 0;
    uint32_t activeUs = 0, longestGap = 0, sequence = 0;
    uint16_t rawEdges = 0; uint8_t segments = 0;
    
    for (;;) {
      BuzzerEdge edge{};
      while (popEdge(edge)) {
        rawEdges++;
        if (edge.active) {
          if (!envelopeOpen) {
            envelopeOpen = true; envelopeStart = edge.us; lastQualifiedEnd = 0;
            activeUs = 0; longestGap = 0; rawEdges = 1; segments = 0;
          }
          if (!segmentOpen) { segmentOpen = true; segmentStart = edge.us; }
        } else if (envelopeOpen && segmentOpen) {
          const uint32_t duration = edge.us - segmentStart;
          if (duration >= config.timing.minBuzzerSegmentUs) {
            if (segments > 0) longestGap = std::max(longestGap, segmentStart - lastQualifiedEnd);
            activeUs += duration; lastQualifiedEnd = edge.us; segments++;
          }
          segmentOpen = false;
        }
      }
      
      const uint32_t now = micros();
      if (envelopeOpen && !segmentOpen && lastQualifiedEnd && now - lastQualifiedEnd >= config.timing.eventEndGapUs) {
        BuzzerEvent ev{};
        ev.sequence = ++sequence; ev.startUs = envelopeStart; ev.endUs = lastQualifiedEnd;
        ev.envelopeUs = lastQualifiedEnd - envelopeStart; ev.activeUs = activeUs;
        ev.longestInternalGapUs = longestGap; ev.rawEdgeCount = rawEdges; ev.segmentCount = segments;
        publishEvent(ev);
        envelopeOpen = false; lastQualifiedEnd = 0; rawEdges = 0; segments = 0;
      } else if (envelopeOpen && !segmentOpen && !lastQualifiedEnd && now - envelopeStart >= config.timing.eventEndGapUs) {
        envelopeOpen = false; rawEdges = 0; segments = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  void readAtomic(uint8_t& o1, uint8_t& o2) const {
    const uint32_t in0 = GPIO.in; const uint32_t in1 = GPIO.in1.val;
    o1 = 0; o2 = 0;
    for (uint8_t i = 0; i < 5; ++i) {
      const int pin = config.pins.o1[i];
      if (pin < 32 ? ((in0 >> pin) & 1U) : ((in1 >> (pin - 32)) & 1U)) o1 |= 1U << i;
    }
    for (uint8_t i = 0; i < 8; ++i) {
      const int pin = config.pins.o2[i];
      if (pin < 32 ? ((in0 >> pin) & 1U) : ((in1 >> (pin - 32)) & 1U)) o2 |= 1U << i;
    }
  }

  void setO2Direction(bool out) const {
    uint32_t mask0 = 0, mask1 = 0;
    for (int pin : config.pins.o2) {
      if (pin < 32) mask0 |= 1UL << pin; else mask1 |= 1UL << (pin - 32);
    }
    if (out) {
      if (mask1) GPIO.enable1_w1ts.val = mask1;
      if (mask0) GPIO.enable_w1ts = mask0;
    } else {
      if (mask1) GPIO.enable1_w1tc.val = mask1;
      if (mask0) GPIO.enable_w1tc = mask0;
    }
  }

  void driveO2(uint8_t value, uint8_t& last) const {
    if (value == last) return;
    uint32_t set0 = 0, clear0 = 0, set1 = 0, clear1 = 0;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const int pin = config.pins.o2[bit];
      const bool high = (value >> bit) & 1U;
      if (pin < 32) (high ? set0 : clear0) |= 1UL << pin;
      else (high ? set1 : clear1) |= 1UL << (pin - 32);
    }
    if (clear1) GPIO.out1_w1tc.val = clear1;
    if (clear0) GPIO.out_w1tc = clear0;
    if (set1) GPIO.out1_w1ts.val = set1;
    if (set0) GPIO.out_w1ts = set0;
    last = value;
  }

  bool updateTracker(PhaseTracker& tracker) const {
    uint8_t raw = 0, ignored = 0; readAtomic(raw, ignored);
    if (raw == tracker.lastRaw) return false;
    tracker.lastRaw = raw;
    if (!isValidPhase(raw)) return false;
    tracker.valid = raw; tracker.validStartUs = micros(); tracker.lastValidChangeMs = millis();
    return true;
  }

  uint8_t targetPhase(ButtonId button) const {
    switch (button) {
      case ButtonId::InstantIncline: case ButtonId::Num1: case ButtonId::Num4:
      case ButtonId::Num5: case ButtonId::Num7: case ButtonId::Num8: return PHASE_A;
      case ButtonId::InstantSpeed: case ButtonId::Enter: case ButtonId::Num0:
      case ButtonId::Num2: case ButtonId::Num3: case ButtonId::Num6: return PHASE_B;
      case ButtonId::Num9: return PHASE_C;
      case ButtonId::Clear: return config.clearMapping.verified ? config.clearMapping.targetPhase : 0xFF;
      default: return 0xFF;
    }
  }

  uint8_t preloadPhase(ButtonId button) const {
    if (button == ButtonId::Num9) return PHASE_B;
    if (button == ButtonId::Clear && config.clearMapping.verified && config.clearMapping.preloadPhase != 0xFF) {
      return config.clearMapping.preloadPhase;
    }
    return 0xFF;
  }

  uint8_t syncPhase(ButtonId button) const {
    const uint8_t preload = preloadPhase(button);
    return (preload != 0xFF) ? preload : targetPhase(button);
  }

  uint8_t o2Response(uint8_t phase, uint32_t ageUs, ButtonId activeButton) const {
    if (phase == IDLE) return ageUs < config.timing.idleSwitchUs ? IDLE_FIRST : IDLE_SECOND;
    uint8_t response = (phase == PHASE_D) ? BASE_D : (phase == PHASE_A || phase == PHASE_B || phase == PHASE_C ? BASE_A : 0xFF);
    
    if (activeButton == ButtonId::InstantIncline && phase == PHASE_A) response = 0xC0;
    if (activeButton == ButtonId::Num1 && phase == PHASE_A) response = 0x82;
    if (activeButton == ButtonId::Num4 && phase == PHASE_A) response = 0x84;
    if (activeButton == ButtonId::Num5 && phase == PHASE_A) response = 0x90;
    if (activeButton == ButtonId::Num7 && phase == PHASE_A) response = 0x88;
    if (activeButton == ButtonId::Num8 && phase == PHASE_A) response = 0xA0;
    if (activeButton == ButtonId::InstantSpeed && phase == PHASE_B) response = 0xC0;
    if (activeButton == ButtonId::Enter && phase == PHASE_B) response = 0x88;
    if (activeButton == ButtonId::Num0 && phase == PHASE_B) response = 0xA0;
    if (activeButton == ButtonId::Num2 && phase == PHASE_B) response = 0x90;
    if (activeButton == ButtonId::Num3 && phase == PHASE_B) response = 0x84;
    if (activeButton == ButtonId::Num6 && phase == PHASE_B) response = 0x82;
    if (activeButton == ButtonId::Num9 && (phase == PHASE_B || phase == PHASE_C)) response = 0x81;
    if (activeButton == ButtonId::Clear && config.clearMapping.verified) {
      if (phase == config.clearMapping.targetPhase) response = config.clearMapping.activeO2;
      if (phase == config.clearMapping.preloadPhase) response = config.clearMapping.preloadO2;
    }
    return response;
  }

  bool waitFreshPhase(PhaseTracker& tracker, uint8_t target, uint32_t timeoutMs, uint8_t& last) {
    const uint32_t startMs = millis(); bool targetWasLeft = (tracker.valid != target);
    while (millis() - startMs < timeoutMs && !abortRequested.load()) {
      const bool changed = updateTracker(tracker);
      if (isValidPhase(tracker.lastRaw) && tracker.lastRaw != target) targetWasLeft = true;
      if (changed && targetWasLeft && tracker.valid == target) return true;
      if (isValidPhase(tracker.valid)) driveO2(o2Response(tracker.valid, micros() - tracker.validStartUs, ButtonId::Unknown), last);
    }
    return false;
  }

  ConsoleOutcome waitAck(PhaseTracker& tracker, uint8_t& last, uint32_t sequenceBefore, uint32_t pressStartUs, uint32_t droppedBefore, AckMetrics& metrics) {
    uint32_t seenSequence = sequenceBefore;
    const uint32_t normalDeadline = millis() + config.timing.ackTimeoutMs;
    const uint32_t finalDeadline = normalDeadline + config.timing.retryCooldownMs;
    uint32_t firstPublishMs = 0;
    ConsoleOutcome result = ConsoleOutcome::NoResponse;

    while (static_cast<int32_t>(finalDeadline - millis()) > 0 && !abortRequested.load()) {
      updateTracker(tracker);
      if (isValidPhase(tracker.valid)) driveO2(o2Response(tracker.valid, micros() - tracker.validStartUs, ButtonId::Unknown), last);
      if (droppedEdges.load() != droppedBefore) return ConsoleOutcome::EdgeLoss;

      BuzzerEvent ev{};
      while (getEventAfter(seenSequence, ev)) {
        seenSequence = ev.sequence;
        if (static_cast<int32_t>(ev.startUs - pressStartUs) < 0) continue;

        if (metrics.eventCount == 0) {
          metrics.startLatencyUs = ev.startUs - pressStartUs;
          metrics.completionLatencyUs = ev.endUs - pressStartUs;
          metrics.envelopeUs = ev.envelopeUs;
          metrics.activeUs = ev.activeUs;
          metrics.longestInternalGapUs = ev.longestInternalGapUs;
          metrics.rawEdgeCount = ev.rawEdgeCount;
          metrics.segmentCount = ev.segmentCount;
          metrics.noisy = ev.rawEdgeCount != 2;
          metrics.late = static_cast<int32_t>(millis() - normalDeadline) > 0;
          firstPublishMs = millis();
        }
        metrics.eventCount++;

        if (metrics.eventCount > 1) return ConsoleOutcome::DistinctMulti;
        if (ev.segmentCount >= 2) return ConsoleOutcome::MergedMulti;
        if (ev.segmentCount != 1) return ConsoleOutcome::Invalid;
        
        result = ev.envelopeUs > config.timing.normalEnvelopeMaxUs ? ConsoleOutcome::NormalLong : ConsoleOutcome::NormalSingle;
      }
      if ((result == ConsoleOutcome::NormalSingle || result == ConsoleOutcome::NormalLong) && firstPublishMs && millis() - firstPublishMs >= config.timing.ackGuardMs) {
        return result;
      }
    }
    return abortRequested.load() ? ConsoleOutcome::Aborted : result;
  }

  ConsoleOutcome pressOwned(PhaseTracker& tracker, uint8_t& last, ButtonId button, uint16_t holdMs, AckMetrics& metrics) {
    if (button == ButtonId::Clear && !config.clearMapping.verified) return ConsoleOutcome::ClearMappingUnverified;
    const uint8_t target = targetPhase(button);
    if (target == 0xFF) return ConsoleOutcome::Invalid;
    const uint8_t start = syncPhase(button);
    if (!waitFreshPhase(tracker, start, 40, last)) return ConsoleOutcome::PhaseTimeout;
    
    const uint32_t seqBefore = eventSequence.load();
    const uint32_t droppedBefore = droppedEdges.load();
    const uint32_t pressStartUs = micros();
    const uint32_t pressStartMs = millis();
    bool releasePending = false;

    while (!abortRequested.load()) {
      const uint8_t validBefore = tracker.valid;
      const bool changed = updateTracker(tracker);
      if (millis() - tracker.lastValidChangeMs > config.timing.phaseWatchdogMs) return ConsoleOutcome::WatchdogTimeout;
      if (millis() - pressStartMs >= holdMs) releasePending = true;
      if (releasePending && changed && validBefore == target && tracker.valid != target) break;
      if (isValidPhase(tracker.valid)) driveO2(o2Response(tracker.valid, micros() - tracker.validStartUs, button), last);
    }
    if (abortRequested.load()) return ConsoleOutcome::Aborted;
    
    if (isValidPhase(tracker.valid)) driveO2(o2Response(tracker.valid, micros() - tracker.validStartUs, ButtonId::Unknown), last);
    return waitAck(tracker, last, seqBefore, pressStartUs, droppedBefore, metrics);
  }

  void emitEvent(const TreadmillCommand& c, CommandStatus status, ConsoleOutcome outcome, ButtonId button,
                 uint8_t step, uint8_t stepCount, uint8_t attempt, uint8_t sequenceRestart,
                 const AckMetrics& ack, const char* detail, float norm) {
    CommandEvent e{};
    e.requestId = c.requestId; e.status = status; e.outcome = outcome; e.button = button;
    e.requestedValue = c.value; e.normalizedValue = norm; e.step = step; e.stepCount = stepCount;
    e.localAttempt = attempt; e.sequenceRestart = sequenceRestart; e.timestampMs = millis(); e.ack = ack;
    if (detail) strlcpy(e.detail, detail, sizeof(e.detail));
    xQueueSend(commandEventQueue, &e, 0);
  }

  bool appendStep(PlanStep* plan, uint8_t& count, ButtonId button, uint16_t hold, uint16_t pause) {
    if (count >= PLAN_CAPACITY) return false;
    plan[count++] = {button, hold, pause};
    return true;
  }

  bool buildPlan(const TreadmillCommand& c, PlanStep* plan, uint8_t& count, float& norm, const char*& err) {
    count = 0; err = nullptr; norm = c.value;
    if (c.type == CommandType::PressSpeedPlus) return appendStep(plan, count, ButtonId::SpeedPlus, config.timing.relayHoldMs, config.timing.relayPauseMs);
    if (c.type == CommandType::PressSpeedMinus) return appendStep(plan, count, ButtonId::SpeedMinus, config.timing.relayHoldMs, config.timing.relayPauseMs);
    
    if (c.type == CommandType::PressButton) {
      if (c.button == ButtonId::Clear && !config.clearMapping.verified) { err = "CLR mapping unverified"; return false; }
      if (c.button == ButtonId::SpeedPlus || c.button == ButtonId::SpeedMinus) return appendStep(plan, count, c.button, config.timing.relayHoldMs, config.timing.relayPauseMs);
      if (targetPhase(c.button) == 0xFF) { err = "Button is not an active output"; return false; }
      
      uint16_t h = isDigit(c.button) ? config.timing.digitHoldMs : config.timing.enterHoldMs;
      uint16_t pa = config.timing.digitPauseMs;
      if (c.button == ButtonId::InstantSpeed || c.button == ButtonId::InstantIncline) { h = config.timing.instantHoldMs; pa = config.timing.postInstantPauseMs; }
      else if (c.button == ButtonId::Enter) { h = config.timing.enterHoldMs; pa = config.timing.enterPauseMs; }
      else if (c.button == ButtonId::Clear) { h = config.timing.clearHoldMs; pa = config.timing.postClearPauseMs; }
      return appendStep(plan, count, c.button, h, pa);
    }
    
    const bool speed = (c.type == CommandType::SetSpeed);
    if (!speed && c.type != CommandType::SetIncline) { err = "Unsupported command"; return false; }
    if (!std::isfinite(c.value)) { err = "Non-finite value"; return false; }
    
    if (speed) {
      norm = std::round(c.value * 10.0f) / 10.0f;
      if (norm < 0.8f || norm > 25.0f) { err = "Speed out of range"; return false; }
    } else {
      if (c.value < 0.0f || c.value > 15.0f || std::round(c.value) != c.value) { err = "Incline must be integer 0-15"; return false; }
      norm = c.value;
    }
    
    int base = std::max(speed ? 1 : 0, std::min(speed ? 25 : 15, (int)std::round(norm)));
    int tenths = speed ? (int)std::round((norm - base) * 10.0f) : 0;
    
    addStepToPlan(plan, count, speed ? ButtonId::InstantSpeed : ButtonId::InstantIncline, config.timing.instantHoldMs, config.timing.postInstantPauseMs);
    
    char ds[4]{}; snprintf(ds, sizeof(ds), "%d", base);
    for (size_t i = 0; i < strlen(ds); i++) {
      addStepToPlan(plan, count, digitButton(ds[i]), i ? config.timing.digitHoldMs : config.timing.firstDigitHoldMs, 
                    i + 1 == strlen(ds) ? config.timing.preEnterPauseMs : config.timing.digitPauseMs);
    }
    
    addStepToPlan(plan, count, ButtonId::Enter, config.timing.enterHoldMs, config.timing.enterPauseMs);
    
    for (int i = 0; i < std::abs(tenths); i++) {
      addStepToPlan(plan, count, tenths > 0 ? ButtonId::SpeedPlus : ButtonId::SpeedMinus, config.timing.relayHoldMs, config.timing.relayPauseMs);
    }
    return true;
  }

  void addStepToPlan(PlanStep* plan, uint8_t& count, ButtonId button, uint16_t hold, uint16_t pause) {
    if (count < PLAN_CAPACITY) plan[count++] = {button, hold, pause};
  }

  bool establishOwnership(PhaseTracker& tracker, uint8_t& last) {
    uint8_t o1, o2; readAtomic(o1, o2); tracker.lastRaw = o1;
    uint32_t st = millis(); bool sync = false;
    while (millis() - st < config.timing.syncTimeoutMs) if (updateTracker(tracker)) { sync = true; break; }
    if (!sync) return false;
    last = o2;
    if (!waitFreshPhase(tracker, IDLE, config.timing.idleWaitTimeoutMs, last)) return false;
    uint8_t base = o2Response(tracker.valid, micros() - tracker.validStartUs, ButtonId::Unknown);
    last = ~base; driveO2(base, last);
    digitalWrite(config.pins.muteAll, LOW); delayMicroseconds(config.timing.muteSwitchDelayUs);
    setO2Direction(true);
    return true;
  }

  void releaseOwnership() {
    setO2Direction(false); delayMicroseconds(config.timing.muteSwitchDelayUs);
    digitalWrite(config.pins.muteAll, HIGH);
  }

  void maintainBaseline(PhaseTracker& tracker, uint8_t& last, uint16_t ms) {
    uint32_t st = millis();
    while (millis() - st < ms && !abortRequested.load()) {
      updateTracker(tracker);
      if (isValidPhase(tracker.valid)) driveO2(o2Response(tracker.valid, micros() - tracker.validStartUs, ButtonId::Unknown), last);
    }
  }

  void pulseRelay(ButtonId b, uint16_t h, uint16_t pa) {
    gpio_num_t p = (b == ButtonId::SpeedPlus) ? config.pins.speedPlusRelay : config.pins.speedMinusRelay;
    digitalWrite(p, LOW); vTaskDelay(pdMS_TO_TICKS(h)); digitalWrite(p, HIGH); vTaskDelay(pdMS_TO_TICKS(pa));
  }

  void executeCommand(const TreadmillCommand& c) {
    active.store(true); abortRequested.store(false);
    PlanStep p[PLAN_CAPACITY]{}; uint8_t n = 0; float norm = 0; const char* err = nullptr;
    
    if (!buildPlan(c, p, n, norm, err)) {
      emitEvent(c, CommandStatus::Rejected, c.button == ButtonId::Clear ? ConsoleOutcome::ClearMappingUnverified : ConsoleOutcome::Invalid, c.button, 0, 0, 0, 0, AckMetrics{}, err, norm);
      active.store(false); return;
    }
    
    emitEvent(c, CommandStatus::Started, ConsoleOutcome::NormalSingle, ButtonId::Unknown, 0, n, 0, 0, AckMetrics{}, "Macro Started", norm);
    bool bus = false; for (uint8_t i = 0; i < n; i++) if (p[i].button != ButtonId::SpeedPlus && p[i].button != ButtonId::SpeedMinus) bus = true;
    
    PhaseTracker t{}; uint8_t last = 0xFF;
    if (bus && !establishOwnership(t, last)) {
      releaseOwnership(); emitEvent(c, CommandStatus::Failed, ConsoleOutcome::SyncFailure, ButtonId::Unknown, 0, n, 0, 0, AckMetrics{}, "Ownership failed", norm);
      active.store(false); return;
    }

    if (c.type == CommandType::PressButton || c.type == CommandType::PressSpeedPlus || c.type == CommandType::PressSpeedMinus) {
      for (uint8_t i = 0; i < n && !abortRequested.load(); i++) {
        if (p[i].button == ButtonId::SpeedPlus || p[i].button == ButtonId::SpeedMinus) {
          emitEvent(c, CommandStatus::StepStarted, ConsoleOutcome::NormalSingle, p[i].button, i + 1, n, 1, 0, AckMetrics{}, "Relay pulse", norm);
          pulseRelay(p[i].button, p[i].holdMs, p[i].pauseMs);
          emitEvent(c, CommandStatus::StepConfirmed, ConsoleOutcome::NormalSingle, p[i].button, i + 1, n, 1, 0, AckMetrics{}, "Relay done", norm);
        } else {
          AckMetrics a{}; uint8_t attempt = 0; ConsoleOutcome o;
          do {
            attempt++;
            if (attempt > 1) maintainBaseline(t, last, config.timing.retryCooldownMs);
            emitEvent(c, attempt == 1 ? CommandStatus::StepStarted : CommandStatus::Retrying, ConsoleOutcome::NoResponse, p[i].button, i + 1, n, attempt, 0, AckMetrics{}, attempt == 1 ? "Sending button" : "Retry NO_RESPONSE", norm);
            a = {}; o = pressOwned(t, last, p[i].button, p[i].holdMs, a);
          } while (o == ConsoleOutcome::NoResponse && attempt <= config.timing.maxLocalRetries && !abortRequested.load());

          if (o != ConsoleOutcome::NormalSingle) {
            emitEvent(c, CommandStatus::Failed, o, p[i].button, i + 1, n, attempt, 0, a, "Button press failed", norm);
            if (bus) releaseOwnership();
            active.store(false); return;
          }
          emitEvent(c, CommandStatus::StepConfirmed, o, p[i].button, i + 1, n, attempt, 0, a, "Button confirmed", norm);
          maintainBaseline(t, last, p[i].pauseMs);
        }
      }
      if (bus) {
        waitFreshPhase(t, IDLE, config.timing.endSyncTimeoutMs, last);
        releaseOwnership();
      }
      emitEvent(c, abortRequested.load() ? CommandStatus::Aborted : CommandStatus::Completed, abortRequested.load() ? ConsoleOutcome::Aborted : ConsoleOutcome::NormalSingle, ButtonId::Unknown, n, n, 0, 0, AckMetrics{}, abortRequested.load() ? "Aborted" : "Completed", norm);
      active.store(false);
      return;
    }

    uint8_t enterIdx = 0;
    bool hasEnter = false;
    for (uint8_t i = 1; i < n; i++) {
      if (p[i].button == ButtonId::Enter) { enterIdx = i; hasEnter = true; break; }
    }
    if (!hasEnter) {
      emitEvent(c, CommandStatus::Failed, ConsoleOutcome::Invalid, ButtonId::Unknown, 0, n, 0, 0, AckMetrics{}, "Invalid plan: Enter missing", norm);
      releaseOwnership();
      active.store(false);
      return;
    }

    // 1. Execute Mode Step (InstantSpeed / InstantIncline)
    AckMetrics modeAck{}; uint8_t modeAttempt = 0; ConsoleOutcome modeOutcome;
    do {
      modeAttempt++;
      if (modeAttempt > 1) maintainBaseline(t, last, config.timing.retryCooldownMs);
      emitEvent(c, modeAttempt == 1 ? CommandStatus::StepStarted : CommandStatus::Retrying, ConsoleOutcome::NoResponse, p[0].button, 1, n, modeAttempt, 0, AckMetrics{}, modeAttempt == 1 ? "Sending mode" : "Retry NO_RESPONSE", norm);
      modeAck = {}; modeOutcome = pressOwned(t, last, p[0].button, p[0].holdMs, modeAck);
    } while (modeOutcome == ConsoleOutcome::NoResponse && modeAttempt <= config.timing.maxLocalRetries && !abortRequested.load());

    if (modeOutcome != ConsoleOutcome::NormalSingle || abortRequested.load()) {
      emitEvent(c, abortRequested.load() ? CommandStatus::Aborted : CommandStatus::Failed, abortRequested.load() ? ConsoleOutcome::Aborted : modeOutcome, p[0].button, 1, n, modeAttempt, 0, modeAck, "Mode selection failed", norm);
      releaseOwnership();
      active.store(false); return;
    }
    emitEvent(c, CommandStatus::StepConfirmed, modeOutcome, p[0].button, 1, n, modeAttempt, 0, modeAck, "Mode confirmed", norm);
    maintainBaseline(t, last, p[0].pauseMs);

    // 2. Execute Digit Sequence with CLR Recovery
    uint8_t sequenceRestarts = 0;
    bool digitsCompleted = false;

    while (!digitsCompleted && !abortRequested.load()) {
      bool digitFailed = false;
      ConsoleOutcome failOutcome = ConsoleOutcome::HardwareError;
      ButtonId failButton = ButtonId::Unknown;
      uint8_t failStep = 0;
      AckMetrics failAck{};

      for (uint8_t i = 1; i < enterIdx && !abortRequested.load(); i++) {
        AckMetrics digitAck{}; uint8_t attempt = 0; ConsoleOutcome o;
        do {
          attempt++;
          if (attempt > 1) maintainBaseline(t, last, config.timing.retryCooldownMs);
          emitEvent(c, attempt == 1 ? CommandStatus::StepStarted : CommandStatus::Retrying, ConsoleOutcome::NoResponse, p[i].button, i + 1, n, attempt, sequenceRestarts, AckMetrics{}, attempt == 1 ? "Sending digit" : "Retry NO_RESPONSE", norm);
          digitAck = {}; o = pressOwned(t, last, p[i].button, p[i].holdMs, digitAck);
        } while (o == ConsoleOutcome::NoResponse && attempt <= config.timing.maxLocalRetries && !abortRequested.load());

        if (abortRequested.load()) break;

        if (o != ConsoleOutcome::NormalSingle) {
          if (!requiresNumericRecovery(o)) {
            emitEvent(c, CommandStatus::Failed, o, p[i].button, i + 1, n, attempt, sequenceRestarts, digitAck, "Digit execution failed", norm);
            releaseOwnership();
            active.store(false);
            return;
          }
          digitFailed = true; failOutcome = o; failButton = p[i].button; failStep = i + 1; failAck = digitAck;
          break; // Stop numeric entry, drop to CLR recovery
        }

        emitEvent(c, CommandStatus::StepConfirmed, o, p[i].button, i + 1, n, attempt, sequenceRestarts, digitAck, "Digit confirmed", norm);
        maintainBaseline(t, last, p[i].pauseMs);
      }

      if (!digitFailed && !abortRequested.load()) {
        digitsCompleted = true;
        break; // All digits successfully confirmed
      }

      if (abortRequested.load()) break;

      // CLR Recovery Flow
      if (sequenceRestarts >= config.timing.maxSequenceRestarts) {
        emitEvent(c, CommandStatus::Failed, failOutcome, failButton, failStep, n, 0, sequenceRestarts, failAck, "Recovery Failed: Max restarts exceeded", norm);
        releaseOwnership();
        active.store(false); return;
      }

      if (!config.clearMapping.verified) {
        emitEvent(c, CommandStatus::Failed, ConsoleOutcome::ClearMappingUnverified, ButtonId::Clear, failStep, n, 0, sequenceRestarts, AckMetrics{}, "Recovery Unavailable: CLR not verified", norm);
        releaseOwnership();
        active.store(false); return;
      }

      emitEvent(c, CommandStatus::RecoveryRequired, failOutcome, ButtonId::Clear, failStep, n, 0, sequenceRestarts, failAck, "Digit error. Sending CLR", norm);

      AckMetrics clrAck{};
      ConsoleOutcome clrRes = pressOwned(t, last, ButtonId::Clear, config.timing.clearHoldMs, clrAck);

      if (clrRes != ConsoleOutcome::NormalSingle) {
        emitEvent(c, CommandStatus::Failed, ConsoleOutcome::ClearFailed, ButtonId::Clear, failStep, n, 0, sequenceRestarts, clrAck, "CLR failed during recovery", norm);
        releaseOwnership();
        active.store(false); return;
      }

      maintainBaseline(t, last, config.timing.postClearPauseMs);
      sequenceRestarts++;
      emitEvent(c, CommandStatus::Retrying, ConsoleOutcome::NormalSingle, ButtonId::Clear, 0, n, 0, sequenceRestarts, clrAck, "CLR confirmed. Restarting digit sequence", norm);
    }

    if (abortRequested.load() || !digitsCompleted) {
      emitEvent(c, CommandStatus::Aborted, ConsoleOutcome::Aborted, ButtonId::Unknown, 0, n, 0, sequenceRestarts, AckMetrics{}, "Command Aborted", norm);
      releaseOwnership();
      active.store(false); return;
    }

    // 3. Enter Step (Strictly Gated - No CLR on failure)
    AckMetrics enterAck{};
    emitEvent(c, CommandStatus::StepStarted, ConsoleOutcome::NoResponse, ButtonId::Enter, enterIdx + 1, n, 1, sequenceRestarts, AckMetrics{}, "Sending Enter", norm);
    ConsoleOutcome enterOutcome = pressOwned(t, last, ButtonId::Enter, p[enterIdx].holdMs, enterAck);

    if (enterOutcome != ConsoleOutcome::NormalSingle || abortRequested.load()) {
      emitEvent(c, abortRequested.load() ? CommandStatus::Aborted : CommandStatus::Failed, abortRequested.load() ? ConsoleOutcome::Aborted : enterOutcome, ButtonId::Enter, enterIdx + 1, n, 1, sequenceRestarts, enterAck, "Enter failed", norm);
      releaseOwnership();
      active.store(false); return;
    }
    emitEvent(c, CommandStatus::StepConfirmed, enterOutcome, ButtonId::Enter, enterIdx + 1, n, 1, sequenceRestarts, enterAck, "Enter confirmed", norm);
    maintainBaseline(t, last, p[enterIdx].pauseMs);

    // 4. Safe Bus Release
    if (!waitFreshPhase(t, IDLE, config.timing.endSyncTimeoutMs, last)) {
      releaseOwnership();
      emitEvent(c, CommandStatus::Failed, ConsoleOutcome::IdleTimeout, ButtonId::Unknown, 0, n, 0, sequenceRestarts, AckMetrics{}, "Reconnect IDLE missing", norm);
      active.store(false); return;
    }
    releaseOwnership();
    vTaskDelay(pdMS_TO_TICKS(config.timing.busToRelayDelayMs));

    // 5. Execute Relay Steps (if any)
    for (uint8_t i = enterIdx + 1; i < n && !abortRequested.load(); i++) {
      if (p[i].button == ButtonId::SpeedPlus || p[i].button == ButtonId::SpeedMinus) {
        emitEvent(c, CommandStatus::StepStarted, ConsoleOutcome::NormalSingle, p[i].button, i + 1, n, 1, sequenceRestarts, AckMetrics{}, "Relay pulse", norm);
        pulseRelay(p[i].button, p[i].holdMs, p[i].pauseMs);
        emitEvent(c, CommandStatus::StepConfirmed, ConsoleOutcome::NormalSingle, p[i].button, i + 1, n, 1, sequenceRestarts, AckMetrics{}, "Relay done", norm);
      }
    }

    emitEvent(c, abortRequested.load() ? CommandStatus::Aborted : CommandStatus::Completed, abortRequested.load() ? ConsoleOutcome::Aborted : ConsoleOutcome::NormalSingle, ButtonId::Unknown, n, n, 0, sequenceRestarts, AckMetrics{}, abortRequested.load() ? "Aborted" : "Completed", norm);
    active.store(false);
  }

  static void commandTaskRouter(void* arg) { static_cast<Impl*>(arg)->commandTask(); }
  void commandTask() { TreadmillCommand c{}; for (;;) if (xQueueReceive(requestQueue, &c, portMAX_DELAY) == pdTRUE) executeCommand(c); }

  ButtonId decodePhysical(uint8_t o1, uint8_t o2) const {
    if (o1 != PHASE_D) return ButtonId::Unknown;
    if (o2 == 0xD0) return ButtonId::QuickStart; if (o2 == 0xC2) return ButtonId::Stop;
    if (o2 == 0xC4) return ButtonId::SpeedPlus; if (o2 == 0xC8) return ButtonId::SpeedMinus;
    if (o2 == 0xC1) return ButtonId::InclinePlus; if (o2 == 0xE0) return ButtonId::InclineMinus;
    return ButtonId::Unknown;
  }

  static void panelTaskRouter(void* arg) { static_cast<Impl*>(arg)->panelTask(); }
  void panelTask() {
    ButtonId cand = ButtonId::Unknown, rep = ButtonId::Unknown;
    uint32_t since = 0, seen = 0, pressed = 0; uint8_t co1 = 0xFF, co2 = 0xFF;
    for (;;) {
      if (active.load()) { cand = rep = ButtonId::Unknown; vTaskDelay(pdMS_TO_TICKS(10)); continue; }
      uint8_t o1, o2; readAtomic(o1, o2); uint32_t now = millis(); ButtonId ob = decodePhysical(o1, o2);
      if (ob != ButtonId::Unknown) {
        if (ob != cand) { cand = ob; since = now; co1 = o1; co2 = o2; }
        seen = now;
        if (rep == ButtonId::Unknown && now - since >= config.timing.passiveStableMs) {
          rep = cand; pressed = now;
          PhysicalButtonEvent e{rep, PhysicalButtonAction::Pressed, now, 0, co1, co2};
          xQueueSend(physicalEventQueue, &e, 0);
        }
      }
      if (cand != ButtonId::Unknown && now - seen >= config.timing.passiveStableMs) {
        if (rep != ButtonId::Unknown) {
          PhysicalButtonEvent e{rep, PhysicalButtonAction::Released, now, now - pressed, co1, co2};
          xQueueSend(physicalEventQueue, &e, 0);
        }
        cand = rep = ButtonId::Unknown;
      }
      vTaskDelay(pdMS_TO_TICKS(config.timing.passivePollMs));
    }
  }
};

ConsoleInterface::Impl* ConsoleInterface::Impl::isrOwner = nullptr;

ConsoleInterface::ConsoleInterface() : impl_(new Impl{}) {}
ConsoleInterface::~ConsoleInterface() { end(); delete impl_; }

bool ConsoleInterface::begin(const ConsoleConfig& c, ConsoleExecutionMode mode) {
  if (impl_->ready.load()) return true;
  impl_->config = c;
  impl_->executionMode = mode;
  impl_->requestQueue = xQueueCreate(c.requestQueueDepth, sizeof(TreadmillCommand));
  impl_->commandEventQueue = xQueueCreate(c.commandEventQueueDepth, sizeof(CommandEvent));
  impl_->physicalEventQueue = xQueueCreate(c.physicalEventQueueDepth, sizeof(PhysicalButtonEvent));
  if (!impl_->requestQueue || !impl_->commandEventQueue || !impl_->physicalEventQueue) {
    end();
    return false;
  }
  
  if (mode == ConsoleExecutionMode::SoftwareSink) {
    // Simulation: Pure software command intent forwarding, zero GPIO, zero tasks
    impl_->ready.store(true);
    return true;
  }

  pinMode(c.pins.txsOe, OUTPUT); digitalWrite(c.pins.txsOe, LOW);
  pinMode(c.pins.muteAll, OUTPUT); digitalWrite(c.pins.muteAll, HIGH);
  pinMode(c.pins.speedPlusRelay, OUTPUT_OPEN_DRAIN); digitalWrite(c.pins.speedPlusRelay, HIGH);
  pinMode(c.pins.speedMinusRelay, OUTPUT_OPEN_DRAIN); digitalWrite(c.pins.speedMinusRelay, HIGH);
  pinMode(c.pins.buzzerIn, INPUT_PULLUP);
  for (int p : c.pins.o1) pinMode(p, INPUT);
  for (int p : c.pins.o2) pinMode(p, INPUT);
  digitalWrite(c.pins.txsOe, HIGH);
  
  Impl::isrOwner = impl_;
  attachInterrupt(digitalPinToInterrupt(c.pins.buzzerIn), Impl::buzzerIsrRouter, CHANGE);
  
  if (xTaskCreatePinnedToCore(Impl::buzzerTaskRouter, "ConsoleBuzzer", 4096, impl_, 4, &impl_->buzzerTaskHandle, 0) != pdPASS ||
      xTaskCreatePinnedToCore(Impl::panelTaskRouter, "ConsolePanel", 4096, impl_, 2, &impl_->panelTaskHandle, 1) != pdPASS ||
      xTaskCreatePinnedToCore(Impl::commandTaskRouter, "ConsoleCommand", 8192, impl_, 18, &impl_->commandTaskHandle, 1) != pdPASS) {
    end(); return false;
  }
  impl_->ready.store(true); return true;
}

bool ConsoleInterface::begin(ConsoleExecutionMode mode) {
  return begin(ConsoleConfig{}, mode);
}

void ConsoleInterface::end() {
  if (!impl_) return;
  impl_->abortRequested.store(true);

  if (impl_->executionMode == ConsoleExecutionMode::HardwareMatrix) {
    detachInterrupt(digitalPinToInterrupt(impl_->config.pins.buzzerIn));
    if (impl_->commandTaskHandle) { vTaskDelete(impl_->commandTaskHandle); impl_->commandTaskHandle = nullptr; }
    if (impl_->panelTaskHandle) { vTaskDelete(impl_->panelTaskHandle); impl_->panelTaskHandle = nullptr; }
    if (impl_->buzzerTaskHandle) { vTaskDelete(impl_->buzzerTaskHandle); impl_->buzzerTaskHandle = nullptr; }
    impl_->releaseOwnership();
    digitalWrite(impl_->config.pins.speedPlusRelay, HIGH);
    digitalWrite(impl_->config.pins.speedMinusRelay, HIGH);
    if (Impl::isrOwner == impl_) Impl::isrOwner = nullptr;
  }

  if (impl_->requestQueue) { vQueueDelete(impl_->requestQueue); impl_->requestQueue = nullptr; }
  if (impl_->commandEventQueue) { vQueueDelete(impl_->commandEventQueue); impl_->commandEventQueue = nullptr; }
  if (impl_->physicalEventQueue) { vQueueDelete(impl_->physicalEventQueue); impl_->physicalEventQueue = nullptr; }
  impl_->active.store(false); impl_->ready.store(false);
}

bool ConsoleInterface::submit(const TreadmillCommand& c, TickType_t w) {
  if (!isReady()) return false;

  if (impl_->executionMode == ConsoleExecutionMode::SoftwareSink) {
    if (impl_->requestQueue) {
      xQueueSend(impl_->requestQueue, &c, w);
    }
    impl_->emitEvent(c, CommandStatus::Queued, ConsoleOutcome::NormalSingle, c.button, 0, 1, 0, 0, AckMetrics{}, "Queued", c.value);

    bool accepted = true;
    ICommandIntentSink* sink = nullptr;
    portENTER_CRITICAL(&impl_->sinkMux);
    sink = impl_->commandSink;
    portEXIT_CRITICAL(&impl_->sinkMux);

    if (sink) {
      accepted = sink->onCommandIntent(c);
    }

    if (accepted) {
      impl_->emitEvent(c, CommandStatus::Completed, ConsoleOutcome::NormalSingle, c.button, 1, 1, 1, 0, AckMetrics{}, "SoftwareSink Completed", c.value);
    } else {
      impl_->emitEvent(c, CommandStatus::Rejected, ConsoleOutcome::Invalid, c.button, 0, 1, 0, 0, AckMetrics{}, "Rejected by sink", c.value);
    }
    return accepted;
  }

  if (xQueueSend(impl_->requestQueue, &c, w) != pdTRUE) return false;
  impl_->emitEvent(c, CommandStatus::Queued, ConsoleOutcome::NormalSingle, c.button, 0, 0, 0, 0, AckMetrics{}, "Queued", c.value);
  return true;
}

bool ConsoleInterface::receiveCommandEvent(CommandEvent& e, TickType_t w) { return impl_->commandEventQueue && xQueueReceive(impl_->commandEventQueue, &e, w) == pdTRUE; }
bool ConsoleInterface::receivePhysicalButtonEvent(PhysicalButtonEvent& e, TickType_t w) { return impl_->physicalEventQueue && xQueueReceive(impl_->physicalEventQueue, &e, w) == pdTRUE; }
bool ConsoleInterface::isActive() const { return impl_ && impl_->active.load(); }
bool ConsoleInterface::isReady() const { return impl_ && impl_->ready.load(); }
void ConsoleInterface::abortActiveCommand() { if (impl_) impl_->abortRequested.store(true); }

ConsoleExecutionMode ConsoleInterface::getExecutionMode() const {
  return impl_ ? impl_->executionMode : ConsoleExecutionMode::HardwareMatrix;
}

void ConsoleInterface::registerCommandSink(ICommandIntentSink* sink) {
  if (!impl_) return;
  portENTER_CRITICAL(&impl_->sinkMux);
  impl_->commandSink = sink;
  portEXIT_CRITICAL(&impl_->sinkMux);
}

void ConsoleInterface::triggerEmergencyStop(bool active) {
  if (!impl_) return;
  ICommandIntentSink* sink = nullptr;
  portENTER_CRITICAL(&impl_->sinkMux);
  sink = impl_->commandSink;
  portEXIT_CRITICAL(&impl_->sinkMux);
  if (sink) {
    sink->onEmergencyStop(active);
  }
}

ConsoleConfig ConsoleInterface::configSnapshot() const { return impl_->config; }

bool ConsoleInterface::setClearMapping(const ButtonMapping& m) {
  if (!impl_ || impl_->active.load() || m.button != ButtonId::Clear || !m.verified || !isValidPhase(m.targetPhase) || m.targetPhase == IDLE) return false;
  if (m.preloadPhase != 0xFF && (!isValidPhase(m.preloadPhase) || m.preloadPhase == IDLE)) return false;
  impl_->config.clearMapping = m; return true;
}

bool ConsoleInterface::clearMappingVerified() const { return impl_ && impl_->config.clearMapping.verified; }

ConsoleOutcome ConsoleInterface::pressButton(ButtonId button, uint16_t holdMs, AckMetrics& metrics) {
  if (!isReady()) return ConsoleOutcome::HardwareError;
  if (impl_->executionMode == ConsoleExecutionMode::SoftwareSink) {
    TreadmillCommand cmd{};
    cmd.type = CommandType::PressButton;
    cmd.button = button;
    cmd.value = 0.0f;
    bool ok = submit(cmd, 0);
    return ok ? ConsoleOutcome::NormalSingle : ConsoleOutcome::Invalid;
  }
  if (impl_->active.exchange(true)) return ConsoleOutcome::HardwareError;
  impl_->abortRequested.store(false); PhaseTracker t{}; uint8_t last = 0xFF;
  if (!impl_->establishOwnership(t, last)) {
    impl_->releaseOwnership();
    impl_->active.store(false);
    return ConsoleOutcome::SyncFailure;
  }
  ConsoleOutcome res = impl_->pressOwned(t, last, button, holdMs, metrics);
  if (!impl_->waitFreshPhase(t, IDLE, impl_->config.timing.endSyncTimeoutMs, last)) {
    impl_->releaseOwnership();
    impl_->active.store(false);
    return res == ConsoleOutcome::NormalSingle ? ConsoleOutcome::IdleTimeout : res;
  }
  impl_->releaseOwnership();
  impl_->active.store(false);
  return res;
}

const char* ConsoleInterface::version() { return "ConsoleInterface/0.2.0 (V5.7 Recovery Engine)"; }

const char* buttonName(ButtonId b) {
  switch (b) {
    case ButtonId::InstantSpeed: return "instant_speed"; case ButtonId::InstantIncline: return "instant_incline";
    case ButtonId::Enter: return "enter"; case ButtonId::Clear: return "clear";
    case ButtonId::Num0: return "num0"; case ButtonId::Num1: return "num1"; case ButtonId::Num2: return "num2";
    case ButtonId::Num3: return "num3"; case ButtonId::Num4: return "num4"; case ButtonId::Num5: return "num5";
    case ButtonId::Num6: return "num6"; case ButtonId::Num7: return "num7"; case ButtonId::Num8: return "num8";
    case ButtonId::Num9: return "num9"; case ButtonId::SpeedPlus: return "speed_plus";
    case ButtonId::SpeedMinus: return "speed_minus"; case ButtonId::InclinePlus: return "incline_plus";
    case ButtonId::InclineMinus: return "incline_minus"; case ButtonId::QuickStart: return "quick_start";
    case ButtonId::Stop: return "stop"; default: return "unknown";
  }
}

const char* outcomeName(ConsoleOutcome o) {
  switch (o) {
    case ConsoleOutcome::NormalSingle: return "NORMAL_SINGLE"; case ConsoleOutcome::NoResponse: return "NO_RESPONSE";
    case ConsoleOutcome::NormalLong: return "NORMAL_LONG"; case ConsoleOutcome::MergedMulti: return "MERGED_MULTI";
    case ConsoleOutcome::DistinctMulti: return "DISTINCT_MULTI"; case ConsoleOutcome::EdgeLoss: return "EDGE_LOSS";
    case ConsoleOutcome::Invalid: return "INVALID"; case ConsoleOutcome::SyncFailure: return "SYNC_FAILURE";
    case ConsoleOutcome::IdleTimeout: return "IDLE_TIMEOUT"; case ConsoleOutcome::PhaseTimeout: return "PHASE_TIMEOUT";
    case ConsoleOutcome::WatchdogTimeout: return "WATCHDOG_TIMEOUT"; case ConsoleOutcome::ClearMappingUnverified: return "CLEAR_MAPPING_UNVERIFIED";
    case ConsoleOutcome::ClearFailed: return "CLEAR_FAILED"; case ConsoleOutcome::RecoveryUnavailable: return "RECOVERY_UNAVAILABLE";
    case ConsoleOutcome::Aborted: return "ABORTED"; default: return "HARDWARE_ERROR";
  }
}

const char* commandStatusName(CommandStatus s) {
  switch (s) {
    case CommandStatus::Queued: return "queued"; case CommandStatus::Started: return "started";
    case CommandStatus::StepStarted: return "step_started"; case CommandStatus::StepConfirmed: return "step_confirmed";
    case CommandStatus::Retrying: return "retrying"; case CommandStatus::RecoveryRequired: return "recovery_required";
    case CommandStatus::Completed: return "completed"; case CommandStatus::Rejected: return "rejected";
    case CommandStatus::Aborted: return "aborted"; default: return "failed";
  }
}

} // namespace stridecontrol