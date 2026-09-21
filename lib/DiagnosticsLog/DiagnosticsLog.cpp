#include "DiagnosticsLog.h"
#include <cstdarg>
#include <cstring>
#include <new>

namespace stridecontrol {

DiagnosticsLog& DiagnosticsLog::instance() {
    static DiagnosticsLog inst;
    return inst;
}

void DiagnosticsLog::addEntry(const char* message) {
    portENTER_CRITICAL(&mux_);
    strncpy(entries_[writeIndex_], message, kMaxEntryLength - 1);
    entries_[writeIndex_][kMaxEntryLength - 1] = '\0';
    entryTimestampMs_[writeIndex_] = millis();
    writeIndex_ = (writeIndex_ + 1) % kMaxEntries;
    if (count_ < kMaxEntries) {
        count_++;
    }
    portEXIT_CRITICAL(&mux_);
}

void DiagnosticsLog::addEntryf(const char* fmt, ...) {
    char buffer[kMaxEntryLength];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    addEntry(buffer);
}

void DiagnosticsLog::dumpTo(Print& output) const {
    // Determine current entry count and allocation requirements under lock
    size_t total = 0;
    size_t start = 0;

    portENTER_CRITICAL(&mux_);
    total = count_;
    start = (count_ < kMaxEntries) ? 0 : writeIndex_;
    
    // Allocate a lightweight temporary snapshot buffer on the stack if small,
    // or allocate dynamically if entries can be up to 150. Given kMaxEntries = 150
    // and kMaxEntryLength = 128 (~20KB), dynamic allocation or a static safe copy is needed.
    portEXIT_CRITICAL(&mux_);

    if (total == 0) {
        return;
    }

    // Use a heap allocation or vector/buffer copy outside the critical section
    struct LogCopy {
        uint32_t ts;
        char msg[kMaxEntryLength];
    };

    LogCopy* copyBuffer = new (std::nothrow) LogCopy[total];
    if (copyBuffer == nullptr) {
        return;
    }

    portENTER_CRITICAL(&mux_);
    const size_t currentTotal = count_;
    const size_t currentStart = (count_ < kMaxEntries) ? 0 : writeIndex_;
    const size_t copyCount = (currentTotal < total) ? currentTotal : total;
    for (size_t i = 0; i < copyCount; ++i) {
        const size_t idx = (currentStart + i) % kMaxEntries;
        copyBuffer[i].ts = entryTimestampMs_[idx];
        std::memcpy(copyBuffer[i].msg, entries_[idx], kMaxEntryLength);
        copyBuffer[i].msg[kMaxEntryLength - 1] = '\0';
    }
    portEXIT_CRITICAL(&mux_);

    // Perform blocking network I/O completely OUTSIDE the critical section
    for (size_t i = 0; i < copyCount; ++i) {
        output.printf("[%lu ms] %s\n", static_cast<unsigned long>(copyBuffer[i].ts), copyBuffer[i].msg);
    }

    delete[] copyBuffer;
}

} // namespace stridecontrol

