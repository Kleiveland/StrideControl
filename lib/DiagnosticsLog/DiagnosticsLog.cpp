#include "DiagnosticsLog.h"
#include <cstdarg>
#include <cstring>

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
    portENTER_CRITICAL(&mux_);
    const size_t total = count_;
    const size_t start = (count_ < kMaxEntries) ? 0 : writeIndex_;
    for (size_t i = 0; i < total; ++i) {
        const size_t idx = (start + i) % kMaxEntries;
        output.printf("[%lu ms] %s\n", static_cast<unsigned long>(entryTimestampMs_[idx]), entries_[idx]);
    }
    portEXIT_CRITICAL(&mux_);
}

} // namespace stridecontrol

