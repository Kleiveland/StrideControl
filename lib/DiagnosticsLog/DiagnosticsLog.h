#pragma once
#include <Arduino.h>

namespace stridecontrol {

class DiagnosticsLog {
public:
    static DiagnosticsLog& instance();

    void addEntry(const char* message);
    void addEntryf(const char* fmt, ...);

    // Writes all buffered entries, oldest to newest, to the given stream.
    void dumpTo(Print& output) const;

private:
    DiagnosticsLog() = default;

    static constexpr size_t kMaxEntries = 150;
    static constexpr size_t kMaxEntryLength = 128;

    char entries_[kMaxEntries][kMaxEntryLength] = {};
    uint32_t entryTimestampMs_[kMaxEntries] = {0};
    size_t writeIndex_ = 0;
    size_t count_ = 0;
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

} // namespace stridecontrol

