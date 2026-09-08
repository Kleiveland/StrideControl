#pragma once

#include <Arduino.h>
#include <LittleFS.h>

namespace stridecontrol {

class FileSystemManager {
public:
    static FileSystemManager& instance();

    // Non-copyable
    FileSystemManager(const FileSystemManager&) = delete;
    FileSystemManager& operator=(const FileSystemManager&) = delete;

    bool begin(bool formatOnFail = true);
    void end();

    bool isMounted() const;
    size_t totalBytes() const;
    size_t usedBytes() const;
    size_t freeBytes() const;
    bool exists(const char* path) const;

private:
    FileSystemManager() = default;
    ~FileSystemManager() = default;

    bool mounted_{false};
};

} // namespace stridecontrol

