#include "FileSystemManager.h"

namespace stridecontrol {

FileSystemManager& FileSystemManager::instance() {
    static FileSystemManager s_instance;
    return s_instance;
}

bool FileSystemManager::begin(bool formatOnFail) {
    if (mounted_) {
        return true;
    }

    Serial.println("[FileSystemManager] Mounting LittleFS filesystem...");
    if (!LittleFS.begin(formatOnFail)) {
        Serial.println("[FileSystemManager] ERROR: Failed to mount LittleFS!");
        mounted_ = false;
        return false;
    }

    mounted_ = true;
    const size_t total = totalBytes();
    const size_t used = usedBytes();
    const size_t free = freeBytes();

    Serial.println("[FileSystemManager] LittleFS mounted successfully.");
    Serial.printf("[FileSystemManager] Storage Stats: Total: %u bytes (%u KB), Used: %u bytes (%u KB), Free: %u bytes (%u KB)\n",
                  static_cast<unsigned int>(total), static_cast<unsigned int>(total / 1024),
                  static_cast<unsigned int>(used), static_cast<unsigned int>(used / 1024),
                  static_cast<unsigned int>(free), static_cast<unsigned int>(free / 1024));

    return true;
}

void FileSystemManager::end() {
    if (mounted_) {
        LittleFS.end();
        mounted_ = false;
        Serial.println("[FileSystemManager] LittleFS unmounted.");
    }
}

bool FileSystemManager::isMounted() const {
    return mounted_;
}

size_t FileSystemManager::totalBytes() const {
    return mounted_ ? LittleFS.totalBytes() : 0;
}

size_t FileSystemManager::usedBytes() const {
    return mounted_ ? LittleFS.usedBytes() : 0;
}

size_t FileSystemManager::freeBytes() const {
    if (!mounted_) return 0;
    const size_t total = LittleFS.totalBytes();
    const size_t used = LittleFS.usedBytes();
    return (total > used) ? (total - used) : 0;
}

bool FileSystemManager::exists(const char* path) const {
    return mounted_ && LittleFS.exists(path);
}

} // namespace stridecontrol

