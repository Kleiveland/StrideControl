#include "SettingsService.h"
#include <Preferences.h>
#include <LittleFS.h>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>

namespace stridecontrol {

void SystemSettingsDeleter::operator()(SystemSettings* p) const {
    if (p) {
        p->~SystemSettings();
        heap_caps_free(p);
    }
}

SystemSettingsPtr makeSystemSettings() {
    void* mem = nullptr;
#if defined(BOARD_HAS_PSRAM) || defined(CONFIG_SPIRAM_SUPPORT)
    if (psramFound()) {
        mem = heap_caps_malloc(sizeof(SystemSettings), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
#endif
    if (!mem) {
        mem = heap_caps_malloc(sizeof(SystemSettings), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    }
    if (!mem) {
        mem = malloc(sizeof(SystemSettings));
    }
    if (!mem) {
        Serial.println("[SettingsService] CRITICAL: Failed to allocate SystemSettings on heap/PSRAM!");
        return nullptr;
    }
    SystemSettings* obj = new (mem) SystemSettings();
    return SystemSettingsPtr(obj);
}

// ===========================================================================
// ENUM STRING CONVERSIONS
// ===========================================================================

const char* durationTypeName(DurationType type) {
    switch (type) {
        case DurationType::METERS: return "METERS";
        case DurationType::TIME_SECONDS:
        default: return "TIME_SECONDS";
    }
}

DurationType parseDurationType(const char* str) {
    if (!str) return DurationType::TIME_SECONDS;
    if (strcasecmp(str, "METERS") == 0 || strcasecmp(str, "meter") == 0 || strcasecmp(str, "distance") == 0) {
        return DurationType::METERS;
    }
    return DurationType::TIME_SECONDS;
}

const char* speedModeName(SpeedMode mode) {
    switch (mode) {
        case SpeedMode::FREE: return "FREE";
        case SpeedMode::FIXED:
        default: return "FIXED";
    }
}

SpeedMode parseSpeedMode(const char* str) {
    if (!str) return SpeedMode::FIXED;
    if (strcasecmp(str, "FREE") == 0 || strcasecmp(str, "manual") == 0) {
        return SpeedMode::FREE;
    }
    return SpeedMode::FIXED;
}

const char* stepRoleName(StepRole role) {
    switch (role) {
        case StepRole::WARMUP: return "WARMUP";
        case StepRole::REST: return "REST";
        case StepRole::COOLDOWN: return "COOLDOWN";
        case StepRole::WORK:
        default: return "WORK";
    }
}

StepRole parseStepRole(const char* str) {
    if (!str) return StepRole::WORK;
    if (strcasecmp(str, "WARMUP") == 0 || strcasecmp(str, "warm") == 0) return StepRole::WARMUP;
    if (strcasecmp(str, "REST") == 0 || strcasecmp(str, "rest") == 0 || strcasecmp(str, "hvile") == 0) return StepRole::REST;
    if (strcasecmp(str, "COOLDOWN") == 0 || strcasecmp(str, "cool") == 0 || strcasecmp(str, "nedjogg") == 0) return StepRole::COOLDOWN;
    return StepRole::WORK;
}

const char* segmentTypeName(SegmentType type) {
    switch (type) {
        case SegmentType::REPEATING_GROUP: return "REPEATING_GROUP";
        case SegmentType::SINGLE_STEP:
        default: return "SINGLE_STEP";
    }
}

SegmentType parseSegmentType(const char* str) {
    if (!str) return SegmentType::SINGLE_STEP;
    if (strcasecmp(str, "REPEATING_GROUP") == 0 || strcasecmp(str, "interval") == 0 || strcasecmp(str, "group") == 0) {
        return SegmentType::REPEATING_GROUP;
    }
    return SegmentType::SINGLE_STEP;
}

// ===========================================================================
// SETTINGSSERVICE IMPLEMENTATION
// ===========================================================================

SettingsService& SettingsService::instance() {
    static SettingsService s_instance;
    return s_instance;
}

SettingsService::SettingsService() {
    mutex_ = xSemaphoreCreateMutex();
}

SettingsService::~SettingsService() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

void SettingsService::begin() {
    if (initialized_) {
        return;
    }

    Serial.println("[SettingsService] Initializing settings subsystem...");
    recoverAndLoadSettings();
    initialized_ = true;
    Serial.println("[SettingsService] Settings subsystem initialized.");
}

// ===========================================================================
// HARDWARE NVS PERSISTENCE (Backwards-Compatible API)
// ===========================================================================

SystemConfig SettingsService::loadAll() {
    SystemConfig cfg{};
    cfg.incline = getInclineConfig();
    cfg.speed = getSpeedConfig();
    cfg.maintenance = getMaintenanceConfig();
    return cfg;
}

InclineConfig SettingsService::getInclineConfig() {
    InclineConfig cfg{};
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Preferences prefs;
        if (prefs.begin(kNvsNamespace, true)) {
            cfg.homingOffset = prefs.getInt("inc_offset", 0);
            cfg.isCalibrated = prefs.getBool("inc_cal", false);
            prefs.end();
        }
        xSemaphoreGive(mutex_);
    }
    return cfg;
}

SpeedConfig SettingsService::getSpeedConfig() {
    SpeedConfig cfg{};
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Preferences prefs;
        if (prefs.begin(kNvsNamespace, true)) {
            cfg.maxAchievableSpeedKmh = prefs.getFloat("spd_max", 25.0f);
            cfg.maxAchievableSpeedVerified = prefs.getBool("spd_max_v", false);
            cfg.commandMapValid = prefs.getBool("spd_map_v", false);
            cfg.pointCount = prefs.getUChar("spd_pts", 0);
            if (cfg.pointCount > kMaxSpeedCalibrationPoints) {
                cfg.pointCount = kMaxSpeedCalibrationPoints;
            }
            prefs.getBytes("spd_data", cfg.points.data(), cfg.pointCount * sizeof(SpeedCalibrationPoint));
            prefs.end();
        }
        xSemaphoreGive(mutex_);
    }
    return cfg;
}

MaintenanceConfig SettingsService::getMaintenanceConfig() {
    MaintenanceConfig cfg{};
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Preferences prefs;
        if (prefs.begin(kNvsNamespace, true)) {
            cfg.totalDistanceMeters = prefs.getULong64("maint_dist", 0);
            cfg.totalTimeSeconds = prefs.getULong64("maint_time", 0);
            prefs.end();
        }
        xSemaphoreGive(mutex_);
    }
    return cfg;
}

bool SettingsService::saveInclineConfig(const InclineConfig& config) {
    bool ok = false;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Preferences prefs;
        if (prefs.begin(kNvsNamespace, false)) {
            prefs.putInt("inc_offset", config.homingOffset);
            prefs.putBool("inc_cal", config.isCalibrated);
            prefs.end();
            ok = true;
        }
        xSemaphoreGive(mutex_);
    }
    return ok;
}

bool SettingsService::saveSpeedConfig(const SpeedConfig& config) {
    bool ok = false;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Preferences prefs;
        if (prefs.begin(kNvsNamespace, false)) {
            prefs.putFloat("spd_max", config.maxAchievableSpeedKmh);
            prefs.putBool("spd_max_v", config.maxAchievableSpeedVerified);
            prefs.putBool("spd_map_v", config.commandMapValid);
            prefs.putUChar("spd_pts", config.pointCount);
            if (config.pointCount > 0) {
                prefs.putBytes("spd_data", config.points.data(), config.pointCount * sizeof(SpeedCalibrationPoint));
            }
            prefs.end();
            ok = true;
        }
        xSemaphoreGive(mutex_);
    }
    return ok;
}

bool SettingsService::saveMaintenanceConfig(const MaintenanceConfig& config) {
    bool ok = false;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Preferences prefs;
        if (prefs.begin(kNvsNamespace, false)) {
            prefs.putULong64("maint_dist", config.totalDistanceMeters);
            prefs.putULong64("maint_time", config.totalTimeSeconds);
            prefs.end();
            ok = true;
        }
        xSemaphoreGive(mutex_);
    }
    return ok;
}

void SettingsService::factoryReset() {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Preferences prefs;
        if (prefs.begin(kNvsNamespace, false)) {
            prefs.clear();
            prefs.end();
        }
        if (!activeSettings_) {
            activeSettings_ = makeSystemSettings();
        }
        if (activeSettings_) {
            populateFactoryDefaults(*activeSettings_);
            saveSystemSettingsAtomic(*activeSettings_);
        }
        xSemaphoreGive(mutex_);
    }
}

// ===========================================================================
// FACTORY DEFAULTS GENERATOR (In-Place Reference Population)
// ===========================================================================

static void populateDefaultWorkout321(WorkoutDefinition& w) {
    w.id = 1;
    strncpy(w.name, "3-2-1 X 2", sizeof(w.name) - 1);
    w.name[sizeof(w.name) - 1] = '\0';
    w.segmentCount = 3;
    w.lastUsedTimestamp = 1000;

    // Segment 0: Warmup
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::TIME_SECONDS, 600, SpeedMode::FREE, 9.0f, 0, false};

    // Segment 1: Interval (3-2-1 repeating group x2)
    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 2;
    w.segments[1].startSpeedKmh = 15.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.0f;
    w.segments[1].stepCount = 5;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, 180, SpeedMode::FIXED, 15.0f, 0, false};
    w.segments[1].steps[1] = {3, StepRole::REST, DurationType::TIME_SECONDS, 90,  SpeedMode::FREE,  6.0f,  0, false};
    w.segments[1].steps[2] = {4, StepRole::WORK, DurationType::TIME_SECONDS, 120, SpeedMode::FIXED, 15.0f, 0, false};
    w.segments[1].steps[3] = {5, StepRole::REST, DurationType::TIME_SECONDS, 90,  SpeedMode::FREE,  6.0f,  0, false};
    w.segments[1].steps[4] = {6, StepRole::WORK, DurationType::TIME_SECONDS, 60,  SpeedMode::FIXED, 15.0f, 0, false};

    // Segment 2: Cooldown
    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {7, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};
}

static void populateDefaultWorkout4x4(WorkoutDefinition& w) {
    w.id = 2;
    strncpy(w.name, "4 X 4 TERSKEL", sizeof(w.name) - 1);
    w.name[sizeof(w.name) - 1] = '\0';
    w.segmentCount = 3;
    w.lastUsedTimestamp = 900;

    // Segment 0: Warmup
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::TIME_SECONDS, 600, SpeedMode::FREE, 9.0f, 0, false};

    // Segment 1: Interval (4x4 min work / 3 min rest)
    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 4;
    w.segments[1].startSpeedKmh = 14.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.0f;
    w.segments[1].stepCount = 2;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, 240, SpeedMode::FIXED, 14.0f, 0, false};
    w.segments[1].steps[1] = {3, StepRole::REST, DurationType::TIME_SECONDS, 180, SpeedMode::FREE,  6.0f,  0, false};

    // Segment 2: Cooldown
    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {4, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};
}

static void populateDefaultWorkout45_15(WorkoutDefinition& w) {
    w.id = 3;
    strncpy(w.name, "45 / 15 PROG", sizeof(w.name) - 1);
    w.name[sizeof(w.name) - 1] = '\0';
    w.segmentCount = 3;
    w.lastUsedTimestamp = 800;

    // Segment 0: Warmup
    w.segments[0].id = 1;
    w.segments[0].type = SegmentType::SINGLE_STEP;
    w.segments[0].repetitions = 1;
    w.segments[0].stepCount = 1;
    w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::TIME_SECONDS, 600, SpeedMode::FREE, 9.0f, 0, false};

    // Segment 1: Interval (10x 45s work / 15s rest with 0.2 km/h progression)
    w.segments[1].id = 2;
    w.segments[1].type = SegmentType::REPEATING_GROUP;
    w.segments[1].repetitions = 10;
    w.segments[1].startSpeedKmh = 14.0f;
    w.segments[1].speedProgressionPerRepKmh = 0.2f;
    w.segments[1].stepCount = 2;
    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, 45, SpeedMode::FIXED, 14.0f, 0, false};
    w.segments[1].steps[1] = {3, StepRole::REST, DurationType::TIME_SECONDS, 15, SpeedMode::FREE,  6.0f,  0, false};

    // Segment 2: Cooldown
    w.segments[2].id = 3;
    w.segments[2].type = SegmentType::SINGLE_STEP;
    w.segments[2].repetitions = 1;
    w.segments[2].stepCount = 1;
    w.segments[2].steps[0] = {4, StepRole::COOLDOWN, DurationType::TIME_SECONDS, 300, SpeedMode::FREE, 8.0f, 0, false};
}

void SettingsService::populateFactoryDefaults(SystemSettings& s) {
    memset(&s, 0, sizeof(SystemSettings));
    s.schemaVersion = 1;

    const char* defaultUserNames[MAX_USERS] = {
        "Bruker 1", "Bruker 2", "Bruker 3", "Bruker 4"
    };

    const std::array<float, 8> defaultSpeedKeys = {6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 15.0f, 16.0f, 18.0f};
    const std::array<uint8_t, 8> defaultInclineKeys = {0, 1, 2, 3, 4, 5, 6, 8};

    for (size_t i = 0; i < MAX_USERS; ++i) {
        UserProfile& u = s.users[i];
        u.id = static_cast<uint8_t>(i + 1);
        strncpy(u.name, defaultUserNames[i], sizeof(u.name) - 1);
        u.name[sizeof(u.name) - 1] = '\0';
        u.hvileSpeedKmh = 6.0f;
        u.dragSpeedKmh = 15.0f;
        u.speedQuickKeys = defaultSpeedKeys;
        u.inclineQuickKeys = defaultInclineKeys;

        u.workoutCount = 3;
        populateDefaultWorkout321(u.workouts[0]);
        populateDefaultWorkout4x4(u.workouts[1]);
        populateDefaultWorkout45_15(u.workouts[2]);

        u.selectedWorkoutId = 1;
        u.recentWorkoutIds = {2, 3};
    }
}

// ===========================================================================
// VALIDATION LOGIC
// ===========================================================================

bool SettingsService::validateSystemSettings(const SystemSettings& s, char* errBuf, size_t errBufLen) {
    auto setErr = [&](const char* msg) {
        if (errBuf && errBufLen > 0) {
            strncpy(errBuf, msg, errBufLen - 1);
            errBuf[errBufLen - 1] = '\0';
        }
    };

    if (s.schemaVersion == 0) {
        setErr("Invalid schemaVersion (must be >= 1)");
        return false;
    }

    for (size_t uIdx = 0; uIdx < MAX_USERS; ++uIdx) {
        const UserProfile& u = s.users[uIdx];
        if (u.id == 0) {
            setErr("User ID cannot be 0");
            return false;
        }
        for (size_t other = uIdx + 1; other < MAX_USERS; ++other) {
            if (u.id == s.users[other].id) {
                setErr("Duplicate User ID detected");
                return false;
            }
        }
        if (strlen(u.name) == 0 || strlen(u.name) > MAX_USER_NAME_LENGTH) {
            setErr("User name missing or exceeds MAX_USER_NAME_LENGTH");
            return false;
        }
        if (u.hvileSpeedKmh < 0.0f || u.hvileSpeedKmh > 25.0f ||
            u.dragSpeedKmh < 0.0f || u.dragSpeedKmh > 25.0f) {
            setErr("User hvile/drag speed out of range (0-25 km/h)");
            return false;
        }
        for (float spd : u.speedQuickKeys) {
            if (spd < 0.0f || spd > 25.0f) {
                setErr("Speed quick key out of range (0-25 km/h)");
                return false;
            }
        }
        for (uint8_t inc : u.inclineQuickKeys) {
            if (inc > 15) {
                setErr("Incline quick key out of range (0-15%)");
                return false;
            }
        }
        if (u.workoutCount > MAX_WORKOUTS_PER_USER) {
            setErr("User workoutCount exceeds MAX_WORKOUTS_PER_USER");
            return false;
        }

        for (size_t wIdx = 0; wIdx < u.workoutCount; ++wIdx) {
            const WorkoutDefinition& w = u.workouts[wIdx];
            if (w.id == 0) {
                setErr("Workout ID cannot be 0");
                return false;
            }
            for (size_t otherW = wIdx + 1; otherW < u.workoutCount; ++otherW) {
                if (w.id == u.workouts[otherW].id) {
                    setErr("Duplicate workout ID within user profile");
                    return false;
                }
            }
            if (strlen(w.name) == 0 || strlen(w.name) > MAX_WORKOUT_NAME_LENGTH) {
                setErr("Workout name missing or exceeds MAX_WORKOUT_NAME_LENGTH");
                return false;
            }
            if (w.segmentCount < 3 || w.segmentCount > MAX_SEGMENTS_PER_WORKOUT) {
                setErr("Workout must have at least 3 segments (Warmup, Interval/Work, Cooldown)");
                return false;
            }

            // Segment 0 must be Warmup (SINGLE_STEP, stepCount == 1)
            const WorkoutSegment& seg0 = w.segments[0];
            if (seg0.type != SegmentType::SINGLE_STEP || seg0.stepCount != 1 || seg0.steps[0].role != StepRole::WARMUP) {
                setErr("First segment must be a SINGLE_STEP Warmup");
                return false;
            }
            if (seg0.steps[0].targetSpeedKmh < 0.0f || seg0.steps[0].targetSpeedKmh > 25.0f) {
                setErr("Warmup step speed target out of range (0-25 km/h)");
                return false;
            }
            if (seg0.steps[0].targetInclinePct > 15) {
                setErr("Warmup step incline target out of range (0-15%)");
                return false;
            }

            // Last segment must be Cooldown (SINGLE_STEP, stepCount == 1)
            const WorkoutSegment& segLast = w.segments[w.segmentCount - 1];
            if (segLast.type != SegmentType::SINGLE_STEP || segLast.stepCount != 1 || segLast.steps[0].role != StepRole::COOLDOWN) {
                setErr("Last segment must be a SINGLE_STEP Cooldown");
                return false;
            }
            if (segLast.steps[0].targetSpeedKmh < 0.0f || segLast.steps[0].targetSpeedKmh > 25.0f) {
                setErr("Cooldown step speed target out of range (0-25 km/h)");
                return false;
            }
            if (segLast.steps[0].targetInclinePct > 15) {
                setErr("Cooldown step incline target out of range (0-15%)");
                return false;
            }

            bool hasActiveStep = (seg0.steps[0].durationValue > 0) || (segLast.steps[0].durationValue > 0);

            // Middle segments must not be Warmup or Cooldown, and zero duration is strictly invalid
            for (size_t sIdx = 1; sIdx < w.segmentCount - 1; ++sIdx) {
                const WorkoutSegment& seg = w.segments[sIdx];
                if (seg.stepCount == 0 || seg.stepCount > MAX_STEPS_PER_GROUP) {
                    setErr("Segment step count out of range (1-8)");
                    return false;
                }
                if (seg.type == SegmentType::SINGLE_STEP && seg.repetitions != 1) {
                    setErr("SINGLE_STEP segment must have repetitions == 1");
                    return false;
                }
                if (seg.type == SegmentType::REPEATING_GROUP && seg.repetitions < 1) {
                    setErr("REPEATING_GROUP segment must have repetitions >= 1");
                    return false;
                }
                for (size_t stepIdx = 0; stepIdx < seg.stepCount; ++stepIdx) {
                    const WorkoutStep& step = seg.steps[stepIdx];
                    if (step.role == StepRole::WARMUP || step.role == StepRole::COOLDOWN) {
                        setErr("Intermediate segment cannot contain Warmup or Cooldown steps");
                        return false;
                    }
                    if (step.durationValue == 0) {
                        setErr("Step duration value must be > 0");
                        return false;
                    }
                    hasActiveStep = true;
                    if (step.targetSpeedKmh < 0.0f || step.targetSpeedKmh > 25.0f) {
                        setErr("Step speed target out of range (0-25 km/h)");
                        return false;
                    }
                    if (step.targetInclinePct > 15) {
                        setErr("Step incline target out of range (0-15%)");
                        return false;
                    }
                }
            }

            if (!hasActiveStep) {
                setErr("Workout must contain at least one step with duration > 0");
                return false;
            }
        }
    }

    return true;
}

// ===========================================================================
// JSON SERIALIZATION & DESERIALIZATION
// ===========================================================================

void SettingsService::serializeSettingsJson(const SystemSettings& s, Print& output) {
    JsonDocument doc;
    doc["schemaVersion"] = s.schemaVersion;

    JsonArray usersArr = doc["users"].to<JsonArray>();
    for (size_t uIdx = 0; uIdx < MAX_USERS; ++uIdx) {
        const UserProfile& u = s.users[uIdx];
        JsonObject uObj = usersArr.add<JsonObject>();
        uObj["id"] = u.id;
        uObj["name"] = u.name;
        uObj["hvileSpeedKmh"] = u.hvileSpeedKmh;
        uObj["dragSpeedKmh"] = u.dragSpeedKmh;

        JsonArray spdKeys = uObj["speedQuickKeys"].to<JsonArray>();
        for (float k : u.speedQuickKeys) spdKeys.add(k);

        JsonArray incKeys = uObj["inclineQuickKeys"].to<JsonArray>();
        for (uint8_t k : u.inclineQuickKeys) incKeys.add(k);

        uObj["selectedWorkoutId"] = u.selectedWorkoutId;

        JsonArray recArr = uObj["recentWorkoutIds"].to<JsonArray>();
        for (uint16_t r : u.recentWorkoutIds) recArr.add(r);

        JsonArray wArr = uObj["workouts"].to<JsonArray>();
        for (size_t wIdx = 0; wIdx < u.workoutCount; ++wIdx) {
            const WorkoutDefinition& w = u.workouts[wIdx];
            JsonObject wObj = wArr.add<JsonObject>();
            wObj["id"] = w.id;
            wObj["name"] = w.name;
            wObj["lastUsedTimestamp"] = w.lastUsedTimestamp;

            JsonArray segArr = wObj["segments"].to<JsonArray>();
            for (size_t sIdx = 0; sIdx < w.segmentCount; ++sIdx) {
                const WorkoutSegment& seg = w.segments[sIdx];
                JsonObject segObj = segArr.add<JsonObject>();
                segObj["id"] = seg.id;
                segObj["type"] = segmentTypeName(seg.type);
                segObj["repetitions"] = seg.repetitions;
                segObj["startSpeedKmh"] = seg.startSpeedKmh;
                segObj["speedProgressionPerRepKmh"] = seg.speedProgressionPerRepKmh;

                JsonArray stArr = segObj["steps"].to<JsonArray>();
                for (size_t stepIdx = 0; stepIdx < seg.stepCount; ++stepIdx) {
                    const WorkoutStep& st = seg.steps[stepIdx];
                    JsonObject stObj = stArr.add<JsonObject>();
                    stObj["id"] = st.id;
                    stObj["role"] = stepRoleName(st.role);
                    stObj["durationType"] = durationTypeName(st.durationType);
                    stObj["durationValue"] = st.durationValue;
                    stObj["speedMode"] = speedModeName(st.speedMode);
                    stObj["targetSpeedKmh"] = st.targetSpeedKmh;
                    stObj["targetInclinePct"] = st.targetInclinePct;
                    stObj["setIncline"] = st.setIncline;
                }
            }
        }
    }

    serializeJson(doc, output);
}

bool SettingsService::deserializeSettingsJson(const uint8_t* jsonBytes, size_t length, SystemSettings& outCandidate, char* errBuf, size_t errBufLen) {
    auto setErr = [&](const char* msg) {
        if (errBuf && errBufLen > 0) {
            strncpy(errBuf, msg, errBufLen - 1);
            errBuf[errBufLen - 1] = '\0';
        }
    };

    if (!jsonBytes || length == 0) {
        setErr("Empty JSON payload");
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, jsonBytes, length);
    if (err) {
        setErr(err.c_str());
        return false;
    }

    memset(&outCandidate, 0, sizeof(SystemSettings));
    outCandidate.schemaVersion = doc["schemaVersion"] | 1;

    JsonArrayConst usersArr = doc["users"].as<JsonArrayConst>();
    if (usersArr.isNull() || usersArr.size() != MAX_USERS) {
        setErr("Payload must contain exactly 4 users");
        return false;
    }

    for (size_t uIdx = 0; uIdx < MAX_USERS; ++uIdx) {
        JsonObjectConst uObj = usersArr[uIdx];
        UserProfile& u = outCandidate.users[uIdx];
        u.id = uObj["id"] | static_cast<uint8_t>(uIdx + 1);

        const char* nameStr = uObj["name"] | "";
        strncpy(u.name, nameStr, sizeof(u.name) - 1);
        u.name[sizeof(u.name) - 1] = '\0';

        u.hvileSpeedKmh = uObj["hvileSpeedKmh"] | 6.0f;
        u.dragSpeedKmh = uObj["dragSpeedKmh"] | 15.0f;

        JsonArrayConst spdKeys = uObj["speedQuickKeys"].as<JsonArrayConst>();
        for (size_t k = 0; k < 8 && k < spdKeys.size(); ++k) {
            u.speedQuickKeys[k] = spdKeys[k] | 0.0f;
        }

        JsonArrayConst incKeys = uObj["inclineQuickKeys"].as<JsonArrayConst>();
        for (size_t k = 0; k < 8 && k < incKeys.size(); ++k) {
            u.inclineQuickKeys[k] = incKeys[k] | 0;
        }

        u.selectedWorkoutId = uObj["selectedWorkoutId"] | 0;
        JsonArrayConst recArr = uObj["recentWorkoutIds"].as<JsonArrayConst>();
        for (size_t r = 0; r < 2 && r < recArr.size(); ++r) {
            u.recentWorkoutIds[r] = recArr[r] | 0;
        }

        JsonArrayConst wArr = uObj["workouts"].as<JsonArrayConst>();
        u.workoutCount = std::min(wArr.size(), MAX_WORKOUTS_PER_USER);
        for (size_t wIdx = 0; wIdx < u.workoutCount; ++wIdx) {
            JsonObjectConst wObj = wArr[wIdx];
            WorkoutDefinition& w = outCandidate.users[uIdx].workouts[wIdx];
            w.id = wObj["id"] | static_cast<uint16_t>(wIdx + 1);
            const char* wName = wObj["name"] | "";
            strncpy(w.name, wName, sizeof(w.name) - 1);
            w.name[sizeof(w.name) - 1] = '\0';
            w.lastUsedTimestamp = wObj["lastUsedTimestamp"] | 0;

            JsonArrayConst segArr = wObj["segments"].as<JsonArrayConst>();
            w.segmentCount = std::min(segArr.size(), MAX_SEGMENTS_PER_WORKOUT);
            for (size_t sIdx = 0; sIdx < w.segmentCount; ++sIdx) {
                JsonObjectConst segObj = segArr[sIdx];
                WorkoutSegment& seg = w.segments[sIdx];
                seg.id = segObj["id"] | static_cast<uint16_t>(sIdx + 1);
                seg.type = parseSegmentType(segObj["type"] | "SINGLE_STEP");
                seg.repetitions = segObj["repetitions"] | 1;
                seg.startSpeedKmh = segObj["startSpeedKmh"] | 0.0f;
                seg.speedProgressionPerRepKmh = segObj["speedProgressionPerRepKmh"] | 0.0f;

                JsonArrayConst stArr = segObj["steps"].as<JsonArrayConst>();
                seg.stepCount = std::min(stArr.size(), MAX_STEPS_PER_GROUP);
                for (size_t stIdx = 0; stIdx < seg.stepCount; ++stIdx) {
                    JsonObjectConst stObj = stArr[stIdx];
                    WorkoutStep& st = seg.steps[stIdx];
                    st.id = stObj["id"] | static_cast<uint16_t>(stIdx + 1);
                    st.role = parseStepRole(stObj["role"] | "WORK");
                    st.durationType = parseDurationType(stObj["durationType"] | "TIME_SECONDS");
                    st.durationValue = stObj["durationValue"] | 0;
                    st.speedMode = parseSpeedMode(stObj["speedMode"] | "FIXED");
                    st.targetSpeedKmh = stObj["targetSpeedKmh"] | 0.0f;
                    st.targetInclinePct = stObj["targetInclinePct"] | 0;
                    st.setIncline = stObj["setIncline"] | false;
                }
            }
        }
    }

    return validateSystemSettings(outCandidate, errBuf, errBufLen);
}

// ===========================================================================
// POWER-FAIL-SAFE ATOMIC PERSISTENCE
// ===========================================================================

bool SettingsService::saveSystemSettingsAtomic(const SystemSettings& settings) {
    if (!LittleFS.exists(kConfigDir)) {
        LittleFS.mkdir(kConfigDir);
    }

    // Step 1 & 2: Write to .tmp and flush
    File tmpFile = LittleFS.open(kSettingsTmpFile, "w");
    if (!tmpFile) {
        Serial.println("[SettingsService] Failed to open tmp file for write.");
        return false;
    }

    serializeSettingsJson(settings, tmpFile);
    tmpFile.flush();
    tmpFile.close();

    // Step 4: Verify readable and valid
    File checkFile = LittleFS.open(kSettingsTmpFile, "r");
    if (!checkFile) {
        Serial.println("[SettingsService] Verification failed: unable to open tmp file.");
        LittleFS.remove(kSettingsTmpFile);
        return false;
    }

    size_t size = checkFile.size();
    if (size == 0) {
        Serial.println("[SettingsService] Verification failed: tmp file empty.");
        checkFile.close();
        LittleFS.remove(kSettingsTmpFile);
        return false;
    }

    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[size]);
    if (!buf) {
        checkFile.close();
        LittleFS.remove(kSettingsTmpFile);
        return false;
    }
    checkFile.read(buf.get(), size);
    checkFile.close();

    if (!candidateSettings_) {
        candidateSettings_ = makeSystemSettings();
        if (!candidateSettings_) {
            Serial.println("[SettingsService] Verification failed: candidate buffer allocation error.");
            LittleFS.remove(kSettingsTmpFile);
            return false;
        }
    }

    char err[128]{};
    if (!deserializeSettingsJson(buf.get(), size, *candidateSettings_, err, sizeof(err))) {
        Serial.printf("[SettingsService] Verification failed on tmp file: %s\n", err);
        LittleFS.remove(kSettingsTmpFile);
        return false;
    }

    // Step 5: Backup current file if exists
    if (LittleFS.exists(kSettingsFile)) {
        if (LittleFS.exists(kSettingsBakFile)) {
            LittleFS.remove(kSettingsBakFile);
        }
        LittleFS.rename(kSettingsFile, kSettingsBakFile);
    }

    // Step 6: Rename .tmp to current
    if (!LittleFS.rename(kSettingsTmpFile, kSettingsFile)) {
        Serial.println("[SettingsService] Failed to rename tmp to settings.json. Restoring backup...");
        if (LittleFS.exists(kSettingsBakFile)) {
            LittleFS.rename(kSettingsBakFile, kSettingsFile);
        }
        return false;
    }

    // Step 7: Remove .bak on successful replacement
    if (LittleFS.exists(kSettingsBakFile)) {
        LittleFS.remove(kSettingsBakFile);
    }

    if (activeSettings_) {
        *activeSettings_ = settings;
    }
    Serial.println("[SettingsService] Settings saved atomically and verified.");
    return true;
}

bool SettingsService::loadFactorySeedFromUsersJson(SystemSettings& target) {
    if (!LittleFS.exists(kUsersSeedFile)) {
        return false;
    }

    File seedFile = LittleFS.open(kUsersSeedFile, "r");
    if (!seedFile) {
        Serial.printf("[SettingsService] Failed to open %s for reading.\n", kUsersSeedFile);
        return false;
    }

    size_t size = seedFile.size();
    if (size == 0 || size > 32768) {
        Serial.printf("[SettingsService] Seed file %s invalid size: %u bytes\n", kUsersSeedFile, static_cast<unsigned int>(size));
        seedFile.close();
        return false;
    }

    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[size]);
    if (!buf) {
        Serial.println("[SettingsService] Failed to allocate buffer for seed file.");
        seedFile.close();
        return false;
    }

    size_t bytesRead = seedFile.read(buf.get(), size);
    seedFile.close();
    if (bytesRead != size) {
        Serial.println("[SettingsService] Failed to read full seed file.");
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf.get(), size);
    if (err) {
        Serial.printf("[SettingsService] Failed to parse %s JSON: %s\n", kUsersSeedFile, err.c_str());
        return false;
    }

    SystemSettingsPtr candidatePtr = makeSystemSettings();
    if (!candidatePtr) {
        Serial.println("[SettingsService] CRITICAL: Failed to allocate candidate for seed import.");
        return false;
    }
    SystemSettings& candidate = *candidatePtr;
    candidate.schemaVersion = 1;

    JsonArrayConst usersArr = doc["users"].as<JsonArrayConst>();
    if (usersArr.isNull() || usersArr.size() != MAX_USERS) {
        Serial.printf("[SettingsService] Seed error: payload must contain exactly %u users.\n", static_cast<unsigned int>(MAX_USERS));
        return false;
    }

    for (size_t uIdx = 0; uIdx < MAX_USERS; ++uIdx) {
        JsonObjectConst uObj = usersArr[uIdx];
        if (uObj.isNull()) return false;

        UserProfile& u = candidate.users[uIdx];
        u.id = static_cast<uint8_t>(uIdx + 1);

        const char* name = uObj["name"] | "";
        if (strlen(name) == 0 || strlen(name) > MAX_USER_NAME_LENGTH) {
            Serial.println("[SettingsService] Seed error: user name missing or exceeds MAX_USER_NAME_LENGTH.");
            return false;
        }
        strncpy(u.name, name, sizeof(u.name) - 1);
        u.name[sizeof(u.name) - 1] = '\0';

        JsonObjectConst presetsObj = uObj["presets"];
        if (presetsObj.isNull()) return false;
        if (!presetsObj["hvile"].is<float>() || !presetsObj["drag"].is<float>()) return false;
        float hvile = presetsObj["hvile"].as<float>();
        float drag = presetsObj["drag"].as<float>();
        if (!std::isfinite(hvile) || !std::isfinite(drag) || hvile < 0.0f || hvile > 25.0f || drag < 0.0f || drag > 25.0f) {
            Serial.println("[SettingsService] Seed error: user hvile/drag speed out of range.");
            return false;
        }
        u.hvileSpeedKmh = hvile;
        u.dragSpeedKmh = drag;

        JsonObjectConst qkObj = uObj["quick_keys"];
        if (qkObj.isNull()) return false;
        JsonArrayConst spdKeys = qkObj["speed"].as<JsonArrayConst>();
        JsonArrayConst incKeys = qkObj["incline"].as<JsonArrayConst>();
        if (spdKeys.isNull() || spdKeys.size() != 8 || incKeys.isNull() || incKeys.size() != 8) {
            Serial.println("[SettingsService] Seed error: quick keys arrays must contain exactly 8 elements.");
            return false;
        }
        for (size_t k = 0; k < 8; ++k) {
            if (!spdKeys[k].is<float>()) return false;
            float spd = spdKeys[k].as<float>();
            if (!std::isfinite(spd) || spd < 0.0f || spd > 25.0f) return false;
            u.speedQuickKeys[k] = spd;

            if (!incKeys[k].is<uint8_t>()) return false;
            uint8_t inc = incKeys[k].as<uint8_t>();
            if (inc > 15) return false;
            u.inclineQuickKeys[k] = inc;
        }

        JsonVariantConst histVar = uObj["history"];
        if (!histVar.is<JsonArrayConst>()) return false;
        JsonArrayConst histArr = histVar.as<JsonArrayConst>();
        if (histArr.size() > 0) {
            Serial.println("[SettingsService] Seed error: non-empty history array is incompatible.");
            return false;
        }

        JsonArrayConst wArr = uObj["workouts"].as<JsonArrayConst>();
        if (wArr.isNull() || wArr.size() != 3) {
            Serial.println("[SettingsService] Seed error: user must contain exactly 3 workouts.");
            return false;
        }
        u.workoutCount = 3;

        for (size_t wIdx = 0; wIdx < 3; ++wIdx) {
            JsonObjectConst wObj = wArr[wIdx];
            if (wObj.isNull()) return false;

            WorkoutDefinition& w = u.workouts[wIdx];
            w.id = static_cast<uint16_t>(wIdx + 1);

            const char* wName = wObj["name"] | "";
            if (strlen(wName) == 0 || strlen(wName) > MAX_WORKOUT_NAME_LENGTH) {
                Serial.printf("[SettingsService] Seed error: workout name '%s' invalid length.\n", wName);
                return false;
            }
            strncpy(w.name, wName, sizeof(w.name) - 1);
            w.name[sizeof(w.name) - 1] = '\0';
            w.lastUsedTimestamp = 0;

            if (!wObj["warmup_m"].is<float>() || !wObj["work_m"].is<float>() ||
                !wObj["rest_m"].is<float>() || !wObj["cooldown_m"].is<float>() ||
                !wObj["reps"].is<uint16_t>()) {
                Serial.println("[SettingsService] Seed error: workout numeric fields missing or wrong type.");
                return false;
            }

            float warmup_m = wObj["warmup_m"].as<float>();
            float work_m = wObj["work_m"].as<float>();
            float rest_m = wObj["rest_m"].as<float>();
            float cooldown_m = wObj["cooldown_m"].as<float>();
            uint16_t reps = wObj["reps"].as<uint16_t>();

            if (!std::isfinite(warmup_m) || warmup_m < 0.0f ||
                !std::isfinite(work_m) || work_m <= 0.0f ||
                !std::isfinite(rest_m) || rest_m < 0.0f ||
                !std::isfinite(cooldown_m) || cooldown_m < 0.0f ||
                reps < 1) {
                Serial.println("[SettingsService] Seed error: workout intervals invalid range.");
                return false;
            }

            uint32_t warmupSec = static_cast<uint32_t>(std::round(warmup_m * 60.0f));
            uint32_t workSec = static_cast<uint32_t>(std::round(work_m * 60.0f));
            uint32_t restSec = static_cast<uint32_t>(std::round(rest_m * 60.0f));
            uint32_t cooldownSec = static_cast<uint32_t>(std::round(cooldown_m * 60.0f));

            w.segmentCount = 3;

            // Segment 0: Warmup
            w.segments[0].id = 1;
            w.segments[0].type = SegmentType::SINGLE_STEP;
            w.segments[0].repetitions = 1;
            w.segments[0].stepCount = 1;
            w.segments[0].steps[0] = {1, StepRole::WARMUP, DurationType::TIME_SECONDS, warmupSec, SpeedMode::FREE, u.hvileSpeedKmh, 0, false};

            // Segment 1: Work / Intervals
            if (rest_m == 0.0f && reps == 1) {
                w.segments[1].id = 2;
                w.segments[1].type = SegmentType::SINGLE_STEP;
                w.segments[1].repetitions = 1;
                w.segments[1].stepCount = 1;
                w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, workSec, SpeedMode::FIXED, u.dragSpeedKmh, 0, false};
            } else {
                w.segments[1].id = 2;
                w.segments[1].type = SegmentType::REPEATING_GROUP;
                w.segments[1].repetitions = reps;
                w.segments[1].startSpeedKmh = u.dragSpeedKmh;
                w.segments[1].speedProgressionPerRepKmh = 0.0f;
                if (restSec > 0) {
                    w.segments[1].stepCount = 2;
                    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, workSec, SpeedMode::FIXED, u.dragSpeedKmh, 0, false};
                    w.segments[1].steps[1] = {3, StepRole::REST, DurationType::TIME_SECONDS, restSec, SpeedMode::FIXED, u.hvileSpeedKmh, 0, false};
                } else {
                    w.segments[1].stepCount = 1;
                    w.segments[1].steps[0] = {2, StepRole::WORK, DurationType::TIME_SECONDS, workSec, SpeedMode::FIXED, u.dragSpeedKmh, 0, false};
                }
            }

            // Segment 2: Cooldown
            w.segments[2].id = 3;
            w.segments[2].type = SegmentType::SINGLE_STEP;
            w.segments[2].repetitions = 1;
            w.segments[2].stepCount = 1;
            uint16_t cdStepId = (w.segments[1].stepCount == 2) ? 4 : 3;
            w.segments[2].steps[0] = {cdStepId, StepRole::COOLDOWN, DurationType::TIME_SECONDS, cooldownSec, SpeedMode::FREE, u.hvileSpeedKmh, 0, false};
        }

        if (!uObj["active_workout_idx"].is<int>()) return false;
        int activeIdx = uObj["active_workout_idx"].as<int>();
        if (activeIdx < 0 || activeIdx >= 3) {
            Serial.println("[SettingsService] Seed error: active_workout_idx out of range.");
            return false;
        }
        u.selectedWorkoutId = u.workouts[activeIdx].id;
        u.recentWorkoutIds = {0, 0};
    }

    char errBuf[128]{};
    if (!validateSystemSettings(candidate, errBuf, sizeof(errBuf))) {
        Serial.printf("[SettingsService] Seed candidate failed validation: %s\n", errBuf);
        return false;
    }

    target = candidate;
    return true;
}

bool SettingsService::recoverAndLoadSettings() {
    if (!activeSettings_) {
        activeSettings_ = makeSystemSettings();
        if (!activeSettings_) {
            Serial.println("[SettingsService] ERROR: Failed to allocate activeSettings_ on heap/PSRAM!");
            return false;
        }
    }

    if (!candidateSettings_) {
        candidateSettings_ = makeSystemSettings();
        if (!candidateSettings_) {
            Serial.println("[SettingsService] ERROR: Failed to allocate candidateSettings_ on heap/PSRAM!");
            return false;
        }
    }

    auto tryLoadFile = [&](const char* path) -> bool {
        if (!LittleFS.exists(path)) return false;
        File f = LittleFS.open(path, "r");
        if (!f) return false;
        size_t size = f.size();
        if (size == 0) { f.close(); return false; }

        std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[size]);
        if (!buf) { f.close(); return false; }
        f.read(buf.get(), size);
        f.close();

        char err[128]{};
        if (deserializeSettingsJson(buf.get(), size, *candidateSettings_, err, sizeof(err))) {
            *activeSettings_ = *candidateSettings_;
            Serial.printf("[SettingsService] Successfully loaded settings from: %s\n", path);
            return true;
        }
        Serial.printf("[SettingsService] Failed to validate settings from %s: %s\n", path, err);
        return false;
    };

    // 1. Try settings.json
    if (tryLoadFile(kSettingsFile)) return true;

    // 2. Try settings.json.tmp
    if (tryLoadFile(kSettingsTmpFile)) {
        saveSystemSettingsAtomic(*activeSettings_);
        return true;
    }

    // 3. Try settings.json.bak
    if (tryLoadFile(kSettingsBakFile)) {
        saveSystemSettingsAtomic(*activeSettings_);
        return true;
    }

    // 4. Try candidate factory seed from /config/users.json
    if (LittleFS.exists(kUsersSeedFile)) {
        if (loadFactorySeedFromUsersJson(*candidateSettings_)) {
            *activeSettings_ = *candidateSettings_;
            Serial.printf("[SettingsService] Successfully imported factory seed from: %s\n", kUsersSeedFile);
            saveSystemSettingsAtomic(*activeSettings_);
            return true;
        } else {
            Serial.printf("[SettingsService] Factory seed %s rejected or invalid. Falling through to hardcoded defaults.\n", kUsersSeedFile);
        }
    }

    // 5. Fallback to hardcoded factory defaults
    Serial.println("[SettingsService] No valid configuration found. Applying hardcoded factory defaults...");
    populateFactoryDefaults(*activeSettings_);
    saveSystemSettingsAtomic(*activeSettings_);
    return true;
}

bool SettingsService::loadSystemSettings() {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        bool ok = recoverAndLoadSettings();
        xSemaphoreGive(mutex_);
        return ok;
    }
    return false;
}

const SystemSettings* SettingsService::getActiveSettings() const {
    return activeSettings_.get();
}

const WorkoutDefinition* SettingsService::findWorkout(uint8_t userId, uint16_t workoutId) const {
    const SystemSettings* settings = getActiveSettings();
    if (settings == nullptr) {
        return nullptr;
    }
    for (const auto& user : settings->users) {
        if (user.id != userId) {
            continue;
        }
        for (uint8_t i = 0; i < user.workoutCount && i < user.workouts.size(); ++i) {
            if (user.workouts[i].id == workoutId) {
                return &user.workouts[i];
            }
        }
        return nullptr;
    }
    return nullptr;
}

bool SettingsService::updateSystemSettings(const SystemSettings& candidate, char* errBuf, size_t errBufLen) {
    if (!validateSystemSettings(candidate, errBuf, errBufLen)) {
        return false;
    }

    bool ok = false;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(2000)) == pdTRUE) {
        ok = saveSystemSettingsAtomic(candidate);
        xSemaphoreGive(mutex_);
    }
    return ok;
}

const char* SettingsService::version() {
    return "SettingsService/1.0.0 (Phase D Atomic Persistence)";
}

} // namespace stridecontrol
