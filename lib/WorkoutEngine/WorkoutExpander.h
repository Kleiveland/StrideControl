#pragma once

#include <cstddef>
#include "SettingsTypes.h"
#include "WorkoutExecutionTypes.h"

namespace stridecontrol {

class WorkoutExpander {
public:
    static bool expand(
        const WorkoutDefinition& input,
        ExpandedWorkout& output,
        char* errorMsg = nullptr,
        size_t errorMsgLen = 0,
        float maxAchievableSpeedKmh = 25.0f
    );
};

} // namespace stridecontrol
