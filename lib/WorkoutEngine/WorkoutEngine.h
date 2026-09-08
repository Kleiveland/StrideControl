#pragma once

#include "WorkoutExecutionTypes.h"
#include "WorkoutExpander.h"

namespace stridecontrol {

class WorkoutEngine {
public:
    static WorkoutEngine& instance();

    WorkoutEngine() = default;
    ~WorkoutEngine() = default;

    WorkoutEngine(const WorkoutEngine&) = delete;
    WorkoutEngine& operator=(const WorkoutEngine&) = delete;

    const ExpandedWorkout& getExpandedWorkout() const { return expandedWorkout_; }
    ExpandedWorkout& getExpandedWorkout() { return expandedWorkout_; }

    bool loadWorkout(const WorkoutDefinition& def, char* errorMsg = nullptr, size_t errorMsgLen = 0) {
        return WorkoutExpander::expand(def, expandedWorkout_, errorMsg, errorMsgLen);
    }

    void reset() {
        expandedWorkout_ = ExpandedWorkout{};
    }

private:
    ExpandedWorkout expandedWorkout_{};
};

} // namespace stridecontrol

