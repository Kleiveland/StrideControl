#include "WorkoutEngine.h"

namespace stridecontrol {

WorkoutEngine& WorkoutEngine::instance() {
    static WorkoutEngine s_instance;
    return s_instance;
}

} // namespace stridecontrol

