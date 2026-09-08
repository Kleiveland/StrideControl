#include "TreadmillControllerAdapter.h"

namespace stridecontrol {

TreadmillControllerAdapter::TreadmillControllerAdapter(TreadmillController& controller)
    : controller_(controller) {}

bool TreadmillControllerAdapter::submitSpeedTarget(float targetSpeedKmh, uint32_t nowMs) {
    return controller_.submitSpeedTarget(targetSpeedKmh, nowMs);
}

bool TreadmillControllerAdapter::submitInclineTarget(float targetInclinePct, uint32_t nowMs) {
    return controller_.submitInclineTarget(targetInclinePct, nowMs);
}

bool TreadmillControllerAdapter::isBusy() const {
    return controller_.isBusy();
}

bool TreadmillControllerAdapter::isReady() const {
    return controller_.isReady();
}

const char* TreadmillControllerAdapter::version() {
    return "TreadmillControllerAdapter/1.0.0";
}

} // namespace stridecontrol

