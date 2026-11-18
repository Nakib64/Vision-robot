#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// update() — Dispatch to the active state handler each loop iteration
// ---------------------------------------------------------------------------
void NavigationFSM::update(const TargetDetection &target) {
    unsigned long now = millis();

    switch (m_state) {
        case RobotState::SWEEP_SCAN:
            handleSweepScan(target, now);
            break;
        case RobotState::APPROACH_TARGET:
            handleApproach(target, now);
            break;
        case RobotState::TARGET_REACHED:
            handleTargetReached(now);
            break;
        case RobotState::CLEARANCE_PIVOT:
            handleClearancePivot(now);
            break;
    }
}

// ---------------------------------------------------------------------------
// stateName() — Human-readable state string for serial logging
// ---------------------------------------------------------------------------
const char *NavigationFSM::stateName(RobotState s) {
    switch (s) {
        case RobotState::SWEEP_SCAN:       return "SWEEP_SCAN";
        case RobotState::APPROACH_TARGET:  return "APPROACH_TARGET";
        case RobotState::TARGET_REACHED:   return "TARGET_REACHED";
        case RobotState::CLEARANCE_PIVOT:  return "CLEARANCE_PIVOT";
        default:                           return "UNKNOWN";
    }
}
