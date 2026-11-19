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
            handleTargetReached(target, now);
            break;
        case RobotState::CLEARANCE_PIVOT:
            handleClearancePivot(now);
            break;
        case RobotState::ROBOT_HALTED:
            handleHalted(now);
            break;
    }
}

// ---------------------------------------------------------------------------
// handleHalted() — Low-power safety stop when no target is found in the room
// ---------------------------------------------------------------------------
void NavigationFSM::handleHalted(unsigned long now) {
    m_motors.stop();

    if (now - m_lastLogTimer >= 3000) {
        Serial.println("[FSM:DECISION] STATE:ROBOT_HALTED | ACTION:POWERED DOWN - NO TARGET AVAILABLE | L:0 R:0");
        m_lastLogTimer = now;
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
        case RobotState::ROBOT_HALTED:     return "ROBOT_HALTED";
        default:                           return "UNKNOWN";
    }
}
