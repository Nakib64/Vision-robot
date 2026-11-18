#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// TARGET_REACHED — Motors stopped, 400 ms pause to stabilize
// ---------------------------------------------------------------------------
void NavigationFSM::handleTargetReached(unsigned long now) {
    m_motors.stop();
    if (now - m_stateTimer >= 400) {
        Serial.println("[FSM:REACHED] Pause complete -> Backing up slightly");
        m_state        = RobotState::CLEARANCE_PIVOT;
        m_stateTimer   = now;
        m_lastLogTimer = 0;
    }
}
