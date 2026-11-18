#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// CLEARANCE_PIVOT — Back up a little, then LOOK FORWARD (avoiding visited one!)
// ---------------------------------------------------------------------------
void NavigationFSM::handleClearancePivot(unsigned long now) {
    (void)now;

    // Guaranteed full backup duration
    Serial.printf("[FSM:DECISION] STATE:CLEARANCE_PIVOT | ACTION:BACKING UP SLIGHTLY | L:-%d R:-%d\n",
                  SPEED_REVERSE, SPEED_REVERSE);
    Serial.flush();

    m_motors.reverse(SPEED_REVERSE);
    delay(REVERSE_DURATION_MS);
    m_motors.stop();

    // Done backing up -> Stop and immediately LOOK FORWARD (SWEEP_SCAN)
    // The just-reached target is marked VISITED in CameraVision, so:
    //   - If another target is visible -> robot drives straight to that one!
    //   - If no other target is visible -> robot rotates RIGHT 45° and looks again!
    m_vision.unlockTarget();
    resetBestTarget();
    m_sweepStep    = 0;
    m_stateTimer   = millis();
    m_lastLogTimer = 0;
    m_state        = RobotState::SWEEP_SCAN;
    Serial.println("[FSM:CLEAR] Backup complete -> Looking forward (avoiding visited target)");
}
