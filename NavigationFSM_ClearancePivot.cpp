#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// CLEARANCE_PIVOT — Option B: Back up, turn 45° away, and resume 360° sweep!
//   1. Backs up for REVERSE_DURATION_MS (600 ms) to clear obstacle / marker wall.
//   2. Pivots right ~45° (PIVOT_AWAY_MS = 310 ms) so camera points away from
//      the marker it just visited.
//   3. Resets tracking & transitions directly to SWEEP_SCAN to search for
//      the next marker in the room.
// ---------------------------------------------------------------------------
void NavigationFSM::handleClearancePivot(unsigned long now) {
    (void)now;

    // 1. Guaranteed full backup duration
    Serial.printf("[FSM:DECISION] STATE:CLEARANCE_PIVOT | ACTION:BACKING UP (%dms) | L:-%d R:-%d\n",
                  REVERSE_DURATION_MS, SPEED_REVERSE, SPEED_REVERSE);
    Serial.flush();

    m_motors.reverse(SPEED_REVERSE);
    delay(REVERSE_DURATION_MS);
    m_motors.stop();
    delay(150);

    // 2. Turn 45° away from the just-reached marker
    Serial.printf("[FSM:DECISION] STATE:CLEARANCE_PIVOT | ACTION:PIVOT AWAY 45 deg (%dms) | L:+%d R:-%d\n",
                  PIVOT_AWAY_MS, SPEED_PIVOT, SPEED_PIVOT);
    Serial.flush();

    m_motors.pivotRight(SPEED_PIVOT);
    delay(PIVOT_AWAY_MS);
    m_motors.stop();
    delay(CHASSIS_SETTLE_MS);

    // 3. Increment cleared marker count & reset vision tracking
    m_clearedCount++;
    m_vision.unlockTarget();
    resetBestTarget();
    m_reachConfirmCount = 0;
    m_sweepStep         = 0;
    m_sweepAttempts     = 0;
    m_stateTimer        = millis();
    m_lastLogTimer      = 0;
    m_state             = RobotState::SWEEP_SCAN;

    Serial.printf("[FSM:CLEAR] Clearance complete (Cleared: %d) -> Resuming 360° sweep scan for next target!\n",
                  m_clearedCount);
}
