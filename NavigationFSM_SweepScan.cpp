#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// SWEEP_SCAN — Multi-checkpoint panoramic sweep with motion-free captures
//   - Look forward for SWEEP_LOOK_MS (chassis completely stationary)
//   - If target visible at ANY angle: immediately lock & approach!
//   - If no target: pivot RIGHT ~45° (SWEEP_LEFT_45_MS), pause, repeat continuously!
// ---------------------------------------------------------------------------
void NavigationFSM::handleSweepScan(const TargetDetection &target, unsigned long now) {
    unsigned long elapsed = now - m_stateTimer;

    // Immediate lock on ANY angle (scanFullFrame selects the NEAREST target)
    if (target.found) {
        m_bestTarget = target;
        Serial.printf(
            "[FSM:SWEEP] Nearest target locked at any angle -> ArUco ID:%d | Dist:%.1f cm | Ang:%+.1f deg\n",
            target.tagId, target.estimatedDistCm, target.angleDeg);

        m_vision.lockTarget(m_bestTarget);
        m_state           = RobotState::APPROACH_TARGET;
        m_targetLostTimer = now;
        m_lastLogTimer    = 0;
        handleApproach(target, now);
        return;
    }

    int sectorNum = (m_sweepStep % 8) + 1; // 1..8 across full 360°

    // Step A: LOOK FORWARD (motors stopped so camera captures razor-sharp frames)
    m_motors.stop();
    if (elapsed == 0 || (now - m_lastLogTimer >= 400)) {
        Serial.printf(
            "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:LOOKING FORWARD (%d/8) | L:0 R:0\n",
            sectorNum);
        m_lastLogTimer = now;
    }

    // Step B: If no unvisited target spotted during look window, rotate RIGHT 45°
    if (elapsed >= SWEEP_LOOK_MS) {
        m_vision.clearFrontVisitedZone();
        Serial.printf(
            "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:NO TARGET -> ROTATE RIGHT 45 deg (%d/8) | L:+%d R:-%d\n",
            sectorNum, SPEED_PIVOT, SPEED_PIVOT);
        Serial.flush();

        m_motors.pivotRight(SPEED_PIVOT);
        delay(SWEEP_LEFT_45_MS); // Guaranteed full 45° rotation
        m_motors.stop();
        delay(150);              // Let chassis stop vibrating before look window starts

        // Advance to next sector (wraps 8 -> 0 continuously for 360° search)
        m_sweepStep    = (m_sweepStep + 1) % 8;
        m_stateTimer   = millis();
        m_lastLogTimer = 0;
    }
}
