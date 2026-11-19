#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// SWEEP_SCAN — Human-Like Dual-Side Sweep Scan with Auto-Shutdown
//
//   Step 0: Pivot Left 45°
//   Step 1: Look Left for SWEEP_LOOK_MS (1.2s), sample leftTarget
//   Step 2: Swing Right 90° (now at +45° Right relative to center)
//   Step 3: Look Right for SWEEP_LOOK_MS (1.2s), sample rightTarget
//   Step 4: Evaluate & Decide:
//     - Both Found: Compare dist! Pivot Left 90° if Left is closer; else lock Right.
//     - Only Left Found: Pivot Left 90° to face it, lock and approach.
//     - Only Right Found: Already facing Right, lock and approach.
//     - Neither Found: Rotate 45° LEFT (user diagnostic test for INB / left pivot)
//                      After 6 attempts with no target: safe battery auto-shutdown!
// ---------------------------------------------------------------------------
void NavigationFSM::handleSweepScan(const TargetDetection &target, unsigned long now) {
    unsigned long elapsed = now - m_stateTimer;

    switch (m_sweepStep) {
        // -------------------------------------------------------------------
        // Step 0: Pivot Left 45°
        // -------------------------------------------------------------------
        case 0: {
            Serial.printf(
                "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:PIVOT LEFT 45 deg (Attempt %d/%d) | L:-%d R:+%d\n",
                m_sweepAttempts + 1, SWEEP_MAX_ATTEMPTS, SPEED_PIVOT, SPEED_PIVOT);
            Serial.flush();

            m_motors.pivotLeft(SPEED_PIVOT);
            delay(SWEEP_LEFT_45_MS); // 310 ms
            m_motors.stop();
            delay(150);

            m_leftTarget   = m_vision.emptyResult();
            m_sweepStep    = 1;
            m_stateTimer   = millis();
            m_lastLogTimer = 0;
            break;
        }

        // -------------------------------------------------------------------
        // Step 1: Look Left for SWEEP_LOOK_MS (1.2s), record closest left target
        // -------------------------------------------------------------------
        case 1: {
            m_motors.stop();

            if (target.found) {
                if (!m_leftTarget.found || target.estimatedDistCm < m_leftTarget.estimatedDistCm) {
                    m_leftTarget = target;
                    Serial.printf("[FSM:SWEEP] Left sample -> ID:%d | Dist:%.1f cm | Ang:%+.1f deg\n",
                                  target.tagId, target.estimatedDistCm, target.angleDeg);
                }
            }

            if (elapsed == 0 || (now - m_lastLogTimer >= 400)) {
                Serial.printf(
                    "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:LOOKING LEFT (%d/%d, Tag:%s) | L:0 R:0\n",
                    m_sweepAttempts + 1, SWEEP_MAX_ATTEMPTS,
                    m_leftTarget.found ? "FOUND" : "SCANNING");
                m_lastLogTimer = now;
            }

            if (elapsed >= SWEEP_LOOK_MS) {
                m_sweepStep  = 2;
                m_stateTimer = millis();
            }
            break;
        }

        // -------------------------------------------------------------------
        // Step 2: Swing Right 90° (now facing +45° Right relative to initial heading)
        // -------------------------------------------------------------------
        case 2: {
            Serial.printf(
                "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:SWING RIGHT 90 deg (Attempt %d/%d) | L:+%d R:-%d\n",
                m_sweepAttempts + 1, SWEEP_MAX_ATTEMPTS, SPEED_PIVOT, SPEED_PIVOT);
            Serial.flush();

            m_motors.pivotRight(SPEED_PIVOT);
            delay(SWEEP_RIGHT_90_MS); // 450 ms
            m_motors.stop();
            delay(150);

            m_rightTarget  = m_vision.emptyResult();
            m_sweepStep    = 3;
            m_stateTimer   = millis();
            m_lastLogTimer = 0;
            break;
        }

        // -------------------------------------------------------------------
        // Step 3: Look Right for SWEEP_LOOK_MS (1.2s), record closest right target
        // -------------------------------------------------------------------
        case 3: {
            m_motors.stop();

            if (target.found) {
                if (!m_rightTarget.found || target.estimatedDistCm < m_rightTarget.estimatedDistCm) {
                    m_rightTarget = target;
                    Serial.printf("[FSM:SWEEP] Right sample -> ID:%d | Dist:%.1f cm | Ang:%+.1f deg\n",
                                  target.tagId, target.estimatedDistCm, target.angleDeg);
                }
            }

            if (elapsed == 0 || (now - m_lastLogTimer >= 400)) {
                Serial.printf(
                    "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:LOOKING RIGHT (%d/%d, Tag:%s) | L:0 R:0\n",
                    m_sweepAttempts + 1, SWEEP_MAX_ATTEMPTS,
                    m_rightTarget.found ? "FOUND" : "SCANNING");
                m_lastLogTimer = now;
            }

            if (elapsed >= SWEEP_LOOK_MS) {
                m_sweepStep  = 4;
                m_stateTimer = millis();
            }
            break;
        }

        // -------------------------------------------------------------------
        // Step 4: Evaluate Both Sides & Commit or Advance Sector
        // -------------------------------------------------------------------
        case 4: {
            m_motors.stop();

            // Case A: Both sides found a target -> Compare physical distances!
            if (m_leftTarget.found && m_rightTarget.found) {
                if (m_leftTarget.estimatedDistCm < m_rightTarget.estimatedDistCm) {
                    Serial.printf(
                        "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:LEFT CLOSER (%.1fcm vs %.1fcm) -> SWING LEFT 90 deg | L:-%d R:+%d\n",
                        m_leftTarget.estimatedDistCm, m_rightTarget.estimatedDistCm, SPEED_PIVOT, SPEED_PIVOT);
                    Serial.flush();
                    m_motors.pivotLeft(SPEED_PIVOT);
                    delay(SWEEP_RIGHT_90_MS);
                    m_motors.stop();
                    delay(CHASSIS_SETTLE_MS);

                    m_bestTarget = m_leftTarget;
                } else {
                    Serial.printf(
                        "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:RIGHT CLOSER (%.1fcm vs %.1fcm) -> LOCK RIGHT!\n",
                        m_rightTarget.estimatedDistCm, m_leftTarget.estimatedDistCm);
                    m_bestTarget = m_rightTarget;
                }

                m_vision.lockTarget(m_bestTarget);
                m_sweepAttempts   = 0;
                m_sweepStep       = 0;
                m_state           = RobotState::APPROACH_TARGET;
                m_targetLostTimer = now;
                m_lastLogTimer    = 0;
                handleApproach(m_bestTarget, now);
                return;
            }

            // Case B: Only Left found -> Swing 90° back to Left and approach!
            if (m_leftTarget.found && !m_rightTarget.found) {
                Serial.printf(
                    "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:ONLY LEFT FOUND (%.1fcm) -> SWING LEFT 90 deg | L:-%d R:+%d\n",
                    m_leftTarget.estimatedDistCm, SPEED_PIVOT, SPEED_PIVOT);
                Serial.flush();
                m_motors.pivotLeft(SPEED_PIVOT);
                delay(SWEEP_RIGHT_90_MS);
                m_motors.stop();
                delay(CHASSIS_SETTLE_MS);

                m_bestTarget = m_leftTarget;
                m_vision.lockTarget(m_bestTarget);
                m_sweepAttempts   = 0;
                m_sweepStep       = 0;
                m_state           = RobotState::APPROACH_TARGET;
                m_targetLostTimer = now;
                m_lastLogTimer    = 0;
                handleApproach(m_bestTarget, now);
                return;
            }

            // Case C: Only Right found -> Already facing Right! Lock and approach!
            if (!m_leftTarget.found && m_rightTarget.found) {
                Serial.printf(
                    "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:ONLY RIGHT FOUND (%.1fcm) -> LOCK RIGHT!\n",
                    m_rightTarget.estimatedDistCm);

                m_bestTarget = m_rightTarget;
                m_vision.lockTarget(m_bestTarget);
                m_sweepAttempts   = 0;
                m_sweepStep       = 0;
                m_state           = RobotState::APPROACH_TARGET;
                m_targetLostTimer = now;
                m_lastLogTimer    = 0;
                handleApproach(m_bestTarget, now);
                return;
            }

            // Case D: Neither side found a target!
            m_sweepAttempts++;
            Serial.printf(
                "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:NO TARGET EITHER SIDE (Attempt %d/%d)\n",
                m_sweepAttempts, SWEEP_MAX_ATTEMPTS);

            if (m_sweepAttempts >= SWEEP_MAX_ATTEMPTS) {
                // 6 attempts exhausted without target -> Auto-shutdown to protect battery!
                m_motors.stop();
                m_state        = RobotState::ROBOT_HALTED;
                m_stateTimer   = now;
                m_lastLogTimer = 0;
                Serial.printf(
                    "[FSM:DECISION] STATE:ROBOT_HALTED | ACTION:POWER DOWN (No target after %d sweeps) | L:0 R:0\n",
                    SWEEP_MAX_ATTEMPTS);
                Serial.println("[FSM:HALT] >>> NO TARGET AVAILABLE (360° search complete) <<<");
                Serial.println("[FSM:HALT] Motors powered down. Press ESP32 RST button to restart.");
                return;
            }

            // Rotate 45° to the LEFT (user test to verify INB / left pivot performance)
            Serial.printf(
                "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:ROTATE LEFT 45 deg AGAIN (%d/%d) | L:-%d R:+%d\n",
                m_sweepAttempts, SWEEP_MAX_ATTEMPTS, SPEED_PIVOT, SPEED_PIVOT);
            Serial.flush();
            m_motors.pivotLeft(SPEED_PIVOT);
            delay(SWEEP_LEFT_45_MS); // 310 ms
            m_motors.stop();
            delay(150);

            // Restart dual-scan from Step 0 at new heading!
            m_sweepStep    = 0;
            m_stateTimer   = millis();
            m_lastLogTimer = 0;
            break;
        }

        default:
            m_sweepStep  = 0;
            m_stateTimer = millis();
            break;
    }
}
