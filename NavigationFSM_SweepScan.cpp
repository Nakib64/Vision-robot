#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// SWEEP_SCAN — Simple & Robust Stepped 360° Search (Stationary Look -> Rotate Right 45°)
//
//   Step 0: Look Stationary for SWEEP_LOOK_MS (1.2s).
//           STRICT RULE: If ANY target is in the frame, LOCK IT IMMEDIATELY!
//           -> Transitions instantly to APPROACH_TARGET.
//           -> APPROACH_TARGET strictly rotates whatever angle the target is at
//              to bring it dead center and drives straight to it!
//
//   Step 1: If no target is seen in this sector:
//           -> Rotate Right 45° into the next sector.
//           -> Settle pause eliminates motion blur.
//           -> After SWEEP_MAX_ATTEMPTS (8 steps * 45° = 360° complete circle):
//              Safe auto-shutdown to protect battery!
// ---------------------------------------------------------------------------
void NavigationFSM::handleSweepScan(const TargetDetection &target, unsigned long now) {
    unsigned long elapsed = now - m_stateTimer;

    switch (m_sweepStep) {
        // -------------------------------------------------------------------
        // Step 0: Stationary Look — Grab sharp frames & detect target
        // -------------------------------------------------------------------
        case 0: {
            m_motors.stop();

            // STRICT TARGET DETECTION: If target is in the frame, LOCK IT IMMEDIATELY!
            if (target.found) {
                m_bestTarget      = target;
                m_vision.lockTarget(m_bestTarget);
                m_sweepAttempts   = 0;
                m_sweepStep       = 0;
                m_state           = RobotState::APPROACH_TARGET;
                m_targetLostTimer = now;
                m_lastLogTimer    = 0;

                Serial.println("\n========================================================");
                Serial.printf("[FSM:LOCK] >>> TARGET SPOTTED! ID:%d | Dist:%.1f cm | Ang:%+.1f deg <<<\n",
                              target.tagId, target.estimatedDistCm, target.angleDeg);
                Serial.printf("[FSM:LOCK] Committing immediately -> strictly aligning to %+.1f deg!\n",
                              target.angleDeg);
                Serial.println("========================================================\n");
                Serial.flush();

                handleApproach(m_bestTarget, now);
                return;
            }

            // Periodic telemetry while looking
            if (elapsed == 0 || (now - m_lastLogTimer >= 400)) {
                Serial.printf(
                    "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:LOOKING FORWARD (Sector %d/%d) | L:0 R:0\n",
                    m_sweepAttempts + 1, SWEEP_MAX_ATTEMPTS);
                m_lastLogTimer = now;
            }

            // If look duration finished and no target found, advance to Step 1
            if (elapsed >= SWEEP_LOOK_MS) {
                m_sweepStep  = 1;
                m_stateTimer = millis();
            }
            break;
        }

        // -------------------------------------------------------------------
        // Step 1: Rotate Right 45° into next sector
        // -------------------------------------------------------------------
        case 1: {
            m_sweepAttempts++;

            if (m_sweepAttempts >= SWEEP_MAX_ATTEMPTS) {
                m_sweepAttempts = 0; // 360° circle complete -> keep scanning continuously!
                Serial.println("[FSM:SWEEP] Full 360° scanned -> continuing endless search loop...");
            }

            Serial.printf(
                "[FSM:DECISION] STATE:SWEEP_SCAN | ACTION:ROTATE RIGHT 45 deg (Sector %d/%d) | L:+%d R:-%d\n",
                m_sweepAttempts, SWEEP_MAX_ATTEMPTS, SPEED_PIVOT, SPEED_PIVOT);
            Serial.flush();

            m_motors.pivotRight(SPEED_PIVOT);
            delay(SWEEP_RIGHT_45_MS); // 310 ms
            m_motors.stop();
            delay(CHASSIS_SETTLE_MS); // Settle so next camera frame is razor sharp!

            // Return to Step 0 to look stationary at new heading
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
