#include "NavigationFSM.h"
#include <math.h>

// ---------------------------------------------------------------------------
// APPROACH_TARGET — Look-Move-Look Halving Approach (Zero Motion Blur!)
//
//   1. Stationary Sharp Capture: Robot only processes vision while stopped.
//   2. Alignment Check: If target is > +-12° off-center, execute gentle aim pulse.
//   3. Half-Distance Step: Calculate remaining distance to STOP_DISTANCE_CM (25cm).
//      Drive HALF that distance using 65mm wheel speed calibration, then STOP!
//   4. Chassis Settle: 350 ms pause lets vibration die down for the next sharp frame.
// ---------------------------------------------------------------------------
void NavigationFSM::handleApproach(const TargetDetection &target, unsigned long now) {
    if (target.found) {
        m_bestTarget      = target;
        m_targetLostTimer = now;

        // 1. Arrival Check: within STOP_DISTANCE_CM (25 cm)
        // Must be confirmed on 2 consecutive frames to prevent single-frame noise glitches
        if (target.estimatedDistCm <= STOP_DISTANCE_CM && target.estimatedDistCm > 1.0f) {
            m_motors.stop();
            m_reachConfirmCount++;
            if (m_reachConfirmCount >= 2) {
                m_vision.unlockTarget();
                m_vision.markVisited(target.tagId, target.centerX, VISITED_IGNORE_MS);
                Serial.printf(
                    "[FSM:DECISION] STATE:TARGET_REACHED | ACTION:TARGET REACHED! (%.1f cm <= %.1f cm) | L:0 R:0\n",
                    target.estimatedDistCm, STOP_DISTANCE_CM);
                m_state        = RobotState::TARGET_REACHED;
                m_stateTimer   = now;
                m_lastLogTimer = 0;
                return;
            }
            // Settle and verify on the very next frame
            return;
        } else {
            m_reachConfirmCount = 0;
        }

        // 2. Alignment Check: Aim-Then-Drive
        float offsetDeg = target.polarAngleDeg - 90.0f;
        float absOffset = fabsf(offsetDeg);

        if (absOffset > ALIGN_TOLERANCE_DEG) {
            // Calibrated turn duration matching 45° = 310 ms physics equation:
            // turnMs = AIM_STICTION_MS + (absOffset * AIM_TURN_RATE_MS)
            // e.g. for 33°: 150 + 33*3.55 = 267 ms
            // e.g. for 45°: 150 + 45*3.55 = 310 ms
            int turnMs = constrain(
                (int)(AIM_STICTION_MS + (absOffset * AIM_TURN_RATE_MS)),
                AIM_TURN_MIN_MS,
                AIM_TURN_MAX_MS
            );

            // Exactly like sweep clearance: Move backward slightly, then rotate (not forward!)
            Serial.printf(
                "[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:AIM BACKWARD CLEARANCE (%dms) | L:-%d R:-%d\n",
                AIM_REVERSE_MS, SPEED_REVERSE, SPEED_REVERSE);
            Serial.flush();
            m_motors.reverse(SPEED_REVERSE);
            delay(AIM_REVERSE_MS);
            m_motors.stop();
            delay(100);

            if (offsetDeg > 0.0f) {
                // Target is LEFT -> rotate LEFT
                Serial.printf(
                    "[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:AIM LEFT %.1f deg (at %.1f deg, %dms) | L:-%d R:+%d\n",
                    absOffset, target.polarAngleDeg, turnMs, SPEED_PIVOT, SPEED_PIVOT);
                Serial.flush();
                m_motors.pivotLeft(SPEED_PIVOT);
                delay(turnMs);
                m_motors.stop();
                delay(CHASSIS_SETTLE_MS);
            } else {
                // Target is RIGHT -> rotate RIGHT
                Serial.printf(
                    "[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:AIM RIGHT %.1f deg (at %.1f deg, %dms) | L:+%d R:+%d\n",
                    absOffset, target.polarAngleDeg, turnMs, SPEED_PIVOT, SPEED_PIVOT);
                Serial.flush();
                m_motors.pivotRight(SPEED_PIVOT);
                delay(turnMs);
                m_motors.stop();
                delay(CHASSIS_SETTLE_MS);
            }
            m_targetLostTimer = millis();
            m_lastLogTimer    = millis();
            return; // Settle and let next frame verify alignment!
        }

        // 3. Aligned with center! Look-Move-Look Halving Approach
        // Distance remaining until 25 cm stop threshold:
        float distToTarget = target.estimatedDistCm;
        float distToCover  = distToTarget - STOP_DISTANCE_CM;

        // In first step, calculate half of the distance to the target.
        // If remaining distance is already small (<= 20 cm), take the final step to 25 cm.
        float moveCm;
        if (distToCover > 20.0f) {
            moveCm = distToCover / 2.0f;  // Move HALF the remaining distance!
        } else {
            moveCm = distToCover;         // Final step directly to 25 cm
        }
        moveCm = max(moveCm, 4.0f);       // Minimum 4 cm so TT motors overcome inertia

        // Calculate drive duration from wheel size calibration:
        // 65mm wheel, ~25 cm/s speed -> DRIVE_MS_PER_CM = 40 ms/cm
        int driveMs = (int)(moveCm * (float)DRIVE_MS_PER_CM);
        driveMs = constrain(driveMs, 100, 1800); // Strict safety cap per step

        Serial.printf(
            "[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:STEP HALF (Dist:%.1f cm, Move:%.1f cm, %dms) | L:+%d R:+%d\n",
            distToTarget, moveCm, driveMs, SPEED_BASE, SPEED_BASE);
        Serial.flush();

        // Step forward for the exact calculated duration
        m_motors.forward(SPEED_BASE);
        delay(driveMs);

        // STOP COMPLETELY after reaching the half distance!
        m_motors.stop();

        // Settle pause: eliminates all motion blur so camera grabs a razor-sharp frame!
        delay(CHASSIS_SETTLE_MS);

        m_targetLostTimer = millis();
        m_lastLogTimer    = millis();
    } else {
        // While stopped, if frame was blurred or briefly occluded, wait calmly before giving up
        m_motors.stop();
        if (now - m_lastLogTimer >= 400) {
            Serial.println("[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:STOPPED - ACQUIRING SHARP FRAME... | L:0 R:0");
            m_lastLogTimer = now;
        }
        if (now - m_targetLostTimer >= TARGET_LOST_MS) {
            Serial.println("[FSM:LOST] Target lost > 2.5s -> returning to sweep search");
            m_motors.stop();
            m_vision.unlockTarget();
            resetBestTarget();
            m_sweepStep    = 0;
            m_stateTimer   = millis();
            m_lastLogTimer = 0;
            m_state        = RobotState::SWEEP_SCAN;
        }
    }
}
