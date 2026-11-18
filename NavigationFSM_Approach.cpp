#include "NavigationFSM.h"
#include <math.h>

// ---------------------------------------------------------------------------
// APPROACH_TARGET — Aim-Then-Drive Alignment & Controlled Step Navigation
//   - If target is within +-ALIGN_TOLERANCE_DEG of 90° center Y-axis:
//       Drive DIRECTLY STRAIGHT in slow, controlled pulse-steps!
//   - If target is beyond tolerance:
//       Gentle proportional turn pulse to center the target, then re-check!
// ---------------------------------------------------------------------------
void NavigationFSM::handleApproach(const TargetDetection &target, unsigned long now) {
    if (target.found) {
        m_bestTarget      = target;
        m_targetLostTimer = now;

        // STOP condition: Marker within STOP_DISTANCE_CM (25 cm)
        if (target.estimatedDistCm <= STOP_DISTANCE_CM) {
            m_motors.stop();
            m_vision.unlockTarget();
            m_vision.markVisited(target.tagId, target.centerX, VISITED_IGNORE_MS);
            m_clearedCount++;
            Serial.printf(
                "[FSM:DECISION] STATE:TARGET_REACHED | ACTION:REACHED & MARKED VISITED (%.1f cm, #%d) | L:0 R:0\n",
                target.estimatedDistCm, m_clearedCount);
            m_state        = RobotState::TARGET_REACHED;
            m_stateTimer   = now;
            m_lastLogTimer = 0;
            return;
        }

        // Aim-Then-Drive Approach Using Radar Polar Angle (Center Y-Axis = 90°)
        //   - polarAngleDeg = 90°  -> straight ahead along center Y-axis
        //   - polarAngleDeg > 90°  -> target is LEFT of center by (polarAngleDeg - 90°)
        //   - polarAngleDeg < 90°  -> target is RIGHT of center by (90° - polarAngleDeg)
        float offsetDeg = target.polarAngleDeg - 90.0f;
        float absOffset = fabsf(offsetDeg);

        if (absOffset > ALIGN_TOLERANCE_DEG) {
            // Gentle proportional rotation pulse — strictly capped so wheels NEVER slip or overshoot!
            int turnMs = constrain(
                (int)(absOffset * AIM_TURN_RATE_MS),
                AIM_TURN_MIN_MS,
                AIM_TURN_MAX_MS
            );

            if (offsetDeg > 0.0f) {
                // Target is LEFT of center -> rotate LEFT that exact angle
                Serial.printf(
                    "[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:AIM LEFT %.1f deg (at %.1f deg, %dms) | L:-%d R:+%d\n",
                    absOffset, target.polarAngleDeg, turnMs, SPEED_PIVOT, SPEED_PIVOT);
                Serial.flush();
                m_motors.pivotLeft(SPEED_PIVOT);
                delay(turnMs);
                m_motors.stop();
                delay(150);
            } else {
                // Target is RIGHT of center -> rotate RIGHT that exact angle
                Serial.printf(
                    "[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:AIM RIGHT %.1f deg (at %.1f deg, %dms) | L:+%d R:+%d\n",
                    absOffset, target.polarAngleDeg, turnMs, SPEED_PIVOT, SPEED_PIVOT);
                Serial.flush();
                m_motors.pivotRight(SPEED_PIVOT);
                delay(turnMs);
                m_motors.stop();
                delay(150);
            }
            m_targetLostTimer = millis();
            m_lastLogTimer    = millis();
        } else {
            // Within +-ALIGN_TOLERANCE_DEG of 90° center Y-axis -> move DIRECTLY STRAIGHT slowly!
            Serial.printf(
                "[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:DRIVE STRAIGHT SLOW (ID:%d, %.1f cm, %.1f deg) | L:+%d R:+%d\n",
                target.tagId, target.estimatedDistCm, target.polarAngleDeg, SPEED_BASE, SPEED_BASE);
            Serial.flush();

            m_motors.forward(SPEED_BASE);
            delay(DRIVE_STEP_MS);   // Controlled short forward step (160 ms)
            m_motors.stop();
            delay(DRIVE_PAUSE_MS);  // Slow settle pause (220 ms) -> sharp next frame!

            m_targetLostTimer = millis();
            m_lastLogTimer    = millis();
        }
    } else {
        // Brief frame blur / occlusion — wait calmly before giving up
        m_motors.stop();
        if (now - m_lastLogTimer >= 400) {
            Serial.println("[FSM:DECISION] STATE:APPROACH_TARGET | ACTION:RE-ACQUIRING TARGET... | L:0 R:0");
            m_lastLogTimer = now;
        }
        if (now - m_targetLostTimer >= TARGET_LOST_MS) {
            Serial.println("[FSM:LOST] Target lost > 2.5s -> returning to LOOK FORWARD / RIGHT 45 deg search");
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
