#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// TARGET_REACHED — Option B: Arrival Pause & Continuous Tour
//   1. Robot stops completely at the marker (<= 25 cm).
//   2. Pauses for TARGET_REACHED_PAUSE_MS (2.0s) so arrival is clearly confirmed.
//   3. Automatically transitions to CLEARANCE_PIVOT to back up, turn 45° away,
//      and resume 360° SWEEP_SCAN for the next marker!
// ---------------------------------------------------------------------------
void NavigationFSM::handleTargetReached(const TargetDetection &target, unsigned long now) {
    m_motors.stop();

    // Safety: If marker was abruptly pulled away (> 35 cm) during the first 500ms, resume approach
    if (target.found && target.estimatedDistCm > (STOP_DISTANCE_CM + 10.0f) && (now - m_stateTimer < 500)) {
        Serial.printf(
            "[FSM:REACHED] Target moved away to %.1f cm (> %.1f cm) -> Resuming approach!\n",
            target.estimatedDistCm, STOP_DISTANCE_CM);
        m_reachConfirmCount = 0;
        m_state             = RobotState::APPROACH_TARGET;
        m_stateTimer        = now;
        m_targetLostTimer   = now;
        m_lastLogTimer      = 0;
        handleApproach(target, now);
        return;
    }

    // Telemetry log during the 2-second arrival pause
    if (now - m_lastLogTimer >= 400) {
        float remainingSec = max(0.0f, (float)(TARGET_REACHED_PAUSE_MS - (now - m_stateTimer)) / 1000.0f);
        Serial.printf(
            "[FSM:DECISION] STATE:TARGET_REACHED | ACTION:ARRIVED AT MARKER! (Touring in %.1fs) | L:0 R:0\n",
            remainingSec);
        m_lastLogTimer = now;
    }

    // Option B: 2.0s pause complete -> Start clearance backup & pivot away to tour next marker!
    if (now - m_stateTimer >= TARGET_REACHED_PAUSE_MS) {
        Serial.println("[FSM:REACHED] 2-second pause complete -> Starting clearance backup & pivot!");
        m_reachConfirmCount = 0;
        m_state             = RobotState::CLEARANCE_PIVOT;
        m_stateTimer        = now;
        m_lastLogTimer      = 0;
        return;
    }
}
