#include "CameraVision.h"

// ---------------------------------------------------------------------------
// lockTarget() — Engage fast ROI tracking on acquired marker
// ---------------------------------------------------------------------------
void CameraVision::lockTarget(const TargetDetection &target) {
    if (target.found) {
        m_locked      = true;
        m_lockedTagId = target.tagId;
        m_trackedRoi  = {target.bboxX, target.bboxY, target.bboxW, target.bboxH};
        Serial.printf("[VISION] >>> TARGET LOCKED <<< ArUco ID:%d at [%d, %d, %d, %d] | Dist:%.1f cm\n",
                      m_lockedTagId, m_trackedRoi.x, m_trackedRoi.y,
                      m_trackedRoi.width, m_trackedRoi.height, target.estimatedDistCm);
    }
}

// ---------------------------------------------------------------------------
// unlockTarget() — Return to full-frame discovery
// ---------------------------------------------------------------------------
void CameraVision::unlockTarget() {
    if (m_locked) {
        Serial.printf("[VISION] >>> TARGET UNLOCKED <<< Released ArUco ID:%d\n", m_lockedTagId);
    }
    m_locked      = false;
    m_lockedTagId = -1;
    m_trackedRoi  = {0, 0, 0, 0};
}

// ---------------------------------------------------------------------------
// markVisited() — Add target to temporary exclusion memory (cooldown)
// ---------------------------------------------------------------------------
void CameraVision::markVisited(int tagId, int centerX, unsigned long durationMs) {
    unsigned long until = millis() + durationMs;
    m_frontZoneActive  = true;
    m_frontZoneCenterX = centerX;
    m_frontZoneUntil   = until;

    if (tagId >= 0 && tagId < 50) {
        m_visitedIdUntil[tagId] = until;
    }
    Serial.printf("[VISION] Marked ID:%d (x=%d) as VISITED for %lu s\n",
                  tagId, centerX, durationMs / 1000UL);
}

// ---------------------------------------------------------------------------
// clearFrontVisitedZone() — Clear front exclusion mask once robot rotates away
// ---------------------------------------------------------------------------
void CameraVision::clearFrontVisitedZone() {
    if (m_frontZoneActive) {
        m_frontZoneActive = false;
        Serial.println("[VISION] Rotated away -> cleared front visited-zone mask");
    }
}

// ---------------------------------------------------------------------------
// isVisited() — Check if a marker ID or front zone is in active cooldown
// ---------------------------------------------------------------------------
bool CameraVision::isVisited(int tagId) const {
    unsigned long now = millis();
    // 1. Check ID cooldown table (active for 6 seconds)
    if (tagId >= 0 && tagId < 50 && m_visitedIdUntil[tagId] > now) {
        return true;
    }
    // 2. Check front zone mask (active for 6 seconds unless cleared)
    if (m_frontZoneActive && m_frontZoneUntil > now) {
        return true;
    }
    return false;
}
