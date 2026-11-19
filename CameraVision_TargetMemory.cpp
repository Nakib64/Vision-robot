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
// markVisited() — (Disabled per user request: target memory off)
// ---------------------------------------------------------------------------
void CameraVision::markVisited(int tagId, int centerX, unsigned long durationMs) {
    (void)tagId;
    (void)centerX;
    (void)durationMs;
    // Memory disabled: never exclude any marker
}

// ---------------------------------------------------------------------------
// clearFrontVisitedZone() — (Disabled per user request)
// ---------------------------------------------------------------------------
void CameraVision::clearFrontVisitedZone() {
    m_frontZoneActive = false;
}

// ---------------------------------------------------------------------------
// isVisited() — Returns false always (visited memory disabled)
// ---------------------------------------------------------------------------
bool CameraVision::isVisited(int tagId) const {
    (void)tagId;
    return false; // Never ignore any detected marker!
}
