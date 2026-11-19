#include "NavigationFSM.h"

// ---------------------------------------------------------------------------
// NavigationFSM Constructor
// ---------------------------------------------------------------------------
NavigationFSM::NavigationFSM(MotorDriver &motors, CameraVision &vision)
    : m_motors(motors)
    , m_vision(vision)
    , m_state(RobotState::SWEEP_SCAN)
    , m_stateTimer(0)
    , m_targetLostTimer(0)
    , m_lastLogTimer(0)
    , m_sweepStep(0)
    , m_sweepAttempts(0)
    , m_clearedCount(0)
    , m_reachConfirmCount(0)
{
    m_leftTarget  = m_vision.emptyResult();
    m_rightTarget = m_vision.emptyResult();
}

// ---------------------------------------------------------------------------
// resetBestTarget() — Clear tracking memory for current sector
// ---------------------------------------------------------------------------
void NavigationFSM::resetBestTarget() {
    m_bestTarget.found           = false;
    m_bestTarget.tagId           = -1;
    m_bestTarget.centerX         = FRAME_CENTER_X;
    m_bestTarget.centerY         = 120;
    m_bestTarget.bboxX           = 0;
    m_bestTarget.bboxY           = 0;
    m_bestTarget.bboxW           = 0;
    m_bestTarget.bboxH           = 0;
    m_bestTarget.apparentWidthPx = 0.0f;
    m_bestTarget.estimatedDistCm = 9999.0f;
    m_bestTarget.error           = 0;
    m_bestTarget.angleDeg        = 0.0f;
    m_bestTarget.polarAngleDeg   = 90.0f;
}

// ---------------------------------------------------------------------------
// init() — Start state machine at continuous 360° radar sweep
// ---------------------------------------------------------------------------
void NavigationFSM::init() {
    m_state           = RobotState::SWEEP_SCAN;
    m_stateTimer      = millis();
    m_targetLostTimer = millis();
    m_lastLogTimer    = 0;
    m_sweepStep       = 0;
    m_sweepAttempts   = 0;
    m_clearedCount    = 0;
    m_reachConfirmCount = 0;
    m_leftTarget      = m_vision.emptyResult();
    m_rightTarget     = m_vision.emptyResult();
    resetBestTarget();
    m_vision.unlockTarget();
    m_motors.stop();
    Serial.println("[FSM] Initialised -> SWEEP_SCAN (Dual-Side Panoramic Search Active)");
}
