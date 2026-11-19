#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"
#include "MotorDriver.h"
#include "CameraVision.h"

/*
 * =====================================================================================
 *  AUTONOMOUS NAVIGATION FSM  –  Panoramic Sweep + Nearest-Target Approach
 *
 *  State flow:
 *    SWEEP_SCAN  ──► APPROACH_TARGET  ──► TARGET_REACHED  ──► CLEARANCE_PIVOT
 *         ▲                                                          │
 *         └──────────────────────────────────────────────────────────┘
 *
 *  SWEEP_SCAN performs a 3-step panoramic arc (−45° → 0° → +45°) and records
 *  the nearest tag spotted across the entire arc before committing to approach.
 *
 *  No odometry, no target memory, no PC dependency.
 * =====================================================================================
 */

class NavigationFSM {
public:
    NavigationFSM(MotorDriver &motors, CameraVision &vision);

    // Reset state machine to initial sweep
    void init();

    // Call once per loop iteration with the latest CameraVision result
    void update(const TargetDetection &target);

    // Accessors
    RobotState getState()        const { return m_state; }
    int        getClearedCount() const { return m_clearedCount; }

private:
    MotorDriver  &m_motors;
    CameraVision &m_vision;
    RobotState    m_state;

    // General-purpose state entry timestamp
    unsigned long m_stateTimer;
    // Timestamps for target-lost and periodic logging
    unsigned long m_targetLostTimer;
    unsigned long m_lastLogTimer;

    // Sweep bookkeeping
    int             m_sweepStep;      // 0=turnLeft, 1=lookLeft, 2=swingRight, 3=lookRight, 4=evaluate
    TargetDetection m_bestTarget;     // Selected winner target
    TargetDetection m_leftTarget;     // Candidate spotted on Left side
    TargetDetection m_rightTarget;    // Candidate spotted on Right side
    int             m_sweepAttempts;  // Counts 0..SWEEP_MAX_ATTEMPTS before auto-shutdown

    // Session statistics
    int           m_clearedCount;
    int           m_reachConfirmCount;

    // ── State handlers ──────────────────────────────────────────────────────
    void handleSweepScan    (const TargetDetection &target, unsigned long now);
    void handleApproach     (const TargetDetection &target, unsigned long now);
    void handleTargetReached(const TargetDetection &target, unsigned long now);
    void handleClearancePivot(unsigned long now);
    void handleHalted       (unsigned long now);

    // Helpers
    void resetBestTarget();
    static const char *stateName(RobotState s);
};
