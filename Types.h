#pragma once

/*
 * =====================================================================================
 *  DATA STRUCTURES & ENUM DEFINITIONS
 *  Platform: ESP32-S3-N16R8 | Standalone ArUco (DICT_4X4_50) Robot
 * =====================================================================================
 */

// ---------------------------------------------------------------------------
// Navigation Finite State Machine States
// ---------------------------------------------------------------------------
enum class RobotState {
    SWEEP_SCAN,       // 3-step panoramic sweep to find nearest ArUco marker
    APPROACH_TARGET,  // Proportional-steering drive toward locked target (Fast ROI)
    TARGET_REACHED,   // Within STOP_DISTANCE_CM: motors off, brief pause
    CLEARANCE_PIVOT   // Reverse + pivot so tag leaves FOV; loop back to sweep
};

// ---------------------------------------------------------------------------
// ArUco Target Detection Result (populated by CameraVision::scan())
// ---------------------------------------------------------------------------
struct TargetDetection {
    bool  found;              // True when marker is detected/tracked in this frame
    int   tagId;              // ArUco tag ID (DICT_4X4_50)
    int   centerX;            // Marker centre X in frame pixels (0–320)
    int   centerY;            // Marker centre Y in frame pixels (0–240)
    int   bboxX;              // Bounding box top-left X (0–320)
    int   bboxY;              // Bounding box top-left Y (0–240)
    int   bboxW;              // Bounding box width (px)
    int   bboxH;              // Bounding box height (px)
    float apparentWidthPx;    // Marker apparent width (px)
    float estimatedDistCm;    // (TARGET_REAL_WIDTH_CM * CAMERA_FX_PX) / apparentWidthPx
    int   error;              // centerX – FRAME_CENTER_X (signed, –160 … +160)
    float angleDeg;           // atan2(error, CAMERA_FX_PX) in degrees
    float polarAngleDeg;      // Radar polar angle: 0°–180°, origin bottom-centre
};

// ---------------------------------------------------------------------------
// Region of Interest (ROI) for Lock-and-Track mode
// ---------------------------------------------------------------------------
struct TrackedRoi {
    int x;
    int y;
    int width;
    int height;
};
