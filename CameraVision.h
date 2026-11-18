#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "Config.h"
#include "Types.h"

/*
 * =====================================================================================
 *  CAMERA VISION SUBSYSTEM (OV2640 & EMBEDDED ARUCO DICT_4X4_50 ENGINE)
 *  Platform: ESP32-S3-WROOM-1-N16R8 | QVGA Grayscale (320x240)
 *
 *  Two-Mode Operation (Lock-and-Track ROI Optimization):
 *    Mode A — Unlocked Discovery (!m_locked, active during SWEEP_SCAN):
 *      Full-frame search for ArUco DICT_4X4_50 markers.
 *      Detects dark square border, samples 6x6 grid, decodes 4x4 inner bits,
 *      verifies quiet zone, and returns the NEAREST marker in view.
 *
 *    Mode B — Locked ROI Tracking (m_locked, active during APPROACH_TARGET):
 *      Skips full pattern decoding! Concentrates on a local ROI expanded by
 *      ROI_PADDING_PX around the last known bounding box. Updates centroid,
 *      apparent width, and distance in ~1 ms while keeping the locked tagId.
 * =====================================================================================
 */

class CameraVision {
public:
    CameraVision();

    // Initialize OV2640 camera in native grayscale QVGA (320x240)
    bool init();

    // Frame capture & processing entry point (always returns frame buffer internally)
    TargetDetection scan();

    // Lock-and-Track API
    void lockTarget(const TargetDetection &target);
    void unlockTarget();
    bool isLocked() const { return m_locked; }

    // Visited-Target Memory API (avoids re-locking a recently reached target)
    void markVisited(int tagId, int centerX, unsigned long durationMs);
    void clearFrontVisitedZone();

private:
    bool configureCamera();
    void configureSensorTuning();
    TargetDetection emptyResult() const;

    // Mode A: Full-frame ArUco candidate discovery
    TargetDetection scanFullFrame(camera_fb_t *fb);

    // Mode B: Fast local ROI tracking
    TargetDetection scanTrackedRoi(camera_fb_t *fb);

    // Helper: decode 4x4 ArUco pattern from sampled 4x4 binary grid against DICT_4X4_50
    int decodeAruco4x4(const uint8_t inner[4][4], int &outHamming);

    // Shared detection constants
    static constexpr int MIN_MARKER_PX   = 22;   // Min size (rejects distant specks/noise < 22px)
    static constexpr int MAX_MARKER_PX   = 300;  // Max bounding box width/height
    static constexpr int CROP_BOTTOM     = 10;   // Ignore bottom 10 rows (chassis edge)
    static constexpr int MIN_TRACK_COUNT = 60;   // Min dark pixels in ROI mode

    bool          m_locked;
    TrackedRoi    m_trackedRoi;
    int           m_lockedTagId;

    // Visited memory:
    //  1. Front zone mask (works even when multiple printed targets share ID:0)
    //  2. Per-ID timer table (works when targets have different IDs, TARGET_ARUCO_ID = -1)
    bool          m_frontZoneActive;
    int           m_frontZoneCenterX;
    unsigned long m_frontZoneUntil;
    unsigned long m_visitedIdUntil[50];
};
