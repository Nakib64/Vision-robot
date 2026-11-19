#include "CameraVision.h"
#include <math.h>

// ---------------------------------------------------------------------------
// MODE B: Locked Fast ROI Tracking (~1 ms)
// ---------------------------------------------------------------------------
TargetDetection CameraVision::scanTrackedRoi(camera_fb_t *fb) {
    const int W        = (int)fb->width;   // 320
    const int H        = (int)fb->height;  // 240
    const int rowLimit = H - CROP_BOTTOM;  // 215

    // Expand tracked ROI by ROI_PADDING_PX
    int roiX0 = max(0, m_trackedRoi.x - ROI_PADDING_PX);
    int roiY0 = max(0, m_trackedRoi.y - ROI_PADDING_PX);
    int roiX1 = min(W - 1, m_trackedRoi.x + m_trackedRoi.width - 1 + ROI_PADDING_PX);
    int roiY1 = min(rowLimit - 1, m_trackedRoi.y + m_trackedRoi.height - 1 + ROI_PADDING_PX);

    if (roiX1 <= roiX0 || roiY1 <= roiY0) {
        return emptyResult();
    }

    // Local dynamic threshold inside ROI
    uint8_t localMin = 255, localMax = 0;
    for (int y = roiY0; y <= roiY1; y += 4) {
        for (int x = roiX0; x <= roiX1; x += 4) {
            uint8_t v = fb->buf[y * W + x];
            if (v < localMin) localMin = v;
            if (v > localMax) localMax = v;
        }
    }
    uint8_t roiThresh = (uint8_t)(localMin + (localMax - localMin) * 0.45f);

    long sumX  = 0, sumY  = 0;
    int  count = 0;
    int  minX  = roiX1, maxX = roiX0;
    int  minY  = roiY1, maxY = roiY0;

    for (int y = roiY0; y <= roiY1; y++) {
        const uint8_t *row = fb->buf + y * W;
        for (int x = roiX0; x <= roiX1; x++) {
            if (row[x] <= roiThresh) {
                sumX += x;
                sumY += y;
                count++;
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }
    }

    if (count < MIN_TRACK_COUNT || maxX <= minX || maxY <= minY) {
        return emptyResult(); // Lost from ROI
    }

    int bw = maxX - minX + 1;
    int bh = maxY - minY + 1;
    float aspect = (float)bh / (float)bw;

    if (aspect < 0.40f || aspect > 2.20f || bw < MIN_MARKER_PX) {
        return emptyResult();
    }

    // Update ROI position for next frame
    m_trackedRoi.x      = minX;
    m_trackedRoi.y      = minY;
    m_trackedRoi.width  = bw;
    m_trackedRoi.height = bh;

    TargetDetection r;
    r.found           = true;
    r.tagId           = m_lockedTagId;
    r.centerX         = (int)(sumX / count);
    r.centerY         = (int)(sumY / count);
    r.bboxX           = minX;
    r.bboxY           = minY;
    r.bboxW           = bw;
    r.bboxH           = bh;
    r.apparentWidthPx = (float)bw;
    r.estimatedDistCm = (TARGET_REAL_WIDTH_CM * CAMERA_FX_PX) / max(r.apparentWidthPx, 1.0f);
    r.error           = r.centerX - FRAME_CENTER_X;
    r.angleDeg        = atan2f((float)r.error, CAMERA_FX_PX) * 180.0f / (float)M_PI;

    // True optical angle relative to 90° center Y-axis
    r.polarAngleDeg   = constrain(90.0f - r.angleDeg, 0.0f, 180.0f);

    return r;
}
