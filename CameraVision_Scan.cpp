#include "CameraVision.h"

// ---------------------------------------------------------------------------
// emptyResult() — Default structure returned when no target is found
// ---------------------------------------------------------------------------
TargetDetection CameraVision::emptyResult() const {
    TargetDetection r;
    r.found           = false;
    r.tagId           = -1;
    r.centerX         = FRAME_CENTER_X;
    r.centerY         = 120;
    r.bboxX           = 0;
    r.bboxY           = 0;
    r.bboxW           = 0;
    r.bboxH           = 0;
    r.apparentWidthPx = 0.0f;
    r.estimatedDistCm = 9999.0f;
    r.error           = 0;
    r.angleDeg        = 0.0f;
    r.polarAngleDeg   = 90.0f;
    return r;
}

// ---------------------------------------------------------------------------
// scan() — Main frame acquisition, processing, and preview streaming entrypoint
// ---------------------------------------------------------------------------
TargetDetection CameraVision::scan() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb || !fb->buf || fb->format != PIXFORMAT_GRAYSCALE) {
        if (fb) esp_camera_fb_return(fb);
        return emptyResult();
    }

    // Always use verified full-frame ArUco decoding (~3 ms on ESP32-S3).
    // Blind ROI blob tracking can merge nearby dark shadows into the marker box.
    TargetDetection result = scanFullFrame(fb);
    if (m_locked && result.found) {
        m_trackedRoi = {result.bboxX, result.bboxY, result.bboxW, result.bboxH};
    }

#if defined(ENABLE_CAMERA_STREAM) && (ENABLE_CAMERA_STREAM == true)
    static unsigned long s_lastStreamTime = 0;
    unsigned long streamNow = millis();
    // Stream compact JPEG every 250 ms (quality 35 = ~2.5 KB, fast over 115200 baud)
    if (streamNow - s_lastStreamTime >= 250) {
        s_lastStreamTime = streamNow;
        uint8_t *jpg_buf = NULL;
        size_t jpg_len   = 0;
        if (fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_GRAYSCALE, 35, &jpg_buf, &jpg_len)) {
            uint8_t header[4] = {0xFF, 0xAA, 0x55, 0x00};
            uint32_t len      = (uint32_t)jpg_len;
            Serial.write(header, 4);
            Serial.write((uint8_t *)&len, 4);
            Serial.write(jpg_buf, jpg_len);
            free(jpg_buf);
        }
    }
#endif

    esp_camera_fb_return(fb);
    return result;
}
