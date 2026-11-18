#include "CameraVision.h"

// ---------------------------------------------------------------------------
// CameraVision Constructor
// ---------------------------------------------------------------------------
CameraVision::CameraVision()
    : m_locked(false)
    , m_lockedTagId(-1)
    , m_frontZoneActive(false)
    , m_frontZoneCenterX(FRAME_CENTER_X)
    , m_frontZoneUntil(0)
{
    m_trackedRoi = {0, 0, 0, 0};
    for (int i = 0; i < 50; i++) {
        m_visitedIdUntil[i] = 0;
    }
}

// ---------------------------------------------------------------------------
// init() — Initialize camera driver and configure sensor
// ---------------------------------------------------------------------------
bool CameraVision::init() {
    if (!configureCamera()) {
        return false;
    }
    configureSensorTuning();
    unlockTarget();
    Serial.println("[OK] CameraVision ready (OV2640 + Adaptive ArUco DICT_4X4_50)");
    return true;
}

// ---------------------------------------------------------------------------
// configureCamera() — Pin mapping and frame buffer allocation
// ---------------------------------------------------------------------------
bool CameraVision::configureCamera() {
    camera_config_t cfg;

    // LEDC_CHANNEL_0 / LEDC_TIMER_0 are used exclusively by the camera driver for XCLK.
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;

    cfg.pin_d0       = CAM_PIN_Y2;
    cfg.pin_d1       = CAM_PIN_Y3;
    cfg.pin_d2       = CAM_PIN_Y4;
    cfg.pin_d3       = CAM_PIN_Y5;
    cfg.pin_d4       = CAM_PIN_Y6;
    cfg.pin_d5       = CAM_PIN_Y7;
    cfg.pin_d6       = CAM_PIN_Y8;
    cfg.pin_d7       = CAM_PIN_Y9;
    cfg.pin_xclk     = CAM_PIN_XCLK;
    cfg.pin_pclk     = CAM_PIN_PCLK;
    cfg.pin_vsync    = CAM_PIN_VSYNC;
    cfg.pin_href     = CAM_PIN_HREF;
    cfg.pin_sccb_sda = CAM_PIN_SIOD;
    cfg.pin_sccb_scl = CAM_PIN_SIOC;
    cfg.pin_pwdn     = CAM_PIN_PWDN;
    cfg.pin_reset    = CAM_PIN_RESET;
    cfg.xclk_freq_hz = 20000000;

    cfg.pixel_format = PIXFORMAT_GRAYSCALE;
    cfg.frame_size   = FRAMESIZE_QVGA;   // 320 x 240

    // Frame buffer: use PSRAM if available, fall back to DRAM
    if (psramFound()) {
        cfg.fb_count    = 2;
        cfg.fb_location = CAMERA_FB_IN_PSRAM;
        cfg.grab_mode   = CAMERA_GRAB_LATEST;
        Serial.println("[CAM] PSRAM detected -> double-buffered frame capture");
    } else {
        cfg.fb_count    = 1;
        cfg.fb_location = CAMERA_FB_IN_DRAM;
        cfg.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
        Serial.println("[CAM] No PSRAM -> single-buffered frame capture (check IDE settings)");
    }
    Serial.flush();

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
        Serial.flush();
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// configureSensorTuning() — Sensor contrast, exposure, sharpness
// ---------------------------------------------------------------------------
void CameraVision::configureSensorTuning() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;

    // Un-mirror OV2640 hardware readout so marker orientation matches
    s->set_hmirror(s, 1);
    s->set_vflip(s, 0);

    // High-contrast tuning for indoor lighting
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_brightness(s, 1);     // Slightly boost exposure
    s->set_contrast(s, 2);       // Strong black/white separation
    s->set_sharpness(s, 2);      // Crisp edges on printed cells
    s->set_whitebal(s, 0);
    s->set_gainceiling(s, (gainceiling_t)4); // 8x-16x amplification for indoor paper
}
