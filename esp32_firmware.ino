/*
 * =====================================================================================
 *  AUTONOMOUS NEAREST-TARGET ARUCO ROBOT  —  MAIN ENTRY POINT
 * =====================================================================================
 *  Platform : ESP32-S3-WROOM-1-N16R8  (16 MB Flash, 8 MB OPI PSRAM)
 *  Camera   : OV2640  —  PIXFORMAT_GRAYSCALE, FRAMESIZE_QVGA (320×240)
 *  Vision   : Embedded ArUco (DICT_4X4_50) with Lock-and-Track ROI Optimization
 *  Motor    : L298N H-Bridge (ENA=GPIO38, IN1=1, IN2=2, IN3=47, IN4=21, ENB=GPIO45)
 *
 *  100% Standalone Autonomous Operation — Zero PC Dependency.
 *  Clean Serial debug telemetry at 115200 baud.
 *
 *  Arduino IDE board settings:
 *    Board            : "ESP32S3 Dev Module"
 *    PSRAM            : "OPI PSRAM"
 *    Flash size       : 16MB
 *    Partition Scheme : "Huge APP (3MB No OTA/1MB SPIFFS)"
 *    USB Mode         : "Hardware CDC and JTAG"
 * =====================================================================================
 */

#include <Arduino.h>
#include "Config.h"
#include "Types.h"
#include "MotorDriver.h"
#include "CameraVision.h"
#include "NavigationFSM.h"

// ── Global subsystem instances ───────────────────────────────────────────────
MotorDriver   motors;
CameraVision  vision;
NavigationFSM fsm(motors, vision);

// ── setup() ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);   // Extra settle time — ensures USB CDC is ready before first print

    Serial.println("\n========================================================");
    Serial.println("  AUTONOMOUS ARUCO (DICT_4X4_50) NEAREST-TARGET ROBOT");
    Serial.println("  ESP32-S3-N16R8 | OV2640 | Lock-and-Track ROI | 115200");
    Serial.println("========================================================");
    Serial.printf("[BOOT] Free heap: %u bytes | PSRAM: %s (%u bytes free)\n",
                  ESP.getFreeHeap(),
                  psramFound() ? "FOUND" : "NOT FOUND",
                  psramFound() ? ESP.getFreePsram() : 0);
    Serial.flush();

    // ── INIT ORDER IS CRITICAL ───────────────────────────────────────────────
    // Camera MUST be initialised before motors.
    // On Arduino Core 3.x, ledcAttach() auto-assigns LEDC channels from 0.
    // esp_camera_init() explicitly claims LEDC_CHANNEL_0 + LEDC_TIMER_0 for XCLK.
    // If motors.init() runs first it grabs ch 0 → camera init crashes (EXCCAUSE 0x1c).
    // ─────────────────────────────────────────────────────────────────────────

    // 1. Camera + Embedded ArUco DICT_4X4_50 Vision Engine  ← MUST BE FIRST
    Serial.println("[INIT] Starting Camera + Vision (LEDC ch 0+1 for XCLK)...");
    Serial.flush();
    if (!vision.init()) {
        Serial.println("[FATAL] Camera initialization failed — halting.");
        Serial.flush();
        while (true) { delay(1000); }
    }
    Serial.println("[OK] Camera (OV2640) + ArUco (DICT_4X4_50) Vision ready");
    Serial.flush();

    // 2. Motor Driver HAL  (ENA=GPIO38, IN1=1, IN2=2, IN3=47, IN4=21, ENB=GPIO45)
    //    Pure digital 3.3V control (matches motor_test.ino — zero LEDC conflict!)
    Serial.println("[INIT] Starting Motor HAL (Pure Digital 3.3V)...");
    Serial.flush();
    motors.init();
    Serial.flush();

    // 3. Autonomous Navigation FSM
    Serial.println("[INIT] Starting Navigation FSM...");
    Serial.flush();
    fsm.init();
    Serial.println("[OK] Navigation FSM started — Beginning continuous 360° radar sweep\n");
    Serial.flush();

#if defined(RUN_BOOT_MOTOR_TEST) && (RUN_BOOT_MOTOR_TEST == true)
    // ── Optional Motor Self-Test (toggle RUN_BOOT_MOTOR_TEST in Config.h) ────
    Serial.println("[TEST] Motor self-test starting — watch the wheels!");
    Serial.flush();
    delay(500);

    Serial.println("[TEST] >> FORWARD");     Serial.flush();
    motors.forward(SPEED_BASE);    delay(500); motors.stop(); delay(200);

    Serial.println("[TEST] >> REVERSE");     Serial.flush();
    motors.reverse(SPEED_REVERSE); delay(500); motors.stop(); delay(200);

    Serial.println("[TEST] >> PIVOT LEFT");  Serial.flush();
    motors.pivotLeft(SPEED_PIVOT); delay(500); motors.stop(); delay(200);

    Serial.println("[TEST] >> PIVOT RIGHT"); Serial.flush();
    motors.pivotRight(SPEED_PIVOT); delay(500); motors.stop(); delay(200);

    Serial.println("[TEST] Motor self-test complete — starting autonomous mode\n");
    Serial.flush();
#endif
}

// ── loop() ──────────────────────────────────────────────────────────────────
void loop() {
    // 1. Grab camera frame and process vision:
    //    Mode A (Unlocked): Full ArUco DICT_4X4_50 pattern discovery (~3-4 ms)
    //    Mode B (Locked)  : Fast local ROI contour tracking (~1 ms)
    TargetDetection target = vision.scan();

    // 2. Feed detection into the navigation state machine and drive motors
    fsm.update(target);
}
