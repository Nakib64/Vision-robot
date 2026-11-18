#pragma once
#include <Arduino.h>

/*
 * =====================================================================================
 *  CENTRAL CONFIGURATION & HARDWARE CONSTANTS
 *  Platform: ESP32-S3-WROOM-1-N16R8 (16 MB Flash, 8 MB OPI PSRAM)
 *  Camera:   OV2640  |  Motor Driver: 4-pin PWM H-Bridge
 *  Vision:   ArUco (DICT_4X4_50) with Lock-and-Track ROI Optimization
 * =====================================================================================
 */

// -----------------------------
// 1. CAMERA PIN DEFINITIONS
//    (CAMERA_MODEL_ESP32S3_EYE)
// -----------------------------
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD     4
#define CAM_PIN_SIOC     5

#define CAM_PIN_Y9      16   // D7
#define CAM_PIN_Y8      17   // D6
#define CAM_PIN_Y7      18   // D5
#define CAM_PIN_Y6      12   // D4
#define CAM_PIN_Y5      10   // D3
#define CAM_PIN_Y4       8   // D2
#define CAM_PIN_Y3       9   // D1
#define CAM_PIN_Y2      11   // D0

#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK    13

// -----------------------------
// 2. MOTOR DRIVER PIN DEFINITIONS
//    L298N H-Bridge with ENA/ENB PWM speed + IN1-IN4 digital direction
//    100% safe GPIOs: avoids camera (4-18), USB (19-20), OPI PSRAM (26-37),
//    and hardware JTAG pins (39-42) which lock up on ESP32-S3!
//
//    ENA ─ PWM speed control   (Left  motor)  → GPIO 38
//    IN1 ─ Direction bit A     (Left  motor)  → GPIO  1
//    IN2 ─ Direction bit B     (Left  motor)  → GPIO  2
//    IN3 ─ Direction bit A     (Right motor)  → GPIO 47
//    IN4 ─ Direction bit B     (Right motor)  → GPIO 21
//    ENB ─ PWM speed control   (Right motor)  → GPIO 45
// -----------------------------
#define PIN_MOTOR_INA    38   // Left  Motor — PWM speed (LEDC)
#define PIN_MOTOR_IN1     1   // Left  Motor — direction A (digital)
#define PIN_MOTOR_IN2     2   // Left  Motor — direction B (digital)
#define PIN_MOTOR_IN3    47   // Right Motor — direction A (digital)
#define PIN_MOTOR_IN4    21   // Right Motor — direction B (digital)
#define PIN_MOTOR_INB    45   // Right Motor — PWM speed (LEDC)

// LEDC PWM channels for INA / INB speed pins
// Channels 0 and 1 are RESERVED by esp_camera_init() for XCLK — do NOT use them!
#define PWM_CH_INA        2   // LEDC channel for Left  motor speed
#define PWM_CH_INB        3   // LEDC channel for Right motor speed
#define PWM_FREQ_HZ    1000   // 1 kHz motor PWM frequency
#define PWM_RESOLUTION_BITS 8 // 8-bit resolution → duty 0–255

// -----------------------------
// 3. VISION & TARGET PARAMETERS
// -----------------------------
#define TARGET_ARUCO_ID        0       // Target marker ID (0 for aruco_a4_id0.png; -1 for any marker)
#define TARGET_REAL_WIDTH_CM   10.0f   // Printed tag physical width (cm)
#define STOP_DISTANCE_CM       25.0f   // Trigger TARGET_REACHED at <= 25 cm

// OV2640 focal length estimate: 320 * 0.85
#define CAMERA_FX_PX          272.0f
#define FRAME_CENTER_X         160     // 320 / 2
#define RADAR_ORIGIN_Y         220     // bottom-center origin on 320x240 frame

// Lock-and-Track ROI Expansion padding (pixels)
#define ROI_PADDING_PX         45      // Expands tracked ROI box for fast local search

// -----------------------------
// 4. MOTION & NAVIGATION TUNING
// -----------------------------
// Tuned for 4WD TT gear motors on L298N (~2.5 V driver drop)
#define SPEED_BASE        220    // Forward cruise PWM (0-255) — strong drive
#define SPEED_PIVOT       180    // Full 100% power for in-place 4WD skid-steer turns
#define SPEED_REVERSE     180    // Reverse PWM

// Flip to true if a side spins backward when driving forward (no rewiring needed!)
// Left uses IN1=HIGH, IN2=LOW (swap OUT1 <-> OUT2 motor wires if Left spins backward).
#define INVERT_LEFT_MOTOR   false
#define INVERT_RIGHT_MOTOR  false

// Set to true once on first test to run a 2-second FWD/REV/LEFT/RIGHT wheel check
#define RUN_BOOT_MOTOR_TEST false

// Aim-Then-Drive alignment & slow pulse-step motion (uses pure digital 3.3V — zero LEDC stall!)
//   - Target within +-ALIGN_TOLERANCE_DEG of 90° -> drive DIRECTLY STRAIGHT in slow controlled steps
//   - Target beyond tolerance                    -> gentle proportional turn pulse to center the target
#define ALIGN_TOLERANCE_DEG  12.0f // +-12 deg from straight (78 deg .. 102 deg) -> drive straight
#define AIM_TURN_RATE_MS     8.5f  // Milliseconds per degree offset (gentle pulse, prevents slip & overshoot)
#define AIM_TURN_MIN_MS      80    // Minimum pulse (ms) to overcome motor static friction
#define AIM_TURN_MAX_MS     220    // Strict maximum pulse (ms) — guarantees target NEVER leaves camera view!
#define DRIVE_STEP_MS       160    // Forward step duration (ms) — lower (e.g. 120) = slower!
#define DRIVE_PAUSE_MS      220    // Settle pause between forward steps (ms) — higher (e.g. 300) = slower!
#define STEERING_KP          0.7f  // (legacy)
#define DEADBAND_PX           18   // (legacy)

// -----------------------------
// 5. SWEEP SCAN TIMING (ms)
//    Simple right-only stepped search: LOOK → (no target?) → ROTATE RIGHT 45°
// -----------------------------
#define SWEEP_LOOK_MS      1500   // Pause & look forward (1.5 s) before rotating — increase (e.g. 2000-3000) for longer scan
#define SWEEP_LEFT_45_MS    450   // Rotate ~45 deg right per search step
#define SWEEP_RIGHT_90_MS   900   // (legacy constant)
#define SWEEP_CENTER_45_MS  450   // (legacy constant)

// -----------------------------
// 6. CLEARANCE, VISITED MEMORY & RECOVERY TIMING (ms)
// -----------------------------
#define REVERSE_DURATION_MS   450   // Back up slightly after reaching target
#define PIVOT_AWAY_MS         600   // (optional, unused in backup-and-look mode)
#define VISITED_IGNORE_MS   30000   // Remember & avoid a visited target for 30 sec
#define TARGET_LOST_MS       2500   // Wait 2.5 s before giving up mid-approach (prevents premature re-sweep)

// -----------------------------
// 7. VISUAL PREVIEW STREAM (PC Debug)
// -----------------------------
// Set to true while testing with preview_camera.py on PC
// Set to false for 100% standalone robot operation on battery (zero PC dependency)
#define ENABLE_CAMERA_STREAM  true