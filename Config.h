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
//    IN2 ─ Direction bit B     (Left  motor)  → GPIO 14 (moved from 2: eliminates onboard LED clamp!)
//    IN3 ─ Direction bit A     (Right motor)  → GPIO 47
//    IN4 ─ Direction bit B     (Right motor)  → GPIO 21
//    ENB ─ PWM speed control   (Right motor)  → GPIO 45
// -----------------------------
#define PIN_MOTOR_INA    38   // Left  Motor — PWM speed (LEDC)
#define PIN_MOTOR_IN1     1   // Left  Motor — direction A (digital)
#define PIN_MOTOR_IN2    14   // Left  Motor — direction B (GPIO 14: 100% clean 3.3V, no onboard LED)
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

// Aim-Then-Drive alignment & Look-Move-Look Halving Approach (Zero Motion Blur!)
//   - Target within +-ALIGN_TOLERANCE_DEG of 90° -> calculate half distance and move forward
//   - Target beyond tolerance                    -> move backward slightly + calibrated rotation pulse
#define ALIGN_TOLERANCE_DEG  20.0f // If target is within 20° of 90° (70°..110°), move straight
#define AIM_STICTION_MS      150   // Static friction break base duration (ms)
#define AIM_TURN_RATE_MS     3.55f // Milliseconds per degree above stiction (45° = 150 + 45*3.55 = 310 ms)
#define AIM_TURN_MIN_MS      180   // Minimum pulse (ms) to overcome motor static friction
#define AIM_TURN_MAX_MS      420   // Maximum pulse (ms) for large angles (up to 80° offset)
#define AIM_REVERSE_MS       200   // Move backward slightly before rotating to break stiction (like sweep clearance)

// 65mm Wheel Speed Calibration & Motion Blur Settle Timing
// Wheel circumference = PI * 65mm = ~20.4 cm
// Under 3x 18650 cells (~11.1V through L298N), 4WD TT chassis moves at ~25 cm/s.
// DRIVE_MS_PER_CM = 1000 / 25.0 = 40 ms per cm. Adjust (e.g. 35-50) if needed.
#define WHEEL_DIAMETER_MM       65.0f // Standard 65mm rubber tire
#define DRIVE_MS_PER_CM         40    // Milliseconds per cm forward drive (40 ms/cm = ~25 cm/s)
#define CHASSIS_SETTLE_MS      350    // Settle pause (ms) after stopping so camera captures razor-sharp frame!
#define STEERING_KP          0.7f  // (legacy)
#define DEADBAND_PX           18   // (legacy)

// -----------------------------
// 5. SWEEP SCAN TIMING & SEARCH LIMITS (ms)
//    Human-Like Dual-Side Scan: Look Left 45° -> Swing Right 90° -> Pick Nearest!
//    - Calibrated 4WD pulses: 310 ms = 45°, 450 ms = 90°
//    - If neither side has target, rotates 45° Right again (max 6 attempts)
//    - After 6 attempts without finding a target: auto-shuts down to protect battery
// -----------------------------
#define SWEEP_LOOK_MS        1200   // Pause & look stationary per side (1.2 s)
#define SWEEP_LEFT_45_MS      310   // Pivot 45° pulse duration (310 ms = 45°)
#define SWEEP_RIGHT_90_MS     450   // Swing 90° pulse duration (450 ms = 90°)
#define SWEEP_MAX_ATTEMPTS      6   // Max full sweep attempts before automatic power-down
#define SWEEP_CENTER_45_MS    310   // (legacy constant)

// -----------------------------
// 6. CLEARANCE, VISITED MEMORY & RECOVERY TIMING (ms)
// -----------------------------
#define TARGET_REACHED_PAUSE_MS 2000   // Option B: Pause 2.0s at target before touring next
#define REVERSE_DURATION_MS      600   // Back up after reaching target (600 ms)
#define PIVOT_AWAY_MS            310   // Rotate 45° away from reached target (310 ms = 45°)
#define VISITED_IGNORE_MS       6000   // Remember & avoid a visited target for 6 sec
#define TARGET_LOST_MS          2500   // Wait 2.5 s before giving up mid-approach (prevents premature re-sweep)

// -----------------------------
// 7. VISUAL PREVIEW STREAM (PC Debug)
// -----------------------------
// Set to true while testing with preview_camera.py on PC
// Set to false for 100% standalone robot operation on battery (zero PC dependency)
#define ENABLE_CAMERA_STREAM  true