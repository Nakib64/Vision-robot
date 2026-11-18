# Autonomous ArUco Robot — Complete Beginner's Guide

> **What is this?**
> A small wheeled robot that can see printed **ArUco markers (`DICT_4X4_50`)** with its camera, pick the closest one, lock onto it with high-speed **Lock-and-Track ROI vision**, drive up to it, and then turn around to hunt for the next one — all by itself, with **zero laptop, zero phone, zero Wi-Fi** needed once it is switched on.

---

## 📋 Table of Contents

1. [How the Robot Thinks — Big Picture](#1-how-the-robot-thinks--big-picture)
2. [What You Need — Parts List](#2-what-you-need--parts-list)
3. [Step 1 — Print the Target Markers](#3-step-1--print-the-target-markers)
4. [Step 2 — Wire Everything Together](#4-step-2--wire-everything-together)
5. [Step 3 — Install Software on Your PC (one time only)](#5-step-3--install-software-on-your-pc-one-time-only)
6. [Step 4 — Open the Code in Arduino IDE](#6-step-4--open-the-code-in-arduino-ide)
7. [Step 5 — Configure Arduino IDE for This Board](#7-step-5--configure-arduino-ide-for-this-board)
8. [Step 6 — Upload the Code to the Robot](#8-step-6--upload-the-code-to-the-robot)
9. [Step 7 — Power On and Watch It Go](#9-step-7--power-on-and-watch-it-go)
10. [Understanding the Serial Debug Log](#10-understanding-the-serial-debug-log)
11. [How the Robot Makes Decisions — FSM Explained](#11-how-the-robot-makes-decisions--fsm-explained)
12. [Tuning & Adjustments (Config.h)](#12-tuning--adjustments-configh)
13. [Troubleshooting](#13-troubleshooting)
14. [File Map — What Every File Does](#14-file-map--what-every-file-does)
15. [Detailed Function Reference (Function-by-Function Explanation)](#15-detailed-function-reference-function-by-function-explanation)

---

## 1. How the Robot Thinks — Big Picture

Imagine you are blindfolded in a room full of people holding signs. You spin slowly until you spot a sign, walk toward the nearest one, tap it, then spin again to find the next. That is exactly what this robot does.

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│   👁️  CAMERA SEES                                               │
│       ↓                                                         │
│   🧠  BRAIN DECIDES  ──►  Which marker is closest?             │
│       ↓                                                         │
│   ⚙️  MOTORS MOVE    ──►  Spin · Drive · Stop · Back up        │
│                                                                 │
│   🔁  Repeat forever — no PC, no phone, no Wi-Fi needed        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### The Robot's Life Cycle (one loop)

```
  ┌──────────┐     Found       ┌──────────────┐    25 cm away    ┌──────────────┐
  │  SWEEPING│ ──────────────► │  APPROACHING │ ───────────────► │   REACHED!   │
  │ (spinning│                 │  (driving to │                  │ (stops, waits│
  │  left &  │                 │   marker)    │                  │  400 ms)     │
  │  right)  │◄─────────────── │              │                  └──────┬───────┘
  └──────────┘  Nothing found  └──────────────┘                         │
        ▲                                                                │ backs up
        │                                                                ▼
        │                                                       ┌──────────────┐
        └────────────────────────────────────────────────────── │  CLEARING    │
                                Done! Find the next one         │ (reverse +   │
                                                                │  turn away)  │
                                                                └──────────────┘
```

---

## 2. What You Need — Parts List

| # | Part | Notes |
|---|------|-------|
| 1 | **ESP32-S3-WROOM-1-N16R8** dev board | The "brain" — has 16 MB storage and 8 MB RAM |
| 2 | **OV2640 camera module** | Plugs into the board's camera connector |
| 3 | **L298N or TB6612 motor driver board** | Controls the wheel motors |
| 4 | **2× DC gear motors** with wheels | Your robot's legs |
| 5 | **Robot chassis / frame** | Any 2-wheel or 4-wheel platform |
| 6 | **Battery pack** (7.4 V LiPo or 6× AA) | Enough power for motors + board |
| 7 | **Jumper wires** (male-to-female) | For all connections |
| 8 | **A4 matte paper** + printer | For printing the target markers |
| 9 | **USB-C cable** | For uploading code once |
| 10 | **A Windows / Mac / Linux PC** | For uploading code once (not needed after) |

> [!NOTE]
> After you upload the code **one single time**, you never need the PC again. The robot runs completely on its own every time you switch it on.

---

## 3. Step 1 — Print the Target Markers (`aruco_a4_id0.png`)

The robot hunts for **ArUco markers (`DICT_4X4_50`)**. The printable A4 sheet **`aruco_a4_id0.png`** is already provided directly in the root of this workspace.

### 3a. Target Sheet Details (`aruco_a4_id0.png`)

- Contains two **10.0 × 10.0 cm** ArUco `DICT_4X4_50` markers (`ID: 0`).
- Each marker features a 6×6 module grid (outer black border + 4×4 data bit pattern) surrounded by white quiet-zone borders.
- Print directly on **matte A4 paper** at **100% scale (actual size)**. Do **not** select "Fit to page" or "Shrink to fit".
- After printing, verify with a ruler that the outer black box is **10.0 cm × 10.0 cm**.

### 3b. What the Sheet Looks Like

```
┌─────────────────── A4 Paper ───────────────────┐
│                                                 │
│    AprilTag tag16h5  ID: 0  |  10 × 10 cm      │
│  ┌───────────────────────────────┐              │
│  │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │              │
│  │  ▓                       ▓  │              │
│  │  ▓  ░░░░  ▓▓░░  ░░░░    ▓  │  10 cm tall  │
│  │  ▓  ░░▓▓  ░░▓▓  ▓▓░░    ▓  │              │
│  │  ▓                       ▓  │              │
│  │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │              │
│  └───────────────────────────────┘              │
│  ←────────────── 10 cm ──────────────→         │
│                                                 │
│- - - - - - ✂ cut here - - - - - - - - - - - - │
│                                                 │
│    AprilTag tag16h5  ID: 1  |  10 × 10 cm      │
│  ┌───────────────────────────────┐              │
│  │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │              │
│  │  ▓  different pattern    ▓  │              │
│  │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  │              │
│  └───────────────────────────────┘              │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 3c. How to Print

1. Open `apriltag_16h5_sheet.png` in any image viewer or photo app.
2. **Print → Page Setup → Scale: 100% (do NOT select "Fit to Page").**
3. Use **matte** paper — glossy paper reflects light and confuses the camera.
4. After printing, measure one tag with a ruler — it must be exactly **10 cm × 10 cm**.
5. Cut along the dashed line to separate the two tags.
6. Place tags flat on the floor or tape them to a wall at camera height.

> [!IMPORTANT]
> If the printed size is wrong (e.g. 8 cm instead of 10 cm), the robot's distance estimate will be wrong. Always verify with a ruler before the first run.

---

## 4. Step 2 — Wire Everything Together

### 4a. Overview Diagram

```
                        ┌─────────────────────────┐
                        │   ESP32-S3 Dev Board    │
                        │                         │
  OV2640 Camera ───────►│  Camera Connector       │
                        │                         │
  Motor Driver ◄────────│  GPIO 1  (L Motor FWD) │
     IN1 ◄──────────────│  GPIO 2  (L Motor REV) │
     IN2 ◄──────────────│  GPIO 41 (R Motor FWD) │
     IN3 ◄──────────────│  GPIO 42 (R Motor REV) │
     IN4 ◄──────────────│                         │
                        │  5V  ──────────────────►│◄── USB-C Power
                        │  GND                    │
                        └─────────────────────────┘
                                    │
                        ┌─────────────────────────┐
                        │    Motor Driver Board   │
                        │    (L298N / TB6612)     │
                        │                         │
  Left Motor  ◄─────────│  OUT1 / OUT2            │◄── Battery (7.4V)
  Right Motor ◄─────────│  OUT3 / OUT4            │
                        └─────────────────────────┘
```

### 4b. Motor Driver Wiring Table

Connect these wires one by one. Use the colour of your jumper wires to keep track.

| Wire goes FROM | Wire goes TO | What it does |
|----------------|--------------|--------------|
| ESP32 **GPIO 1** | Motor Driver **IN1** | Left wheel — go forward |
| ESP32 **GPIO 2** | Motor Driver **IN2** | Left wheel — go backward |
| ESP32 **GPIO 41** | Motor Driver **IN3** | Right wheel — go forward |
| ESP32 **GPIO 42** | Motor Driver **IN4** | Right wheel — go backward |
| ESP32 **GND** | Motor Driver **GND** | Shared ground (very important!) |
| Battery **+** | Motor Driver **VIN** | Power for the motors |
| Battery **−** | Motor Driver **GND** | Battery ground |
| Motor Driver **5V OUT** | ESP32 **5V** | Power for the brain board |

> [!WARNING]
> **Always connect GND between the ESP32 and the motor driver.** Without a shared ground, the signals have no reference and the robot will behave randomly or not move at all.

### 4c. Camera Connection

The OV2640 camera plugs directly into the camera FPC (ribbon) connector on the ESP32-S3 board. The connector has a small brown latch — lift it, insert the ribbon cable (shiny contacts facing the board), then push the latch down to lock.

```
   Camera ribbon cable
         ││││││││││││
   ┌─────┴──────────┴─────┐
   │  ↑ lift latch        │
   │  insert ribbon       │  ← Camera FPC connector on board
   │  push latch down ↓   │
   └──────────────────────┘
```

---

## 5. Step 3 — Install Software on Your PC (one time only)

### 5a. Install Arduino IDE

1. Go to **[https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)**
2. Download and install **Arduino IDE 2.x** (the latest version).

### 5b. Add the ESP32-S3 Board Package

This teaches Arduino IDE how to talk to your specific chip.

1. Open Arduino IDE.
2. Click **File → Preferences** (or **Arduino IDE → Settings** on Mac).
3. Find the box labelled **"Additional boards manager URLs"**.
4. Paste this URL in and click **OK**:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
5. Click **Tools → Board → Boards Manager**.
6. Search for **`esp32`**.
7. Click **Install** next to **"esp32 by Espressif Systems"** (wait — it downloads ~200 MB).

```
Boards Manager
┌─────────────────────────────────────────────────────┐
│  Search: esp32                                      │
│                                                     │
│  ┌──────────────────────────────────────────────┐  │
│  │ esp32  by Espressif Systems          v3.x.x  │  │
│  │ [INSTALL]                                    │  │
│  └──────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

---

## 6. Step 4 — Open the Code in Arduino IDE

1. Open the folder `esp32_firmware` on your PC.
2. Double-click **`esp32_firmware.ino`**.
3. Arduino IDE opens and you will see **multiple tabs** across the top — one per file. This is normal and expected.

```
┌─────────────────────────────────────────────────────────────────────────┐
│  esp32_firmware.ino  │ Config.h │ Types.h │ MotorDriver.h │ ... more   │
│────────────────────────────────────────────────────────────────────────│
│  /*                                                                     │
│   * AUTONOMOUS APRILTAG ROBOT — MAIN ENTRY POINT                       │
│   ...                                                                   │
└─────────────────────────────────────────────────────────────────────────┘
```

> [!TIP]
> You never need to touch any file other than `Config.h` for normal tuning. Everything else is automatic.

---

## 7. Step 5 — Configure Arduino IDE for This Board

Click the **Tools** menu and set each option **exactly** as shown:

| Tools Menu Option | What to Select |
|-------------------|----------------|
| **Board** | `ESP32S3 Dev Module` |
| **PSRAM** | `OPI PSRAM` |
| **Flash Size** | `16MB (128Mb)` |
| **Flash Mode** | `QIO 80MHz` |
| **Partition Scheme** | `Huge APP (3MB No OTA/1MB SPIFFS)` |
| **CPU Frequency** | `240MHz (WiFi/BT)` |
| **Upload Speed** | `921600` |
| **USB Mode** | `Hardware CDC and JTAG` |
| **Port** | *(the COM port that appears when you plug in the USB cable)* |

> [!IMPORTANT]
> **PSRAM must be set to "OPI PSRAM"** — without this, the camera has no memory to store images and the robot will crash immediately on startup.

### How to Find the Right COM Port

1. Unplug the USB cable from the PC.
2. Open **Tools → Port** — note which ports are listed.
3. Plug in the USB cable.
4. Open **Tools → Port** again — a **new port** appeared. That is your robot's port. Select it.

```
Before plugging in:    After plugging in:
  COM1                   COM1
  COM3                   COM3
                       ► COM7  ← NEW! This is your robot
```

---

## 8. Step 6 — Upload the Code to the Robot

1. Make sure the USB cable is plugged in and the correct **Port** is selected.
2. Click the **→ Upload** button (the right-pointing arrow at the top).

```
  ┌────────────────────────────────────────────────┐
  │  ✓  ⇒  🔍  ...  Arduino IDE toolbar          │
  │  │   │                                         │
  │  │   └─ Click this UPLOAD button               │
  │  └─ This is the Verify/Compile button          │
  └────────────────────────────────────────────────┘
```

3. Watch the bottom bar — it will say **"Compiling..."** then **"Uploading..."**
4. Wait for: **"Done uploading."**

```
Progress bar during upload:
▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░░░░░  78%  Uploading...
▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  Done uploading ✓
```

> [!NOTE]
> If you see `A fatal error occurred: Failed to connect`, hold down the **BOOT** button on the board while clicking Upload, then release it once "Uploading..." appears. This forces the chip into programming mode.

5. Once uploaded, **unplug the USB cable**. The robot is now self-contained.

---

## 9. Step 7 — Power On and Watch It Go

### Setup Checklist Before Power-On

- [ ] Printed markers placed on the floor, at least 50 cm apart
- [ ] Markers lying flat (not crumpled, not shiny)
- [ ] All wires connected (especially the shared GND!)
- [ ] Robot placed on a smooth flat surface
- [ ] Battery charged and plugged in

### What Happens When You Switch It On

```
Time 0s   → Robot powers up
Time 0.5s → Camera wakes up, AprilTag detector starts
Time 1s   → Robot begins slowly spinning LEFT  ←
Time 1.5s → Robot begins slowly spinning RIGHT →
             (scanning a 90° arc to find the nearest marker)
             │
             ▼ (marker spotted!)
Time ~3s  → Robot STOPS spinning
           → Drives straight toward the nearest marker
           → Steers left/right to stay centred on the marker
             │
             ▼ (25 cm away)
           → Robot STOPS
           → Waits 400 ms (a brief pause)
           → Reverses ~5 cm
           → Spins away so the marker is out of view
           → Starts spinning again to find the NEXT marker
           → Repeats forever ♻️
```

### Robot Body Language

| What you see the robot doing | What it means |
|------------------------------|---------------|
| Spinning slowly left then right | 🔍 Searching for a marker |
| Driving forward with small corrections | 🎯 Found one, heading toward it |
| Completely stopped for a moment | ✅ Reached a marker! |
| Short reverse + spin sideways | 🔄 Moving away to find the next one |
| Spinning one direction indefinitely | ⚠️ No markers in view — place one in front of it |

---

## 10. Understanding the Serial Debug Log

You can watch the robot's thoughts in real time by plugging the USB cable back in **without uploading** — just for monitoring.

Open **Tools → Serial Monitor** in Arduino IDE and set baud rate to **115200**.

```
=====================================================
  AUTONOMOUS APRILTAG NEAREST-TARGET ROBOT STARTING
  ESP32-S3-N16R8 | OV2640 | tag16h5 | Standalone
=====================================================
[OK] Motor HAL initialised (pins 1,2,41,42)
[OK] Camera (OV2640) + AprilTag (tag16h5) ready
[OK] NavigationFSM started — beginning panoramic sweep
```

| Log Message | Plain English |
|-------------|---------------|
| `[FSM:SWEEP] Step0 done → pivoting RIGHT 90°` | Finished left swing, now swinging right |
| `[FSM:SWEEP] Step1 – new best: Tag0 82.3cm +12.1°` | Spotted marker ID 0, it's 82 cm away and a bit to the right |
| `[FSM:SWEEP] NEAREST TAG → ID:1 \| Dist:54.2cm` | Best target chosen: marker 1 at 54 cm |
| `[CAM] Tag ID:1 \| W:50.2px \| Dist:54.2cm \| Err:+22` | Camera frame report: tag width in pixels, distance, steering error |
| `[FSM:APPROACH] Tag1 \| Dist: 38.5cm \| Err:+10 \| L:147 R:133` | Closing in, slightly right of centre, left wheel a bit faster |
| `[FSM:REACHED] Tag1 reached at 24.8cm! Total cleared: 1` | Robot is at the marker! |
| `[FSM:CLEAR] Clearance done. Scanning for next target...` | Backed up and turned, now hunting again |
| `[FSM:LOST] Tag lost. Returning to SWEEP_SCAN...` | Marker disappeared from view, restarting search |

---

## 11. How the Robot Sees & Decides — Lock-and-Track ROI Vision

The robot uses a **two-mode vision engine** designed specifically for real-time responsiveness on the ESP32-S3:

```
MODE A: UNLOCKED DISCOVERY (active during SWEEP_SCAN)
┌────────────────────────────────────────────────────────┐
│ 1. Scan 320x240 frame for candidate square blobs       │
│ 2. Check 1-cell black border & white quiet-zone ring   │
│ 3. Sample 4x4 data grid & decode DICT_4X4_50 ID        │
│ 4. If multiple tags visible: select the NEAREST one!   │
│ 5. Lock target & seed ROI box                          │
└──────────────────────────┬─────────────────────────────┘
                           │ Target locked!
                           ▼
MODE B: LOCKED ROI TRACKING (active during APPROACH_TARGET)
┌────────────────────────────────────────────────────────┐
│ 1. SKIP FULL PATTERN DECODING completely!              │
│ 2. Expand ROI by ROI_PADDING_PX (45 px) around target  │
│ 3. Fast contour & centroid update (~1 ms latency)      │
│ 4. Steer smoothly toward locked target                 │
│ 5. Stop when distance <= STOP_DISTANCE_CM (25 cm)      │
└────────────────────────────────────────────────────────┘
```

```
                   ┌─────────────────────────────────────────────────────────┐
                   │                                                         │
   POWER ON        ▼                                                         │
     │      ┌─────────────┐                                                  │
     └─────►│ SWEEP_SCAN  │◄──────────────────────────────┐                 │
            │             │                               │                 │
            │ Robot spins │  No marker found in arc        │ Done clearing   │
            │ left 45°    │──────────────────────────────► (stays in SWEEP) │
            │ then right  │                                │                 │
            │ 90°, logging│                                │                 │
            │ all markers │                                │                 │
            └──────┬──────┘                                │                 │
                   │ Nearest marker found!                 │                 │
                   ▼                                       │                 │
            ┌──────────────┐                               │                 │
            │   APPROACH   │  Marker disappears > 1.5 sec  │                 │
            │   TARGET     │──────────────────────────────►(back to SWEEP)  │
            │              │                                                  │
            │ Steers toward│                                                  │
            │ marker using │                                                  │
            │ proportional │                                                  │
            │ control      │                                                  │
            └──────┬───────┘                                                  │
                   │ Distance ≤ 25 cm                                         │
                   ▼                                                           │
            ┌──────────────┐                                                   │
            │   TARGET     │                                                   │
            │   REACHED    │  400 ms pause                                     │
            └──────┬───────┘                                                   │
                   │                                                            │
                   ▼                                                            │
            ┌──────────────┐                                                    │
            │  CLEARANCE   │  Reverse 350 ms                                    │
            │   PIVOT      │  Spin away 600 ms ─────────────────────────────────┘
            └──────────────┘
```

### Steering — How the Robot Stays on Target

The camera watches where the marker is in the frame. If the marker is in the exact centre, both wheels spin at the same speed. If the marker drifts to the right, the **right wheel slows down** (and left speeds up) to steer right. This correction is proportional — a small drift causes a small correction; a big drift causes a big correction.

```
Marker CENTRED          Marker to the RIGHT     Marker to the LEFT
      │                        │                        │
  ────┼────               ─────┼──                  ──┼─────
      ↓                        ↓                        ↓
  L=140  R=140           L=154  R=126           L=126  R=154
  (straight)             (turns right)          (turns left)
```

---

## 12. Tuning & Adjustments (`Config.h`)

Open [`Config.h`](file:///e:/academic%20documents/2.2/hardware/robot_v1/esp32_firmware/Config.h) in Arduino IDE. These are the values you can safely change without breaking anything. After any change, re-upload the code.

### Speed Settings

```cpp
#define SPEED_BASE    140   // How fast the robot drives toward a marker (0–255)
#define SPEED_PIVOT   130   // How fast it spins during the left/right search sweep
#define SPEED_REVERSE 120   // How fast it backs away after reaching a marker
```

> Increase numbers → robot moves faster. Decrease → slower and more careful.
> Never exceed 220 — motors overheat.

### Stopping Distance

```cpp
#define STOP_DISTANCE_CM  25.0f   // Stop when marker is this many cm away
```

> Increase this (e.g. `35.0f`) to stop further away. Decrease (e.g. `15.0f`) to get closer.

### Steering Sensitivity

```cpp
#define STEERING_KP   0.7f   // Steering correction strength
#define DEADBAND_PX    18    // Ignore drift smaller than this (pixels)
```

> If the robot **wiggles / oscillates** while approaching → lower `STEERING_KP` to `0.4f`.
> If the robot approaches **too slowly and drifts wide** → raise to `0.9f`.

### Sweep Arc Timing

```cpp
#define SWEEP_LEFT_45_MS    450   // ms to pivot 45° left
#define SWEEP_RIGHT_90_MS   900   // ms to sweep 90° right
#define SWEEP_CENTER_45_MS  450   // ms to return to centre if nothing found
```

> If the robot consistently **undershoots** the 45° arc, increase these values.
> If it **overshoots** (goes past 45°), decrease them.

---

## 13. Troubleshooting

### Robot does not move at all

```
✅ Check: Is the battery charged and connected?
✅ Check: Is the GND wire connected between ESP32 and motor driver?
✅ Check: Tools → Port — is the board detected when USB is plugged in?
✅ Check: Serial Monitor output — does it show [OK] messages?
```

### Robot spins forever and never approaches a marker

```
✅ Check: Is the marker flat on the floor in the camera's field of view?
✅ Check: Is the marker exactly 10 × 10 cm? (measure with ruler)
✅ Check: Is the marker printed on MATTE paper? (not glossy)
✅ Check: Is lighting OK? (avoid very bright backlight directly behind marker)
✅ Check: Serial Monitor — does [CAM] ever appear? If not, camera sees nothing.
```

### Robot overshoots and bumps into the marker

```
✅ Fix: Increase STOP_DISTANCE_CM from 25.0 to 35.0 in Config.h, re-upload.
```

### Robot wobbles left-right while approaching

```
✅ Fix: Decrease STEERING_KP from 0.7 to 0.4 in Config.h, re-upload.
```

### "Failed to connect" error during upload

```
✅ Fix: Hold the BOOT button on the board, click Upload in Arduino IDE,
        release BOOT once "Uploading..." appears in the status bar.
```

### Upload succeeds but nothing happens (Serial Monitor is blank)

```
✅ Check: Serial Monitor baud rate is set to 115200 (bottom-right dropdown).
✅ Check: Tools → USB Mode is set to "Hardware CDC and JTAG".
✅ Try:   Press the RST (Reset) button on the board once after uploading.
```

---

## 14. File Map — What Every File Does

Think of these files like the organs of a living creature — each one has one specific job:

```
esp32_firmware/
│
├── 📄 esp32_firmware.ino      ← THE HEART
│                                 Starts everything up. Calls camera
│                                 and brain on every tick. (~60 lines)
│
├── ⚙️  Config.h               ← THE SETTINGS PANEL
│                                 All numbers you might want to change
│                                 live here: pin numbers, speeds, distances.
│
├── 🗂️  Types.h                ← THE VOCABULARY
│                                 Defines what a "Target Detection" looks
│                                 like as data (distance, angle, ID, etc.)
│
├── 🔧 MotorDriver.h           ← LEFT/RIGHT WHEEL CONTROLLER (interface)
├── 🔧 MotorDriver.cpp         ← LEFT/RIGHT WHEEL CONTROLLER (how it works)
│                                 Tells each motor how fast and which
│                                 direction to spin using PWM signals.
│
├── 👁️  CameraVision.h         ← EYES (interface & detection constants)
├── 👁️  CameraVision_Init.cpp  ← Camera driver hardware setup & sensor tuning
├── 👁️  CameraVision_Scan.cpp  ← Frame capture & JPEG preview streaming
├── 👁️  CameraVision_ScanFullFrame.cpp ← Mode A: adaptive full-frame ArUco discovery
├── 👁️  CameraVision_ScanTrackedRoi.cpp ← Mode B: fast local ROI window tracking
├── 👁️  CameraVision_Decode.cpp ← 4x4 dictionary pattern decode & hamming distance
├── 👁️  CameraVision_TargetMemory.cpp ← Visited targets cooldown & exclusion memory
│
├── 🧠 NavigationFSM.h         ← BRAIN (interface & state machine definitions)
├── 🧠 NavigationFSM_Init.cpp  ← Constructor & state machine reset
├── 🧠 NavigationFSM_Update.cpp ← Loop tick state router & human-readable state names
├── 🧠 NavigationFSM_SweepScan.cpp ← State 1: 360° stepped radar search & settle
├── 🧠 NavigationFSM_Approach.cpp  ← State 2: Aim-Then-Drive anti-slip alignment & approach
├── 🧠 NavigationFSM_TargetReached.cpp ← State 3: Stop pause & mark visited
├── 🧠 NavigationFSM_ClearancePivot.cpp ← State 4: Backup clearance & resume sweep
│
├── 🐍 generate_apriltag_sheet.py  ← TAG PRINTER HELPER
│                                    Run once on your PC to create the
│                                    printable A4 marker sheet (ID 0 + ID 1).
│
└── 📖 README.md               ← THIS FILE — the guide you are reading
```

---

## 15. Detailed Function Reference (Function-by-Function Explanation)

Below is a complete guide to every function across all firmware modules.

---

### A. Main Entry Point (`esp32_firmware.ino`)

* **`setup()`**:
  * Initializes Serial debugging at 115200 baud.
  * **Starts Camera & Vision first**: Allocates LEDC Timer 0 / Channel 0 strictly for the OV2640 20 MHz XCLK (preventing any timer conflicts).
  * **Starts MotorDriver second**: Sets up GPIOs 38, 1, 2, 47, 21, 45 in pure digital mode.
  * **Starts NavigationFSM third**: Boots the state machine into 360° radar sweep mode.
* **`loop()`**:
  * Runs repeatedly forever.
  * Calls `vision.scan()` to acquire a frame and detect ArUco markers.
  * Passes the detection result to `fsm.update(target)` to drive robot decisions and motor movement.

---

### B. Motor Driver Subsystem (`MotorDriver.h` / `MotorDriver.cpp`)

* **`init()`**: Configures direction and enable pins as `OUTPUT` and stops all wheels immediately.
* **`applyMotor(pinA, pinB, enPin, speed)`**: Low-level pure digital driver. Applies `HIGH`/`LOW` to direction pins and `HIGH` to the enable pin (`ENA`/`ENB`) for 100% solid 3.3V gate logic (no PWM timer interference with camera XCLK).
* **`setSpeeds(leftSpeed, rightSpeed)`**: Applies speed commands to both sides simultaneously; applies inversion flags (`INVERT_LEFT_MOTOR`, `INVERT_RIGHT_MOTOR`) if enabled.
* **`forward(speed)`**: Drives all 4 wheels forward.
* **`reverse(speed)`**: Drives all 4 wheels backward.
* **`pivotLeft(speed)`**: Tank turn in place: Left wheels backward, Right wheels forward.
* **`pivotRight(speed)`**: Tank turn in place: Left wheels forward, Right wheels backward.
* **`stop()`**: Shuts off all 6 L298N pins to brake the chassis immediately.

---

### C. Camera Vision Subsystem (`CameraVision.h` & `CameraVision_*.cpp`)

#### 1. Hardware Initialization ([CameraVision_Init.cpp](CameraVision_Init.cpp))
* **`CameraVision()`** *(Constructor)*: Initializes state variables, resets lock flags, and clears visited target timestamps.
* **`init()`**: Calls camera configuration and sensor tuning routines; prepares vision system for frame capture.
* **`configureCamera()`**: Maps the 8-bit parallel D0–D7 camera data bus, sync pins (VSYNC, HREF, PCLK), and allocates a double-buffered frame buffer in 8 MB OPI PSRAM.
* **`configureSensorTuning()`**: Un-mirrors the OV2640 sensor readout (`set_hmirror(1)`), and optimizes contrast, sharpness, and gain for indoor printed paper.

#### 2. Frame Acquisition & PC Streaming ([CameraVision_Scan.cpp](CameraVision_Scan.cpp))
* **`scan()`**: Grabs the active grayscale frame buffer (`esp_camera_fb_get()`), compresses and sends an optional 35% quality JPEG over Serial to `preview_camera.py`, calls `scanFullFrame()`, and safely releases the frame buffer back to the memory pool.
* **`emptyResult()`**: Helper that returns a blank `TargetDetection` object when no marker is in view.

#### 3. Full-Frame Detection ([CameraVision_ScanFullFrame.cpp](CameraVision_ScanFullFrame.cpp))
* **`scanFullFrame(fb)`** *(Mode A — Discovery)*:
  1. **Dynamic Lighting Adaptation**: Samples ambient lighting to calculate a mean-anchored threshold that is immune to bright 255 light bulbs or dark shadows.
  2. **Connected Component Run-Length Encoding (RLE)**: Uses static buffers to group dark pixels into candidate blobs in ~2 ms without stack overflows.
  3. **Quiet Zone Verification**: Checks that white paper surrounds all 4 sides of the candidate marker.
  4. **4-Edge Midpoint Sampling**: Measures true contrast at the 4 edges of the candidate box (handles marker tilts up to ~20°).
  5. **Inner 4×4 Bit Extraction**: Samples the 4×4 data grid and validates that it contains a mix of black and white cells.
  6. **Dictionary Matching**: Matches pattern against `DICT_4X4_50` using `decodeAruco4x4()`.
  7. **Multi-Target Nearest Selection**: Evaluates all valid markers, filters out visited ones, and returns the single closest target. Computes physical optical polar angle ($\text{polarAngleDeg} = 90^\circ - \text{angleDeg}$).

#### 4. Fast Local Tracking ([CameraVision_ScanTrackedRoi.cpp](CameraVision_ScanTrackedRoi.cpp))
* **`scanTrackedRoi(fb)`** *(Mode B — ROI Tracking)*:
  * Narrows the search area to a padded box around the previously locked target. Runs in ~1 ms to update centroid and distance during rapid forward motion.

#### 5. Dictionary Decoding ([CameraVision_Decode.cpp](CameraVision_Decode.cpp))
* **`decodeAruco4x4(inner, outHamming)`**: Compares a 4×4 matrix across all 4 rotations and horizontal flip against the 50 predefined codes in `ARUCO_DICT_4X4_50` using bitwise XOR and popcount. Strict zero hamming distance (`hamming == 0`) ensures zero false positives.

#### 6. Target Memory & Cooldown ([CameraVision_TargetMemory.cpp](CameraVision_TargetMemory.cpp))
* **`lockTarget(target)`**: Sets `m_locked = true` and saves target bounding box.
* **`unlockTarget()`**: Releases tracking lock so the robot returns to open 360° search.
* **`markVisited(tagId, centerX, durationMs)`**: Remembers reached marker for 30 seconds so the robot ignores it when looking forward after backing up.
* **`clearFrontVisitedZone()`**: Resets the front exclusion zone once the robot rotates away to another sector.

---

### D. Navigation State Machine (`NavigationFSM.h` & `NavigationFSM_*.cpp`)

#### 1. Setup & Reset ([NavigationFSM_Init.cpp](NavigationFSM_Init.cpp))
* **`NavigationFSM(motors, vision)`** *(Constructor)*: Links references to motors and camera vision, and resets internal state timers.
* **`init()`**: Sets state to `SWEEP_SCAN`, resets sector step to 0, stops motors, and unlocks any previous target.
* **`resetBestTarget()`**: Clears candidate target memory.

#### 2. Loop Tick Router ([NavigationFSM_Update.cpp](NavigationFSM_Update.cpp))
* **`update(target)`**: Called once every loop cycle; dispatches execution to the handler matching `m_state`.
* **`stateName(state)`**: Returns human-readable state name string for serial diagnostic logs.

#### 3. Continuous 360° Radar Search ([NavigationFSM_SweepScan.cpp](NavigationFSM_SweepScan.cpp))
* **`handleSweepScan(target, now)`**:
  * **Stationary Look Window**: Halts motors and looks forward for `SWEEP_LOOK_MS` (1.5 s) so the camera takes blur-free captures.
  * **Instant Lock**: If a target is seen at ANY angle, immediately locks and transitions to `APPROACH_TARGET`.
  * **Stepped Rotation**: If no target is seen, executes a crisp in-place 45° right turn (`SWEEP_LEFT_45_MS = 450 ms`), pauses 150 ms to settle chassis vibration, and repeats across 8 sectors (full 360°).

#### 4. Aim-Then-Drive Approach ([NavigationFSM_Approach.cpp](NavigationFSM_Approach.cpp))
* **`handleApproach(target, now)`**:
  * **Arrival Check**: If target distance is $\le 25\text{ cm}$, stops motors, marks target as VISITED, and transitions to `TARGET_REACHED`.
  * **Aim-Then-Drive Alignment**:
    * If target is within $\pm 12^\circ$ of center ($78^\circ \dots 102^\circ$): Drives straight forward slowly in controlled pulse steps (`DRIVE_STEP_MS = 160 ms`, pause 220 ms).
    * If target is $> 12^\circ$ off-center: Calculates a gentle proportional turn pulse (`AIM_TURN_RATE_MS = 8.5 ms/deg`, clamped to 80–220 ms). Executes a gentle in-place nudge toward the target and pauses 150 ms to settle. Strictly prevents wheel slip and keeps the target in the camera's field of view.
  * **Target Lost Watchdog**: If the target is occluded or lost for $> 2.5\text{ s}$, safely falls back to `SWEEP_SCAN`.

#### 5. Stop & Mark Visited ([NavigationFSM_TargetReached.cpp](NavigationFSM_TargetReached.cpp))
* **`handleTargetReached(now)`**: Stops motors, stabilizes the chassis for 400 ms, logs target arrival, and switches to `CLEARANCE_PIVOT`.

#### 6. Backup & Resume Search ([NavigationFSM_ClearancePivot.cpp](NavigationFSM_ClearancePivot.cpp))
* **`handleClearancePivot(now)`**: Reverses motors (`reverse(SPEED_REVERSE)`) for 450 ms to gain clearance from the reached marker, resets tracking memory, and returns to `SWEEP_SCAN` looking forward while ignoring the visited marker.

---

## Quick Reference Card

> Cut this out and stick it on your robot! 🤖

```
┌─────────────────────────────────────────────────────────────┐
│          ARUCO ROBOT — QUICK REFERENCE CARD                 │
├─────────────────────────────────────────────────────────────┤
│  L298N PINS:  ENA=38  IN1=1  IN2=2  IN3=47  IN4=21  ENB=45  │
│  STOP DIST:   25 cm from marker                             │
│  AIM ZONE:    ±12° from center (78°..102° drives straight)  │
│  SERIAL BAUD: 115200                                        │
├─────────────────────────────────────────────────────────────┤
│  LOOKING  = Stationary look forward for ArUco tag (1.5 s)   │
│  PIVOT 45 = No target -> turn right 45° to next sector      │
│  AIMING   = Gentle turn pulse to center tag (zero slip)     │
│  FORWARD  = Tag centered -> slow step drive (160ms on/pause)│
│  STOPPED  = Reached 25 cm -> marked VISITED (ignored 30 s)  │
│  BACKING  = Backup clearance -> resume 360° search          │
└─────────────────────────────────────────────────────────────┘
```

---

*Last updated: November 2026 — Firmware v2.0 (AprilTag standalone, ESP32-S3-N16R8)*
