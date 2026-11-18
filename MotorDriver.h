#pragma once
#include <Arduino.h>
#include "Config.h"

/*
 * =====================================================================================
 *  MOTOR DRIVER HAL  –  INA/INB-style H-Bridge (e.g. L298N with Enable pins)
 *
 *  Control scheme:
 *    ENA (GPIO 38) — PWM duty  → Left  motor speed  (LEDC ch 2 / Timer 1)
 *    IN1 (GPIO  1) — HIGH/LOW  → Left  motor direction A
 *    IN2 (GPIO  2) — HIGH/LOW  → Left  motor direction B
 *    IN3 (GPIO 47) — HIGH/LOW  → Right motor direction A
 *    IN4 (GPIO 21) — HIGH/LOW  → Right motor direction B
 *    ENB (GPIO 45) — PWM duty  → Right motor speed  (LEDC ch 3 / Timer 1)
 *
 *  Truth table (one side):
 *    Forward : IN_A=HIGH, IN_B=LOW,  PWM > 0
 *    Reverse : IN_A=LOW,  IN_B=HIGH, PWM > 0
 *    Stop    : IN_A=LOW,  IN_B=LOW,  PWM = 0
 *
 *  Compatible with ESP32 Arduino Core 2.x AND 3.x LEDC APIs.
 * =====================================================================================
 */

class MotorDriver {
public:
    MotorDriver();

    // Attach LEDC to INA/INB, set IN1-IN4 as outputs, ensure motors are stopped
    void init();

    // Signed differential drive (–255 … +255 per side)
    void setSpeeds(int leftSpeed, int rightSpeed);

    // High-level motion primitives
    void forward(int speed = SPEED_BASE);
    void reverse(int speed = SPEED_REVERSE);
    void pivotLeft(int speed = SPEED_PIVOT);
    void pivotRight(int speed = SPEED_PIVOT);
    void stop();

private:
    // Set direction pins and write PWM speed for one motor side
    void applyMotor(uint8_t pinA, uint8_t pinB, uint8_t pwmPinOrCh, int speed);
};
