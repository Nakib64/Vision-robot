#include "MotorDriver.h"
#include "driver/gpio.h"

MotorDriver::MotorDriver() {}

// ---------------------------------------------------------------------------
// init() — Pure Digital Control (EXACT same proven setup as motor_test.ino!)
//
//   NO LEDC PWM timers are used — completely eliminates any LEDC channel/timer
//   conflict with esp_camera_init() and guarantees full solid 3.3V on ENA & ENB.
//
//   Pins (from Config.h):
//     Left  Motor: ENA=38, IN1=1,  IN2=2   (INVERT_LEFT_MOTOR  = true)
//     Right Motor: ENB=45, IN3=47, IN4=21  (INVERT_RIGHT_MOTOR = false)
// ---------------------------------------------------------------------------
void MotorDriver::init() {
    pinMode(PIN_MOTOR_INA, OUTPUT);
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);
    pinMode(PIN_MOTOR_IN3, OUTPUT);
    pinMode(PIN_MOTOR_IN4, OUTPUT);
    pinMode(PIN_MOTOR_INB, OUTPUT);

    // Boost all motor control GPIOs to maximum drive capability (40 mA)
    gpio_set_drive_capability((gpio_num_t)PIN_MOTOR_INA, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)PIN_MOTOR_IN1, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)PIN_MOTOR_IN2, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)PIN_MOTOR_IN3, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)PIN_MOTOR_IN4, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)PIN_MOTOR_INB, GPIO_DRIVE_CAP_3);

    stop();
    Serial.println("[OK] MotorDriver ready (Pure Digital 3.3V Max-Drive — zero LEDC conflict)");
}

// ---------------------------------------------------------------------------
// applyMotor() — Drive one side of the L298N with pure digital HIGH / LOW
//   pinA, pinB : direction GPIO pins (IN1/IN2 or IN3/IN4)
//   enPin      : enable GPIO pin (ENA=38 or ENB=45)
//   speed      : > 0 forward, < 0 reverse, 0 stop
// ---------------------------------------------------------------------------
void MotorDriver::applyMotor(uint8_t pinA, uint8_t pinB, uint8_t enPin, int speed) {
    if (speed > 0) {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
        digitalWrite(enPin, HIGH);
    } else if (speed < 0) {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
        digitalWrite(enPin, HIGH);
    } else {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        digitalWrite(enPin, LOW);
    }
}

// ---------------------------------------------------------------------------
// setSpeeds() — Signed differential drive for both sides simultaneously
// ---------------------------------------------------------------------------
void MotorDriver::setSpeeds(int leftSpeed, int rightSpeed) {
#if defined(INVERT_LEFT_MOTOR) && (INVERT_LEFT_MOTOR == true)
    leftSpeed = -leftSpeed;
#endif
#if defined(INVERT_RIGHT_MOTOR) && (INVERT_RIGHT_MOTOR == true)
    rightSpeed = -rightSpeed;
#endif

    applyMotor(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_INA, leftSpeed);
    applyMotor(PIN_MOTOR_IN3, PIN_MOTOR_IN4, PIN_MOTOR_INB, rightSpeed);
}

void MotorDriver::forward(int speed)    { setSpeeds( speed,  speed); }
void MotorDriver::reverse(int speed)    { setSpeeds(-speed, -speed); }

// Power Swing Turn: Drives Right forward while Left rolls freely (ENA=LOW).
// Completely eliminates the L298N active dynamic brake lock caused by IN2 requiring 5V,
// allowing the right wheels to swing the robot left smoothly on the floor with full torque.
void MotorDriver::pivotLeft(int speed) {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    digitalWrite(PIN_MOTOR_INA, LOW); // Left coast / freewheel (no brake!)

    applyMotor(PIN_MOTOR_IN3, PIN_MOTOR_IN4, PIN_MOTOR_INB, speed); // Right full forward
}

void MotorDriver::pivotRight(int speed) { setSpeeds( speed, -speed); }

void MotorDriver::stop() {
    digitalWrite(PIN_MOTOR_INA, LOW);
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    digitalWrite(PIN_MOTOR_IN3, LOW);
    digitalWrite(PIN_MOTOR_IN4, LOW);
    digitalWrite(PIN_MOTOR_INB, LOW);
}
