/*
 * ============================================================
 *  DIRECT MOTOR RUNNER — ALL 4 DIRECTIONS TEST
 * ============================================================
 *  Pins:
 *    ENA (Green)  -> GPIO 38  (Left Speed Enable)
 *    IN1 (Yellow) -> GPIO  1  (Left Direction A)
 *    IN2 (Orange) -> GPIO 48  (Left Direction B — Right Header pin between 45 and 47!)
 *    IN3 (Red)    -> GPIO 47  (Right Direction A)
 *    IN4 (Brown)  -> GPIO 21  (Right Direction B)
 *    ENB (Red)    -> GPIO 45  (Right Speed Enable)
 * ============================================================
 */

#include <Arduino.h>

#define PIN_ENA  38   // Green  wire (GPIO 38)
#define PIN_IN1   1   // Yellow wire (GPIO 1)
#define PIN_IN2  48   // Orange wire (GPIO 48 — Right Header between 45 and 47!)
#define PIN_IN3  47   // Red    wire (GPIO 47)
#define PIN_IN4  21   // Brown  wire (GPIO 21)
#define PIN_ENB  45   // Red    wire (GPIO 45)

void stopMotors() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    digitalWrite(PIN_ENA, LOW);
    digitalWrite(PIN_ENB, LOW);
}

void forwardMotors() {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    digitalWrite(PIN_ENA, HIGH);
    digitalWrite(PIN_ENB, HIGH);
}

void reverseMotors() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    digitalWrite(PIN_ENA, HIGH);
    digitalWrite(PIN_ENB, HIGH);
}

void pivotLeftMotors() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);  // Left reverse
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);   // Right forward
    digitalWrite(PIN_ENA, HIGH);
    digitalWrite(PIN_ENB, HIGH);
}

void pivotRightMotors() {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);   // Left forward
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);  // Right reverse
    digitalWrite(PIN_ENA, HIGH);
    digitalWrite(PIN_ENB, HIGH);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n==========================================");
    Serial.println("  4-DIRECTION MOTOR TEST (FORWARD/REV/L/R)");
    Serial.println("  Orange wire on GPIO 14 (IN2)            ");
    Serial.println("==========================================");

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);

    stopMotors();
}

void loop() {
    Serial.println("\n[TEST] 1. ALL 4 WHEELS FORWARD (1 second)...");
    forwardMotors();
    delay(1000);
    stopMotors();
    delay(1000);

    Serial.println("[TEST] 2. ALL 4 WHEELS REVERSE (1 second)...");
    reverseMotors();
    delay(1000);
    stopMotors();
    delay(1000);

    Serial.println("[TEST] 3. PIVOT LEFT - All 4 wheels should spin (1 second)!");
    pivotLeftMotors();
    delay(1000);
    stopMotors();
    delay(1000);

    Serial.println("[TEST] 4. PIVOT RIGHT - All 4 wheels should spin (1 second)!");
    pivotRightMotors();
    delay(1000);
    stopMotors();
    delay(2000);
}
