/*
 * ============================================================
 *  DIRECT MOTOR RUNNER — 100% NON-JTAG SAFE PINS
 * ============================================================
 *  GPIO 39-42 on ESP32-S3 are hardware JTAG pins (stuck at 0V / 1.9V).
 *  We move IN1-IN4 to 100% pure, non-JTAG GPIOs on the SAME top row:
 *
 *    ENA (Green)  -> GPIO 38  (stays where it is — already 3.3V!)
 *    IN1 (Yellow) -> GPIO  1  (pure GPIO, side-by-side with 2)
 *    IN2 (Orange) -> GPIO  2  (pure GPIO, lights up IO2 LED!)
 *    IN3 (Red)    -> GPIO 47  (pure GPIO, side-by-side with 21)
 *    IN4 (Brown)  -> GPIO 21  (pure GPIO, side-by-side with 47)
 *    ENB (Red)    -> GPIO 45  (stays where it is — already 3.3V!)
 * ============================================================
 */

#include <Arduino.h>

#define PIN_ENA  38   // Green  wire (keep on 38)
#define PIN_IN1   1   // Yellow wire (move from 39 -> 1)
#define PIN_IN2   2   // Orange wire (move from 40 -> 2)
#define PIN_IN3  47   // Red    wire (move from 41 -> 47)
#define PIN_IN4  21   // Brown  wire (move from 42 -> 21)
#define PIN_ENB  45   // Red    wire (keep on 45)

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n==========================================");
    Serial.println("  MOTOR DRIVE TEST — SAFE NON-JTAG PINS   ");
    Serial.println("==========================================");

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);

    // Enable both sides (3.3V)
    digitalWrite(PIN_ENA, HIGH);
    digitalWrite(PIN_ENB, HIGH);

    // Drive FORWARD using IN1=HIGH, IN2=LOW and IN3=HIGH, IN4=LOW
    // (If Left spins backward, swap the two motor wires in OUT1 <-> OUT2 on the L298N!)
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);

    Serial.println("[OK] ENA(38)=3.3V  IN1(1)=3.3V   IN2(2)=0V  (Left FWD)");
    Serial.println("[OK] ENB(45)=3.3V  IN3(47)=3.3V  IN4(21)=0V (Right FWD)");
    Serial.println(">>> Wheels should spin FORWARD now! <<<");
}

void loop() {
    delay(1000);
    Serial.println("[RUNNING] Motors active (forward)...");
}
