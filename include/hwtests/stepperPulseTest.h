// This test is to produce step rate, direction, number of steps
// #include <Arduino.h>
#include "StepperMotor.h"  // Assuming PlatformIO includes lib/ in the path
#include "Ctrl_config.h"   // For any config defines

// Create a StepperMotor instance (using pins from your config or example)
StepperMotor motor(PIN_PUL, PIN_DIR);  // STEP pin 18, DIR pin 25

void setup() {
    Serial.begin(9600);
    Serial.println("StepperMotor Test Started");
    Serial.println("Commands:");
    Serial.println("  v - set pulsing speed motor");
    Serial.println("  s<steps> - Step forward (e.g., s100 for 100 steps)");
    Serial.println("  r<steps> - Step reverse (e.g., r50 for 50 steps)");
    Serial.println("  p<steps>,<pulse> - Step with pulse (e.g., p200,1000 for 200 steps at 1000us pulse)");
    Serial.println("  t - Stop motor");
    Serial.println("  a - Pause motor");
    Serial.println("  e - Resume motor");
    Serial.println("  g - Get remaining steps");
    Serial.println("  c - Check if stepping");
    Serial.println("  d - Check if stopped");
    Serial.println("  b - Check if paused");
    motor.init();
}
void loop(){
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        Serial.print("Received: ");
        Serial.println(cmd);
        if(cmd.startsWith("v")){
            long Pulse = cmd.substring(1).toInt();
            motor.setPulse(Pulse);
            Serial.print("Pulsing speed: ");
            Serial.print(Pulse);
            Serial.println(" steps");
        } else if (cmd.startsWith("s")) {
            long steps = cmd.substring(1).toInt();
            if (motor.step(steps, HIGH)) {  // HIGH for forward
                Serial.print("Stepping forward ");
                Serial.print(steps);
                Serial.println(" steps");
            } else {
                Serial.println("Motor is already stepping");
            }
        } else if (cmd.startsWith("r")) {
            long steps = cmd.substring(1).toInt();
            if (motor.step(steps, LOW)) {  // LOW for reverse
                Serial.print("Stepping reverse ");
                Serial.print(steps);
                Serial.println(" steps");
            } else {
                Serial.println("Motor is already stepping");
            }
        } else if (cmd.startsWith("p")) {
            int commaIndex = cmd.indexOf(',');
            if (commaIndex > 0) {
                long steps = cmd.substring(1, commaIndex).toInt();
                long pulse = cmd.substring(commaIndex + 1).toInt();
                if (motor.step(steps, HIGH, pulse)) {  // Forward with custom pulse
                    Serial.print("Stepping ");
                    Serial.print(steps);
                    Serial.print(" steps with pulse ");
                    Serial.print(pulse);
                    Serial.println(" us");
                } else {
                    Serial.println("Motor is already stepping");
                }
            } else {
                Serial.println("Invalid format: p<steps>,<pulse>");
            }
        } else if (cmd == "t") {
            long remaining = motor.stop();
            Serial.print("Stopped, remaining steps: ");
            Serial.println(remaining);
        } else if (cmd == "a") {
            motor.pause();
            Serial.println("Paused");
        } else if (cmd == "e") {
            motor.resume();
            Serial.println("Resumed");
        } else if (cmd == "g") {
            long remaining = motor.getRemainingSteps();
            Serial.print("Remaining steps: ");
            Serial.println(remaining);
        } else if (cmd == "c") {
            bool stepping = motor.isStepping();
            Serial.print("Is stepping: ");
            Serial.println(stepping ? "Yes" : "No");
        } else if (cmd == "d") {
            bool stopped = motor.isStopped();
            Serial.print("Is stopped: ");
            Serial.println(stopped ? "Yes" : "No");
        } else if (cmd == "b") {
            bool paused = motor.isPaused();
            Serial.print("Is paused: ");
            Serial.println(paused ? "Yes" : "No");
        } else {
            Serial.println("Unknown command");
        }
    }

    // Optional: Print status periodically
    static unsigned long lastPrint = 0;
    // if (millis() - lastPrint > 1000) {
    //     lastPrint = millis();
    //     Serial.print("Status - Remaining: ");
    //     Serial.print(motor.getRemainingSteps());
    //     Serial.print(", Stepping: ");
    //     Serial.print(motor.isStepping() ? "Yes" : "No");
    //     Serial.print(", Paused: ");
    //     Serial.println(motor.isPaused() ? "Yes" : "No");
    // }
    
    // Optionally init here, or wait for 'i' command
    // motor.init();
}