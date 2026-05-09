#include "Ctrl_config.h"
#include "Arduino.h"
bool home = false;
void hometrig() {
    home = !home;
}
void setup() {
    pinMode(ENC_STEPPER_1HOME, INPUT_PULLUP);

    // Use program interrupt
    attachInterrupt(digitalPinToInterrupt(ENC_STEPPER_1HOME), hometrig, CHANGE);
    Serial.begin(9600);
}
void loop() {
    Serial.println("home: " + (String)home);
    digitalWrite(2, home);
    delay(5);
}