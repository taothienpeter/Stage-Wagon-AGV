#include "Ctrl_config.h"
// #include "Arduino.h"

const int pinA = 32;
const int pinB = 33;

volatile long position = 0;

void printPosition() {
    Serial.println("Position: " + (String)position);
}

void triggerA() {
    if (digitalRead(pinB) == digitalRead(pinA)) {
        position++;
    } else {
        position--;
    }
}
void triggerB() {
    if (digitalRead(pinA)!=digitalRead(pinB)) {
        position++;
    } else {
        position--;
    }
}

void setup() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    // Use program interrupt
    attachInterrupt(digitalPinToInterrupt(pinA), triggerA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), triggerB, CHANGE);
    Serial.begin(9600);
}
void loop() {
    printPosition();
    delay(100);
}