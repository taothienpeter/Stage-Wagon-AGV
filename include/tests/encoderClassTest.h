#include "Encoder.h"
#include "Ctrl_config.h"

Encoder encoder(ENC_TEST_PINA, ENC_TEST_PINB, 0, false);
// void ARDUINO_ISR_ATTR triggerA() { encoder.triggerA(); }
// void ARDUINO_ISR_ATTR triggerB() { encoder.triggerB(); }

void setup() {
    encoder.reset();
    // attachInterrupt(digitalPinToInterrupt(pinA), triggerA, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(pinB), triggerB, CHANGE);
    Serial.begin(9600);
}

void loop() {
    Serial.println("Position " + (String)encoder.getAngle()+" Ticks " + (String)encoder.getTicks());
    delay(10);
}