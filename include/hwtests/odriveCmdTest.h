#include "Ctrl_config.h"
#include "Arduino.h"
enum Phase { START, ARM, READY };
Phase State = START;

void setup() {
    Serial.begin(9600);   // PC
    Serial2.begin(ODRIVE_BAUD, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX); // ODrive
    Serial.println("ODrive Booting...");
    delay(500); 
    Serial2.print("sc\n"); // Clear errors
}
void loop() {
        switch(State) {
            case START:{
                Serial.println("Calibrating MOTOR ...");
                Serial2.print("w axis0.requested_state 4\n");
                Serial2.print("w axis1.requested_state 4\n");
                Serial2.print("r axis0.error\n");
                delay(5000);
                State = ARM;
                break;
            }
            case ARM:{
                Serial.println("Calibrating ENCODER ...");
                Serial2.print("w axis0.requested_state 7\n");
                Serial2.print("w axis1.requested_state 7\n");
                State = READY;
                break;
            }    
            case READY:{
                delay(10000);
                Serial2.print("w axis0.controller.input_pos 0\n"); // Zero position target
                Serial2.print("w axis1.controller.input_pos 0\n");
                Serial2.print("w axis0.requested_state 8\n"); // Arm
                Serial2.print("w axis1.requested_state 8\n");
                Serial.println("ARMED! Motor is holding.");
                Serial.println("Type your 'p' command to move.");
                break;
            }
    }
    // 2. Simple Two-Way Serial Relay
    if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();
        if (msg.length() > 0) {
            Serial2.print(msg + "\n");
        }
    }

    if (Serial2.available()) {
        Serial2.print("r axis0.error\n");
        Serial.print("ODrive: " + Serial2.readString());
    }
}