#include "AgvFrame.h"
SwervePin pinsFrontLeft = {
    .stepDir = MOTOR_1_PIN_DIR,
    .stepPul = MOTOR_1_PIN_PUL,
    // .stepEn = MOTOR_1_PIN_EN,

    .encStepA = ENC_STEPPER_1A,
    .encStepB = ENC_STEPPER_1B,
    .encHome = ENC_STEPPER_1HOME,

    .SerialOdr = &Serial2,
    .SerialMonitor = &Serial,
    // .isCW = {0x00}
};
Swerve_controls swerve(pinsFrontLeft);

enum Phase { START, ARM, READY };
Phase State = START; // phases for odrive init

void setup(){
    Serial.begin(9600);
    // swerve.initSwerve();
    // Serial.println("ODrive Booting...");
    
    // pinsFrontLeft.SerialOdr->begin(ODRIVE_BAUD, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);
    // Serial2.begin(ODRIVE_BAUD, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);
    // delay(500); 
    // Serial2.print("sc\n"); // Clear errors
    // init();
}
void loop(){
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      // data.trim();
      int inter = data.substring(1).toInt();
      Serial.println("Received from laptop: "+data);
    // if (data[0] == 'm') swerve.move(((inter*MOTOR_MICROSTEPS)/360)+111);
    // if (data[0] == 'p') stepper->moveTo(((inter*MOTOR_MICROSTEPS)/360)+111);
    // if (data[0] == 's') stepper->setSpeedInUs(inter);
    // if (data[0] == 'a') stepper->setAcceleration(inter);
    // if (data[0] == 'd') Serial.println("Position: "+ (String)stepper->getCurrentPosition() +"\nSpeed: " + (String)stepper->getSpeedInUs()+ "\nAcceleration: " + (String)stepper->getAcceleration());
    // if (data[0] == 'h') homingSeq();
    // } 
    // if (stepper && stepper->isRunning()) {
    //   Serial.println("Position: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + " \tEncoder: " + (String)stepper_1_encoder.getAngle());
    //   // if(digitalRead(14)){Serial.println("Not homing");} 
    //   // else {updateH1();};
    // } else {
    //   Serial.print("Position: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + " \tEncoder: " + (String)stepper_1_encoder.getTicks());
    //   Serial.println(" All moves completed!");
    //   while (!Serial.available()) {
    //     // Serial.println("Position: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + " \tEncoder: " + (String)stepper_1_encoder.getTicks());
    //     // Serial.println("Stepper idle");
    //     // delay(10);
    //   }
    }

    // BLDC
    if (State != READY) {
        switch(State) {
            case START:{
                Serial.println("Calibrating MOTOR ...");
                Serial2.print("w axis0.requested_state 4\n");
                Serial2.print("w axis1.requested_state 4\n");
                // Serial2.print("r axis0.error\n");
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