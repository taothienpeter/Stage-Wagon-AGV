#include "AgvFrame.h"
#include "FrameEnum.h"
SwervePin pins[2] = {{
    .stepDir = MOTOR_1_PIN_DIR,
    .stepPul = MOTOR_1_PIN_PUL,

    .encStepA = ENC_STEPPER_1A,
    .encStepB = ENC_STEPPER_1B,
    .encHome = ENC_STEPPER_1HOME,

    .MotorNum = 0,

    .SerialOdr = &Serial2,
    .SerialMonitor = &Serial,
}, {
    .stepDir = MOTOR_2_PIN_DIR,
    .stepPul = MOTOR_2_PIN_PUL,

    .encStepA = ENC_STEPPER_2A,
    .encStepB = ENC_STEPPER_2B,
    .encHome = ENC_STEPPER_2HOME,

    .MotorNum = 1,

    .SerialOdr = &Serial2,
    .SerialMonitor = &Serial,
}};
wheelPositions wheelPos[2] = {
    wheelPositions(WHEEL_POSITIONS_W/2, WHEEL_POSITIONS_B/2),
    wheelPositions(-WHEEL_POSITIONS_W/2, -WHEEL_POSITIONS_B/2)
};
Swerve_module_kinematics *swerve[2] = {new Swerve_module_kinematics(pins[0], wheelPos[0], {0x00}), 
                                       new Swerve_module_kinematics(pins[1], wheelPos[1], {0x00})};
// Swerve_module_controls *swerve[2] = {new Swerve_module_controls(pins[0]), new Swerve_module_controls(pins[1])};
// ctrlValues ref = {0.0,0.0,0.0,0.0}; // only use posTurn and velDrive

void setup(){
    // Initialize Serial first
    Serial.begin(MONITOR_BAUDRATE);
    delay(500);
    
    // Initialize encoders sequentially with delays to avoid PCNT ISR conflicts
    swerve[0]->initSwerveModule();
    swerve[1]->initSwerveModule();
    // delay(100);
    delay(100);
}
void loop(){
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      // data.trim();
      int inter = data.substring(1).toInt();
      Serial.println("Received from laptop: "+data);
      
        switch(data[0]){
            case 'n':
                ref.position = 1;
                ref.angle = 0;
                break;
            case 'w':
                ref.position = 1;
                ref.angle = -90;
                break;
            case 's':
                ref.position = -1;
                ref.angle = 0;
                break;
            case 'e':
                ref.position = 1;
                ref.angle = 90;
                break;
            case 'r':
                ref.position = 0;
                ref.angle = 0;
                swerve[0]->resetVars();
                swerve[0]->resetVars();
        }
        for (short i=0; i<2; i++){
            swerve[i]->driveSwerve(&ref);
        }
    }
    delay(5); // delta t
}