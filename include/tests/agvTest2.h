#include "AgvFrame.h"
#include "FrameEnum.h"
FastAccelStepperEngine engine = FastAccelStepperEngine();
SwervePin pins[2] = {{
    .stepDir = MOTOR_1_PIN_DIR,
    .stepPul = MOTOR_1_PIN_PUL,

    .encStepA = ENC_STEPPER_1A,
    .encStepB = ENC_STEPPER_1B,
    .encHome = ENC_STEPPER_1HOME,

    .MotorNum = 0,

    .SerialOdr = &Serial2,
    .SerialMonitor = &Serial,
    .stepperEngine = &engine,
}, {
    .stepDir = MOTOR_2_PIN_DIR,
    .stepPul = MOTOR_2_PIN_PUL,

    .encStepA = ENC_STEPPER_2A,
    .encStepB = ENC_STEPPER_2B,
    .encHome = ENC_STEPPER_2HOME,

    .MotorNum = 1,

    .SerialOdr = &Serial2,
    .SerialMonitor = &Serial,
    .stepperEngine = &engine,
}}; 
wheelPositions wheelPos[2] = {
    wheelPositions(WHEEL_POSITIONS_W/2, WHEEL_POSITIONS_B/2),
    wheelPositions(-WHEEL_POSITIONS_W/2, -WHEEL_POSITIONS_B/2)
};
Swerve_module_kinematics *swerve[2] = {new Swerve_module_kinematics(pins[0], wheelPos[0], {0x00}), 
                                       new Swerve_module_kinematics(pins[1], wheelPos[1], {0x00})};
pose Pose = {0, 0, 0};
void setup(){
    _initswerve();
    // Serial.println("Stalling...");
    // while(1);
}
void loop(){    
    
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      data.trim();
      int inter = data.substring(1).toInt();
      Serial.println("Received from laptop: "+data);
      
        switch(data[0]){
            case 'd':
                Pose = {1, 0};
                break;
            case 's':
                Pose = {0, -1};
                break;
            case 'a':
                Pose = {-1, 0};
                break;
            case 'w':
                Pose = {0, 1};
                break;
            case 'r':
                Pose = {0, 0, 90};
                break;
            default:
                Serial.println("Invalid command");
                break;
        }
        
    }
    // for (short i=0; i<2; i++){
    //         swerve[i]->driveSwervePose(Pose);
    // }
    Serial.println("Pose: "+ (String)Pose.x + " " + (String)Pose.y + " " + (String)Pose.theta);
    delay(5); // delta t = 0.005s
}