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
vel Vel = {0, 0, 0};
void setup(){
    // _initswerve();
    swerve[0]->initSwerveModule();
    swerve[1]->initSwerveModule();
}
void loop(){    
    
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      data.trim();
    //   Serial.println(data);
        _cmdParse(data);
        // Serial.println("Receive data:"+(String)Vel.velx + "\t" + (String)Vel.vely + "\t" + (String)Vel.omega);
        for (short i=0; i<2; i++){
            swerve[i]->driveSwerveVel(Vel);
        }
    }
    _logData();
    // Serial.println("Pose: "+ (String)Pose.x + " " + (String)Pose.y + " " + (String)Pose.theta);
    delay(50); // delta t = 0.005s
}