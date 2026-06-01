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
    .SerialMonitor = &Serial1,
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
Swerve_module_controls *swerve = new Swerve_module_controls(pins[1]);

void setup(){
    // Initialize Serial first
    Serial.begin(MONITOR_BAUDRATE);
    swerve->initSwerve();
    delay(100);
}
void loop(){
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      // data.trim();
      int inter = data.substring(1).toInt();
      Serial.println("Received from laptop: "+data);
      
        switch(data[0]){
            case 't':
                swerve->runTurnAngle(inter);
                break;
            case 'd':
                swerve->runDriveDistance(inter);
                break;
            case 'e':
                swerve->getDriveBldcPos();
                break;
            case 'r':
                swerve->resetVars();
                break;
            case 'h':
                // swerve[0]->home();
                break;
            default:
                Serial.println("Invalid command");
                break;
        }
        // for (short i=0; i<2; i++){
        //     swerve[i]->driveSwerve(&ref);
        // }
    }
}