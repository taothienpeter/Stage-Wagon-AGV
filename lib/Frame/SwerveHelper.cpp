#include "AgvFrame.h"
#include "FrameEnum.h"

extern Swerve_module_kinematics* swerve[2];

agvEr home(SwervePin pins){
    // if (homingSeq(pins, engine) == SWERVE_ERROR_NO_HOME_DETECTED) pins.SerialMonitor->println("Dir motor no home pins, next for BLDC");
    // if (engineInitialized) {
    // reset bldc parrameters
    pins.SerialMonitor->println("Calibrating MOTOR ...");
    pins.SerialOdr->print("w axis0.requested_state 4\n");
    pins.SerialOdr->print("w axis1.requested_state 4\n");
    pins.SerialOdr->print("r axis0.error\n");
    // State = ARM;
    delay(5000);
    pins.SerialMonitor->println("Calibrating ENCODER ...");
    pins.SerialOdr->print("w axis0.requested_state 7\n");
    pins.SerialOdr->print("w axis1.requested_state 7\n");
    pins.SerialOdr->print("r axis0.error\n");
    // State = READY;
    delay(10000);
    pins.SerialOdr->print("w axis0.controller.input_pos 0\n"); // Zero position target
    pins.SerialOdr->print("w axis1.controller.input_pos 0\n");
    pins.SerialOdr->print("w axis0.requested_state 8\n"); // Arm
    pins.SerialOdr->print("w axis1.requested_state 8\n");
    pins.SerialMonitor->println("ARMED! Motor is holding.");
    pins.SerialOdr->print("r axis0.error\n");
    // }
    return SWERVE_INFO_HOME;
};
agvEr _initswerve(){
    // initSwerveModule();
    swerve[0]->initSwerveModule();
    // calib swerve
    Serial.println("Homing sequence started");
    u_int16_t sT = millis(); // start time
    u_int16_t eT = 5000; // elapsed time
    calibState stepperState[2] = {stepCW, stepCW}, bldcState = bldcMotor;
    swerve[0]->calibSwerve(bldcMotor);
    swerve[1]->calibSwerve(bldcMotor);
    swerve[0]->calibSwerve(stepCW); // inlucded all the init of stepper
    swerve[1]->calibSwerve(stepCW); 
    while(eT != 0)
    {
        // if (seT < (millis() - sT) && seT != 0){
        for(int i = 0; i<2; i++){
            if(!swerve[i]->swerveCtrl->stepper->isRunning()) {
                if (stepperState[i] != stepTruehome) stepperState[i] = static_cast<calibState>(stepperState[i] + 1);
                swerve[i]->calibSwerve(stepperState[i]); 
            } 
        }
        // }
        if(eT <  (millis() - sT)){
            if (bldcState != bldcArmed) bldcState = static_cast<calibState>(bldcState + 1);
            swerve[0]->calibSwerve(bldcState);
            swerve[1]->calibSwerve(bldcState);
            if((millis() - sT)>15000) {eT = 0;}
            else if((millis() - sT)>5000){eT = 15000;}
        }
    }    
    return SWERVE_OK;
};
agvEr homingSeq(SwervePin pins, FastAccelStepper* stepper){ 
    return SWERVE_INFO_HOME;
}

// agvEr Swerve_module_controls::stepperRoughHomeSeq(){
//     pins.SerialMonitor->println("Homing sequence started");
//     stepper->forceStopAndNewPosition(0);
//     stepper->moveTo(deg2pos(180,MOTOR_MICROSTEPS));
//     // delay(4000);
//     stepper->moveTo(deg2pos(-180,MOTOR_MICROSTEPS));
//     // delay(4000);
//     if(!digitalRead(pins.encHome)) return SWERVE_ERROR_NO_HOME_DETECTED;
//     return SWERVE_INFO_HOME;
// }
// agvEr Swerve_module_controls::stepperTrueHomeSeq(){
//     enum HomingState {
//         START_SEEK,
//         FINDING_EDGE_A, // Looking for Switch ON
//         FINDING_EDGE_B, // Looking for Switch OFF
//         MOVING_TO_CENTER,
//         HOMING_COMPLETE
//     };

//     // State variables for each module
//     HomingState stateM1 = START_SEEK;

//     // Edge storage
//     long edgeA1, edgeB1;
//     bool homingActive = true; 

//     if (!homingActive) return SWERVE_ERROR_NO_HOME_DETECTED;

//     // --- MODULE 1 STATE MACHINE ---
//     switch (stateM1) {
//         case START_SEEK:
//         stepper->setSpeedInHz(2000);
//         stepper->runForward();
//         stateM1 = FINDING_EDGE_A;
//         break;

//         case FINDING_EDGE_A:
//             if (digitalRead(ENC_STEPPER_1HOME) == LOW) {
//                 edgeA1 = Step_enc->getTicks();
//                 stateM1 = FINDING_EDGE_B;
//                 Serial.println("M1: Edge A Found");
//             }
//             break;

//         case FINDING_EDGE_B:
//             if (digitalRead(ENC_STEPPER_1HOME) == HIGH) {
//                 edgeB1 = Step_enc->getTicks();
//                 stepper->stopMove();
//                 stateM1 = MOVING_TO_CENTER;
//                 Serial.println("M1: Edge B Found");
//             }
//             break;

//         case MOVING_TO_CENTER:
//             if (!stepper->isRunning()) {
//                 long center = (edgeA1 + edgeB1) / 2;
//                 stepper->setCurrentPosition(Step_enc->getTicks());
//                 stepper->setSpeedInHz(1000);
//                 stepper->moveTo(center);
//                 stateM1 = HOMING_COMPLETE;
//             }
//             break;

//         case HOMING_COMPLETE:
//             if (!stepper->isRunning()) {
//                 // Final Zeroing
//                 // Step_enc->();
//                 stepper->setCurrentPosition(0);
//                 stepper->setSpeedInUs(50);
//                 Serial.println("M1: Homing Finished");
//             }
//             break;
//     };
//     // Check if overall homing is finished
//     // if (stateM1 == HOMING_COMPLETE && stateM2 == HOMING_COMPLETE && 
//     //     !stepper1->isRunning() && !stepper2->isRunning()) {
//     //     homingActive = false;
//     //     Serial.println("ALL SYSTEMS ZEROED AND READY");
//     // }
//     // }
//     return SWERVE_INFO_HOME;
// }
