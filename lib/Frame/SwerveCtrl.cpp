#include "AgvFrame.h"

Swerve_controls::Swerve_controls(const SwervePin& pins, isClockWise isCW): pins(pins){
    engine = FastAccelStepperEngine();
    stepper = engine.stepperConnectToPin(pins.stepPul);
    Step_enc = new Encoder(pins.encStepA, pins.encStepB, false);
    _motorNum = pins.MotorNum; // for Odrive target selection
}

#ifdef MOTOR_USES_PIN_ENABLE
Swerve_controls::Swerve_controls(uint8_t pinStep_Dir, uint8_t pinStep_Pul, uint8_t pinStep_En){
    FastAccelStepperEngine engine = FastAccelStepperEngine();
    FastAccelStepper *stepper = NULL;
    // initStepper();
}
#else
// Swerve_controls::initPin();
// Swerve_controls::Swerve_controls(uint8_t pinStep_Dir, uint8_t pinStep_Pul){
//     FastAccelStepperEngine engine = FastAccelStepperEngine();
//     FastAccelStepper *stepper = NULL;
//     // initStepper();
// }
#endif
void Swerve_controls::home(){
    this->homingSeq();
    // reset bldc parrameters
    enum Phase { START, ARM, READY };
    Phase State = START; // phases for odrive init
    do {switch(State) {
            case START:{
                pins.SerialMonitor->println("Calibrating MOTOR ...");
                pins.SerialOdr->print("w axis0.requested_state 4\n");
                pins.SerialOdr->print("w axis1.requested_state 4\n");
                // Serial2.print("r axis0.error\n");
                State = ARM;
                delay(5000);
                break;
            }
            case ARM:{
                pins.SerialMonitor->println("Calibrating ENCODER ...");
                pins.SerialOdr->print("w axis0.requested_state 7\n");
                pins.SerialOdr->print("w axis1.requested_state 7\n");
                State = READY;
                delay(8000);
                break;
            }    
            case READY:{
                pins.SerialOdr->print("w axis0.controller.input_pos 0\n"); // Zero position target
                pins.SerialOdr->print("w axis1.controller.input_pos 0\n");
                pins.SerialOdr->print("w axis0.requested_state 8\n"); // Arm
                pins.SerialOdr->print("w axis1.requested_state 8\n");
                pins.SerialMonitor->println("ARMED! Motor is holding.");
                // pins.SerialMonitor->println("Type your 'p' command to move.");
                break;
            }
        }
    } while(State != READY);
}

agvEr Swerve_controls::initSwerve(short spd, short acc){
    engine.init();
    stepper = engine.stepperConnectToPin(pins.stepPul);
    stepper->setDirectionPin(pins.stepDir);
    #ifdef MOTOR_USES_PIN_ENABLE
    stepper->setEnablePin(pins.stepEn); 
    #else 

    #endif
    stepper->setAutoEnable(true);
    
    stepper->setSpeedInUs(spd);
    stepper->setAcceleration(acc);
    
    Step_enc->reset();
    this->initInterrupts();
    delay(500); // optional
    this->home();
    this->bldc_clcEr();
    return SWERVE_INFO_INIT;
}
agvEr Swerve_controls::resetVars(bool needHome){ 
    needHome?this->home():stepper->forceStopAndNewPosition(0);
    this->bldc_clcEr();
    this->setDriveAbsolutePos(0);
    Step_enc->reset();
    return SWERVE_INFO_RESET;
}
agvEr Swerve_controls::homingSeq(){ 
    pins.SerialMonitor->println("Homing sequence started");
    stepper->forceStopAndNewPosition(0);
    // scan for homing position
    while (digitalRead(pins.encHome)&&pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)<181) {
        stepper->move(deg2pos(1,MOTOR_MICROSTEPS));  // Move forward 180 degs
        delay(5);  // Wait for move to complete or be stopped
    }
    while (digitalRead(pins.encHome)&&pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)>-181) {
        stepper->move(deg2pos(-1,MOTOR_MICROSTEPS));  // Then move backward 360 degs
        delay(5);  // Wait for move to complete or be stopped
    }
    if(!digitalRead(pins.encHome)) return SWERVE_ERROR_NO_HOME_DETECTED;
    // get true home position
    short calib = stepper->getCurrentPosition();
    while (digitalRead(pins.encHome)) {
        if(stepper->getCurrentPosition()>0){stepper->move(-1);}
        else{stepper->move(1);}
        delay(10);
    }
    calib -= stepper->getCurrentPosition();
    Serial.println("Position: " + (String)(calib));
    stepper->move(-calib/2);
    delay(10);
    // move to start position
    if(stepper->getCurrentPosition()>0) {
        stepper->forceStopAndNewPosition(deg2pos(180,MOTOR_MICROSTEPS)); 
        stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));
    }
    else if (stepper->getCurrentPosition()<0) {
        stepper->forceStopAndNewPosition(deg2pos(-180,MOTOR_MICROSTEPS)); 
        stepper->forceStopAndNewPosition(0);
    }
    else {stepper->setCurrentPosition(deg2pos(0,MOTOR_MICROSTEPS));}
    // set home 
    stepper->forceStopAndNewPosition(0);
    return SWERVE_INFO_HOME;
}
void Swerve_controls::bldc_ReBoot(){ 
    pins.SerialOdr->print("sr\n");
}
void Swerve_controls::bldc_clcEr(){
    pins.SerialOdr->print("sc\n"); // Clear errors
}
agvEr Swerve_controls::bldc_prtEr(){
    if (!pins.SerialOdr->available()) return SWERVE_ERROR_NO_ODRIVE_FOUND;
    pins.SerialOdr->print("r axis"+(String)_motorNum+".error\n");
    pins.SerialMonitor->print("ODrive: " + pins.SerialOdr->readString());
    return SWERVE_ERROR_FEEDBACK_OK;
}
agvEr Swerve_controls::bldc_sendCmd(String msg){
    if (!pins.SerialMonitor->available()) return AGV_ERROR_NO_SERIALMONITOR_FOUND;
    if (!pins.SerialOdr->available()) return SWERVE_ERROR_NO_ODRIVE_FOUND;

    if(msg == "serialMonitor"){
        msg = pins.SerialMonitor->readStringUntil('\n');
        msg.trim();
        if (msg.length() > 0) {
            pins.SerialOdr->print(msg + "\n");
        }
        return this->bldc_prtEr()==SWERVE_ERROR_FEEDBACK_OK? SWERVE_OK : SWERVE_ERROR_NO_ODRIVE_FOUND;
    }

    pins.SerialOdr->print(msg + "\n");
    return SWERVE_OK;
}
agvEr Swerve_controls::runTurnAngle(double angle){ 
    // if (!this->checkStepSkiping()) return SWERVE_ERROR_STEP_MISSMATCH;
    stepper->moveTo(deg2pos(angle, MOTOR_MICROSTEPS));
    if (!this->checkStepSkiping()) return SWERVE_ERROR_STEP_MISSMATCH;
    return SWERVE_OK;
}
agvEr Swerve_controls::setTurnSpeed(double speedHz){ 
    stepper->setSpeedInHz(speedHz);
    return SWERVE_OK;
}
agvEr Swerve_controls::setTurnAccel(int16_t step_s_s){
    stepper->setAcceleration(step_s_s);
    return SWERVE_OK;
}
agvEr Swerve_controls::runDriveSpeed(double turns){
    // if (this->bldc_prtEr() != SWERVE_OK) return this->bldc_prtEr();
    return this->bldc_sendCmd("v "+(String)_motorNum+" "+(String)turns);
}
agvEr Swerve_controls::setDriveAccel(int16_t inertia){
    return this->bldc_sendCmd("w axis"+(String)_motorNum+".controller.config.inertia = "+ (String)inertia);
}
agvEr Swerve_controls::setDriveAbsolutePos(int16_t pos){
    // if (this->bldc_prtEr() != SWERVE_OK) return this->bldc_prtEr();
    return this->bldc_sendCmd("es " + (String)_motorNum + " " + (String)pos);
}
agvEr Swerve_controls::runDriveDistance(double turns){
    return this->bldc_sendCmd("p "+ (String)_motorNum +" "+ (String)turns);
}
agvEr Swerve_controls::setDriveTorque(uint8_t torque){
    return this->bldc_sendCmd("c " + (String)_motorNum + " " + (String)torque);
}
short Swerve_controls::getDirectionEncoderPos(unit u){
    return u==degree? Step_enc->getAngle() : Step_enc->getTicks();
}
double Swerve_controls::getDirectionEncoderPos_UnitOne(){
    return this->getDirectionEncoderPos(degree)/360;
}
// double Swerve_controls::getDirectionEncoderVelo(float dt, unit u){
//     if(u==radian) return Step_enc->getAngle()*DEG_TO_RAD / dt;
//     return u==degree? Step_enc->getAngle() / dt : Step_enc->getTicks() / dt;
// }
// double Swerve_controls::getDirectionEncoderVelo_UnitOne(float dt){
//     return this->getDirectionEncoderPos_UnitOne() / dt;
// }
//   Get direction position
long Swerve_controls::getDirectionPosiotion(unit u){
    return u==degree? pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS):  stepper->getCurrentPosition();
}
double Swerve_controls::getDirectionPosiotion_UnitOne(){ 
    return this->getDirectionPosiotion(degree)/360;
}
double Swerve_controls::getDirectionVelocity(unit u){
    return u==degree? pos2deg(stepper->getSpeedInTicks(),MOTOR_MICROSTEPS) : stepper->getSpeedInTicks();
}
double Swerve_controls::getDirectionVelocity_UnitOne(){ 
    return this->getDirectionVelocity(degree)/360;   
}
bool Swerve_controls::checkStepSkiping(){
    return (abs(this->getDirectionEncoderPos_UnitOne()/this->getDirectionPosiotion_UnitOne())<0.005)?true: false;
}
float Swerve_controls::getWheelPosition(){
    if(this->bldc_sendCmd("r "+(String)_motorNum+".controller.input_pos")!=SWERVE_OK) return 0;
    return this->parseWheelVar(pins.SerialOdr->readString());
}
float Swerve_controls::getWheelVelocity(){
    if(this->bldc_sendCmd("r "+(String)_motorNum+".controller.input_vel")!=SWERVE_OK) return 0;
    return this->parseWheelVar(pins.SerialOdr->readString());
}
float Swerve_controls::getWheelTorque(){
    if(this->bldc_sendCmd("r "+(String)_motorNum+".controller.input_torque")!=SWERVE_OK) return 0;
    return this->parseWheelVar(pins.SerialOdr->readString());
}
float Swerve_controls::getWheelEncoderVariables(){
    
    pins.SerialOdr->print("r axis"+(String)_motorNum+".encoder.pos_estimate\n");
    return this->parseWheelVar(pins.SerialOdr->readString());
} 
float Swerve_controls::parseWheelVar(String s){
    s = pins.SerialOdr->readString();
    s.trim();
    return s.toFloat();
}
agvEr Swerve_controls::checkWheelEncoderInfos(){
    String cmd[3] = {"error", "encoder.config.phase_offset", "encoder.config.direction"};
    for(short i=0; i<3; i++){
        agvEr agver = this->bldc_sendCmd("r "+(String)_motorNum+"."+cmd[i]);
        if(agver!=SWERVE_OK) return agver;
    };
    return SWERVE_OK;
}

boolean Swerve_controls::printPosStats(){ // set option for printing the right amount of data
    pins.SerialMonitor->print(
        "Stepper motor: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + "\tdegrees\t\n" +
        "Direction Encoder: " + (String)Step_enc->getAngle() + "\tdegrees\t\n" +
        "BLDC motor: "  + (String) + "\tmeters\t\n" +
        "Drive Encoder: " + (String) + "\tticks\t\n" +
        "=================================================================="
    );
    return 0;
}
boolean Swerve_controls::printStatus(String msg){
    #ifdef SERIAL_DEBUG
        pins.SerialMonitor->println(msg);
        return 0;
    #endif
    return 1;
}

