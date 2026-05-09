#include "AgvFrame.h"

Swerve_module_controls::Swerve_module_controls(const SwervePin pins, isClockWise isCW): pins(pins){
    // Defer encoder creation to initSwerve to prevent multiple PCNT ISR installations
    stepper = NULL;
    Step_enc = NULL;
    _motorNum = pins.MotorNum;
}
agvEr Swerve_module_controls::calibStepper(calibState state){   
    switch (state){
        case stepCW:  
        // Create encoder if not already created (happens only once per module)
            Step_enc = new Encoder(pins.encStepA, pins.encStepB, false);
            Step_enc->reset();
            // pins.SerialMonitor->println("init encoder" + (String)_motorNum);
            this->initInterrupts();
            stepper = pins.stepperEngine->stepperConnectToPin(pins.stepPul);
            stepper->setDirectionPin(pins.stepDir);
            stepper->setAutoEnable(true);
            stepper->setSpeedInUs(20);
            stepper->setAcceleration(20000);
        // find home first
            isHome = false;
        // stepper moves 180 and -180 to find home automatically
            stepper->forceStopAndNewPosition(0);
            stepper->moveTo(deg2pos(180,MOTOR_MICROSTEPS));
            pins.SerialMonitor->println("Calibrating STEPPER ...");
            break;
        case stepCCW:
            if(isHome)Serial.print("home found");
            if(!isHome) stepper->moveTo(deg2pos(-180,MOTOR_MICROSTEPS));
            break;
        case stepTruehome:
            if(isHome)Serial.print("home found");
            if(stepper->getCurrentPosition()>180) {stepper->forceStopAndNewPosition(deg2pos(180+3,MOTOR_MICROSTEPS)); stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));}
            else if (stepper->getCurrentPosition()<-180) {stepper->forceStopAndNewPosition(0);}
            else {stepper->setCurrentPosition(deg2pos(0,MOTOR_MICROSTEPS));}
            break;
    }
    return SWERVE_INFO_HOME;
}
agvEr Swerve_module_controls::calibBLDC(calibState state){
    switch(state){
    case bldcMotor:
    // BLDC calib
        pins.SerialMonitor->println("Calibrating BLDC MOTOR ...");
        pins.SerialOdr->print("w axis" + (String)_motorNum +".requested_state 4\n");
        // pins.SerialOdr->print("w axis1.requested_state 4\n");
        pins.SerialOdr->print("r axis"+ (String)_motorNum +".error\n");
        // State = ARM;
        break;
    case bldcEncoder:
        pins.SerialMonitor->println("Calibrating BLDC ENCODER ...");
        pins.SerialOdr->print("w axis"+ (String)_motorNum +".requested_state 7\n");
        // pins.SerialOdr->print("w axis1.requested_state 7\n");
        pins.SerialOdr->print("r axis"+ (String)_motorNum +".error\n");
        // State = READY;
        break;
    case bldcArmed:
        pins.SerialOdr->print("w axis"+ (String)_motorNum +".controller.input_pos 0\n"); // Zero position target
        // pins.SerialOdr->print("w axis1.controller.input_pos 0\n");
        pins.SerialOdr->print("w axis"+ (String)_motorNum +".requested_state 8\n"); // Arm
        // pins.SerialOdr->print("w axis1.requested_state 8\n");
        pins.SerialOdr->print("r axis"+ (String)_motorNum +".error\n");
        pins.SerialMonitor->println("ARMED! Motor is holding.");
        break;
    }
    return SWERVE_OK;
}
agvEr Swerve_module_controls::initSwerve(){
    pins.SerialMonitor->begin(MONITOR_BAUDRATE);
    pins.SerialOdr->begin(ODRIVE_BAUD, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);
    pins.stepperEngine->init();
    pins.SerialMonitor->println("init swerve");
    
    
    return SWERVE_INFO_INIT;
}
agvEr Swerve_module_controls::homingSeq(){
    if(stepper->getCurrentPosition()>180) {
       stepper->forceStopAndNewPosition(deg2pos(360,MOTOR_MICROSTEPS)); 
       stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));}
    else if (stepper->getCurrentPosition()<180) {
        stepper->forceStopAndNewPosition(0);
    }
    else {stepper->setCurrentPosition(deg2pos(0,MOTOR_MICROSTEPS));};
    return SWERVE_OK;
}
agvEr Swerve_module_controls::resetVars(bool needHome){ 
    // if(needHome) {this->home();}else{ stepper->forceStopAndNewPosition(0);};
    this->bldc_clcEr();
    this->setDriveAbsolutePos(0);
    Step_enc->reset();
    return SWERVE_INFO_RESET;
}
void Swerve_module_controls::bldc_ReBoot(){ 
    pins.SerialOdr->print("sr\n");
}
void Swerve_module_controls::bldc_clcEr(){
    pins.SerialOdr->print("sc\n"); // Clear errors
}
agvEr Swerve_module_controls::bldc_prtEr(){
    if (!pins.SerialOdr->available()) return SWERVE_ERROR_NO_ODRIVE_FOUND;
    pins.SerialOdr->print("r axis"+(String)_motorNum+".error\n");
    pins.SerialMonitor->print("ODrive: " + pins.SerialOdr->readString());
    return SWERVE_ERROR_FEEDBACK_OK;
}
agvEr Swerve_module_controls::bldc_sendCmd(String msg){
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
// call this function once to go to the desired position
agvEr Swerve_module_controls::runTurnAngle(double angle){ 
    // if (!this->checkStepSkiping()) return SWERVE_ERROR_STEP_MISSMATCH;
    stepper->moveTo(deg2pos(angle, MOTOR_MICROSTEPS));
    if (!this->checkStepSkiping()) return SWERVE_ERROR_STEP_MISSMATCH;
    return SWERVE_OK;
}
agvEr Swerve_module_controls::setTurnSpeed(double speed, unit u){ 
    switch (u) {
        case Hz:
            stepper->setSpeedInHz(speed);
            break;
        case Us:   
            stepper->setSpeedInUs(speed);
            break;
        case tick:
            stepper->setSpeedInTicks(speed);
            break;
        default:
            stepper->setSpeedInUs(speed);
            break;
    };
    return SWERVE_OK;
}
agvEr Swerve_module_controls::setTurnAccel(int16_t step_s_s){
    stepper->setAcceleration(step_s_s);
    return SWERVE_OK;
}
agvEr Swerve_module_controls::runDriveSpeed(double turns){
    // if (this->bldc_prtEr() != SWERVE_OK) return this->bldc_prtEr();
    return this->bldc_sendCmd("v "+(String)_motorNum+" "+(String)turns);
}
agvEr Swerve_module_controls::setDriveAccel(int16_t inertia){
    return this->bldc_sendCmd("w axis"+(String)_motorNum+".controller.config.inertia = "+ (String)inertia);
}
agvEr Swerve_module_controls::setDriveAbsolutePos(int16_t pos){
    // if (this->bldc_prtEr() != SWERVE_OK) return this->bldc_prtEr();
    return this->bldc_sendCmd("es " + (String)_motorNum + " " + (String)pos);
}
agvEr Swerve_module_controls::runDriveDistance(double turns){
    return this->bldc_sendCmd("p "+ (String)_motorNum +" "+ (String)turns + " 0 0");
}
agvEr Swerve_module_controls::setDriveTorque(uint8_t torque){
    return this->bldc_sendCmd("c " + (String)_motorNum + " " + (String)torque + " 0 )");
}
double Swerve_module_controls::getDirectionEncoderPos(unit u){
    return u==degree? Step_enc->getAngle() : Step_enc->getTicks();
}
#ifndef temp
double Swerve_module_controls::getDirectionEncoderPos_UnitOne(){
    return this->getDirectionEncoderPos(degree)/360;ref.velx * sin(pose.theta) + ref.vely * cos(pose.theta)
}
#endif
#ifndef temp
double Swerve_module_controls::getDirectionEncoderVelo(float dt, unit u){
    // if(u==radian) return Step_enc->getAngle()*DEG_TO_RAD / dt;
    // return u==degree? Step_enc->getAngle() / dt : Step_enc->getTicks() / dt;
    pins.SerialMonitor->print("Sorry this function is not implemented yet");
    return 0.0;
}
#endif
#ifndef temp
double Swerve_module_controls::getDirectionEncoderVelo_UnitOne(float dt){
    // return this->getDirectionEncoderPos_UnitOne() / dt;
    pins.SerialMonitor->print("Sorry this function is not implemented yet");
    return 0.0;
}
#endif
//   Get direction position
long Swerve_module_controls::getDirectionPosition(unit u){
    return u==degree? pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS):  stepper->getCurrentPosition();
}
#ifndef temp
double Swerve_module_controls::getDirectionPosition_UnitOne(){ 
    return this->getDirectionPosition(degree)/360;
}
#endif
double Swerve_module_controls::getDirectionVelocity(unit u){
    return u==degree? pos2deg(stepper->getSpeedInTicks(),MOTOR_MICROSTEPS) : stepper->getSpeedInTicks();
}
#ifndef temp
double Swerve_module_controls::getDirectionVelocity_UnitOne(){ 
    return this->getDirectionVelocity(degree)/360;   
}
#endif
bool Swerve_module_controls::checkStepSkiping(){
    return (abs(this->getDirectionEncoderPos(degree)/this->getDirectionPosition(degree))<0.005)?true: false;
}
float Swerve_module_controls::getWheelPosition(){
    if(this->bldc_sendCmd("r "+(String)_motorNum+".controller.input_pos")!=SWERVE_OK) return 0;
    return this->parseWheelVar(pins.SerialOdr->readString());
}
float Swerve_module_controls::getWheelVelocity(){
    if(this->bldc_sendCmd("r "+(String)_motorNum+".controller.input_vel")!=SWERVE_OK) return 0;
    return this->parseWheelVar(pins.SerialOdr->readString());
}
#ifndef temp
float Swerve_module_controls::getWheelTorque(){
    if(this->bldc_sendCmd("r "+(String)_motorNum+".controller.input_torque")!=SWERVE_OK) return 0;
    return this->parseWheelVar(pins.SerialOdr->readString());
}
#endif
float Swerve_module_controls::getWheelEncoderVariables(){
    
    pins.SerialOdr->print("r axis"+(String)_motorNum+".encoder.pos_estimate\n");
    return this->parseWheelVar(pins.SerialOdr->readString());
} 
float Swerve_module_controls::parseWheelVar(String s){
    s = pins.SerialOdr->readString();
    s.trim();
    return s.toFloat();
}
agvEr Swerve_module_controls::checkWheelEncoderInfos(){
    String cmd[3] = {"error", "encoder.config.phase_offset", "encoder.config.direction"};
    for(short i=0; i<3; i++){
        agvEr agver = this->bldc_sendCmd("r "+(String)_motorNum+"."+cmd[i]);
        if(agver!=SWERVE_OK) return agver;
    };
    return SWERVE_OK;
}

bool Swerve_module_controls::printPosStats(){ // set option for printing the right amount of data
    pins.SerialMonitor->print(
        "Stepper motor: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + "\tdegrees\t\n" +
        "Direction Encoder: " + (String)Step_enc->getAngle() + "\tdegrees\t\n" +
        "BLDC motor: "  + (String) + "\tmeters\t\n" +
        "Drive Encoder: " + (String) + "\tticks\t\n" +
        "=================================================================="
    );
    return 0;
}
bool Swerve_module_controls::printStatus(String msg){
    #ifdef SERIAL_DEBUG
        pins.SerialMonitor->println(msg);
        return 0;
    #endif
    return 1;
}

