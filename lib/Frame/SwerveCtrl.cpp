#include "AgvFrame.h"

// Static member initialization
FastAccelStepperEngine Swerve_module_controls::engine;

Swerve_module_controls::Swerve_module_controls(const SwervePin pins, isClockWise isCW): pins(pins){
    // Defer encoder creation to initSwerve to prevent multiple PCNT ISR installations
    stepper = NULL;
    Step_enc = NULL;
    _motorNum = pins.MotorNum;
}

#ifdef MOTOR_USES_PIN_ENABLE
Swerve_module_controls::Swerve_module_controls(uint8_t pinStep_Dir, uint8_t pinStep_Pul, uint8_t pinStep_En){
    FastAccelStepperEngine engine = FastAccelStepperEngine();
    FastAccelStepper *stepper = NULL;
    // initStepper();
}
#else
#endif
void Swerve_module_controls::home(){
    if (this->homingSeq() == SWERVE_ERROR_NO_HOME_DETECTED) pins.SerialMonitor->println("Dir motor no home pins, next for BLDC");
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
    pins.SerialMonitor->println("Type your 'p' command to move.");
    pins.SerialOdr->print("r axis0.error\n");
}

agvEr Swerve_module_controls::initSwerve(short spd, short acc){
    static bool engineInitialized = false;
    
    pins.SerialMonitor->begin(MONITOR_BAUDRATE);
    pins.SerialOdr->begin(ODRIVE_BAUD, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);
    
    // Initialize engine only once globally
    if (!engineInitialized) {
        Swerve_module_controls::engine.init();
        engineInitialized = true;
        delay(100);
    }
    
    // Create stepper if not already created
    if (stepper == NULL) {
        stepper = Swerve_module_controls::engine.stepperConnectToPin(pins.stepPul);
        stepper->setDirectionPin(pins.stepDir);
    }
    
    // Create encoder if not already created (happens only once per module)
    if (Step_enc == NULL) {
        Step_enc = new Encoder(pins.encStepA, pins.encStepB, false);
        delay(50);  // Allow PCNT service to stabilize
    }
    
    #ifdef MOTOR_USES_PIN_ENABLE
    stepper->setEnablePin(pins.stepEn); 
    #else 

    #endif
    stepper->setAutoEnable(true);
    stepper->setSpeedInUs(spd);
    stepper->setAcceleration(acc);
    Step_enc->reset();
    
    this->initInterrupts();
    this->home();
    // this->bldc_clcEr();
    return SWERVE_INFO_INIT;
}
agvEr Swerve_module_controls::resetVars(bool needHome){ 
    needHome?this->home():stepper->forceStopAndNewPosition(0);
    this->bldc_clcEr();
    this->setDriveAbsolutePos(0);
    Step_enc->reset();
    return SWERVE_INFO_RESET;
}
agvEr Swerve_module_controls::homingSeq(){ 
    pins.SerialMonitor->println("Homing sequence started");
    stepper->forceStopAndNewPosition(0);
    // scan for homing position
    while (digitalRead(pins.encHome)&&pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)<181) {
        stepper->move(deg2pos(1,MOTOR_MICROSTEPS));  // Move forward 180 degs
        if(stepper->getCurrentPosition()>0) {stepper->forceStopAndNewPosition(deg2pos(360,MOTOR_MICROSTEPS)); stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));}
        else if (stepper->getCurrentPosition()<0) {stepper->forceStopAndNewPosition(0);}
        else {stepper->setCurrentPosition(deg2pos(0,MOTOR_MICROSTEPS));};
        delay(5);  // Wait for move to complete or be stopped
    }
    while (digitalRead(pins.encHome)&&pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)>-181) {
        stepper->move(deg2pos(-1,MOTOR_MICROSTEPS));  // Then move backward 360 degs
        if(stepper->getCurrentPosition()>0) {stepper->forceStopAndNewPosition(deg2pos(360,MOTOR_MICROSTEPS)); stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));}
        else if (stepper->getCurrentPosition()<0) {stepper->forceStopAndNewPosition(0);}
        else {stepper->setCurrentPosition(deg2pos(0,MOTOR_MICROSTEPS));};
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
    // Serial.println("Position: " + (String)(calib));
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

