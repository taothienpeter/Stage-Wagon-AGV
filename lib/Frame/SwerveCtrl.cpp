#include "AgvFrame.h"

Swerve_module_controls::Swerve_module_controls(const SwervePin pins, isClockWise isCW): pins(pins){
    // Defer encoder creation to initSwerve to prevent multiple PCNT ISR installations
    pins.SerialMonitor->begin(MONITOR_BAUDRATE);
    pins.SerialOdr->begin(ODRIVE_BAUD, SERIAL_8N1, ODRIVE_RX, ODRIVE_TX);
    stepper = NULL;
    Step_enc = NULL;
    _motorNum = pins.MotorNum;
    settings = {CONFIG_STEPPER_UNIT, CONFIG_BLDC_UNIT, CONFIG_STEPPER_SPEED, CONFIG_STEPPER_ACCEL, 
        CONFIG_BLDC_SPEED, CONFIG_BLDC_ACCEL_MAX, CONFIG_BLDC_ACCEL_MIN, isCW}; // preset controller
    pins.stepperEngine->init();
}
agvEr Swerve_module_controls::initSwerve(){
    stepper = pins.stepperEngine->stepperConnectToPin(pins.stepPul);
    stepper->setDirectionPin(pins.stepDir);
    stepper->setAutoEnable(true);
    stepper->setSpeedInUs(settings.stepperSpeed);
    stepper->setAcceleration(settings.stepperAccel);
    // find home first
    isHome = false;
    stepper->forceStopAndNewPosition(0);
    return SWERVE_INFO_INIT;
}
agvEr Swerve_module_controls::calibStepper(calibState state){   
    switch (state){
        case stepCW:  
        // Create encoder if not already created (happens only once per module)
            Step_enc = new Encoder(pins.encStepA, pins.encStepB, false);
            Step_enc->reset();
            this->initInterrupts();
            this->initSwerve();
            stepper->moveTo(deg2pos(180,MOTOR_MICROSTEPS));
            pins.SerialMonitor->println("Calibrating STEPPER ...");
            // stepper moves 180 and -180 to find home automatically
            break;
        case stepCCW:
            if(isHome)Serial.print("home found");
            if(!isHome) stepper->moveTo(deg2pos(-180,MOTOR_MICROSTEPS));
            break;
        case stepTruehome:
            if(isHome)Serial.print("home found");
            if(stepper->getCurrentPosition()>180) {stepper->forceStopAndNewPosition(deg2pos(180+3,MOTOR_MICROSTEPS)); stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));}
            else if (stepper->getCurrentPosition()<-180) {stepper->forceStopAndNewPosition(deg2pos(-3,MOTOR_MICROSTEPS)); stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));}
            else {stepper->setCurrentPosition(deg2pos(0,MOTOR_MICROSTEPS));}
            break;
    }
    return SWERVE_INFO_HOME;
}
agvEr Swerve_module_controls::calibBLDC(calibState state){
    switch(state){
    case bldcMotor:
        pins.SerialMonitor->println("Calibrating BLDC MOTOR ...");
        pins.SerialOdr->print("w axis" + (String)_motorNum +".requested_state 4\n");
        pins.SerialOdr->print("r axis"+ (String)_motorNum +".error\n");
        break;
    case bldcEncoder:
        pins.SerialMonitor->println("Calibrating BLDC ENCODER ...");
        pins.SerialOdr->print("w axis"+ (String)_motorNum +".requested_state 7\n");
        pins.SerialOdr->print("r axis"+ (String)_motorNum +".error\n");
        break;
    case bldcArmed:
        // pins.SerialOdr->print("w axis1.requested_state 8\n");
        pins.SerialOdr->print("w axis"+ (String)_motorNum +".controller.input_pos 0\n"); // Zero position target
        pins.SerialOdr->print("w axis"+ (String)_motorNum +".requested_state 8\n"); // Arm
        pins.SerialOdr->print("r axis"+ (String)_motorNum +".error\n");
        pins.SerialMonitor->println("ARMED! Motor is holding.");
        break;
    }
    return SWERVE_OK;
}
agvEr Swerve_module_controls::resetVars(bool needHome){ 
    this->bldc_clcEr();
    this->setDriveAbsolutePos(0);
    Step_enc->reset();
    return SWERVE_INFO_RESET;
}
agvEr Swerve_module_controls::ctrlSS(wheelState setState){
    // input speed and angle are m/s and rad 
    if (fabs(setState.speed) < JITTER_PERCENTAGE) {
        setState.speed = 0;
        return SWERVE_OK; // skip to prevent jitter
    }
    // optimize turn angle
    double delta = wrapAngle(setState.angle - (this->getTurnStepperPos()*DEG_TO_RAD));
    if (delta > HALF_PI) {
        setState.angle -= PI;
        setState.speed *= -1;
    }else if(delta < -HALF_PI){
        setState.angle += PI;
        setState.speed *= -1;
    }
    setState.angle = wrapAngle(setState.angle);
    // setState.speed *= cos(delta);
    this->runTurnAngle(setState.angle*RAD_TO_DEG);
    this->runDriveSpeed(setState.speed);
    pins.SerialMonitor->println("driving:"+ (String)setState.speed+"\t"+(String)(setState.angle*RAD_TO_DEG));
    return SWERVE_OK;
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
    return SWERVE_OK;
}
agvEr Swerve_module_controls::setTurnSpeed(double speed){ 
    switch (settings.unitTurn) {
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
agvEr Swerve_module_controls::setTurnAccel(int16_t accel){
    stepper->setAcceleration(accel);
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
double Swerve_module_controls::getTurnEncoderPos(){
    return Step_enc->getAngle();
}
double Swerve_module_controls::getTurnStepperPos(){
    return pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS);
}
// double Swerve_module_controls::getDirectionStepperVel(unit u){
//     return u==degree? pos2deg(stepper->getSpeedInTicks(),MOTOR_MICROSTEPS) : stepper->getSpeedInTicks();
// }

float Swerve_module_controls::getDriveBldcPos(){
    if(this->bldc_sendCmd("r "+(String)_motorNum+".controller.input_pos")!=SWERVE_OK) return 0;
    return this->parseWheelVar(pins.SerialOdr->readString());
}
float Swerve_module_controls::getDriveBldcVel(){
    if(this->bldc_sendCmd("r "+(String)_motorNum+".controller.input_vel")!=SWERVE_OK) return 0;
    return this->parseWheelVar(pins.SerialOdr->readString());
}
float Swerve_module_controls::getDriveEncoderVariables(){
    pins.SerialOdr->print("r axis"+(String)_motorNum+".encoder.pos_estimate\n");
    return this->parseWheelVar(pins.SerialOdr->readString());
} 
bool Swerve_module_controls::checkStepSkiping(){
    return (fabs(this->getTurnEncoderPos()/this->getTurnStepperPos())<0.005)?true: false;
}
float Swerve_module_controls::parseWheelVar(String s){
    s = pins.SerialOdr->readString();
    s.trim();
    pins.SerialMonitor->println(s);
    return s.toFloat();
}
agvEr Swerve_module_controls::checkDriveEncoderInfos(){
    String cmd[3] = {"error", "encoder.config.phase_offset", "encoder.config.direction"};
    for(short i=0; i<3; i++){
        agvEr agver = this->bldc_sendCmd("r "+(String)_motorNum+"."+cmd[i]);
        if(agver!=SWERVE_OK) return agver;
    };
    return SWERVE_OK;
}
agvEr Swerve_module_controls::loadSettings(ctrlSettings settings){
    this->settings = settings;
    
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

