#include "AgvFrame.h"
#include "FrameEnum.h"

Swerve_module_kinematics::Swerve_module_kinematics(const SwervePin& pins, isClockWise isCW, wheelPositions wheelPos): pins(pins), wP(wheelPos){
    swerveCtrl = new Swerve_module_controls(pins, isCW);
}
// init all the swerve modules
agvEr Swerve_module_kinematics::initSwerveModule(){
    return swerveCtrl->initSwerve();
}
// agvEr initAgv(){for (short i = 0; i < 2; i++){swerve[i]->initSwerve();}};
void Swerve_module_kinematics::resetVars(){
    swerveCtrl->resetVars();
}; // calling homeNow() will call home() for all swerve modules
// ctrlValues preRef;

// agvEr getInfo();
// ctrlValues ref[2]; // 0 is ref now, 1 is preref
void Swerve_module_kinematics::driveSwerve(ctrlValues* ref){
    ref = this->toSwerveModuleStates(ref, true);
    swerveCtrl->runTurnAngle(ref->posTurn);
    swerveCtrl->runDriveDistance(ref->posDrive);
}
ctrlValues* Swerve_module_kinematics::toSwerveModuleStates(ctrlValues* ref, bool posCtrl){
    if(posCtrl){
        double x = ref->position*cos(ref->angle) + cos(ref->angle)*wP.posB;
        double y = ref->position*sin(ref->angle) + sin(ref->angle)*wP.posW;
        ref->posTurn = atan(y/x);
        ref->posDrive = sqrt(x*x+y*y);
        ref = this->optimize(ref);
        return ref;
    }
    double x = ref->velocity*cos(ref->angle) + cos(ref->angularVel)*wP.posB;
    double y = ref->velocity*sin(ref->angle) + sin(ref->angularVel)*wP.posW;
    ref->velTurn = atan(y/x);
    ref->velDrive = sqrt(x*x+y*y);
    ref = this->optimize(ref);
    ref = this->optimize(ref);

    return ref;
}
ctrlValues* Swerve_module_kinematics::optimize(ctrlValues* ref){
    // prevent jitter
    if(abs(ref->velTurn) < JITTER_PERCENTAGE) ref->velTurn = 0;
    if(abs(ref->velDrive) < JITTER_PERCENTAGE) ref->velDrive = 0;
    // get the angle
    double delta = ref->posTurn - swerveCtrl->getDirectionEncoderPos(degree);
    // clamp to -180 to 180
    delta = fmod(delta + 180.0, 360.0);
    if (delta < 0) delta += 360.0;
    delta -= 180.0;
    // check optimal turn angle
    if (abs(delta) > 90.0) {
        delta -= (delta > 0 ? 180.0 : -180.0);
        ref->posDrive *= -1.0; // Reverse the motor power via pointer
    }
    // cosin compensation
    ref->velDrive *= cos(delta);
    ref->posTurn +=  delta;

  return ref;
}
// required init Serial before calling this function.
void Swerve_module_kinematics::getInfo_Serialprint(){
    if(!swerveCtrl->printPosStats()) pins.SerialMonitor->println("Swerve print error!");
}  