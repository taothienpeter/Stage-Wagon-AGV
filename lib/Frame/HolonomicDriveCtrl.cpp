#include "AgvFrame.h"
#include "FrameEnum.h"

Swerve_module_kinematics::Swerve_module_kinematics(const SwervePin& pins, wheelPositions wheelPos, isClockWise isCW): pins(pins), wP(wheelPos){
    swerveCtrl = new Swerve_module_controls(pins, isCW);
}
// init all the swerve modules
agvEr Swerve_module_kinematics::initSwerveModule(){
    return swerveCtrl->initSwerve();
}
// agvEr initAgv(){for (short i = 0; i < 2; i++){swerve[i]->initSwerve();}};
void Swerve_module_kinematics::getInfo_Serialprint(){
    // if(!swerveCtrl->printPosStats()) pins.SerialMonitor->println("Swerve print error!");
}  
void Swerve_module_kinematics::resetVars(){
    swerveCtrl->resetVars();
}; // calling homeNow() will call home() for all swerve modules

void Swerve_module_kinematics::driveSwerve(ctrlValues* ref, bool posCtrl){
    ref = this->toSwerveModuleStates(ref, true);
    swerveCtrl->runTurnAngle(ref->posTurn);
    if(posCtrl){
        swerveCtrl->runDriveDistance(ref->posDrive);
    }else{
        swerveCtrl->runDriveSpeed(ref->velDrive);  // check for type of controller
    }
}
agvEr Swerve_module_kinematics::computeVelocity(pose current, pose target, vel& ref){
    // Error in world frame
    double etheta = target.theta - current.theta;
    // Normalize angle
    while (etheta > 180) etheta -= 360;
    while (etheta < -180) etheta += 360;
    // P controller
    ref.velx = kp * (cos(current.theta) * (target.x - current.x) + sin(current.theta) * (target.y - current.y));
    ref.vely = kp * (-sin(current.theta) * (target.x - current.x) + cos(current.theta) * (target.y - current.y));
    ref.omega = ktheta * etheta;
    return AGV_INFO_COMPUTE_VELO;
}
wheelState Swerve_module_kinematics::computeWheel(vel ref){
    double vix = ref.velx - ref.omega * wP.posB;
    double viy = ref.vely + ref.omega * wP.posW;
    wheelState w;
    w.speed = sqrt(vix*vix + viy*viy);
    w.angle = atan2(viy, vix);
    return w;
};
// ctrlValues* Swerve_module_kinematics::toSwerveModuleStates(ctrlValues* ref, bool posCtrl){
    // if(posCtrl){
    //     double x = ref->position*cos(ref->angle) + cos(ref->angle)*wP.posB;
    //     double y = ref->position*sin(ref->angle) + sin(ref->angle)*wP.posW;
    //     ref->posTurn = atan(y/x);
    //     ref->posDrive = sqrt(x*x+y*y);
    //     ref = this->optimize(ref, posCtrl);
    //     return ref;
    // }
    // double x = ref->velocity*cos(ref->angle) + cos(ref->angularVel)*wP.posB;
    // double y = ref->velocity*sin(ref->angle) + sin(ref->angularVel)*wP.posW;
    // ref->posTurn = atan(y/x);
    // ref->velDrive = sqrt(x*x+y*y);
    // ref = this->optimize(ref, posCtrl);
//     return ref;
// }
// ctrlValues* Swerve_module_kinematics::optimize(ctrlValues* ref, bool posCtrl){
//     // scale speed for accurate position control

//     // prevent jitter
//     if(!posCtrl && abs(ref->velDrive) < JITTER_PERCENTAGE) ref->velDrive = 0;
//     // get the angle
//     double delta = ref->posTurn - swerveCtrl->getDirectionEncoderPos(degree);
//     // clamp to -180 to 180
//     delta = fmod(delta + 180.0, 360.0);
//     if (delta < 0) delta += 360.0;
//     delta -= 180.0;
//     // check optimal turn angle
//     if (abs(delta) > 90.0) {
//         delta -= (delta > 0 ? 180.0 : -180.0);
//         ref->posDrive *= -1.0; // Reverse the motor power via pointer
//     }
//     // cosin compensation
//     if(!posCtrl) ref->velDrive *= cos(delta);
//     // Pre-normalize

//     // Post-normalize
    
//     ref->posTurn +=  delta;
//   return ref;
// }
// ctrlValues* Swerve_module_kinematics::cvModuleStates2Chassis(ctrlValues* ref){
//     return ref;
// }
// ctrlValues* Swerve_module_kinematics::getModuleState(ctrlValues* ref){
//     return ref;
// }
// required init Serial before calling this function.

