#include "AgvFrame.h"
#include "FrameEnum.h"

Swerve_module_kinematics::Swerve_module_kinematics(const SwervePin& pins, wheelPositions wheelPos, isClockWise isCW): pins(pins), wP(wheelPos){
    swerveCtrl = new Swerve_module_controls(pins, isCW);
    wS = {0,0};
}
// init all the swerve modules
agvEr Swerve_module_kinematics::calibSwerve(calibState state){
    // scan for homing position
    if(state == stepCW || state == stepCCW || state == stepTruehome) {
        swerveCtrl->calibStepper(state);
    }else{
        swerveCtrl->calibBLDC(state);
    };
    return SWERVE_OK;
}
agvEr Swerve_module_kinematics::initSwerveModule(){return swerveCtrl->initSwerve();}
void Swerve_module_kinematics::getInfo_Serialprint(){if(!swerveCtrl->printPosStats()) pins.SerialMonitor->println("Swerve print error!");}  
agvEr Swerve_module_kinematics::resetVars(){return swerveCtrl->resetVars();};
// agvEr Swerve_module_kinematics::driveSwervePose(pose& pose){
//     if(this->computeVelocity(prePose, pose, ref)==AGV_INFO_COMPUTE_FRAMEVELO) pins.SerialMonitor->print("ok");
//     this->driveSwerveVel(ref);
//     prePose = pose;
//     return AGV_INFO_DRIVE_FRAMEVELO;
// }
agvEr Swerve_module_kinematics::driveSwerveVel(vel vel){
    this->setDesireState(vel);
    swerveCtrl->ctrlSS(wS);
    return AGV_INFO_DRIVE_OK;
}
agvEr Swerve_module_kinematics::setDesireState(vel ref){
    double vix = ref.velx + ref.omega * wP.posB;
    double viy = ref.vely - ref.omega * wP.posW;
    wS.speed = sqrt(vix*vix + viy*viy); // m/s
    wS.angle = atan2(viy, vix); // radian
    return AGV_INFO_COMPUTE_WHEELSTATE;
};
wheelState Swerve_module_kinematics::getDrivingState(){
    return wS;
};

// agvEr Swerve_module_kinematics::computeVelocity(pose current, pose target, vel& ref){
//     // Normalize angle error in world frame
//     double etheta = wrapAngle(target.theta - current.theta);
//     // P controller
//     ref.velx = kp * (cos(current.theta) * (target.x - current.x) + sin(current.theta) * (target.y - current.y));
//     ref.vely = kp * (-sin(current.theta) * (target.x - current.x) + cos(current.theta) * (target.y - current.y));
//     ref.omega = ktheta * etheta;
//     return AGV_INFO_COMPUTE_FRAMEVELO;
// }

agvEr Swerve_module_kinematics::getChassisVelo(vel& ref){
    ref.velx += wS.speed * cos(wS.angle);
    ref.vely += wS.speed * sin(wS.angle);

    double r2 = wP.posW * wP.posW + wP.posB * wP.posB;
    if (r2 > 1e-6) {
        ref.omega += (wS.speed * sin(wS.angle) * wP.posW - wS.speed * cos(wS.angle) * wP.posB) / r2;
    }
    return AGV_INFO_COMPUTE_FRAMEVELO;
}
// agvEr Swerve_module_kinematics::updateOdometry(pose& pose, vel ref, float dt){
//     pose.x += (ref.velx * cos(pose.theta) - ref.vely * sin(pose.theta)) * dt;
//     pose.y += (ref.velx * sin(pose.theta) + ref.vely * cos(pose.theta)) * dt;
//     pose.theta += ref.omega * dt;
//     pose.theta = wrapAngle(pose.theta);
//     return AGV_INFO_COMPUTE_FRAMEVELO;
// }
