#ifndef FrameEnum_h
#define FrameEnum_h

enum agvEr{
//     AGV_OK = 0x00,
    AGV_ERROR_NO_SERIALMONITOR_FOUND,
//     AGV_ERROR_NO_HOME_DETECTED,
//     AGV_ERROR_NO_ENCODER_ATTACHED,
//     AGV_ERROR_NO_HOMING_SENSOR,
//     AGV_ERROR_NO_STEPPER_MOTOR,
//     AGV_WARNING_BATTERY_LOW,
//     AGV_WARNING,
    AGV_INFO_CLAMP,
    AGV_INFO_COMPUTE_WHEELSTATE,
    AGV_INFO_COMPUTE_FRAMEVELO,
    // AGV_INFO_DRIVE_FRAMEVELO,
    AGV_INFO_DONE_HOMING,   
    AGV_INFO_DRIVE_OK,
//     AGV_INFO_DONE_RESET,

//     SWERVE_WARNING,
    SWERVE_INFO_INIT,
    SWERVE_INFO_RESET,
    SWERVE_INFO_HOME,

    SWERVE_ERROR_NO_HOME_DETECTED,
    SWERVE_ERROR_NO_ODRIVE_FOUND,
    SWERVE_ERROR_STEP_MISSMATCH,
    SWERVE_ERROR_FEEDBACK_OK, // from error checker function, should be output in printStatus()
    SWERVE_OK
};
struct logEr{
    // 4 states logs
    agvEr swerveErr[4];
    agvEr swerveInfo[4];
    agvEr swerveWarning[4];
    // 2 states logs
    agvEr agvErr[2];
    agvEr agvInfo[2];
    agvEr agvWarning[2];
};
struct isClockWise{
    bool isEnc_StepCW:1; 
    bool isEnc_BldcCW:1; 
    bool isStepCW:1; 
    bool isOdrCW:1;
};
struct SwervePin {
    uint8_t stepDir;
    uint8_t stepPul;
    #ifdef MOTOR_USES_PIN_ENABLE
    uint8_t stepEn;  // optional
    #endif
    uint8_t encStepA;
    uint8_t encStepB;
    uint8_t encHome;

    bool MotorNum;

    HardwareSerial* SerialOdr;  // Communicate with Odrive
    HardwareSerial* SerialMonitor; // Coms with PC
    FastAccelStepperEngine* stepperEngine;
    // isClockWise isCW;
};

enum calibState{ 
    stepCW,
    stepCCW,
    stepTruehome,
    bldcMotor,
    bldcEncoder,
    bldcArmed,
};
enum unit{
    degree,
    radian,
    revolution,
    meter,
    tick,
    Us,
    Hz
};
struct ctrlSettings{
    // uint8_t bldcAccel;
    unit unitTurn;
    unit unitDrive;
    
    uint8_t stepperSpeed; // best in Us or Hz (tick, Us, Hz)
    uint16_t stepperAccel; 

    uint8_t bldcSpeed; // best in m/s (original is revolutions/s)
    uint8_t bldcAccelMax;
    uint8_t bldcAccelMin;

    isClockWise isCW;
};
struct wheelPositions { 
    float posW;
    float posB;

    wheelPositions() : posW(0), posB(0) {}
    wheelPositions(float w, float b) : posW(w), posB(b) {}
    float getR() const { return sqrt(posW*posW+posB*posB); }
    float getTheta() const { return atan2(posB, posW); }
};
struct wheelState{
    double angle;
    double speed;
};
struct pose{
    double x, y , theta;
};
struct vel{
    double velx, vely, omega;
};
#endif