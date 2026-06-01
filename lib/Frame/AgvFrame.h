#include "Encoder.h"
#include "FastAccelStepper.h"
#include "Ctrl_config.h"
#include "Arduino.h"
#include "FrameEnum.h"

agvEr _initswerve(); 
short _cmdParse(String cmd);
short _logData();
inline int pos2deg(long pos, short res){return ((pos*360)/res);};
inline long deg2pos(short deg, short res){return ((deg*res)/360);};
class Swerve_module_controls{
    public: // initialization
        Swerve_module_controls(const SwervePin pins, isClockWise isCW = {1,1,1,1});
        agvEr calibStepper(calibState state);
        agvEr calibBLDC(calibState state); 
        agvEr initSwerve();
        agvEr resetVars(bool needHome = true);
        agvEr ctrlSS(wheelState setState);
        bool isHomeState(){
            return (!stepper->isRunning() || isHome)? false : true;
        }
        
        static FastAccelStepperEngine engine;
        FastAccelStepper *stepper;
        Encoder *Step_enc; // Using a pointer so we can initialize it dynamically
        bool _motorNum; // 0 = Front, 1 = Back motor
        bool isHome;
        SwervePin pins;
        ctrlSettings settings;
    private:
        void initInterrupts(){
            attachInterruptArg(digitalPinToInterrupt(pins.encStepA), updateEA, this, CHANGE);
            attachInterruptArg(digitalPinToInterrupt(pins.encStepB), updateEB, this, CHANGE);
            // attachInterruptArg(digitalPi nToInterrupt(pins.encHome), homeNow, this, CHANGE);
        }
        void bldc_ReBoot(); // should be public
        void bldc_clcEr();
        agvEr bldc_prtEr();
    protected:
        static void updateEA(void* arg) {
            Swerve_module_controls* instance = static_cast<Swerve_module_controls*>(arg);
            if (instance->Step_enc != nullptr) {
                instance->Step_enc->triggerA();
            }
        }
        static void updateEB(void* arg) {
            Swerve_module_controls* instance = static_cast<Swerve_module_controls*>(arg);
            if (instance->Step_enc != nullptr) {
                instance->Step_enc->triggerB();
            }
        }
        static void homeNow(void* arg) {
            Swerve_module_controls* instance = static_cast<Swerve_module_controls*>(arg); 
            instance->stepper->forceStopAndNewPosition(0);
            instance->isHome = !instance->isHome;
        }
    public: // setting swerve function
        agvEr runTurnAngle(double Angle); // changing swerve direction
        agvEr runDriveSpeed(double turns);
        agvEr runDriveDistance(double turns); // changing swerve direction
    private: 
        agvEr bldc_sendCmd(String msg);
    public: // getting functions
        double getTurnEncoderPos(); // encoders
        double getTurnStepperPos(); // stepper ramp
        // double getTurnStepperVel(); // stepper ramp speed

        float getDriveBldcPos(); 
        float getDriveBldcVel(); 
        float getDriveEncoderVariables(); // bldc position
        agvEr checkDriveEncoderInfos(); // return err, dir, phase offset
    private:  
        bool checkStepSkiping(); 
        float parseWheelVar(String s);

        agvEr setTurnSpeed(double speed); //or update speed()
        agvEr setTurnAccel(int16_t accel); // changing swerve acceleration

        agvEr setDriveAccel(int16_t inertia); 
        agvEr setDriveAbsolutePos(int16_t pos);
        agvEr setDriveTorque(uint8_t torque); // torque in Nm 
    public:

        agvEr loadSettings(ctrlSettings settings);
        bool printPosStats();
        bool printStatus(String msg);
};
inline double wrapAngle(double a) {
    while (a > PI)  a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}
class Swerve_module_kinematics{
    public:
        Swerve_module_kinematics(const SwervePin& pins, wheelPositions wheelPos = wheelPositions(), isClockWise isCW = {1,1,1,1});
        agvEr calibSwerve(calibState state);
        agvEr initSwerveModule();   
        /// @brief 
        void getInfo_Serialprint();
        agvEr resetVars();
        // agvEr driveSwervePose(pose& pose); // control using next pose
        agvEr driveSwerveVel(vel vel); // control usign velocity
        Swerve_module_controls *swerveCtrl;     
        wheelState wS;
    private:
        // FastAccelStepperEngine engine;
        SwervePin pins;
        wheelPositions wP;
        vel ref; // velx, vely, omega input
        float kp = 1.0; // pos gains
        float ktheta = 1.0; // angle gains
        // agvEr setPrePose();
        agvEr setDesireState(vel ref);
        wheelState getDrivingState();
        // agvEr computeVelocity(pose current, pose target, vel& ref);

        agvEr getChassisVelo(vel& ref);
        // agvEr updateOdometry(pose& pose, vel ref, float dt);

        logEr _logErr;
    public:
        String getLogEr(bool Serialprint = false); // takes in-class logerr and returns
};