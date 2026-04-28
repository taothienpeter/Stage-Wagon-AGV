#include "Encoder.h"
#include "FastAccelStepper.h"
#include "Ctrl_config.h"
#include "Arduino.h"
#include "FrameEnum.h"
// #include "OdriveUART.h"
// #define enablePinStepper 2
#define SERIAL_DEBUG
#define DELTA_T PROCESS_DELTA_T
#define temp
/// @brief 0x.. hexagall number which in decamal: 
///             first num is type of error: 1 for ERROR, 2 for WARING, 3 for INFO
///             second device's code:       1 for Encoders, 2 for Steppers, 3 for Home sens, 4 for BLDCs, 5 for UWBs, 6 for IMU, 7 for Battery
///             third device's number   

inline int pos2deg(short deg, short res){return ((deg*res)/360);};
inline long deg2pos(long pos, short res){return ((pos*360)/res);};
class Swerve_module_controls{
    public: // initialization
        /// @brief Constructor for the Swerve_module_controls class.
        /// @details Initializes swerve module with pin configuration and orientation settings.
        ///          Sets up FastAccelStepper engine, stepper motor, and direction encoder.
        /// @param pins Reference to SwervePin structure containing all pin assignments.
        /// @param isCW Union specifying clockwise orientation for each motor (default {0x00}).
        Swerve_module_controls(const SwervePin pins, isClockWise isCW = {0x00});
        
        /// @brief Initializes the swerve module with speed and acceleration parameters.
        /// @details Sets up stepper motor engine, configures pins, speed, acceleration,
        ///          resets encoder, performs homing and clears errors.
        /// @param spd Speed in microseconds per step (lower = faster). Default: 50 µs/step.
        /// @param acc Acceleration in steps per second squared. Default: 10000 steps/s².
        agvEr initSwerve(short spd = 50, short acc = 10000);
        /// @brief Initiates the homing sequence for the swerve module.
        /// @details Calls homingSeq() and calibrates BLDC motor via ODrive.
        ///          Transitions through START, ARM, and READY phases for proper initialization.
        void home();
        

        /// @brief Resets all variables and states to their initial values.
        /// @details Clears errors, resets encoder/stepper positions, optionally performs homing.
        /// @param needHome If true (default), performs full homing sequence after reset.
        /// @return agvEr Returns SWERVE_OK on success, or appropriate error code on failure.
        agvEr resetVars(bool needHome = true);
        
    private:
        static FastAccelStepperEngine engine;
        FastAccelStepper *stepper;
        Encoder *Step_enc; // Using a pointer so we can initialize it dynamically
        // ODriveUART *odrive;
        bool _motorNum; // 0 = Front, 1 = Back motor
        SwervePin pins;
        void initInterrupts(){
            attachInterruptArg(digitalPinToInterrupt(pins.encStepA), updateEA, this, CHANGE);
            attachInterruptArg(digitalPinToInterrupt(pins.encStepB), updateEB, this, CHANGE);
            attachInterruptArg(digitalPinToInterrupt(pins.encHome), homeNow, this, LOW);
        }
        agvEr homingSeq();
        
        void bldc_ReBoot(); // should be public
        /// @brief Clears error flags from the ODrive motor controller.
        /// @details Sends clear error command ("sc") to reset accumulated error states.
        void bldc_clcEr();
        
        /// @brief Reads and prints error information from the ODrive motor controller.
        /// @details Queries ODrive for error status and prints response to serial monitor.
        /// @return agvEr Returns SWERVE_ERROR_NO_ODRIVE_FOUND if unavailable, 
        ///              or SWERVE_ERROR_FEEDBACK_OK on success.
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
            if (instance->stepper->getCurrentPosition() > 0) {
                instance->stepper->forceStopAndNewPosition(deg2pos(360, MOTOR_MICROSTEPS)); 
                instance->stepper->moveTo(deg2pos(0, MOTOR_MICROSTEPS));
            } else if (instance->stepper->getCurrentPosition() < 0) {
                instance->stepper->forceStopAndNewPosition(0);
            } else {
                instance->stepper->setCurrentPosition(deg2pos(0, MOTOR_MICROSTEPS));
            }
        }
    public: // setting swerve function
               
        /// @brief Executes a turn operation by specifying the angle.
        ///        Sets the turning speed for the steering motor.
        ///        Sets the acceleration for the steering motor.
        /// @param Angle The angle in degrees to turn. Positive values indicate clockwise direction,
        ///              while negative values indicate counterclockwise direction.
        /// @param SpeedHz The desired turning speed in Hertz (steps per second).
        /// @param step_s_s The acceleration value in steps per second squared
        /// @return agvEr Returns an error code indicating success or failure of the operation.
        agvEr runTurnAngle(double Angle); // changing swerve direction
        agvEr setTurnSpeed(double speed, unit u = Us); //or update speed()
        agvEr setTurnAccel(int16_t step_s_s); // changing swerve acceleration
        
        /// @brief Runs the drive motor at a specified speed.
        ///        Sets the acceleration for the drive motor based on system inertia.
        ///        Sets the absolute position for the drive motor.
        ///        Runs the drive motor for a specific distance.
        ///        Sets the torque output of the drive motor.
        /// @param turns The speed in turns per second for the drive motor.
        /// @param inertia A value which correlates acceleration (in turns/sec^2) and motor torque. 
        ///                It is 0 by default. It is optional, but can improve response of your system 
        ///                if correctly tuned. Keep in mind this will need to change with the load/mass of your system.
        /// @param pos The target absolute position for the drive motor in integer steps.
        /// @param tuns The distance in turns for the drive motor to travel.
        /// @param torque The desired torque value in Newton-meters (Nm).
        /// @return agvEr Returns an error code indicating success or failure of the operation.
        agvEr runDriveSpeed(double turns);
        
        /// @brief Sets the acceleration for the drive motor based on system inertia.
        /// @details Configures how motor torque responds to acceleration commands.
        ///          Tune this value based on system load and mass for optimal response.
        /// @param inertia Value correlating acceleration (turns/sec²) to motor torque. Default is 0.
        /// @return agvEr Returns SWERVE_OK on success.
        agvEr setDriveAccel(int16_t inertia); 
        agvEr setDriveAbsolutePos(int16_t pos);
        agvEr runDriveDistance(double turns); // changing swerve direction
        agvEr setDriveTorque(uint8_t torque); // torque in Nm 
    private:
        /// @brief Sends a command to the ODrive motor controller via serial communication.
        /// @details If msg is "serialMonitor", reads command from serial monitor and sends to ODrive,
        ///          then reads and prints the response. Otherwise, sends the provided message directly.
        /// @param msg The command string to send. Default is "serialMonitor" to read from serial input.
        /// @return agvEr Returns AGV_ERROR_NO_SERIALMONITOR_FOUND if serial monitor unavailable,
        ///              SWERVE_ERROR_NO_ODRIVE_FOUND if ODrive unavailable, or result of bldc_prtEr().
        agvEr bldc_sendCmd(String msg);
    public: // getting functions
        /// @brief Retrieves the current position of the direction encoder in the specified unit.
        /// @param u The unit in which the position should be returned (degree, radian, revolution, tick, Hz, meter).
        /// @return short Returns the position of the direction encoder in the specified unit.
        double getDirectionEncoderPos(unit u);
        #ifndef temp
        /// @return double Returns normalized position: -1 = full reverse, 0 = center, 1 = full forward.
        double getDirectionEncoderPos_UnitOne();
        #endif
        #ifndef temp
        /// @brief Retrieves the velocity of the direction encoder over a given time interval `dt`, in the specified unit.
        /// @param dt The time interval (in seconds) over which the velocity is calculated.
        /// @param u The unit in which the velocity should be returned (default is degree).
        /// @return double Returns the velocity of the direction encoder in the specified unit.
        double getDirectionEncoderVelo(float dt, unit u = degree);
        #endif
        #ifndef temp
        /// @return double Returns the normalized velocity of the direction encoder (-1 to 1).
        double getDirectionEncoderVelo_UnitOne(float dt);
        #endif
        /// @brief Retrieves the current position of the directional motor in the specified unit.
        /// @param u The unit in which the position should be returned (degree, radian, revolution, tick, Hz, meter).
        /// @return long Returns the position of the directional motor in the specified unit.
        long getDirectionPosition(unit u); 
        #ifndef temp
        /// @return double Returns the normalized position of the directional motor (-1 to 1).
        double getDirectionPosition_UnitOne(); 
        #endif
        /// @brief Retrieves the velocity of the directional motor in the specified unit (default is degree per second).
        /// @param u The unit in which the velocity should be returned (default is degree).
        /// @return double Returns the velocity of the directional motor in the specified unit.
        double getDirectionVelocity(unit u = degree); 
        #ifndef temp
        /// @return double Returns the normalized velocity of the directional motor (-1 to 1).
        double getDirectionVelocity_UnitOne(); 
        #endif
        /// @brief Checks if there has been any accumulated error due to skipped steps in the stepper motor.
        /// @return bool Returns true if step skipping has occurred, false otherwise.
        bool checkStepSkiping(); 

        /// @brief Retrieves the current angular position of the wheel motor (BLDC).
        /// @details These function uses string parsing and may impact MCU memory and overall performance. Use with caution.
        /// @return float Returns the angular position of the wheel motor.
        float getWheelPosition(); 
        /// @return float Returns the velocity of the wheel motor.
        float getWheelVelocity(); 
        #ifndef temp
        /// @return float Returns the torque output of the wheel motor.
        float getWheelTorque();
        #endif
        /// @brief Retrieves additional variables from the wheel encoder.
        /// @return float Returns the parsed variable from the wheel encoder.
        float getWheelEncoderVariables();

        /// @brief Checks and retrieves error information from the wheel encoder.
        /// @return agvEr Returns an error code indicating the status of the wheel encoder.
        agvEr checkWheelEncoderInfos();
    private:  
        /// @brief Parses a string containing wheel-related variables and extracts relevant data.
        /// @param s The input string containing wheel-related variables.
        /// @return float Returns the parsed value extracted from the input string.
        float parseWheelVar(String s);
    public:
        /// @brief Prints current position statistics of stepper motor and encoder.
        /// @details Prints stepper position in degrees and encoder angle in degrees to serial monitor.
        /// @return boolean Returns 0 (false) always after printing.
        bool printPosStats();
        
        /// @brief Prints a status message if SERIAL_DEBUG is enabled.
        /// @details Conditionally prints debug messages based on SERIAL_DEBUG preprocessor definition.
        /// @param msg The status message string to print.
        /// @return boolean Returns 0 if printed (SERIAL_DEBUG enabled), 1 if suppressed.
        bool printStatus(String msg);
        
};
inline double wrapAngle(double a) {
    while (a > M_PI)  a -= 360;
    while (a < -M_PI) a += 360;
    return a;
}
class Swerve_module_kinematics{
    public:
        Swerve_module_kinematics(const SwervePin& pins, wheelPositions wheelPos = wheelPositions(), isClockWise isCW = {0x00});
        agvEr initSwerveModule();   
        /// @brief 
        void getInfo_Serialprint();
        agvEr resetVars();
        agvEr driveSwervePose(pose& pose); // control using next pose
        agvEr driveSwerveVel(vel& vel); // control usign velocity
    private:
        Swerve_module_controls *swerveCtrl;     
        SwervePin pins;
        wheelPositions wP;
        wheelState wS;
        vel ref;
        pose prePose = {0,0,0};
        float kp = 1.0; // pos gains
        float ktheta = 2.0; // angle gains
        // agvEr setPrePose();
        agvEr computeVelocity(pose current, pose target, vel& ref);
        wheelState computeWheel(vel ref, double currentSteerAngle);

        agvEr computeChassisVelocity(wheelState wS, vel& ref);
        agvEr updateOdometry(pose& pose, vel ref, float dt);
};