#include <math.h>
// Ebedded System config
#define MONITOR_BAUDRATE 9600
#define ODRIVE_BAUDRATE 115200

#define JITTER_PERCENTAGE 0.001
// Kinematic config
#define WHEEL_DIAMETER     0.145 // meter
// #define WHEEL_RADIUS (WHEEL_DIAMETER/2)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define WHEEL_POSITIONS_W  0.64 // meter
#define WHEEL_POSITIONS_B  0.64
// #define FRAME_HEIGHT       0.21925

// Encoder config
#define ENC_TEST_PINA ENC_STEPPER_2A
#define ENC_TEST_PINB ENC_STEPPER_2B

#define ENC_STEPPER_1A 22
#define ENC_STEPPER_1B 21
#define ENC_STEPPER_2A 5
#define ENC_STEPPER_2B 4

#define ENC_STEPPER_1HOME 35
#define ENC_STEPPER_2HOME 34

#define ENC_BLDC_TPR (6000) // BLDC encoder's ticks per revolution 
#define ENC_STEP_TPR (6000) // Stepper encoder's ticks per revolution 

#define BACKLASH_ENC 7
#define BACKLASH_STEP 5

// Stepper config
#define PIN_PUL MOTOR_1_PIN_PUL
#define PIN_DIR MOTOR_1_PIN_DIR

#define MOTOR_1_PIN_PUL 26
#define MOTOR_1_PIN_DIR 25
// #define MOTOR_1_PIN_ENABLE
#define MOTOR_2_PIN_PUL 14
#define MOTOR_2_PIN_DIR 27
// #define MOTOR_2_PIN_ENABLE

#if defined(MOTOR_1_PIN_ENABLE) || defined(MOTOR_2_PIN_ENABLE)
    #define MOTOR_USES_PIN_ENABLE
#endif

#define MOTOR_ANGLE 1.8
#define MOTOR_STEPS ((360/MOTOR_ANGLE)*2.5*5)
#define MOTOR_MICROSTEPS_SCALE 16 // 2 4 8 16 32
#define MOTOR_MICROSTEPS ((int16_t)MOTOR_STEPS*MOTOR_MICROSTEPS_SCALE)
#define MOTOR_MICROANGLE (MOTOR_ANGLE / MOTOR_MICROSTEPS_SCALE)

#define MOTOR_PulseWidth_MinUs 2.5
#define MOTOR_DirChange_MinUs 5
#define TIMER_MAX_FREQUENCY (1000000/MOTOR_DirChange_MinUs) // => Max timer frequency alowable according to stepper driver

// (MOTORPULSES * 1000000 / MOTORSTEPS)

// #define MOTOR_FREQUENCY_MIN 16 // due to limitation of ESP32's MCPWM 
// #define MOTOR_FREQUENCY_MAX 2500/2
// #define STEPPER_MOTOR_STEADY_STATE_ERROR    2          // pulse
// #define STEPPER_MOTOR_ACCELERATION_RPM      10         // rpm/s
// #define MAX_STEPPER_MOTOR_SPEED_RPM         120        // rpm

#define ODRIVE_TX 16
#define ODRIVE_RX 17
#define ODRIVE_BAUD ODRIVE_BAUDRATE // 115200  

#define ODRIVE_CONFIG
// Controller config

#define PROCESS_DELTA_T 0.05