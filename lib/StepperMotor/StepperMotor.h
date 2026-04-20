#pragma once
#include "Arduino.h"

class StepperMotor
{
public:
    StepperMotor(unsigned int pinStep, unsigned int pinDir);
    static void IRAM_ATTR sendPulse();
    
    void init();
    void setPulse(long pulse);
    bool step(long steps, uint8_t direction);
    bool step(long steps, uint8_t direction, long pulse);
    long getRemainingSteps();

    long stop();
    void pause();
    void resume();

    bool isStepping();
    bool isStopped();
    bool isPaused();
    static void ticking();
protected: // half private
    static hw_timer_t * sharedTimer;
    static portMUX_TYPE timerMux;
    static const int MAX_INSTANCES = 2;
    static StepperMotor* instances[MAX_INSTANCES];
    static short instanceCount;
private:
    byte pinStep;
    byte pinDir;
    int pulse_width;
    bool active;
public:
    volatile long ticksRemaining;
    bool paused;
};
