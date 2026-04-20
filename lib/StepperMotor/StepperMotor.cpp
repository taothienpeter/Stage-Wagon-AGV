#include "StepperMotor.h"
#include "Ctrl_config.h"

portMUX_TYPE StepperMotor::timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * StepperMotor::sharedTimer = NULL;

StepperMotor* StepperMotor::instances[MAX_INSTANCES];
// int StepperMotor::instanceCount = 0;

StepperMotor::StepperMotor(unsigned int _pinStep, unsigned int _pinDir)
  : pinStep(_pinStep), pinDir(_pinDir) {
  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);
  if (instanceCount < MAX_INSTANCES) {
    instances[instanceCount++] = this;
  }
}

void StepperMotor::init() {
  if (instanceCount == 1) {  // First instance sets up the shared timer
    sharedTimer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz ticks
    timerAttachInterrupt(sharedTimer, ticking, true);
    timerAlarmWrite(sharedTimer, 100, true);  // 100 ticks = 100 us period (10 kHz)
    timerAlarmEnable(sharedTimer);
  }
}

void StepperMotor::setPulse(long pulse_us) {
  // Calculate reload based on timer period (100 us)
  pulse_widthTick = pulse_us / TIMER_MAX_FREQUENCY;  // e.g., 1000 us -> 10
  if (pulse_widthTick < 1) pulse_widthTick = 1;
}

bool StepperMotor::step(long steps, uint8_t direction) {
  if (this->isStepping()) return false;
  ticksRemaining = steps * 2;
  digitalWrite(pinDir, direction);
  
  paused = false;
  return true;
}
bool StepperMotor::step(long steps, uint8_t direction, long pulse) {
  if(this->isStepping())return false; // if already running, the command will be ignored
    // if not running, start
    this->setPulse(pulse);
    this->resume();
    return  this->step(steps, direction);
}

long StepperMotor::getRemainingSteps() {
  return ticksRemaining / 2;
}
// stop by de
long StepperMotor::stop(){
    //each step = 2 ticks
    long stepsRemaining = this->getRemainingSteps();
    timerDetachInterrupt(sharedTimer); // stop timer int
    ticksRemaining & 1 ? ticksRemaining = 1: ticksRemaining = 0; // reset ticksRemaining
    return stepsRemaining;
}

void StepperMotor::pause() {
  timerDetachInterrupt(sharedTimer); // stop timer int
  paused = true;
}

void StepperMotor::resume(){
  if(paused){
    timerAttachInterrupt(sharedTimer, ticking, true); // activate timer int
    paused = false;
  }
}

bool StepperMotor::isStepping(){
    return (ticksRemaining > 0);
}

bool StepperMotor::isStopped() {
  return ticksRemaining <= 0;
}

bool StepperMotor::isPaused(){
  return paused;
}

void IRAM_ATTR StepperMotor::ticking() {
  // portENTER_CRITICAL_ISR(&timerMux);
  for (int i = 0; i < instanceCount; i++) {
    StepperMotor* ins_p = instances[i];
    if (ins_p->pulse_widthTick-- <= 0) {
    if (ins_p->ticksRemaining > 0) {
      
        digitalWrite(ins_p->pinStep, !digitalRead(ins_p->pinStep));
        ins_p->ticksRemaining--;
        // ins_p->stepCounter = ins_p->stepCounterReload;
        
      }
      
    }
  }
  // portEXIT_CRITICAL_ISR(&timerMux);
}
