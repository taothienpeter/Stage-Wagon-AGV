#pragma once
#include "Arduino.h"
#include "Ctrl_config.h"
#define ENC_TICKS_PER_REV ENC_STEP_TPR

// ISR function declarations
extern void encoderISR_A();
extern void encoderISR_B();
extern void encoderISR_Home();

class Encoder
{
private:
  uint8_t pinA;
  uint8_t pinB;
  uint8_t pinHome;
  volatile long ticks = 0;
  int increment = 0;
  void (*homeCallback)(void) = nullptr;
  
public:

  Encoder(
      uint8_t pinA,
      uint8_t pinB,
      bool isClockwise = false);

  Encoder(
      uint8_t pinA,
      uint8_t pinB,
      uint8_t pinHome,
      bool isClockwise = false,
      void (*homeCallback)(void) = nullptr);
  // void init();
  void triggerA();
  void triggerB();
  // void homeDetected();
  volatile long getTicks();
  void reset();
  int getPinA();
  int getPinB();
  double getAngle();
};