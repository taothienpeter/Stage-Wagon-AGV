#include "Encoder.h"
Encoder::Encoder(
  uint8_t pinA,
  uint8_t pinB,
  bool isClockwise)
  : pinA(pinA), pinB(pinB)
  {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  increment = (2*isClockwise-1);
}

// Encoder::Encoder(
//     uint8_t pinA,
//     uint8_t pinB,
//     uint8_t pinHome,
//     bool isClockwise,
//     void (*homeCallback)(void))
//     : pinA(pinA), pinB(pinB), pinHome(pinHome), homeCallback(homeCallback)
// {
//   pinMode(pinA, INPUT);
//   pinMode(pinB, INPUT);
//   pinMode(pinHome, INPUT);
//   increment = (2*isClockwise-1);
// }
void Encoder::triggerA(){
  if (digitalRead(pinA) == digitalRead(pinB)) {ticks += increment;}
  else {ticks -= increment;}
}
void Encoder::triggerB(){
  if (digitalRead(pinA) != digitalRead(pinB)) {ticks += increment;}
  else {ticks -= increment;}
}

volatile long Encoder::getTicks() {return ticks;}

double Encoder::getAngle() {return (double)ticks/ ENC_TICKS_PER_REV *360;}

void Encoder::reset() {ticks = 0;}

int Encoder::getPinA() {return pinA;}

int Encoder::getPinB() {return pinB;}