#include "Encoder.h"
#include "FastAccelStepper.h"
#include "Ctrl_config.h"
#define enablePinStepper 2
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
short calib = 0;
Encoder stepper_1_encoder(ENC_STEPPER_2A, ENC_STEPPER_2B, false);
bool isHome = false;
long deg2pos(int deg, int res){return ((deg*res)/360);}
int pos2deg(long pos, int res){return ((pos*360)/res);}

void updateE1A() { stepper_1_encoder.triggerA(); }
void updateE1B() { stepper_1_encoder.triggerB(); }
void sethome() {
  // stepper->forceStopAndNewPosition(0);
  // stepper->stopMove();
  // stepper->setPositionAfterCommandsCompleted(0);
  digitalRead(ENC_STEPPER_2HOME)? isHome = false: isHome = true;
}
void homingSeq(){
  Serial.println("Homing...");
  stepper->forceStopAndNewPosition(0);
  while (digitalRead(14)&&pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)<181) {
    stepper->move(deg2pos(1,MOTOR_MICROSTEPS));  // Move forward 180 degs
    // sethome();
    Serial.println("Position: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + " \tEncoder: " + (String)stepper_1_encoder.getAngle());
    delayMicroseconds(2);  // Wait for move to complete or be stopped
  }
  while (digitalRead(14)&&pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)>-181) {
    stepper->move(deg2pos(-1,MOTOR_MICROSTEPS));  // Then move backward 360 degs
    // sethome();
    Serial.println("Position: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + " \tEncoder: " + (String)stepper_1_encoder.getAngle());
    delayMicroseconds(200);  // Wait for move to complete or be stopped
  }
  calib = stepper->getCurrentPosition();
  Serial.println("Position: " + (String)(stepper->getCurrentPosition()) + " \tEncoder: " + (String)stepper_1_encoder.getTicks());
  while (!digitalRead(14)) {
    if(stepper->getCurrentPosition()>0){stepper->move(1);}
    else{stepper->move(-1);}
    delay(10);
  }
  calib = stepper->getCurrentPosition()-calib;
  Serial.println("Position: " + (String)(calib));
  stepper->move(-calib);
  // sethome();
  // stepper->forceStopAndNewPosition(0);
  // stepper->stopMove();
  // stepper->setCurrentPosition(0);
  stepper_1_encoder.reset();
}
void homingSeq2(){
  Serial.printf("homingSeq2");
  stepper->forceStopAndNewPosition(0);
  // find home first
  stepper->moveTo(deg2pos(180,MOTOR_MICROSTEPS));
  while(!isHome &&stepper->isRunning()); // replace with while loop to remove delay
  if(isHome){
    Serial.print("home found");
    stepper->setSpeedInUs(200);
    stepper->setAcceleration(1000);
    stepper->moveTo(deg2pos(20,MOTOR_MICROSTEPS));
  }
  else{
    stepper->moveTo(deg2pos(-180,MOTOR_MICROSTEPS));
    while(!isHome &&stepper->isRunning());
    if(isHome){
      Serial.print("home found");
      stepper->setSpeedInUs(200);
      stepper->setAcceleration(1000);
      stepper->moveTo(deg2pos(-20,MOTOR_MICROSTEPS));
    }

  }

  // find true home
  /*
  // calib = stepper->getCurrentPosition()+1;
  // stepper->moveTo(-1000);
  // while(!isHome) {calib = stepper->getCurrentPosition()-1;}
  // if(!isHome)stepper->moveTo(1000);
  // while(!isHome) {calib = stepper->getCurrentPosition()-1;} 
  // Serial.print(calib);
  // stepper->move(-calib/2);
  */  
  if(stepper->getCurrentPosition()>180) {stepper->setPositionAfterCommandsCompleted(deg2pos(180,MOTOR_MICROSTEPS)); stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));}
  else if (stepper->getCurrentPosition()<-180) {stepper->setPositionAfterCommandsCompleted(deg2pos(-175,MOTOR_MICROSTEPS)); stepper->moveTo(deg2pos(0,MOTOR_MICROSTEPS));}
  else {stepper->setPositionAfterCommandsCompleted(deg2pos(0,MOTOR_MICROSTEPS));}
  stepper->setSpeedInUs(100);
  stepper->setAcceleration(10000);
}
void setup(){
    Serial.begin(9600);
    stepper_1_encoder.reset();
    attachInterrupt(digitalPinToInterrupt(ENC_STEPPER_2A), updateE1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_STEPPER_2B), updateE1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_STEPPER_2HOME), sethome, CHANGE);

    engine.init();
    stepper = engine.stepperConnectToPin(MOTOR_2_PIN_PUL);
    stepper->setDirectionPin(MOTOR_2_PIN_DIR);
    
    stepper->setSpeedInUs(100);
    stepper->setAcceleration(10000);
    // test this for ensur
    // stepper->runForward();
}
void loop(){
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      // data.trim();
      int inter = data.substring(1).toInt();
      Serial.println("Received from laptop: "+data);
    if (data[0] == 'm') stepper->move(((inter*MOTOR_MICROSTEPS)/360)+111);
    if (data[0] == 'p') stepper->moveTo(((inter*MOTOR_MICROSTEPS)/360)+111);
    if (data[0] == 's') stepper->setSpeedInUs(inter);
    if (data[0] == 'a') stepper->setAcceleration(inter);
    if (data[0] == 'd') Serial.println("Position: "+ (String)stepper->getCurrentPosition() +"\nSpeed: " + (String)stepper->getSpeedInUs()+ "\nAcceleration: " + (String)stepper->getAcceleration());
    if (data[0] == 'h') homingSeq2();
    } 
    if (stepper && stepper->isRunning()) {
      Serial.println("Position: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + " \tEncoder: " + (String)stepper_1_encoder.getAngle() + "\t Ticks: " +(String)stepper_1_encoder.getTicks());
      // if(digitalRead(14)){Serial.println("Not homing");} 
      // else {updateH1();};
    } else {
      Serial.print("Position: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + " \tEncoder: " + (String)stepper_1_encoder.getAngle() + "\t Ticks: " +(String)stepper_1_encoder.getTicks());
      Serial.println(" All moves completed!");
      while (!Serial.available()) {
        // Serial.println("Position: " + (String)(pos2deg(stepper->getCurrentPosition(),MOTOR_MICROSTEPS)) + " \tEncoder: " + (String)stepper_1_encoder.getTicks());
        // Serial.println("Stepper idle");
        // delay(10);
      }
    }
    
    // delay(10);
}