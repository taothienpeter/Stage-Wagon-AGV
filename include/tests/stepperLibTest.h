
#include "FastAccelStepper.h"
#include "Ctrl_config.h"
// As in StepperDemo for Motor 1 on AVR
#define enablePinStepper 2

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(9600);

  Serial.println("Starting Stepper Test");
  Serial.flush();
  engine.init();
  stepper = engine.stepperConnectToPin(PIN_PUL);
  Serial.println("Stepper Initialized \nStepper Pin: " + (String)PIN_PUL);
  Serial.flush();
  if (stepper) {
    stepper->setDirectionPin(PIN_DIR); // Dir
    stepper->setEnablePin(enablePinStepper); //
    stepper->setAutoEnable(true);

    stepper->setSpeedInUs(50);  // 2000 us/step = 500 steps/s
    stepper->setAcceleration(10000);
    // stepper->move(-20000);
  } else {
    Serial.println("Stepper Not initialized!");
    delay(1000);
  }
  Serial.print("F_CPU=");
  Serial.println(F_CPU);
  Serial.print("TICKS_PER_S=");
  Serial.println(TICKS_PER_S);
  Serial.flush();
}

void loop() {
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        long steps = data.toInt();
        Serial.println("Received: " + data + ", moving " + (String)steps + " steps");
        stepper->move(steps);
    };
    
  //   if (stepper && stepper->isRunning()) {
  //     Serial.print("Position: "); Serial.println((stepper->getCurrentPosition()*360)/200000);
  //   } else {
  //     Serial.println("All moves completed!");
  //     while (true) {
  //       Serial.println("Stepper idle");
  //       delay(1000);
  //   }
  // } 
}
/*
#include "FastAccelStepper.h"
#include "Ctrl_config.h"
// As in StepperDemo for Motor 1 on AVR
#define dirPinStepper 18
#define enablePinStepper 2
#define stepPinStepper 5

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(9600);

  Serial.println("Starting");
  Serial.flush();
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  Serial.println("Starting");
  Serial.print("Stepper Pin:");
  Serial.println(stepPinStepper);
  Serial.flush();
  Serial.println((unsigned int)stepper);
  Serial.println((unsigned int)&engine);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    stepper->setSpeedInUs(1000);  // the parameter is us/step !!!
    stepper->setAcceleration(10000);
    // stepper->move(7000);
    stepper->runForward();
  } else {
    Serial.println("Stepper Not initialized!");
    delay(1000);
  }
  Serial.print("    F_CPU=");
  Serial.println(F_CPU);
  Serial.print("    TICKS_PER_S=");
  Serial.println(TICKS_PER_S);
  Serial.flush();
}

void loop() {
  delay(100);
  if (stepper) {
    if (stepper->isRunning()) {
      Serial.print("@");
      Serial.println(stepper->getCurrentPosition());
    }
  } else {
    Serial.println("Stepper died?");
    Serial.flush();
    delay(10000);
  }
}
*/