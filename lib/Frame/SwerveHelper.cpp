#include "AgvFrame.h"
#include "FrameEnum.h"

extern Swerve_module_kinematics* swerve[2];
extern vel Vel;
agvEr _initswerve(){
    // calib swerve
    Serial.println("Homing sequence started");
    u_int16_t sT = millis(); // start time
    u_int16_t eT = 5000; // elapsed time
    calibState stepperState[2] = {stepCW, stepCW}, bldcState = bldcMotor;
    swerve[0]->calibSwerve(bldcMotor);
    swerve[1]->calibSwerve(bldcMotor);
    swerve[0]->calibSwerve(stepCW); // inlucded all the init of stepper
    swerve[1]->calibSwerve(stepCW); 
    while(eT != 0)
    {
        // if (seT < (millis() - sT) && seT != 0){
        for(int i = 0; i<2; i++){
            if(!swerve[i]->swerveCtrl->stepper->isRunning()) {
                if (stepperState[i] != stepTruehome) stepperState[i] = static_cast<calibState>(stepperState[i] + 1);
                swerve[i]->calibSwerve(stepperState[i]); 
            } 
        }
        // }
        if(eT <  (millis() - sT)){
            if (bldcState != bldcArmed) bldcState = static_cast<calibState>(bldcState + 1);
            swerve[0]->calibSwerve(bldcState);
            swerve[1]->calibSwerve(bldcState);
            if((millis() - sT)>15000) {eT = 0;}
            else if((millis() - sT)>5000){eT = 15000;}
        }
    }    
    Serial.println(swerve[0]->swerveCtrl->getTurnStepperPos());
    return SWERVE_OK;
};

short _cmdParse(String cmd) { 
  // 1. CRITICAL FIX: Trim whitespace/newlines before evaluating lengths or copying
//   cmd.trim(); 
  
  if (cmd.length() == 0) return -1; 
  
  // Convert Arduino String to a char array buffer for tokenization
  char buf[cmd.length() + 1]; 
  cmd.toCharArray(buf, sizeof(buf));
  
  // Extract the first token (The Command Name)
  char* token = strtok(buf, " "); 
  if (token == NULL) return -1;
  
  String cmdType = String(token);
  cmdType.toUpperCase(); 

  // --- EXTENSION SECTION ---
  
  // 1. Motion Command ("M velx vely omega")
  if (cmdType == "M") {
    token = strtok(NULL, " "); // Move to velx
    if (token != NULL) Vel.velx = atof(token); 
    
    token = strtok(NULL, " "); // Move to vely
    if (token != NULL) Vel.vely = atof(token);
    
    token = strtok(NULL, " "); // Move to omega
    if (token != NULL) Vel.omega = atof(token);
    
    return 0; // Return success/unique ID for this command
  }
  
  return -1; // Unknown command
/*
  // 2. Teleoperated Mode Command
  else if (cmdType == "TELEOPERATED") {
    Serial.println("Switching to Teleoperated Mode...");
    // Add your execution logic here
    return 1;
  }
  
  // 3. Autonomous Mode Command
  else if (cmdType == "AUTONOMOUS") {
    Serial.println("Switching to Autonomous Mode...");
    // Add your execution logic here
    return 2;
  }
  
  // 4. Practice Mode Command
  else if (cmdType == "PRACTISE" || cmdType == "PRACTICE") {
    Serial.println("Switching to Practice Mode...");
    return 3;
  }
  
  // 5. Test Mode Command
  else if (cmdType == "TEST") {
    Serial.println("Running Diagnostics/Test Mode...");
    return 4;
  }

  // If no commands match
  Serial.print("Unknown command received: ");
  Serial.println(cmdType);
    return -1;
*/
}
short _logData(){
    // Example transmission output structure for your loop:
    Serial.println("W " + (String)degrees(swerve[0]->wS.angle) +" "+ (String)swerve[0]->wS.speed+
                    " " + (String)degrees(swerve[1]->wS.angle)+ " "+  (String)swerve[1]->wS.speed);
    // Serial.println("R " + (String)swerve[0]->swerveCtrl->getTurnEncoderPos() +" "+ (String)swerve[0]->swerveCtrl->getDriveBldcVel()+
    //                 " " + (String)swerve[1]->swerveCtrl->getTurnEncoderPos()+ " "+  (String)swerve[1]->swerveCtrl->getDriveBldcVel());
    return -1;
}