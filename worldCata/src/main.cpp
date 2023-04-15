/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// left1                motor         7               
// left2                motor         8               
// left3                motor         9               
// right1               motor         17              
// right2               motor         18              
// right3               motor         19              
// Controller1          controller                    
// ExpandR              digital_out   B               
// ExpandL              digital_out   C               
// Rotation9            rotation      20              
// Inertial10           inertial      5               
// cataclang            digital_out   A               
// CATAP                motor_group   2, 3            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

//Settings must tune
double kP = 0.01;
double kI = 0.000;
double kD = 0.003;
double turnkP = 0.02;
double turnkI = 0.008;
double turnkD = 0.002;

//Autonomous Settings
int desiredValue = 200;
int desiredTurnValue = 0;

double maxSpeed = 1;
double maxTurnSpeed = 1; 

int error; //sensorValue - desiredValue (deltaX) -- Positional
int prevError = 0; //Position 20 ms ago
int derivative; //error - previousError : Speed
int totalError = 0; //totalError = totalError + Error

int turnError; //sensorValue - desiredValue (deltaX) -- Positional
int turnPrevError = 0; //Position 20 ms ago
int turnDerivative; //error - previousError : Speed
int turnTotalError = 0; //totalError = totalError + Error

bool resetDriveSensor = false;


//Variables Modified for Use
bool enableDrivePID = true;

int drivePID(){

while(1){
  while(enableDrivePID){
    
    //reset items
    if(resetDriveSensor){
      resetDriveSensor = false;
      left1.setPosition(0, degrees);
      right1.setPosition(0,degrees);
    }


    //Get the position of both motors
    int leftMotorPosition = left1.position(degrees); 
    int rightMotorPosition = right1.position(degrees);

    //------------------------------------------------
    //Lateral Movement--------------------------------
    //------------------------------------------------
    //Get average of motors
    int averagePosition = (leftMotorPosition + rightMotorPosition) /2;

    //Positoinal
    error =desiredValue - averagePosition ;

    //Derivative
    derivative = error - prevError;

    //Integral
    totalError += error;

    if(totalError > 5){
      totalError = 5;
    }else if(totalError <-5){
      totalError = -5;
    }

    double lateralMotorPower = (error *kP + derivative* kD + totalError *kI)*maxSpeed;
    //-----------------------------------------------------------------------


    //------------------------------------------------
    //Turning Movement--------------------------------
    //------------------------------------------------
    //Get average of motors
    int turnDifference = (leftMotorPosition - rightMotorPosition);

    //Potential
    turnError = desiredTurnValue - turnDifference;

    //Derivative
    turnDerivative = turnError - turnPrevError;

    //Integral
    if(error < 50 && error >-50) {
    turnTotalError += turnError;
    }
    else {
    turnTotalError = 0;
    }

    if(turnTotalError > 100){
      turnTotalError = 100;
    }else if(turnTotalError <-100){
      turnTotalError = -100;
    }

    double turnMotorPower = (turnError *turnkP + turnDerivative* turnkD + turnTotalError *turnkI)*maxTurnSpeed;


    //--------------------------------------------------end
    left1.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    left2.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    left3.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);


    right1.spin(forward, (lateralMotorPower - turnMotorPower), voltageUnits::volt);
    right2.spin(forward, (lateralMotorPower - turnMotorPower), voltageUnits::volt);
    right3.spin(forward, (lateralMotorPower - turnMotorPower), voltageUnits::volt);




    //items
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

    
    Brain.Screen.setCursor(6,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(turnDifference);

  }

  return 1;
}
}

double CkP = 2;
double CkI = 0;
double CkD = 0;

int CdesiredValue = -82;

int Cerror; //sensorValue - desiredValue (deltaX) -- Positional
int CprevError = 0; //Position 20 ms ago
int Cderivative; //error - previousError : Speed
int CtotalError = 0; //totalError = totalError + Error

bool CresetDriveSensor = true;

bool x = true;


//Variables Modified for Use
bool CenableDrivePID = true;

int CdrivePID(){

while(1){
  while(CenableDrivePID){
    
    //reset items
    if(CresetDriveSensor){
      CresetDriveSensor = false;
      Rotation9.setPosition(0, degrees);
    }


    //Get the position of both motors
    //int leftMotor Position = CATA.position(degrees);
    int MotorPosition = Rotation9.position(degrees);
    //------------------------------------------------
    //Lateral Movement--------------------------------
    //------------------------------------------------
    //Get average of motors

    //Positoinal
    Cerror =CdesiredValue - MotorPosition;

    //Derivative
    Cderivative = Cerror - CprevError;

    //Integral
    CtotalError += Cerror;

    //intergral limiting
    if(CtotalError > 5){
      CtotalError = 5;
    }else if(CtotalError <-5){
      CtotalError = -5;
    }

    
    //set power with values
    double lateralMotorPower = (Cerror *CkP + Cderivative* CkD + CtotalError *CkI);
    //-----------------------------------------------------------------------


    //initiate motor
    //--------------------------------------------------end
    if(error < 2 ){
      CATAP.spin(forward, 0, voltageUnits::volt);
      
    }else{
      CATAP.spin(forward, lateralMotorPower, voltageUnits::volt);
    }
    

    //items
    CprevError = Cerror;
    
    vex::task::sleep(20);

  }

  vex::task::sleep(20);
}

  return 1;
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  left1.setVelocity(100,percent);
  left2.setVelocity(100,percent);
  left3.setVelocity(100,percent);
  right1.setVelocity(100,percent);
  right3.setVelocity(100,percent);
  right2.setVelocity(100,percent);
  CATAP.setVelocity(100,percent);

  ExpandL.set(true);
  ExpandR.set(true);

  vex::task randomName2(CdrivePID);


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

bool armOn = true;

void shootarm(){
  armOn = false;
  CATAP.spinFor(0.25, turns);
  CenableDrivePID = true;
  vex::task::sleep(1000);
  CenableDrivePID = false;
  armOn = true;
}

void autonomous(void) {
  
  //start task
  vex::task randomName(drivePID);

  //drive onto roller first
  resetDriveSensor = true;
  desiredValue = 300;
  desiredTurnValue = 0;
  
  
  vex::task::sleep(1000);

  
  //back away from roller
  resetDriveSensor = true;
  desiredValue = -500;
  desiredTurnValue = 0;
  
  vex::task::sleep(1000);//

  //turn towards center of field
  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = 560;
  
  
  vex::task::sleep(1000);//

  //drive towards center of field
  resetDriveSensor = true;
  desiredValue = -2000;
  desiredTurnValue = 0;
  
  
  vex::task::sleep(2500);

  //turn towards goal
  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = -960;
  
  
  vex::task::sleep(1700);

  //shoot sequence

  ///set pnuematic to strong shoot
  cataclang.set(true);

  shootarm();

  cataclang.set(false);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = false;

void leftDrivestop(){
  left1.stop();
  left2.stop();
  left3.stop();
}

void rightDrivestop(){
  right1.stop();
  right2.stop();
  right3.stop();
}




// All Pneumatics programming///////////
void catapultRight(){
  //if(Brain.Timer.value() > 95){
    ExpandR.set(false);
    ExpandL.set(false);
  //}
}



bool helpme = true;
void cataclanger(){
  if(helpme){
  cataclang.set(true);
  helpme = false;
  }else{
    cataclang.set(false);
    helpme = true;
  }
}







///////////////////////////////////////


/////////////////GOOD USERCONTROLLER/////////////////

void usercontrol(void) {

  enableDrivePID = false;

  //Controller1.ButtonA.pressed(cataSet);
  Controller1.ButtonB.pressed(shootarm);
  Controller1.ButtonR1.pressed(shootarm);
  // User control code here, inside the loop

  

  //special rotation for michael
  
  //turned off see function for more info
  //Controller1.ButtonX.pressed(shoot);

  //speed change

  //pneumatics for time lols
  Controller1.ButtonRight.pressed(catapultRight);
  //Controller1.ButtonLeft.pressed(catapultLeft);
  Controller1.ButtonDown.pressed(cataclanger);

  //broken not using
  //Controller1.ButtonB.pressed(release);

  //FUNCTION CALL BACKS

  //maybe brain clock

  Brain.Timer.reset();

  while (1) {


      //////////////////////////////////////////////////
  ////////////////DRIVE TRAIN TRAIN///////////////////
  //////////////////////////////////////////////////
  //////////////////////////////////////////////////
  //////////////////////////////////////////////////
  //////////////////////////////////////////////////

      int drivetrainLeftSideSpeed = (int)(Controller1.Axis3.position());
      int drivetrainRightSideSpeed = (int)(Controller1.Axis2.position());


      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          leftDrivestop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          rightDrivestop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {

        left1.setVelocity(drivetrainLeftSideSpeed, percent);
        left2.setVelocity(drivetrainLeftSideSpeed, percent);
        left3.setVelocity(drivetrainLeftSideSpeed, percent);

        left1.spin(forward);
        left2.spin(forward);
        left3.spin(forward);
        
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        right1.setVelocity(drivetrainRightSideSpeed, percent);
        right2.setVelocity(drivetrainRightSideSpeed, percent);
        right3.setVelocity(drivetrainRightSideSpeed, percent);

        right1.spin(forward);
        right2.spin(forward);
        right3.spin(forward);
      }





      /*////////Flywheel //////////////////////
      ////////////////////////////////////////
      ////////////////////////////////////////
      //////////////////////////////////////////
      /////////////////////////////////////////
      */

      
      
      
      
      
      if(Controller1.ButtonL1.pressing())
      {
        //if go spin
        if(armOn == true){
        CATAP.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
      }
      else if(Controller1.ButtonL2.pressing())
      {
        //if go spin t
        if(armOn ==true){
        CATAP.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);

        }
      }
      else {
        //if not go dont go
        if(armOn == true){
        CATAP.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        }
        
      }

    
    //print out time maybe
    
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(Rotation9.position(degrees));
    

    /*STOP THE FUNCTION HERE STOP IT STOP*/
    wait(20, msec); 
  }
}




////////////////////////////////////////////////////////////////
// Main will set up the competition functions and callbacks.///
///////////////////////////////////////////////////////////////
//DONT TOUCH

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
