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
// CATA                 motor         10              
// Intake               motor         1               
// ExpandR              digital_out   B               
// ExpandL              digital_out   C               
// Rotation9            rotation      20              
// Inertial10           inertial      6               
// cataclang            digital_out   A               
// LTrack               rotation      3               
// STrack               rotation      16              
// GPS15                gps           15              
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
double turnkI = 0.0005;
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
    turnTotalError += turnError;

    if(turnTotalError > 8){
      turnTotalError = 8;
    }else if(turnTotalError <-8){
      turnTotalError = -8;
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

  }

  return 1;
}
}

double CkP = 1;
double CkI = 0;
double CkD = 0;

int CdesiredValue = -79; //-58

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
    CATA.spin(forward, lateralMotorPower, voltageUnits::volt);




    //items
    CprevError = Cerror;
    
    vex::task::sleep(20);

  }

  vex::task::sleep(20);
}

  return 1;
}

///////////////////////////////////////////odom//////////////////////////////////////
//CONSTANTS / Hard-Coded Values
//Radius of tracking wheels in inches
double WHEEL_RADIUS = 1.369; //was 1.379 //1.43

//Starting angle (relative to field) (RADIANS)
double THETA_START = 0; //imagine the field is a unit circle

//The starting x and y coordinates of the bot (INCHES)
  //These distances are relative to some point (0,0) on the field
  //Relative to: BOTTOM LEFT CORNER
double X_START = 0; //19.1
double Y_START = 0; //8.5

//Distances of tracking wheels from tracking center (INCHES)
double LTrackRadius = 0.375; //was 6.87
//double RTrackRadius = 0; //6.8335
double STrackRadius = -6.125;

//Calculated Values (every loop)
//Angles (DEGREES) *NEEDS TO BE CONVERTED TO RADIANS FOR MATH*
double LPos = 0;
//double RPos = 0;
double SPos = 0;

double LPrevPos = 0;
//double RPrevPos = 0;
double SPrevPos = 0;

//Distances traveled by tracking wheels each loop (INCHES)
double deltaDistL = 0;
//double deltaDistR = 0;
double deltaDistS = 0;

//Distance summations (since last reset)
double totalDeltaDistL = 0;
//double totalDeltaDistR = 0;

//The current angle of the bot (RADIANS)
double currentAbsoluteOrientation = THETA_START;
//The previous angle of the bot (RADIANS)
double previousTheta = THETA_START;

//The change in Theta each loop (RADIANS)
double deltaTheta = 0;

//The Average angle Theta (In RADIANS) throughout the arc
  //currentAbsoluteOrientation + (deltaTheta / 2)
double avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

//The changes in the X and Y positions (INCHES)
/*These are calculated on a local basis each loop,
then converted to global position changes */
double deltaXLocal = 0;
double deltaYLocal = 0;

//The X and Y offsets converted from their local forms (INCHES)
double deltaXGlobal = 0;
double deltaYGlobal = 0;

//The global position of the bot (INCHES)
double xPosGlobal = X_START;
double yPosGlobal = Y_START;


int positionTracking() {
  while(1) {
    //Get encoder values (DEGREES)
    
    LPos = LTrack.position(rotationUnits::deg);
    //RPos = RTrack.position(rotationUnits::deg);
    SPos = STrack.position(rotationUnits::deg);

    //Calculate distance traveled by tracking each wheel (INCHES)
      //Converts degrees to radians
    deltaDistL = ((LPos - LPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    //deltaDistR = ((RPos - RPrevPos) * M_PI / 180) * WHEEL_RADIUS;
    deltaDistS = ((SPos - SPrevPos) * M_PI / 180) * WHEEL_RADIUS;

    //Update previous values to be used next loop (DEGREES)
    LPrevPos = LPos;
    //RPrevPos = RPos;
    SPrevPos = SPos;

    //Total change in each of the L and R encoders since last reset (INCHES)
    //These are used to calculate the absolute orientation of the bot
    totalDeltaDistL += deltaDistL;
    //totalDeltaDistR += deltaDistR;

    //Calculate the current absolute orientation (RADIANS)
    //currentAbsoluteOrientation = THETA_START - ( (totalDeltaDistL - totalDeltaDistR) / (LTrackRadius + RTrackRadius) );
    currentAbsoluteOrientation = (360 - Inertial10.heading()) * M_PI / 180.0;

    //Calculate the change in the angle of the bot (RADIANS)
    deltaTheta = currentAbsoluteOrientation - previousTheta;

    //Update the previous Theta value (RADIANS)  
    previousTheta = currentAbsoluteOrientation;

    //If we didn't turn, then we only translated

    
    

    if(deltaTheta == 0) {
      deltaXLocal = deltaDistS;
      // could be either L or R, since if deltaTheta == 0 we assume they're =
      deltaYLocal = deltaDistL;
    }
    //Else, caluclate the new local position
    else {
      //Calculate the changes in the X and Y values (INCHES)
      //General equation is:
        //Distance = 2 * Radius * sin(deltaTheta / 2)
      deltaXLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistS / deltaTheta) + STrackRadius);
      deltaYLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistL / deltaTheta) - LTrackRadius);

      
    }

    

    //The average angle of the robot during it's arc (RADIANS)
    avgThetaForArc = currentAbsoluteOrientation - (deltaTheta / 2);

    deltaXGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
    deltaYGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));

    //Wraps angles back around if they ever go under 0 or over 2 pi
    while(currentAbsoluteOrientation >= 2 * M_PI) {
      currentAbsoluteOrientation -= 2 * M_PI;
    }
    
    while(currentAbsoluteOrientation < 0) {
      currentAbsoluteOrientation += 2 * M_PI;
    }

    //Update global positions
    xPosGlobal += deltaXGlobal;
    yPosGlobal += deltaYGlobal;


    // Brain.Screen.setCursor(4,8);
    // Brain.Screen.print(deltaDistS/deltaTheta);
    // Brain.Screen.setCursor(5,8);
    // Brain.Screen.print((deltaDistL+deltaDistR)/2);
    // Brain.Screen.setCursor(6,8);
    // Brain.Screen.print(deltaDistS);
    // Brain.Screen.setCursor(7,8);
    // Brain.Screen.print(deltaTheta);

    Brain.Screen.setCursor(8,1);
    Brain.Screen.print(xPosGlobal);
    
    Brain.Screen.setCursor(9,1);
    Brain.Screen.print(yPosGlobal);

    Brain.Screen.setCursor(10,1);
    Brain.Screen.print(currentAbsoluteOrientation);

    Brain.Screen.setCursor(11,1);
    Brain.Screen.print(Inertial10.heading());






    //loop every 20 milliseconds
    task::sleep(10);

  }
  return 1;
}




/////////////////////////////////////////////////////////////////////////////////////

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  left1.setVelocity(100,percent);
  left2.setVelocity(100,percent);
  left3.setVelocity(100,percent);
  right1.setVelocity(100,percent);
  right3.setVelocity(100,percent);
  right2.setVelocity(100,percent);
  Intake.setVelocity(100,percent);

  ExpandL.set(true);
  ExpandR.set(true);

  vex::task randomName2(CdrivePID);
  vex::task randomName3(positionTracking);


  LTrack.setPosition(0,degrees);
  STrack.setPosition(0,degrees);




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
void shootarm(){
  CenableDrivePID = false;
  CATA.spinFor(-0.25,turns);
  CenableDrivePID = true;
}

void autonomous(void) {
  
  //start task
  vex::task randomName(drivePID);

  /*

  wait(1,seconds);

  CenableDrivePID = false;
  cataclang = false;
  CATA.spinFor(-0.25,turns);
  cataclang = true;
  CenableDrivePID = true;
  */

  //put catapult down
  
  /*
  while(cata.pressing() == false){
      Catapult.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
  Catapult.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
  */
  
  //drive forward
  /*
  resetDriveSensor = true;
  desiredValue = 1500;
  desiredTurnValue = 0;
  
  Intake.spin(reverse);
  
  vex::task::sleep(1000);

  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = -1400;
  
  
  vex::task::sleep(750);

  resetDriveSensor = true;
  desiredValue = 800;
  desiredTurnValue = 0;
  
  Intake.spin(forward);
  
  vex::task::sleep(650);

  resetDriveSensor = true;
  desiredValue = -500;
  desiredTurnValue = 0;
  
  vex::task::sleep(1000);

  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = -2000;
  
  
  vex::task::sleep(750);

  resetDriveSensor = true;
  desiredValue = 6000;
  desiredTurnValue = 0;
  
  
  vex::task::sleep(4000);
  ///

  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = 600;
  
  
  vex::task::sleep(750);

  ///

  resetDriveSensor = true;
  desiredValue = 1000;
  desiredTurnValue = 0;
  
  
  vex::task::sleep(1000);

  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = -1400;
  
  
  vex::task::sleep(750);


  

  resetDriveSensor = true;
  desiredValue = 2000;
  desiredTurnValue = 0;
  
  
  vex::task::sleep(1500);

  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = 1400;
  
  
  vex::task::sleep(750);


  resetDriveSensor = true;
  desiredValue = 900;
  desiredTurnValue = 0;
  
  Intake.spin(forward);
  
  vex::task::sleep(800);

  resetDriveSensor = true;
  desiredValue = -500;
  desiredTurnValue = 0;
  
  vex::task::sleep(1000);
*/




  /*

  //turn right
  resetDriveSensor = true;
  desiredValue = -400;
  desiredTurnValue = -1400;
  
vex::task::sleep(1500);

  //move forward
  resetDriveSensor = true;
  desiredValue = -2300;
  desiredTurnValue = -400;

  vex::task::sleep(2000);

  Catapult.spinFor(forward,1,turns);
  */
  /*
  resetDriveSensor = true;
  desiredValue = -5000;
  desiredTurnValue = -500;

  vex::task::sleep(2000);

  
  resetDriveSensor = true;
  desiredValue = -2200;
  desiredTurnValue = -400;

   vex::task::sleep(3000);
   
    resetDriveSensor = true;
  desiredValue = -1000;
  desiredTurnValue = 0;

   vex::task::sleep(1250);

    resetDriveSensor = true;
  desiredValue = -1000;
  desiredTurnValue = -800;

   vex::task::sleep(1500);

   resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = 2000;

   vex::task::sleep(1500);
   resetDriveSensor = true;
  desiredValue = -3000;
  desiredTurnValue = 0;

   vex::task::sleep(1500);

    

  */

  /*
  resetDriveSensor = true;
  desiredValue = -600;
  desiredTurnValue = 1700;
  */
  /*
  
  
  
  
  

  vex::task::sleep(3000);
  
  //turn to shooting position set true to open
  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = -600;

  //shoot pneumatic one set true to open

  
  
  vex::task::sleep(1500);

  Right.set(true);

  vex::task::sleep(1500);

  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = -600;

  vex::task::sleep(3000);

  Left.set(true);
  */

  //shoot pnuematic 2

  /*
  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = 1640;
  
  
  vex::task::sleep(3000);

  resetDriveSensor = true;
  desiredValue = -2500;
  desiredTurnValue = 0;

  vex::task::sleep(3000);

  resetDriveSensor = true;
  desiredValue = 0;
  desiredTurnValue = 1640;
  
  
  vex::task::sleep(3000);
  */

  
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

/*
int desiredValue = 0;
int error;


int throwPos(){
  while(1){
    

    error = desiredValue - Catapult.position(degrees);




    left1.spin(reverse,error, voltageUnits::volt);
  }
}
*/



//spazzes sometimes not using right now something with the two function conflicting
/*
void shoot(){
  num++;
  Catapult.spinFor(forward,1,turns);
  wait(0.25,sec);
  num--;
}
*/

//speed change sentry and other function


//break out of limit switch thing in case of issue
//not being used doesnt work dont know why
/*
void release(){
  num = 0;
}
*/

// All Pneumatics programming///////////
void catapultRight(){
  //if(Brain.Timer.value() > 95){
    ExpandR.set(false);
    ExpandL.set(false);
  //}
}



void catapultLeft(){
  //if(Brain.Timer.value() > 95){
    ExpandL.set(true);
    wait(0.5,seconds);
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



void cataSet(){
  if(CenableDrivePID){
    CenableDrivePID = false;
  }else{
    CenableDrivePID = true;
  }
}

void shootarmcorrect(){
  
  CenableDrivePID = false;
  CATA.stop();
  CATA.spinFor(0.8,turns);
  CenableDrivePID = true;
}

///////////////////////////////////////


/////////////////GOOD USERCONTROLLER/////////////////

void usercontrol(void) {

  enableDrivePID = false;

  //Controller1.ButtonA.pressed(cataSet);
  Controller1.ButtonB.pressed(shootarm);
  // User control code here, inside the loop

  Controller1.ButtonY.pressed(shootarmcorrect);

  //special rotation for michael
  
  //turned off see function for more info
  //Controller1.ButtonX.pressed(shoot);

  //speed change

  //pneumatics for time lols
  Controller1.ButtonRight.pressed(catapultRight);
  //Controller1.ButtonLeft.pressed(catapultLeft);
  Controller1.ButtonDown.pressed(cataclanger);

  Controller1.ButtonR1.pressed(shootarm);

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
        Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      }
      else if(Controller1.ButtonL2.pressing())
      {
        //if go spin other
        Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      }
      else {
        //if not go dont go
        
        Intake.spin(vex::directionType::fwd, 0, vex::velocityUnits::pct);
        
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
