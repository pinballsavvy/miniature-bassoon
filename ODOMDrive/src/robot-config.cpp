#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor left1 = motor(PORT7, ratio6_1, true);
motor left2 = motor(PORT8, ratio6_1, true);
motor left3 = motor(PORT9, ratio6_1, true);
motor right1 = motor(PORT17, ratio6_1, false);
motor right2 = motor(PORT18, ratio6_1, false);
motor right3 = motor(PORT19, ratio6_1, false);
controller Controller1 = controller(primary);
motor CATA = motor(PORT10, ratio36_1, false);
motor Intake = motor(PORT1, ratio36_1, true);
digital_out ExpandR = digital_out(Brain.ThreeWirePort.B);
digital_out ExpandL = digital_out(Brain.ThreeWirePort.C);
rotation Rotation9 = rotation(PORT20, false);
inertial Inertial10 = inertial(PORT6);
digital_out cataclang = digital_out(Brain.ThreeWirePort.A);
rotation LTrack = rotation(PORT3, false);
rotation STrack = rotation(PORT16, false);
gps GPS15 = gps(PORT15, 0.00, 203.20, mm, 0);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}