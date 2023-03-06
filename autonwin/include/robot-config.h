using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor left1;
extern motor left2;
extern motor left3;
extern motor right1;
extern motor right2;
extern motor right3;
extern controller Controller1;
extern motor CATA;
extern motor Intake;
extern digital_out ExpandR;
extern digital_out ExpandL;
extern rotation Rotation9;
extern inertial Inertial10;
extern digital_out cataclang;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );