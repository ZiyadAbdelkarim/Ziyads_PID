/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       theabdelkarims                                            */
/*    Created:      9/21/2025, 9:39:57 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

competition Competition;

// Setting up motors and gyroscope:

motor LMB(PORT1, ratio6_1, true);
motor LMM(PORT2, ratio6_1, true);
motor LMF(PORT3, ratio6_1, true);

motor RMB(PORT4, ratio6_1, true);
motor RMM(PORT5, ratio6_1, true);
motor RMF(PORT6, ratio6_1, true);

inertial gyro(PORT7);



void pre_auton(void) {

}
double kp = 0.0;
double kd = 0.0;
double ki = 0.0;

int error;
int prev_error;
int derivitive;
int intergral;
bool enable_pid = true 
int PID_Drive(){
  while(PID_Drive){
    
    prev_error=error;
    task::sleep(20);
  }
}
void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
