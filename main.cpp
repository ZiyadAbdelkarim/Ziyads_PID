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
float wheel_diameter = 3.75;
float pi = 3.14:
float wheel_circumfrence = wheel_diameter*pi;
int desired_value_deg = 200;
int error;
int prev_error=0;
int derivitive;
int intergral=0;
bool enable_pid = true; 
int PID_Drive(){
  while(PID_Drive){
  // Grabs the pos from left motors
  int LMB_pos = LMB.position(degrees);
  int LMM_pos = LMM.position(degrees);
  int LMF_pos = LMF_pos.position(degrees);

  // Grabs the pos from right motors
  int RMBB_pos = RMB.position(degrees);
  int RMM_pos = RMM.position(degrees);
  int RMF_pos = RMF_pos.position(degrees);

  // Gets avg pos to see the distance traveled
  int avg_pos = (LMB_pos+LMM_pos+LMR_pos+RMB_pos+RMM_pos+RMR_pos)/2;

  // Checks error for porp
  int error = avg_pos- desired_value;

  // Checks error for Intergral
  total_error+= error;

  // Checks error for derivitive
  int derivitive = erro-prev_error
  double motor_power = kp*error+kd*derivitive+ki*intergral;
    
    prev_error=error;
    task::sleep(20);
  }
  return(1);
}
void autonomous(void) {
  vex::task PID_IS_GOATED(PID_Drive)



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
