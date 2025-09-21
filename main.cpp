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
  gyro.calibrate()
}
// Lateral movment
double kp = 0.0;
double kd = 0.0;
double ki = 0.0;
int error;
int prev_error=0;
int derivitive;
int intergral=0;
// Genreral info
float wheel_diameter = 3.75;
float pi = 3.14:
float wheel_circumfrence = wheel_diameter*pi;
// Angular Movment
double turn_kp = 0.0;
double turn_kd = 0.0;
double turn_ki = 0.0;

int turn_error;
int turn_prev_error=0;
int turn_derivitive;
int turn_intergral=0;

bool enable_pid = true; 
int PID_Drive(int desired_value, int desired_turn){

  while(enabled_pid && error <=.05){
// Lateral displacement 
  int turns=0;

  // Grabs the pos from left motors
  int LMB_pos = LMB.position(degrees);
  int LMM_pos = LMM.position(degrees);
  int LMF_pos = LMF_pos.position(degrees);

  // Grabs the pos from right motors
  int RMB_pos = RMB.position(degrees);
  int RMM_pos = RMM.position(degrees);
  int RMF_pos = RMF_pos.position(degrees);

  // Gets avg pos to see the distance traveled
  int avg_pos_deg = (LMB_pos+LMM_pos+LMR_pos+RMB_pos+RMM_pos+RMR_pos)/2;

  int turns = avg_pos_deg/360
  int avg_pos = turns*wheel_circumfrence;

  // Checks error for porp
  int error = avg_pos- desired_value;

  // Checks error for Intergral
  intergral+= error;

  // Checks error for derivitive
  int derivitive = erro-prev_error
  double motor_power = kp*error+kd*derivitive+ki*intergral;
  // Motors go forward at desired speed to reach their target dist
  LMB.spin(forward, motor_power, voltageUnits::volt);
  LMM.spin(forward, motor_power, voltageUnits::volt);
  LMF.spin(forward, motor_power, voltageUnits::volt);

  RMB.spin(forward, motor_power, voltageUnits::volt);
  RMM.spin(forward, motor_power, voltageUnits::volt);
  RMF.spin(forward, motor_power, voltageUnits::volt);
    task::sleep(20);
  }

  avg_pos=0;
}
int PID_turn(int desired_turn){
  while(enabled_pid && turn_error <=.05){
    int avg_turn = gyro.rotation();
    int turn_error = avg_turn-desired_turn;
    turn_intergral+=turn_error;
    int derivitive = turn_error-turn_prev_error;
    double turn_motor_power = turn_kp*turn_error+turn_kd*turn_derivitive+turn_ki*turn_intergral;
    LMB.spin(forward, turn_motor_power, voltageUnits::volt);
    LMM.spin(forward, turn_motor_power, voltageUnits::volt);
    LMF.spin(forward, turn_motor_power, voltageUnits::volt);

    RMB.spin(forward, -turn_motor_power, voltageUnits::volt);
    RMM.spin(forward, -turn_motor_power, voltageUnits::volt);
    RMF.spin(forward, -turn_motor_power, voltageUnits::volt);

    turn_prev_error=turn_error;
    task::sleep(20);
}
  gyro.reset();
}
void autonomous(void) {
  vex::task PID_IS_GOATED(PID_Drive)
  vex::task PID_IS_GOATED(PID_turn)
  



}

void usercontrol(void) {

  while (1) {


    wait(20, msec); 
  }
}


int main() {

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  pre_auton();


  while (true) {
    wait(100, msec);
  }
}

