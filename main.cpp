/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       azabd                                                     */
/*    Created:      9/26/2025, 4:56:39 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
// Left motors
motor LMB(PORT1, ratio18_1,true);
motor LMM(PORT2, ratio18_1,true);
motor LMF(PORT3, ratio18_1,true);

// Right Motors
motor RMB(PORT4, ratio18_1,false);
motor RMM(PORT5, ratio18_1,false);
motor RMF(PORT6, ratio18_1,false);

// Gyro
inertial Gyro(PORT7);

void drive(int left_speed, int right_speed){
  // Left motors
  LMB.spin(forward, left_speed, voltageUnits::mV);
  LMM.spin(forward, left_speed, voltageUnits::mV);
  LMF.spin(forward, left_speed, voltageUnits::mV);
// Right motors
  RMB.spin(forward, right_speed, voltageUnits::mV);
  RMM.spin(forward, right_speed, voltageUnits::mV);
  RMF.spin(forward, right_speed, voltageUnits::mV);
}

void drive_brake(){
  LMB.stop(brake);
  LMM.stop(brake);
  LMF.stop(brake);
  RMB.stop(brake);
  RMM.stop(brake);
  RMF.stop(brake);
}
void reset_drive(){
  LMB.resetRotation();
  LMM.resetRotation();
  LMF.resetRotation();
  RMB.resetRotation();
  RMM.resetRotation();
  RMF.resetRotation();
}
// Constants
float wheel_dia = 3.25;
float pi = 3.14;
float circum = wheel_dia*pi;
void PID_drive(float target_dist){
  reset_drive();
// Errors
  float error = target_dist;
  float intergral = 0.0;
  float derivitave = 0.0;
  float prev_error = 0.0;
  float dist_travled = 0.0;
// KP,KI,KD
  float KP = 200.0;
  float KD = 5.0;
  float KI = 0.1;

  while (fabs(error)>0.1){
    float avg_pos_deg = (LMB.position(deg) + LMM.position(deg) + LMF.position(deg) + RMB.position(deg) + RMM.position(deg) + RMF.position(deg))/6;
    // degrees - inches
    dist_travled = (avg_pos_deg/360.0)*circum;
    error = target_dist-dist_travled;
    derivitave = error - prev_error;
    intergral += error;
    if (fabs(intergral)>1000){
      intergral=1000*(intergral/fabs(intergral));
    }
    drive(error*KP+derivitave*KD+intergral*KI, error*KP+derivitave*KD+intergral*KI);
    task::sleep(20);
    prev_error=error;
  }
  drive_brake();
}

void Gyro_turn(float target_angle){
  float t_error = target_angle;
  float t_intergral = 0.0;
  float t_derivitave = 0.0;
  float t_prev_error = 0.0;
// KP,KI,KD
  float TKP = 0.1;
  float TKD = 0.01;
  float TKI = 0.001;
  float heading = 0.0;
  while(fabs(t_error)>0.5){
    heading = Gyro.rotation(degrees);
    t_error = target_angle-heading;
    t_derivitave =t_error-t_prev_error;
    t_intergral+=t_error;
    if (fabs(t_intergral)>1000){
      t_intergral=1000*(t_intergral/fabs(t_intergral));
    }
    float speed =t_error*TKP+t_derivitave*TKD+t_intergral*TKI;
    drive(speed,-speed);
    task::sleep(20);
    t_prev_error=t_error;
  }
  Gyro.resetRotation();
  drive_brake();
}
void pre_auton(void) {
  Gyro.calibrate();
  while(Gyro.isCalibrating()) {
    task::sleep(20); 
  }
}
void autonomous(void) {
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
