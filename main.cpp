/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       azabd                                                     */
/*    Created:      9/26/2025, 4:56:39 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
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
digital_out piston(Brain.ThreeWirePort.A);
// Gyro
inertial Gyro(PORT7);

motor FI(PORT8, ratio18_1, false);
motor HI(PORT9, ratio18_1, false);
motor TI(port10, ratio18_1, false);
distance DistanceSensor(PORT11);
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
double wheel_dia = 3.25;
double pi = 3.14;
double circum = wheel_dia*pi;
void PID_drive(double target_dist){
  reset_drive();
// Errors
  double error = target_dist;
  double intergral = 0.0;
  double derivitave = 0.0;
  double prev_error = 0.0;
  double dist_travled = 0.0;
// KP,KI,KD
  double KP = 4.5;
  double KD = 0.8;
  double KI = 0.015;

  while (fabs(error)>0.5 && timer <=2000){
    double avg_pos_deg = (LMB.position(deg) + LMM.position(deg) + LMF.position(deg) + RMB.position(deg) + RMM.position(deg) + RMF.position(deg))/6;
    // degrees - inches
    dist_travled = (avg_pos_deg/360.0)*circum;
    error = target_dist-dist_travled;
    derivitave = error - prev_error;
    intergral += error;
    if (fabs(intergral)>1000){
      intergral=1000*(intergral/fabs(intergral));
    }
    double speed = error*KP+derivitave*KD+intergral*KI;
  if (speed > 12000) {
    speed = 12000;
  }
if (speed < -12000) {
  speed = -12000;
}
    drive(speed,speed);
    task::sleep(20);
    prev_error=error;
  }
  drive_brake();
}

void Gyro_turn(double target_angle, bool gyro_reset){
  if (gyro_reset == true){
    Gyro.resetRotation();
  }
  double t_error = target_angle;
  double t_intergral = 0.0;
  double t_derivitave = 0.0;
  double t_prev_error = 0.0;
// KP,KI,KD
  double TKP = 3.8;
  double TKD = 0.3;
  double TKI = 0.005;
  double heading = 0.0;
  int timer =  0;
  while(fabs(t_error)>1 && timer <=2000){
    heading = Gyro.rotation(degrees);
    t_error = target_angle-heading;
    t_derivitave =t_error-t_prev_error;
    t_intergral+=t_error;
    if (fabs(t_intergral)>1000){
      t_intergral=1000*(t_intergral/fabs(t_intergral));
    }
    double speed =t_error*TKP+t_derivitave*TKD+t_intergral*TKI;
    drive(speed,-speed);
    task::sleep(20);
    t_prev_error=t_error;
    timer+=20;
  }
  drive_brake();
}
double x_pos_original = 107.6;
double y_pos_original = 28.9;
void point_drive(double x_pos, double y_pos, double angle_orentaiton){
  double differencex = x_pos-x_pos_original;
  double differencey = y_pos - y_pos_original;
  double target_angle_ptp = atan2(differencey, differencex) * (180.0 / pi);
  double target_dist_ptp = sqrt(pow(differencex, 2)+pow(differencey, 2));    
  Gyro_turn(target_angle_ptp, false);
  wait(.5, sec);
  PID_drive(target_dist_ptp);   
  wait(.5, sec);
  Gyro_turn(angle_orentaiton);
  x_pos_original=x_pos;
  y_pos_original=y_pos;
    
  }
void store_in_hoard(int time){
  FI.spin(forward, 12000,voltageUnits::mV);
  HI.spin(forward, 12000,voltageUnits::mV);
  wait(time, sec);
}
void score_middle(int time){
  while(DistanceSensor.objectDistance(inches)<.75){
    FI.spin(reverse,12000,voltageUnits::mV):
  }
  else(){
    FI.spin(reverse,12000,voltageUnits::mV):
    HI.spin(reverse,12000,voltageUnits::mV):
    wait(time, sec);
  }
}  
void score_lower(int time){
  while(DistanceSensor.objectDistance(inches)<.75){
    FI.spin(forward,12000,voltageUnits::mV):
}
  else(){
    FI.spin(forward,12000,voltageUnits::mV):
    HI.spin(forward,12000,voltageUnits::mV):
    wait(time, sec);
  }
}
void score_long_goal(int time){
    while(DistanceSensor.objectDistance(inches)<.75){
    FI.spin(forward,12000,voltageUnits::mV):
    TI.spin(forward,12000,voltageUnits::mV):
}
  else(){
    FI.spin(forward,12000,voltageUnits::mV):
    FI.spin(forward,12000,voltageUnits::mV):
    HI.spin(forward,12000,voltageUnits::mV):
    wait(time, sec);
  }
}

void go_to_long_goal(){
  point_drive(48.81, 117.00, 90);
  score_long_goal(5);
}
void go_to_lower_middle_goal(){
  point_drive(78.19,62.21, 135);
  score_lower(3);
}
 void go_to_upper_middle_goal(){
  point_drive(78.19,78.21, 315);
  score_middle(3);
 }
 void go_to_match_loader(){
  point_drive(116.96,4.64,180);
  while(DistanceSensor.objectDistance(inches)<10){
    store_in_hoard(3);
  }
  else{
    score_long_goal(7);
  }

 }
 void go_pick_up_those_three_blocks(){
  point_drive(92.14,50.51,135);
  store_in_hoard(5);
}
 void pick_up_blocks_under_the_long_goal(){
  point_drive(117.95,68.57,90);
  store_in_hoard(4);
 }
void pre_auton(void) {
  Gyro.calibrate();
  while(Gyro.isCalibrating()) {
    task::sleep(20); 
  }
}
void autonomous(void) {
  bool regularPID = true;
  if (regularPID){
  Gyro_turn(-18.21, true);
  wait(.5,sec);
  PID_drive(37.2178657);
  wait(.5,sec);
  Gyro_turn(90.00, true);
  wait(.5,sec);
  PID_drive(44.1758909);
  wait(0.5, sec);
  PID_drive(-3.0);
  wait(0.5, sec);
  Gyro_turn(0.0, true);
  wait(0.5, sec);
  PID_drive(2.0);
  wait(0.5, sec);
  Gyro_turn(90.0, true);
  wait(0.5,sec);
  PID_drive(22.640);
  wait(1,sec);
  Gyro_turn(0.0, true);
  wait(0.5, sec);
  PID_drive(2.0);
  wait(1,sec);
  PID_drive(-2.0);
  wait(1,sec);
  Gyro_turn(90, true);
  wait(0.5, sec);
  PID_drive(-22.640);
  wait(0.5, sec);
  Gyro_turn(180.0, true);
  wait(0.5, sec);
  PID_drive(2.0);
  wait(0.5, sec);
  Gyro_turn(90, true);
  wait(0.5, sec);
  PID_drive(2.0);
  wait(0.5, sec);
  PID_drive(-3.0);
  wait(0.25, sec);
  PID_drive(3.0);
  wait(0.25, sec);
  PID_drive(-2.0);
  wait(0.25, sec);
  Gyro_turn(172, true);
  wait(0.25, sec);
  PID_drive(40.2);
}
else {
  go_to_match_loader();
  go_to_long_goal();
  go_pick_up_those_three_blocks();
  go_to_long_goal();
  pick_up_blocks_under_the_long_goal();
  go_to_upper_middle_goal();
  go_to_lower_middle_goal();
 
}
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
