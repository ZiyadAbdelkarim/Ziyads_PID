/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       azabd                                                     */
/*    Created:      9/26/2025, 4:56:39 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// EDITED
// EDITED
#include "vex.h"
#include <cmath>
using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
// Left motors
motor LMB(PORT1, ratio18_1,true);
motor LMM(PORT2, ratio18_1,true);
motor LMF(PORT3, ratio18_1,true);

// Right Motors
motor RMB(PORT4, ratio18_1,false);
motor RMM(PORT5, ratio18_1,false);
motor RMF(PORT6, ratio18_1,false);
digital_out scraper(Brain.ThreeWirePort.A);
digital_out hood(Brain.ThreeWirePort.B);
// Gyro
inertial Gyro(PORT7);

motor FI(PORT8, ratio18_1, false);
motor HI(PORT9, ratio18_1, false);
motor TI(PORT10, ratio18_1, false);
distance DistanceSensor(PORT11);
distance DistanceSensor2(PORT12);
int timeout=0;
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
  double derivative = 0.0;
  double prev_error = 0.0;
  double dist_travled = 0.0;
// KP,KI,KD
  int timer = 0;
  while (fabs(error)>0.5 && timer <=5000){
    double avg_pos_deg = (LMB.position(deg) + LMM.position(deg) + LMF.position(deg) + RMB.position(deg) + RMM.position(deg) + RMF.position(deg))/6;
    // degrees - inches
    dist_travled = (avg_pos_deg/360.0)*circum;
    error = target_dist-dist_travled;
    derivative = error - prev_error;
    intergral += error;
    double abs_error = fabs(error);
    double abs_derivative = fabs(derivative);
    double abs_intergral = fabs(intergral);
    double KP = 0.90 + 4.60 * (1 - exp(-abs_error / 25));
    double KD = 0.08 + 0.40 * (1 - exp(-abs_derivative / 20.0));
    double KI = 0.0005 + 0.015 * (1 - exp(-abs_intergral / 60.0));
    if (fabs(intergral)>1000){
      intergral=1000*(intergral/fabs(intergral));
    }
    double speed = error*KP+derivative*KD+intergral*KI;
  if (speed > 700) {
    speed = 700;
  }
if (speed < -700) {
  speed = -700;
}
    drive(speed,speed);
    task::sleep(20);
    prev_error=error;

    timer+=20;
  }
  drive_brake();
}

void Gyro_turn(double target_angle, bool gyro_reset){
  if (gyro_reset == true){
    Gyro.resetRotation();
  }
  double t_error = target_angle;
  double t_intergral = 0.0;
  double t_derivative = 0.0;
  double t_prev_error = 0.0;
  double heading = 0.0;
  int timer =  0;
  while(fabs(t_error)>1 && timer <=5000){
    heading = Gyro.rotation(degrees);
    t_error = target_angle-heading;
    t_derivative =t_error-t_prev_error;
    t_intergral+=t_error;
    double abs_t_error = fabs(t_error);
    double abs_t_derivative = fabs(t_derivative);
    double abs_t_intergral = fabs(t_intergral);
    // KP,KI,KD
    double TKP = 0.9 + 4.6 * (1 - exp(-abs_t_error / 25));
    double TKD = 0.10 + 0.4 * (1 - exp(-abs_t_derivative / 18));
    double TKI = 0.00001 + 0.008 * (1 - exp(-abs_t_intergral / 60));
    if (fabs(t_intergral)>1000){
      t_intergral=1000*(t_intergral/fabs(t_intergral));
    }

    double speed =t_error*TKP+t_derivative*TKD+t_intergral*TKI;
    if (speed > 700) {
    speed = 700;
  }
    if (speed < -700) {
  speed = -700;
}
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
  wait(0.5, sec);
  PID_drive(target_dist_ptp);   
  wait(0.5, sec);
  Gyro_turn(angle_orentaiton, false);
  x_pos_original=x_pos;
  y_pos_original=y_pos;
    
  }
enum class Corner{left_red,right_red,left_blue,right_blue, not_selected };
Corner selected_corner = Corner::not_selected; 
void GUI_selection(){
  Brain.Screen.setFont(FontType::PROP40);
  Brain.Screen.setFillColor(red); 
  Brain.Screen.drawRectangle(0,0,120,(272/2));
  Brain.Screen.setFillColor(red); 
  Brain.Screen.drawRectangle(240,0,120,(272/2));
  Brain.Screen.setFillColor(blue); 
  Brain.Screen.drawRectangle(0,(272/2),120,(272/2));
  Brain.Screen.setFillColor(blue); 
  Brain.Screen.drawRectangle(240,(272/2),120,(272/2));
  Brain.Screen.setFillColor(black);
  Brain.Screen.drawLine(240,0,240, 272); 
  Brain.Screen.setFillColor(black); 
  Brain.Screen.drawLine(0,(272/2),480, (272/2));
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.drawText(10, (272/4), "Red Left Corner");
  Brain.Screen.drawText(250, (272/4), "Red Right Corner");
  Brain.Screen.drawText(10, (3*272/4), "Blue Left Corner");
  Brain.Screen.drawText(250, (3*(272/4)), "Blue Right Corner");
  while (selected_corner == Corner::not_selected){
    wait(20, msec);
    if (Brain.Screen.pressing()){
      int x = Brain.Screen.xPosition();
      int y = Brain.Screen.yPosition();
      if(x < 240 && y < 136){
      selected_corner = Corner::left_red;
    }
    if (x>240 && y<136){
      selected_corner = Corner::right_red;
   }
   if (x<240 && y>136){
     selected_corner = Corner::left_blue;
   }
   if (x>240 && y>136){
      selected_corner = Corner::right_blue;
  }
  }
}
  if(selected_corner != Corner::not_selected){
    Brain.Screen.clearScreen();
  }
}
void store_in_hoard(int time){
  FI.spin(forward, 12000,voltageUnits::mV);
  HI.spin(forward, 12000,voltageUnits::mV);
  wait(time, msec);
}
void score_middle(int time){
  timeout= 0;
  while(DistanceSensor.objectDistance(inches)<.75 && timeout < 2000){
    FI.spin(reverse,12000,voltageUnits::mV);
    wait(time, msec);
    timeout+=1;
  }
  if (DistanceSensor.objectDistance(inches)>.75){
    FI.spin(reverse,12000,voltageUnits::mV);
    HI.spin(reverse,12000,voltageUnits::mV);
    wait(time, msec);
    timeout+=1;
  }
}  
void score_lower(int time){
  timeout= 0;
  while(DistanceSensor.objectDistance(inches)<.75 && timeout < 2000){
    FI.spin(forward,12000,voltageUnits::mV);
    timeout+=1;
}
  if(DistanceSensor.objectDistance(inches)>.75){
    timeout= 0;
    FI.spin(forward,12000,voltageUnits::mV);
    HI.spin(forward,12000,voltageUnits::mV);
    wait(time, msec);
    timeout+=1;
  }
}
void score_long_goal(int time){
    timeout= 0;
    while(DistanceSensor.objectDistance(inches)<.75 && timeout < 2000){
    FI.spin(forward,12000,voltageUnits::mV);
    TI.spin(forward,12000,voltageUnits::mV);
    wait(time, msec);
    timeout+=1;
}
  if(DistanceSensor.objectDistance(inches)>.75 && timeout < 2000){
    FI.spin(forward,12000,voltageUnits::mV);
    TI.spin(forward,12000,voltageUnits::mV);
    HI.spin(forward,12000,voltageUnits::mV);
    wait(time, msec);
    timeout+=1;
  }
}
void go_to_long_goal(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){
    point_drive(48.81, 117.00, 90);
  }
 if (selected_corner == Corner::right_red || selected_corner == Corner::left_blue){
    point_drive(-48.81, -117.00, 270);
  }
  score_long_goal(5);
}
void go_to_lower_middle_goal(){
  if (selected_corner != Corner::not_selected){
    point_drive(78.19,62.21, 135);
}
  score_lower(3);
}
 void go_to_upper_middle_goal(){
  if (selected_corner != Corner::not_selected){
    point_drive(78.19,78.21, 315);
  }
  score_middle(3);
 }
 void go_to_match_loader(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){
    point_drive(116.96,4.64,180);
 }
  else if (selected_corner == Corner::right_red || selected_corner == Corner::left_blue){
    point_drive(-116.96,-4.64,180);
 }
}
 void go_pick_up_those_three_blocks(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){
    point_drive(92.14,50.51,135);
  }
  else if (selected_corner == Corner::left_blue || selected_corner == Corner::right_red){
    point_drive(-92.14,-50.51,45);
  }
  store_in_hoard(5);
}
 void pick_up_blocks_under_the_long_goal(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){ 
    point_drive(117.95,68.57,270);
 }
   else if (selected_corner == Corner::right_red || selected_corner == Corner::left_blue){ 
    point_drive(-117.95,-68.57,270);
 }
    store_in_hoard(4);
}
void pre_auton(void) {
  Gyro.calibrate();
  wait(0.1, sec);
  while(Gyro.isCalibrating() && timeout <2000) {
    task::sleep(20); 
    timeout+=1;
  }
  timeout=0;
  GUI_selection();
}
void autonomous(void) {
  bool regularPID = false;
  bool match_auton = true;
  if (regularPID){
  Gyro_turn(-18.21, false);
  wait(.5,sec);
  scraper.set(true);
  hood.set(false);
  store_in_hoard(10);
  wait(2, sec);
  PID_drive(37.2178657);
  wait(.5,sec);
  Gyro_turn(90.00, false);
  wait(.5,sec);
  PID_drive(44.1758909);
  wait(0.5, sec);
  hood.set(true);
  scraper.set(false);
  PID_drive(-3.0);
  wait(0.5, sec);
  Gyro_turn(0.0, false);
  wait(0.5, sec);
  PID_drive(2.0);
  wait(0.5, sec);
  Gyro_turn(90.0, false);
  wait(0.5,sec);
  PID_drive(22.640);
  wait(1,sec);
  Gyro_turn(0.0, false);
  wait(0.5, sec);
  PID_drive(2.0);
  wait(1,sec);
  PID_drive(-2.0);
  wait(1,sec);
  Gyro_turn(90, false);
  wait(0.5, sec);
  PID_drive(-22.640);
  wait(0.5, sec);
  Gyro_turn(180.0, false);
  wait(0.5, sec);
  PID_drive(2.0);
  wait(0.5, sec);
  Gyro_turn(90, false);
  wait(0.5, sec);
  PID_drive(2.0);
  wait(0.5, sec);
  PID_drive(-3.0);
  wait(0.25, sec);
  PID_drive(3.0);
  wait(0.25, sec);
  PID_drive(-2.0);
  wait(0.25, sec);
  Gyro_turn(172, false);
  wait(0.25, sec);
  PID_drive(40.2);
} 
else if(!regularPID) {
  if (match_auton == true){
  scraper.set(true);
  hood.set(false);
  go_to_match_loader();
  scraper.set(false);
  PID_drive(2);
  hood.set(true);
  PID_drive(-2);
  go_pick_up_those_three_blocks();
  go_to_long_goal();
  PID_drive(-3);
  Gyro_turn(0, false);
  wait(0.5, sec);
  PID_drive(2);
  pick_up_blocks_under_the_long_goal();
  go_to_upper_middle_goal();
  PID_drive(-2);
  go_to_lower_middle_goal();
  PID_drive(-2); 
  hood.set(false);
  go_to_long_goal();
  PID_drive(-3);
  PID_drive(3);
 //HELLO :)
  }
}
}
void usercontrol(void) {
  while (1) {
    int leftPower = Controller1.Axis3.position();
    int rightPower = Controller1.Axis2.position();
    drive(leftPower * 12, rightPower * 12);
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
