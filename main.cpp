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
#include <iostream>
#include <vector>
competition Competition;
brain Brain;
controller Controller1;
bool regularPID = false;
bool match_auton = false;
// Left motors
motor LMB(PORT1, gearSetting::ratio18_1,true);
motor LMM(PORT2, gearSetting::ratio18_1,true);
motor LMF(PORT3, gearSetting::ratio18_1,true);

// Right Motors
motor RMB(PORT4, gearSetting::ratio18_1,false);
motor RMM(PORT5, gearSetting::ratio18_1,false);
motor RMF(PORT6, gearSetting::ratio18_1,false);
digital_out scraper(Brain.ThreeWirePort.A);
digital_out hood(Brain.ThreeWirePort.B);
// Gyro
inertial Gyro(PORT7);

motor FI(PORT8, gearSetting::ratio18_1, false);
motor HI(PORT9, gearSetting::ratio18_1, false);
motor TI(PORT10, gearSetting::ratio18_1, false);
distance DistanceSensor(PORT11);
distance DistanceSensor2(PORT12);
void stop_intake(){
  FI.stop();
  TI.stop();
  HI.stop();
}
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
  LMB.resetPosition();
  LMM.resetPosition();
  LMF.resetPosition();
  RMB.resetPosition();
  RMM.resetPosition();
  RMF.resetPosition();
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
void checkTorque(){
  double motor_torque_LMB = LMB.torque(torqueUnits::Nm);
  double motor_torque_LMM = LMM.torque(torqueUnits::Nm);
  double motor_torque_LMF = LMF.torque(torqueUnits::Nm);

  double motor_torque_RMB = RMB.torque(torqueUnits::Nm);
  double motor_torque_RMM = RMM.torque(torqueUnits::Nm);
  double motor_torque_RMF = RMF.torque(torqueUnits::Nm);
  double motor_torque[] = {motor_torque_LMB, motor_torque_LMF,motor_torque_LMM,motor_torque_RMB,motor_torque_RMF,motor_torque_RMM};
  std::vector<int> motorsIncluded;
  std::string motorNames[] = { "LMB", "LMF", "LMM", "RMB", "RMF", "RMM" };
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.print("LMB Torque: %.2f Nm", motor_torque_LMB);
  Brain.Screen.newLine();
  Brain.Screen.print("LMM Torque: %.2f Nm", motor_torque_LMM);
  Brain.Screen.newLine();
  Brain.Screen.print("LMF Torque: %.2f Nm", motor_torque_LMF);
  Brain.Screen.newLine();
  Brain.Screen.print("RMB Torque: %.2f Nm", motor_torque_RMB);
  Brain.Screen.newLine();
  Brain.Screen.print("RMM Torque: %.2f Nm", motor_torque_RMM);
  Brain.Screen.newLine();
  Brain.Screen.print("RMF Torque: %.2f Nm", motor_torque_RMF);
  Brain.Screen.newLine();
  Brain.Screen.print("LOW TOURQUE MOTORS:");
  for (int i = 0; i < 6; ++i) {
    if (motor_torque[i] < 1.2) {
      motorsIncluded.push_back(i);
    }
  }
  if (motorsIncluded.empty()){
      Brain.Screen.setPenColor(color::white);
      Brain.Screen.print("NONE");
  }
  else {
    Brain.Screen.setPenColor(color::red);
    for (int idx : motorsIncluded) {
      Brain.Screen.print(" %s(%.2f)", motorNames[idx].c_str(), motor_torque[idx]);
    }
  drive_brake();
}

}
void checkSpeed(){
  double motor_velocity_LMB = LMB.velocity(velocityUnits::rpm);
  double motor_velocity_LMM = LMM.velocity(velocityUnits::rpm);
  double motor_velocity_LMF = LMF.velocity(velocityUnits::rpm);

  double motor_velocity_RMB = RMB.velocity(velocityUnits::rpm);
  double motor_velocity_RMM = RMM.velocity(velocityUnits::rpm);
  double motor_velocity_RMF = RMF.velocity(velocityUnits::rpm);
  double motor_speed[] = {motor_velocity_LMB, motor_velocity_LMM,motor_velocity_LMF,motor_velocity_RMB,motor_velocity_RMM,motor_velocity_RMF};
  std::vector<int> motorsIncluded;
  std::string motorNames[] = { "LMB", "LMM", "LMF", "RMB", "RMM", "RMF" };
  Brain.Screen.setCursor(1,30);
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.print("LMB velocity: %.2f rpm", motor_velocity_LMB);
  Brain.Screen.newLine();
  Brain.Screen.print("LMM velocity: %.2f rpm", motor_velocity_LMM);
  Brain.Screen.newLine();
  Brain.Screen.print("LMF velocity: %.2f rpm", motor_velocity_LMF);
  Brain.Screen.newLine();
  Brain.Screen.print("RMB velocity: %.2f rpm", motor_velocity_RMB);
  Brain.Screen.newLine();
  Brain.Screen.print("RMM velocity: %.2f rpm", motor_velocity_RMM);
  Brain.Screen.newLine();
  Brain.Screen.print("RMF velocity: %.2f rpm", motor_velocity_RMF);
  Brain.Screen.newLine();
  Brain.Screen.print("LOW VELOCITY MOTORS:");
  Brain.Screen.newLine();
  for (int i = 0; i < 6; ++i) {
    if (motor_speed[i] < 1.2) {
      motorsIncluded.push_back(i);
    }
  }
  if (motorsIncluded.empty()){
      Brain.Screen.setPenColor(color::white);
      Brain.Screen.print("NONE");
  }
  else {
    Brain.Screen.setPenColor(color::red);
    for (int idx : motorsIncluded) {
      Brain.Screen.print(" %s(%.2f)", motorNames[idx].c_str(), motor_speed[idx]);
      Brain.Screen.newLine();
    }
  
  }
}
void checkTempature(){
  double motor_temp_LMB = LMB.temperature(temperatureUnits::celsius);
  double motor_temp_LMM = LMM.temperature(temperatureUnits::celsius);
  double motor_temp_LMF = LMF.temperature(temperatureUnits::celsius);

  double motor_temp_RMB = RMB.temperature(temperatureUnits::celsius);
  double motor_temp_RMM = RMM.temperature(temperatureUnits::celsius);
  double motor_temp_RMF = RMF.temperature(temperatureUnits::celsius);
  double motor_temp[] = {motor_temp_LMB, motor_temp_LMM,motor_temp_LMF,motor_temp_RMB,motor_temp_RMM,motor_temp_RMF};
  std::vector<int> motorsIncluded;
  std::string motorNames[] = { "LMB", "LMM", "LMF", "RMB", "RMM", "RMF" };
  Brain.Screen.setCursor(1,65);
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.print("LMB temp: %.2f C", motor_temp_LMB);
  Brain.Screen.newLine();
  Brain.Screen.print("LMM temp: %.2f C", motor_temp_LMM);
  Brain.Screen.newLine();
  Brain.Screen.print("LMF temp: %.2f C", motor_temp_LMF);
  Brain.Screen.newLine();
  Brain.Screen.print("RMB temp: %.2f C", motor_temp_RMB);
  Brain.Screen.newLine();
  Brain.Screen.print("RMM temp: %.2f C", motor_temp_RMM);
  Brain.Screen.newLine();
  Brain.Screen.print("RMF temp: %.2f C", motor_temp_RMF);
  Brain.Screen.setPenColor(color::orange);
  Brain.Screen.newLine();
  Brain.Screen.print("HIGH TEMPERATURE MOTORS:");
  Brain.Screen.newLine();
 for (int i = 0; i < 6; ++i) {
    if (motor_temp[i] >70) {
      motorsIncluded.push_back(i);
    }
  }
  if (motorsIncluded.empty()){
      Brain.Screen.setPenColor(color::white);
      Brain.Screen.print("NONE");
  }
  else {
    Brain.Screen.setPenColor(color::red);
    for (int idx : motorsIncluded) {
      Brain.Screen.print(" %s(%.2f)", motorNames[idx].c_str(), motor_temp[idx]);
      Brain.Screen.newLine();
    }
  
  drive_brake();
  }
}
double x_pos_original = 107.6;
double y_pos_original = 28.9;
void point_drive(double x_pos, double y_pos, double angle_orentaiton){
  double differencex = x_pos-x_pos_original;
  double differencey = y_pos - y_pos_original;
  double target_angle_ptp = atan2(differencey, differencex) * (180.0 / pi);
  double target_dist_ptp = sqrt(pow(differencex, 2)+pow(differencey, 2));    
  Gyro_turn(target_angle_ptp, false);
  task::sleep(0.5);
  PID_drive(target_dist_ptp);   
  task::sleep(0.5);
  Gyro_turn(angle_orentaiton, false);
  x_pos_original=x_pos;
  y_pos_original=y_pos;
    
  }
enum class Corner{left_red,right_red,left_blue,right_blue, not_selected };
Corner selected_corner = Corner::not_selected; 
void GUI_selection(){
  Brain.Screen.setFillColor(red); 
  Brain.Screen.drawRectangle(0,0,120,136);
  Brain.Screen.setFillColor(red); 
  Brain.Screen.drawRectangle(240,0,120, 136);
  Brain.Screen.setFillColor(blue); 
  Brain.Screen.drawRectangle(0,(272/2),120,136);
  Brain.Screen.setFillColor(blue); 
  Brain.Screen.drawRectangle(240,(272/2),120,136);
  Brain.Screen.setFillColor(black);
  Brain.Screen.drawLine(240,(272/2),0, (272/2)); 
  Brain.Screen.setFillColor(black); 
  Brain.Screen.drawLine(0,(272/2),480,0);
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.printAt(10, (272/4), "Red Left Corner");
  Brain.Screen.printAt(250, (272/4), "Red Right Corner");
  Brain.Screen.printAt(10, (3*(272/4)), "Blue Left Corner");
  Brain.Screen.printAt(250, (3*(272/4)), "Blue Right Corner");
  if (match_auton == true){
    while (selected_corner == Corner::not_selected){
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
  }
   Brain.Screen.setCursor(1,1);
   Brain.Screen.print("please press brain when ready");
}
void store_in_hoard(int time){
  int timeout =0;
  while(DistanceSensor2.objectDistance(inches)<0.75 && DistanceSensor2.objectDistance(inches)> 0.0 && timeout < 200){
    FI.spin(forward, 12000,voltageUnits::mV);
    HI.spin(forward, 12000,voltageUnits::mV);
    task::sleep(time);
   timeout+=time;
}
  if(DistanceSensor2.objectDistance(inches)<0.75 && DistanceSensor2.objectDistance(inches)> 0.0 && DistanceSensor.objectDistance(inches)>0.75&& timeout < 200){
    FI.spin(reverse, 12000,voltageUnits::mV);
    HI.spin(forward, 12000,voltageUnits::mV);
    TI.spin(reverse, 12000,voltageUnits::mV);
    task::sleep(time);
   timeout+=time;
  }
   stop_intake();
}
void score_middle(int time){
  int timeout = 0;
  while(DistanceSensor.objectDistance(inches)<0.75 && DistanceSensor.objectDistance(inches)> 0.0 && timeout < 200){
    FI.spin(reverse,12000,voltageUnits::mV);
   timeout+=time;
  }
  if (DistanceSensor.objectDistance(inches)>0.75){
    FI.spin(reverse,12000,voltageUnits::mV);
    HI.spin(reverse,12000,voltageUnits::mV);
    task::sleep(time);
    timeout+=time;
  }
  stop_intake();
}  
void score_lower(int time){
  int timeout = 0;
  while(DistanceSensor.objectDistance(inches)<0.75 &&  DistanceSensor.objectDistance(inches)> 0.0 && timeout < 200){
    FI.spin(forward,12000,voltageUnits::mV);
    timeout+=time;
}
  if(DistanceSensor.objectDistance(inches)>0.75){
    FI.spin(forward,12000,voltageUnits::mV);
    HI.spin(forward,12000,voltageUnits::mV);
    task::sleep(time);
    timeout+=time;
  }
   stop_intake();
}
void score_long_goal(int time){
  int timeout = 0;
    while(DistanceSensor.objectDistance(inches)<0.75 && DistanceSensor.objectDistance(inches)> 0.0 && timeout < 200){
    FI.spin(forward,12000,voltageUnits::mV);
    TI.spin(forward,12000,voltageUnits::mV);
    task::sleep(time);
    timeout+=time;
}
  if(DistanceSensor.objectDistance(inches)>0.75){
    FI.spin(forward,12000,voltageUnits::mV);
    TI.spin(forward,12000,voltageUnits::mV);
    HI.spin(forward,12000,voltageUnits::mV);
    task::sleep(time);
    timeout+=time;
  }
   stop_intake();
}
void go_to_long_goal(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){
    point_drive(48.81, 117.00, 90);
  }
 if (selected_corner == Corner::right_red || selected_corner == Corner::left_blue){
    point_drive(-48.81, -117.00, 270);
  }
  score_long_goal(5000);
}
void go_to_lower_middle_goal(){
  if (selected_corner != Corner::not_selected){
    point_drive(78.19,62.21, 135);
}
  score_lower(3000);
}
 void go_to_upper_middle_goal(){
  if (selected_corner != Corner::not_selected){
    point_drive(78.19,78.21, 315);
  }
  score_middle(3000);
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
  store_in_hoard(5000);
}
 void pick_up_blocks_under_the_long_goal(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){ 
    point_drive(117.95,68.57,270);
 }
   else if (selected_corner == Corner::right_red || selected_corner == Corner::left_blue){ 
    point_drive(-117.95,-68.57,270);
 }
    store_in_hoard(4000);
}
void go_to_long_goal2(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){
    point_drive(23,48, 90);
  }
  score_long_goal(5000);
}
void go_to_match_loader2(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){
    point_drive(23,4.5, 90);
  }
  store_in_hoard(5000);
}
void go_pick_up_those_three_blocks2(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){
    point_drive(47,50, 45);
  }
  store_in_hoard(3000);
}
 void pick_up_blocks_under_the_long_goal2(){
  if (selected_corner == Corner::left_red || selected_corner == Corner::right_blue){ 
    point_drive(24,68.57,270);
 }
    store_in_hoard(4000);
}
 void park(){
  point_drive(62,14.86,270);
  store_in_hoard(4000);
 }
 void park2(){
  point_drive(62, 141,90);
  store_in_hoard(2000);
 }
 void pick_up_side_blocks(){
  point_drive(0.9, 26, 0);
  store_in_hoard(3);
  point_drive(139, 26,0);
  store_in_hoard(4000);
 }
 void pick_up_block_continusly(){
  store_in_hoard(100000);
 }
 void go_pick_up_those_three_blocks3(){
  point_drive(90,95,0);
  point_drive(45,95,0);
  PID_drive(-45);
 }

void pre_auton(void) {
  int timeout = 0;
  Gyro.calibrate();
  task::sleep(0.1); 
  while(Gyro.isCalibrating() && timeout <2000) {
    task::sleep(500); 
    timeout+=500;
  }
  timeout=0;
  GUI_selection();
}
void autonomous(void) {
  if (regularPID){
  Gyro_turn(-18.21, false);
  task::sleep(100);
  scraper.set(true);
  hood.set(false);
  store_in_hoard(100);
  task::sleep(2);
  PID_drive(37.2178657);
  task::sleep(100);
  Gyro_turn(90.00, false);
  task::sleep(100);
  PID_drive(44.1758909);
  task::sleep(100);
  hood.set(true);
  scraper.set(false);
  PID_drive(-3.0);
  task::sleep(100);
  Gyro_turn(0.0, false);
  task::sleep(100);
  PID_drive(2.0);
  task::sleep(100);
  Gyro_turn(90.0, false);
  task::sleep(100);
  PID_drive(22.640);
  task::sleep(100);
  Gyro_turn(0.0, false);
  task::sleep(100);
  PID_drive(2.0);
  task::sleep(100);
  PID_drive(-2.0);
  task::sleep(100);
  Gyro_turn(90, false);
  task::sleep(100);
  PID_drive(-22.640);
  task::sleep(100);
  Gyro_turn(180.0, false);
  task::sleep(100);
  PID_drive(2.0);
  task::sleep(100);
  Gyro_turn(90, false);
  task::sleep(100);
  PID_drive(2.0);
  task::sleep(100);
  PID_drive(-3.0);
  task::sleep(100);
  PID_drive(3.0);
  task::sleep(100);
  PID_drive(-2.0);
  task::sleep(100);
  Gyro_turn(172, false);
  task::sleep(100);
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
  task::sleep(0.5);
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
if (match_auton==false){
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
  task::sleep(0.5);
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
  PID_drive(-1);
  scraper.set(true);
  hood.set(false);
  go_to_match_loader2();
  scraper.set(false);
  PID_drive(2);
  hood.set(true);
  PID_drive(-2);
  go_pick_up_those_three_blocks2();
  go_to_long_goal2();
  PID_drive(-3);
  Gyro_turn(0, false);
  task::sleep(0.5);
  PID_drive(2);
  pick_up_blocks_under_the_long_goal2();
  go_to_upper_middle_goal();
  PID_drive(-2);
  go_to_lower_middle_goal();
  PID_drive(-2); 
  hood.set(false);
  go_to_long_goal2();
  PID_drive(-3);
  PID_drive(3);
  park();
  go_to_long_goal2();
  pick_up_side_blocks();
  go_to_long_goal2();
  park();
  PID_drive(-5);
  park2();
  go_to_long_goal2();
  pick_up_block_continusly();
  PID_drive(-4);
  go_pick_up_those_three_blocks3();
  go_to_long_goal2();
  park();
}
}
}
void usercontrol(void) {
  bool toggle_scrapper_new = true;
  bool toggle_scrapper_old = false;
  bool toggle_hood_new = true;
  bool toggle_hood_old = false;
  while (1) {
    if (Controller1.ButtonR1.pressing()){
      stop_intake();
      task::sleep(10);
      score_long_goal(1000000);
    }
    if (Controller1.ButtonR2.pressing()){
     task::sleep(10);
      stop_intake();
      store_in_hoard(10000000);
    }
    if (Controller1.ButtonL1.pressing()){
      task::sleep(10);
      stop_intake();
      score_middle(10000000);
    }
    if (Controller1.ButtonL2.pressing()){
     task::sleep(10);
      stop_intake();
      score_lower(1000000000);
    }
    if (Controller1.ButtonA.pressing()){
      if (toggle_scrapper_new == true && toggle_scrapper_old == false){
        task::sleep(10);
        scraper.set(true);
        toggle_scrapper_old = !toggle_scrapper_old;
      }
      if (toggle_scrapper_new == false && toggle_scrapper_old == true){
        task::sleep(10);
        scraper.set(false);
        toggle_scrapper_old = !toggle_scrapper_old;        
      }
    if (Controller1.ButtonB.pressing())
      if (toggle_hood_old == false && toggle_hood_new == true){
        task::sleep(10);
        hood.set(true);
        toggle_hood_new = !toggle_hood_new;
      }
      if (toggle_hood_new == false && toggle_hood_old==true){
        task::sleep(10);
        hood.set(false);
        toggle_hood_old= !toggle_hood_old;        
      }
    }
      task::sleep(20); 
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();

  while (true) {
    task::sleep(100);
  }
}
