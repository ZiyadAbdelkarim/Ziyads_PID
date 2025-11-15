/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       theabdelkarims                                            */
/*    Created:      11/15/2025, 7:45:12 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <iostream>
#include <vector>
using namespace vex;
competition Competition;
brain Brain;
motor LMB(PORT1, gearSetting::ratio18_1,true);
motor LMM(PORT2, gearSetting::ratio18_1,true);
motor LMF(PORT3, gearSetting::ratio18_1,true);

// Right Motors
motor RMB(PORT4, gearSetting::ratio18_1,false);
motor RMM(PORT5, gearSetting::ratio18_1,false);
motor RMF(PORT6, gearSetting::ratio18_1,false);
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
  }
}
void pre_auton(void) {

}
void autonomous(void) {

}
void usercontrol(void) {

  while (1) {
    checkTorque();
    checkSpeed();
    checkTempature();
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

