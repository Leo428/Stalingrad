#include "main.h"
#include "userincludes/subsystems/bigscreentv.hpp"
#include "userincludes/robot.hpp"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    BigScreenTV* bigTV = new BigScreenTV();
    bigTV->createUIComponents();
    
    RobotStates * robotStates = RobotStates::getInstance();
    Robot * robot = Robot::getInstance();
    printf("initializing! \n");
    pros::Task refresh(bigTV->updateScreen);
    pros::delay(200);
    //each controller can only have 3 paths at a time so sad
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
    // 4 to 0.65
    Robot::profileController->generatePath({Point{4_ft, 0_ft, 0_deg}, Point{0.7_ft, 0_ft, 0_deg}}, "B");
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "90deg");
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.65_ft, 0_ft, 0_deg}}, "short"); //0.25 //0.35
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3.25_ft, -0.3_ft, 0_deg}}, "low");
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2_ft, 0.3_ft, 0_deg}}, "back2mid");
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "turn2cap");
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3_ft, 0_ft, 0_deg}}, "flipCap");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    
}