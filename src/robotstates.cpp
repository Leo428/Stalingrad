#include "userincludes/robotstates.hpp"

RobotStates* RobotStates::instance = 0;
const double RobotStates::ROBOT_WIDTH_in = 0;
const double RobotStates::ROBOT_LENGTH_in = 0;
const double RobotStates::ROBOT_MAX_VELOCITY_in = 0;
const double RobotStates::ROBOT_MAX_ACCEL_in = 0;
const double RobotStates::ROBOT_MAX_JERK_in = 0;

RobotStates::FieldColor RobotStates::fieldColor; //initialize the static stuff!
RobotStates::AutoChoice RobotStates::autoChoice;

bool RobotStates::is_Flywheel_Running; 
bool RobotStates::is_autoAiming;
bool RobotStates::is_Aimed;
bool RobotStates::is_Collecting_Ball;
bool RobotStates::is_Shooting_Ball;
bool RobotStates::is_autoAligning;
bool RobotStates::is_Aligned;
bool RobotStates::is_Static_Cam_Detecting; 
bool RobotStates::is_assistant_Shooting;
bool RobotStates::is_autoHooding;
bool RobotStates::is_Hooded;
bool RobotStates::is_oneShot;

double RobotStates::targetY = 0.0;
double RobotStates::targetFlag_Y = 0.0;
double RobotStates::targetFlag_X = 0.0;
double RobotStates::hortizontal_correction = 0.0;
double RobotStates::vertical_correction = 0.0;


RobotStates::RobotStates() {
    
}

RobotStates* RobotStates::getInstance() {
    if(instance == 0) {
        instance = new RobotStates();
    }
    return instance;
}