#include "userincludes/subsystems/nuclauncher.hpp"

//make sure to initialize all the static members
// double NucLauncher::hoodPID_Output;
// double NucLauncher::target_Y; 

okapi::Motor * NucLauncher::hood_Motor = 0;
okapi::Motor * NucLauncher::flywheel_Motor = 0;
ADIPotentiometer * NucLauncher::pot = nullptr;

NucLauncher::NucLauncher(){
    initMotors();
    pot = new ADIPotentiometer('h');
}

void NucLauncher::initMotors() {
    flywheel_Motor = new okapi::Motor(RobotStates::FLYWHEEL_PORT);
    hood_Motor = new okapi::Motor(RobotStates::HOOD_PORT);
    flywheel_Motor->setGearing(AbstractMotor::gearset::blue);
    hood_Motor->setGearing(AbstractMotor::gearset::green);
    // hood_Motor->setBrakeMode(AbstractMotor::brakeMode::hold);
    // hood_Motor->setReversed(true);
}

// this is for speeding up the motor to a certain range of velocity 
// and then switch into another control method to stabilize 
void NucLauncher::stabilize_the_Flywheel() {
    flywheel_Motor->moveVelocity(600);
}

void NucLauncher::stop_Flywheel() {
    flywheel_Motor->moveVelocity(0);
}

void NucLauncher::toggle_Flywheel() {
	RobotStates::is_Flywheel_Running = !RobotStates::is_Flywheel_Running;
    pros::delay(200);
}

void NucLauncher::hoodUp() {
    hood_Motor->move(-68);
    // if(hood_Motor->getPosition() < 190) {
        
    // }
}

void NucLauncher::hoodDown() {
    hood_Motor->move(68);
    // if(hood_Motor->getPosition() > 10) {
        
    // }
}

void NucLauncher::hoodStop() {
    hood_Motor->move(0);
}

void NucLauncher::autoAim(double err) {
    int baseSpeed = 50; 
    if(!RobotStates::is_Aimed) {
        int output = (err > 0) ? (baseSpeed + 0.5 * err) : (-baseSpeed + 0.5 * err);
        if(fabs(err) > 5) {
            hood_Motor->move(output);
        } else {
            hood_Motor->move(0);
            RobotStates::is_Aimed = true;
            RobotStates::is_autoAiming = false;
        }
        printf("hood output: %d \n", output);
    }
}

void NucLauncher::toggleAutoAim() {
    RobotStates::is_autoAiming = !RobotStates::is_autoAiming;
    RobotStates::is_Aimed = false;
    pros::delay(200);
}