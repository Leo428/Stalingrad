#include "main.h"
#include "userincludes/robot.hpp"
#include <cmath>

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


// double in2meter(double in) {
//     return (in / 39.37);
// }

void prepareToTurn() {
    Robot::rightFront_Motor->setReversed(false);
    Robot::rightBack_Motor->setReversed(false);
}

void prepareToDrive() {
    Robot::rightFront_Motor->setReversed(true);
    Robot::rightBack_Motor->setReversed(true);
}

void manyFlagsAuto() {
    // Timer * t = new Timer();
    // auto turnController = AsyncControllerFactory::motionProfile(
    //     in2meter(20.0),  // Maximum linear velocity of the Chassis in m/s
    //     in2meter(30.0),  // Maximum linear acceleration of the Chassis in m/s/s
    //     in2meter(300.0), // Maximum linear jerk of the Chassis in m/s/s/s
    //     *Robot::base // Chassis Controller
    // );
    
    // auto profileController = AsyncControllerFactory::motionProfile(
    //     in2meter(40.0),  // Maximum linear velocity of the Chassis in m/s
    //     in2meter(60.0),  // Maximum linear acceleration of the Chassis in m/s/s
    //     in2meter(500.0), // Maximum linear jerk of the Chassis in m/s/s/s
    //     *Robot::base // Chassis Controller
    // );

    // auto hoodTime = TimeUtilFactory::withSettledUtilParams(30, 5.0, 250_ms);
    // auto hoodController = AsyncControllerFactory::posIntegrated(*Robot::nuc->hood_Motor, 100);
    Robot::nuc->hood_Motor->setReversed(true);

    //angle = distance * 2 / width 
    // distance = angle * width / 2
    // double d = 90.0 * M_PI / 180.0 * 12.5 / 2;

    //prepareToDrive();
    //prepareToTurn();

    //enable flywheel and collector control tasks
    Task auto_flyWheelTask(Robot::operate_Flywheel);
    Task auto_collectorTask(Robot::operate_BallCollector);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);

    RobotStates::is_Flywheel_Running = true;
    //4ft //before chino: 3.8 
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
    Robot::profileController->setTarget("A");
    RobotStates::is_Collecting_Ball = true;
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("A");
    //4.25 - 0 //4 //3.8
    Robot::profileController->generatePath({Point{4_ft, 0_ft, 0_deg}, Point{0.65_ft, 0_ft, 0_deg}}, "B");
    Robot::profileController->setTarget("B", true);
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("B");
    // pros::delay(200);

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.6_ft, 0_ft, 0_deg}}, "small");
    // Robot::profileController->setTarget("small");
    // Robot::profileController->waitUntilSettled();
    // Robot::profileController->removePath("small");

    prepareToTurn();
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "90deg");
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("90deg");
        RobotStates::hortizontal_correction = -15.0;
    } else {
        Robot::turnController->setTarget("90deg", true);
        RobotStates::hortizontal_correction = 15.0;
    }
    Robot::turnController->waitUntilSettled();
    RobotStates::is_Collecting_Ball = false;
    Robot::turnController->removePath("90deg");
    prepareToDrive();
    
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;

    pros::delay(200);
    while(!RobotStates::is_Aligned) {
        pros::delay(50);
    }
    RobotStates::hortizontal_correction = 0.0;

    Robot::hoodController->reset();
    Robot::hoodController->flipDisable(false);
    Robot::hoodController->setTarget(60); //60 //40
    Robot::hoodController->waitUntilSettled();
    RobotStates::is_Shooting_Ball = true;
    pros::delay(100);
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = true;
    pros::delay(500);
    RobotStates::is_Collecting_Ball = false;
    
    // Robot::hoodController->reset();
    // Robot::hoodController->flipDisable(false);
    Robot::hoodController->setTarget(130); //130
    Robot::hoodController->waitUntilSettled();


    RobotStates::is_Shooting_Ball = true;
    pros::delay(200);
    RobotStates::is_Shooting_Ball = false;

    prepareToTurn();
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.375_ft, 0_ft, 0_deg}}, "turn2Mid");
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.5_ft, 0_ft, 0_deg}}, "turn2Mid_Red");
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2Mid", true);
        RobotStates::hortizontal_correction = -15;
    } else {
        Robot::turnController->setTarget("turn2Mid_Red");
        RobotStates::hortizontal_correction = 15;
    }
    
    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("turn2Mid");
    prepareToDrive();

    // RobotStates::hortizontal_correction = 0;//10.0;
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;
    pros::delay(200);
    while(true) {
        if(RobotStates::is_Aligned) break;
        pros::delay(50);
    }
    RobotStates::hortizontal_correction = 0.0;
    //0.65 //0.8
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.825_ft, 0_ft, 0_deg}}, "C");
    Robot::profileController->setTarget("C");
    Robot::profileController->waitUntilSettled();
    // Robot::profileController->removePath("C");


    RobotStates::is_Collecting_Ball = true;
    Robot::collector->capDownSlowly();
    pros::delay(1000);
    Robot::collector->capStop();

    // Robot::collector->capUp();
    // pros::delay(200);
    // Robot::collector->capStop();

    Robot::profileController->setTarget("C");
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("C");
    Robot::collector->capStop();
    pros::delay(1200);
    RobotStates::is_Collecting_Ball = false;
    RobotStates::is_Shooting_Ball = true;
    Robot::collector->capUp(); //addedd before Chino
    pros::delay(200);
     Robot::collector->capStop(); //addedd before Chino
    RobotStates::is_Shooting_Ball = false;

    // Robot::hoodController->reset();
    RobotStates::is_Collecting_Ball = true;
    Robot::hoodController->reset();
    Robot::hoodController->setTarget(-60);
    Robot::hoodController->waitUntilSettled();
    pros::delay(500);
    RobotStates::is_Collecting_Ball = false;
    RobotStates::is_Shooting_Ball = true;
    // pros::delay(200);
    // RobotStates::is_Shooting_Ball = false;
    
    Timer * end = new Timer();
    while(end->getDtFromStart() < 2000_ms) {
        if(end->getDtFromStart() > 500_ms && end->getDtFromStart() < 600_ms) {
            Robot::collector->capDownSlowly();
        } else {
            Robot::collector->capStop();
        }
        Robot::base->forward(200);
    }
    
    Robot::base->stop();
    // Timer * t = new Timer();
    // Robot::collector->capUpSlowly();
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2_ft, 0_ft, 0_deg}}, "D");
    // Robot::profileController->setTarget("D");
    // if(t->getDtFromStart() > 200_ms) Robot::collector->capStop();
    // Robot::profileController->waitUntilSettled();

    // RobotStates::is_Collecting_Ball = false;
    // RobotStates::is_Flywheel_Running = false;
    // Robot::nuc->hood_Motor->setReversed(false);
}

void testing() {
    Timer * end = new Timer();
    while(end->getDtFromStart() < 2000_ms) {
        if(end->getDtFromStart() > 500_ms && end->getDtFromStart() < 600_ms) {
            Robot::collector->capDownSlowly();
        } else {
            Robot::collector->capStop();
        }
    }
}

void autonomous() {

    switch(RobotStates::autoChoice){
		case (RobotStates::AutoChoice::FOUR_FLAGS):
			manyFlagsAuto();
			break;
		default:
			break;
	}
    // testing();
}
