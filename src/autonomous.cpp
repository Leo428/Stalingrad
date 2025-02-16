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
void capDownBit(void * param) {
    Robot::collector->capDown();
    delay(400);
    Robot::collector->capStop();
}

void capUpBit(void * param) {
    Robot::collector->capUp();
    delay(1000);
    Robot::collector->capStop();
}

void knockBalls(void * param) {
    delay(1000);
    Robot::collector->capDown();
    delay(300);
    Robot::collector->capStop();
    Robot::base->forward(-200);
    delay(200);
    RobotStates::is_Collecting_Ball = true;
    Robot::base->forward(200);
    delay(200);
    Robot::base->stop();
}

void knockFlag(void * param) {
    delay(1250);
    Robot::collector->capDown();
    delay(800);
    Robot::collector->capStop();
}

void knockPLat(void * param) {
    delay(400);
    Robot::collector->capDown();
    delay(500); //1000 //750
    Robot::collector->capStop();
    Robot::collector->capUp();
    delay(500);
    Robot::collector->capStop();
}

void flipCap(void * param) {
    delay(1500);
    Robot::collector->capUp();
    delay(750);
    Robot::collector->capStop();
}

void collectFromPlat(void * param) {
    delay(750); //100
    Robot::collector->capDown();
    delay(250);
    Robot::collector->capStop();
    delay(500);
    Robot::collector->capUp();
    delay(500);
    Robot::collector->capStop();
}

void resetHood(void * param) {
    Robot::hoodController->flipDisable(true);
    Robot::nuc->hood_Motor->setReversed(false);
    Robot::nuc->hoodUp();
    delay(500);
    Robot::nuc->hoodStop();
}

void manyFlagsAuto() {
    Robot::nuc->hood_Motor->setReversed(true);
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
    // Robot::profileController->generatePath({Point{4_ft, 0_ft, 0_deg}, Point{0.65_ft, 0_ft, 0_deg}}, "B");
    Robot::profileController->setTarget("B", true);
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("B");
    // pros::delay(200);

    prepareToTurn();
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "90deg");
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

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.25_ft, 0_ft, 0_deg}}, "short");
    Robot::profileController->setTarget("short");
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("short");

    Robot::hoodController->reset();
    Robot::hoodController->tarePosition();
    Robot::hoodController->flipDisable(false);
    Robot::hoodController->setTarget(0); //60 //40
    Robot::hoodController->waitUntilSettled();
    RobotStates::is_Shooting_Ball = true;
    pros::delay(100);
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = true;
    pros::delay(250);
    RobotStates::is_Collecting_Ball = false;
    
    Robot::hoodController->setTarget(130);
    //getting rid of the swereve (3.25, -0.3)
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "low");
    Robot::hoodController->waitUntilSettled();

    RobotStates::is_Shooting_Ball = true;
    pros::delay(200);
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = false;
    RobotStates::is_Flywheel_Running = false;

    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.375_ft, 0_ft, 0_deg}}, "turn2Mid");
    } else {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.45_ft, 0_ft, 0_deg}}, "turn2Mid_Red");
    }
    
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2Mid", true);
        RobotStates::hortizontal_correction = -15;
    } else {
        Robot::turnController->setTarget("turn2Mid_Red");
        RobotStates::hortizontal_correction = -10;
    }

    //0.825
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{1.5_ft, 0_ft, 0_deg}}, "C");

    Robot::turnController->waitUntilSettled();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->removePath("turn2Mid");
    } else {
        Robot::turnController->removePath("turn2Mid_Red");
    }
    
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
    
    Robot::profileController->setTarget("C");
    
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.825_ft, 0_ft, 0_deg}}, "C");
    Robot::profileController->waitUntilSettled();
    // Robot::profileController->removePath("C");

    // RobotStates::is_Collecting_Ball = true;
    Task getBallsTask(knockBalls);
    
    // pros::delay(1200);
    // RobotStates::is_Collecting_Ball = false;
    // RobotStates::is_Shooting_Ball = true;
    // Robot::collector->capUp(); //addedd before Chino
    // pros::delay(200);
    //  Robot::collector->capStop(); //addedd before Chino
    // RobotStates::is_Shooting_Ball = false;

    // // Robot::hoodController->reset();
    // RobotStates::is_Collecting_Ball = true;
    // Robot::hoodController->reset();
    // Robot::hoodController->setTarget(-60);
    // Robot::hoodController->waitUntilSettled();
    // pros::delay(500);
    // RobotStates::is_Collecting_Ball = false;
    // RobotStates::is_Shooting_Ball = true;
    
    
}

void autoForState() {
    Robot::nuc->hood_Motor->setReversed(true);
    //enable flywheel and collector control tasks
    Task auto_flyWheelTask(Robot::operate_Flywheel);
    Task auto_collectorTask(Robot::operate_BallCollector);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);
    Task auto_doubleShot(Robot::assistShooting_withVision); //added for double shot

    RobotStates::is_Flywheel_Running = true;
    //4ft //before chino: 3.8 
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
    Robot::profileController->setTarget("A");
    RobotStates::is_Collecting_Ball = true;
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("A");
    //4.25 - 0 //4 //3.8
    // Robot::profileController->generatePath({Point{4_ft, 0_ft, 0_deg}, Point{0.65_ft, 0_ft, 0_deg}}, "B");
    Robot::profileController->setTarget("B", true);
    Robot::profileController->waitUntilSettled();
    RobotStates::is_Collecting_Ball = false;
    Robot::profileController->removePath("B");
    // pros::delay(200);

    prepareToTurn();
    
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("90deg");
        RobotStates::hortizontal_correction = -15.0;
    } else {
        Robot::turnController->setTarget("90deg", true);
        RobotStates::hortizontal_correction = 15.0;
    }
    Robot::turnController->waitUntilSettled();
    RobotStates::is_Collecting_Ball = false;
    
    prepareToDrive();
    
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;

    pros::delay(200);
    while(!RobotStates::is_Aligned) {
        pros::delay(50);
    }
    RobotStates::hortizontal_correction = 0.0;

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.25_ft, 0_ft, 0_deg}}, "short");
    Robot::profileController->setTarget("short");
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("short");

    Robot::getInstance()->toggle_AssistShooting();
    delay(500);

    //getting rid of the swereve (3.25, -0.3)
    //2.75 for low flag
    Robot::turnController->removePath("90deg");
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.65_ft, -0.25_ft, 0_deg}}, "low");
    } else {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.65_ft, 0.25_ft, 0_deg}}, "low");
    }
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.65_ft, 0_ft, 0_deg}}, "low");
    // Robot::hoodController->waitUntilSettled();

    // RobotStates::is_Shooting_Ball = true;
    // pros::delay(200);

    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = false;
    // RobotStates::is_Flywheel_Running = false;

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3.25_ft, -0.3_ft, 0_deg}}, "low");
    Robot::profileController->setTarget("low");
    Task auto_Knock(knockFlag);
    
    //add 1.5 ft to 1.25
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "back2mid");
    Robot::profileController->waitUntilSettled();
    
    Robot::profileController->removePath("low");
    pros::delay(50);
    
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2_ft, 0.3_ft, 0_deg}}, "back2mid");
    Robot::profileController->setTarget("back2mid", true);
    Robot::collector->capUp();
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.425_ft, 0_ft, 0_deg}}, "turn2cap");
    Robot::profileController->waitUntilSettled();
    Robot::collector->capStop();
    Robot::profileController->removePath("back2mid");

    prepareToTurn();
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "turn2cap");
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2cap", true);
        RobotStates::hortizontal_correction = 30.0;
    } else {
        Robot::turnController->setTarget("turn2cap");
        RobotStates::hortizontal_correction = -30.0;
    }

    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("turn2cap");
    prepareToDrive();

    // RobotStates::is_Aligned = false;
    // RobotStates::is_autoAligning = true;

    // pros::delay(200);
    // while(!RobotStates::is_Aligned) {
    //     pros::delay(50);
    // }
    RobotStates::hortizontal_correction = 0.0;
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = true;

    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{1.6_ft, 0_ft, 0_deg}}, "2cap");
    Robot::profileController->setTarget("2cap");
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.75_ft, 0_ft, 0_deg}}, "2shoot");
    Robot::profileController->waitUntilSettled();

    Robot::collector->capDown();
    // Robot::profileController->removePath("2cap");
    delay(750);
    Robot::collector->capStop();

    Robot::profileController->setTarget("2shoot", true);
    delay(500);
    Robot::collector->capUp();
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("2shoot");

    delay(500);
    Robot::collector->capStop();
    delay(1500);

    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = false;
    delay(200);

    Robot::getInstance()->toggle_AssistShooting();

    // Robot::base->forward(200);
    // delay(1500);
    // Robot::base->stop();
}

void threeFlags_twoCaps() { //3 flags, 2 caps, no plat
    Robot::nuc->hood_Motor->setReversed(true);
    //enable flywheel and collector control tasks
    Task auto_flyWheelTask(Robot::operate_Flywheel);
    Task auto_collectorTask(Robot::operate_BallCollector);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);
    Task auto_doubleShot(Robot::assistShooting_withVision); //added for double shot

    RobotStates::is_Flywheel_Running = true;
    //4ft //before chino: 3.8 
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
    Robot::profileController->setTarget("A");
    RobotStates::is_Collecting_Ball = true;
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("A");
    //4.25 - 0 //4 //3.8
    // Robot::profileController->generatePath({Point{4_ft, 0_ft, 0_deg}, Point{0.65_ft, 0_ft, 0_deg}}, "B");
    Robot::profileController->setTarget("B", true);
    delay(750);
    RobotStates::is_Collecting_Ball = false;
    Robot::profileController->waitUntilSettled();
    // RobotStates::is_Collecting_Ball = false;
    Robot::profileController->removePath("B");
    // pros::delay(200);

    prepareToTurn();
    
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("90deg");
        RobotStates::hortizontal_correction = -15.0;
    } else {
        Robot::turnController->setTarget("90deg", true);
        RobotStates::hortizontal_correction = 15.0;
    }
    Robot::turnController->waitUntilSettled();
    RobotStates::is_Collecting_Ball = false;
    
    prepareToDrive();
    
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;

    pros::delay(200);
    while(!RobotStates::is_Aligned) {
        pros::delay(50);
    }
    RobotStates::hortizontal_correction = 0.0;

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.25_ft, 0_ft, 0_deg}}, "short");
    Robot::profileController->setTarget("short");
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("short");

    Robot::getInstance()->toggle_AssistShooting();

    //getting rid of the swereve (3.25, -0.3)
    //2.75 for low flag
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.65_ft, -0.25_ft, 0_deg}}, "low");
    } else {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.65_ft, 0.25_ft, 0_deg}}, "low");
    }
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.65_ft, 0_ft, 0_deg}}, "low");
    // Robot::hoodController->waitUntilSettled();

    // RobotStates::is_Shooting_Ball = true;
    // pros::delay(200);
    // RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = true;
    RobotStates::is_Flywheel_Running = false;

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3.25_ft, -0.3_ft, 0_deg}}, "low");
    Robot::profileController->setTarget("low");
    Task auto_Knock(knockFlag);
    //getting rid of the swerve and shorter distance  (2, 0.3)
    //back to mid reduce from 1.5 to 1.25 // 1.25 to 1.1
    //back more
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{1.25_ft, 0_ft, 0_deg}}, "back2mid");
    Robot::profileController->waitUntilSettled();
    
    Robot::profileController->removePath("low");
    pros::delay(50);
    
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2_ft, 0.3_ft, 0_deg}}, "back2mid");
    Robot::profileController->setTarget("back2mid", true);
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("back2mid");

    Robot::turnController->removePath("90deg");
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.875_ft, 0_ft, 0_deg}}, "turn2cap");
    prepareToTurn();
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "turn2cap");
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2cap", true);
    } else {
        Robot::turnController->setTarget("turn2cap");
    }

    //changed from 3 to 2.5 to align perfectly to the plat
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "flipCap");
    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("turn2cap");

    Robot::collector->capDown();
    delay(200);
    Robot::collector->capStop();

    prepareToDrive();
    
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = true;
    
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.85_ft, 0_ft, 0_deg}}, "2cap");
    Robot::turnController->setTarget("2cap");
    Task flipGG(flipCap);
    //0.425
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.41_ft, 0_ft, 0_deg}}, "2midFlags");
    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("2cap");

    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("2midFlags");
    } else {
        Robot::turnController->setTarget("2midFlags", true);
    }
    Robot::turnController->waitUntilSettled();
    prepareToDrive();
    Robot::turnController->removePath("2midFlags");

    Robot::base->forward(200);
    delay(1000);
    Task auto_cap_down(capDownBit);
    // delay(200); //1500
    Robot::base->stop();
    Robot::base->forward(-200);
    delay(300);
    Robot::base->stop();
}

void threeFlags_Only_Plat() {
    Robot::nuc->hood_Motor->setReversed(true);
    //enable flywheel and collector control tasks
    Task auto_flyWheelTask(Robot::operate_Flywheel);
    Task auto_collectorTask(Robot::operate_BallCollector);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);
    //auto corrects once before the shot
    Task auto_doubleShot(Robot::assistShooting_withVision); //added for double shot

    RobotStates::is_Flywheel_Running = true;
    //4ft //before chino: 3.8 
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
    Robot::profileController->setTarget("A");
    RobotStates::is_Collecting_Ball = true;
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("A");
    //4.25 - 0 //4 //3.8
    // Robot::profileController->generatePath({Point{4_ft, 0_ft, 0_deg}, Point{0.65_ft, 0_ft, 0_deg}}, "B");
    Robot::profileController->setTarget("B", true);
    Robot::profileController->waitUntilSettled();
    RobotStates::is_Collecting_Ball = false;
    Robot::profileController->removePath("B");
    // pros::delay(200);

    prepareToTurn();
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "90deg");
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
    
    //commented out because its redundant 
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;

    pros::delay(200);
    while(!RobotStates::is_Aligned) {
        pros::delay(50);
    }
    RobotStates::hortizontal_correction = 0.0;

    
    // if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
    //     // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.25_ft, 0_ft, 0_deg}}, "short");
    // } else {
    //     Robot::profileController->removePath("short");
    //     Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.625_ft, 0_ft, 0_deg}}, "short");
    // }

    Robot::profileController->setTarget("short");
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("short");

    Robot::getInstance()->toggle_AssistShooting();
    delay(500);
    
    //2.75 for low flag //2.5 a bit too much 
    // if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
    //     Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.25_ft, 0_ft, 0_deg}}, "low");
    // } else {
    //     Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.25_ft, 0_ft, 0_deg}}, "low");
    // }

    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.65_ft, 0_ft, 0_deg}}, "low");
    } else {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.65_ft, 0_ft, 0_deg}}, "low");
    }

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.25_ft, 0_ft, 0_deg}}, "low");
    // Robot::hoodController->waitUntilSettled();

    // RobotStates::is_Shooting_Ball = true;
    // pros::delay(200);
    // RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = true;
    RobotStates::is_Flywheel_Running = false;

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3.25_ft, -0.3_ft, 0_deg}}, "low");
    Robot::profileController->setTarget("low");
    //TODO: enable for the cap flipper; disable for the descorer
    Task auto_Knock(knockFlag); 

    //getting rid of the swerve and shorter distance  (2, 0.3)
    Robot::profileController->waitUntilSettled();
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{5_ft, 0_ft, 0_deg}}, "back2plat"); //5.5

    Robot::profileController->removePath("low");
    pros::delay(50);

    //5.5 too much 
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        //4.75
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{4.75_ft, 0_ft, 0_deg}}, "back2plat"); //5.5
    } else {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{5_ft, 0_ft, 0_deg}}, "back2plat"); //5.5
    }
    //5.25 //5
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{4.75_ft, 0_ft, 0_deg}}, "back2plat"); //5.5
    
    Robot::profileController->setTarget("back2plat", true);

    //TODO: enable for the cap flipper; disable for the descorer
    Task auto_flipCap(flipCap);

    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("back2plat");

    prepareToTurn();
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "90deg");
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "turn2cap");
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("90deg", true);
    } else {
        Robot::turnController->setTarget("90deg");
    }

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "flipCap");
    Robot::turnController->waitUntilSettled();
    prepareToDrive();
    Robot::turnController->removePath("90deg");
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = true;
    
    Robot::base->forward(200);
    // TODO: enable for the cap flipper; disable for the descorer
    Task knockPlat_task(knockPLat);

    // Task resetHood_task(resetHood);
    pros::delay(1750);
    Robot::base->stop();
}

void skills() {
    Robot::nuc->hood_Motor->setReversed(true);
    //enable flywheel and collector control tasks
    Task auto_flyWheelTask(Robot::operate_Flywheel);
    Task auto_collectorTask(Robot::operate_BallCollector);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);
    Task auto_doubleShot(Robot::assistShooting); //added for double shot

    RobotStates::is_Flywheel_Running = true;
    //4ft //before chino: 3.8 
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
    Robot::profileController->setTarget("A");
    RobotStates::is_Collecting_Ball = true;
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("A");
    //4.25 - 0 //4 //3.8
    // Robot::profileController->generatePath({Point{4_ft, 0_ft, 0_deg}, Point{0.65_ft, 0_ft, 0_deg}}, "B");
    Robot::profileController->setTarget("B", true);
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("B");
    // pros::delay(200);

    prepareToTurn();
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "90deg");
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

    
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.25_ft, 0_ft, 0_deg}}, "short");
    } else {
        Robot::profileController->removePath("short");
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.625_ft, 0_ft, 0_deg}}, "short");
    }

    Robot::profileController->setTarget("short");
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("short");

    Robot::getInstance()->toggle_AssistShooting();
    delay(500);

    //getting rid of the swereve (3.25, -0.3)
    //2.75 for low flag //2.5 a bit too much 
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.25_ft, 0_ft, 0_deg}}, "low");
    } else {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.5_ft, 0_ft, 0_deg}}, "low");
    }
    
    RobotStates::is_Collecting_Ball = true;
    RobotStates::is_Flywheel_Running = false;

    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3.25_ft, -0.3_ft, 0_deg}}, "low");
    Robot::profileController->setTarget("low");
    Task auto_Knock(knockFlag);
    //getting rid of the swerve and shorter distance  (2, 0.3)
    Robot::profileController->waitUntilSettled();
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{5_ft, 0_ft, 0_deg}}, "back2plat"); //5.5

    Robot::profileController->removePath("low");
    pros::delay(50);

    //5.5 too much 
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{4.75_ft, 0_ft, 0_deg}}, "back2plat"); //5.5
    } else {
        Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{5_ft, 0_ft, 0_deg}}, "back2plat"); //5.5
    }
    //5.25 //5
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{4.75_ft, 0_ft, 0_deg}}, "back2plat"); //5.5
    
    Robot::profileController->setTarget("back2plat", true);
    Task auto_flipCap(flipCap);
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("back2plat");

    prepareToTurn();
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "90deg");
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.8181_ft, 0_ft, 0_deg}}, "turn2cap");
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("90deg", true);
    } else {
        Robot::turnController->setTarget("90deg");
    }

    Robot::turnController->waitUntilSettled();
    prepareToDrive();
    Robot::turnController->removePath("90deg");
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = true;
    
    Robot::base->setBrakeMode(AbstractMotor::brakeMode::hold);
    Robot::base->forward(200);
    Task knockPlat_task(knockPLat);
    // Task resetHood_task(resetHood);
    pros::delay(2500); //2500 //2750
    Robot::base->stop();
}

void backTile() {
    Robot::nuc->hood_Motor->setReversed(true);
    //enable flywheel and collector control tasks
    Task auto_flyWheelTask(Robot::operate_Flywheel);
    Task auto_collectorTask(Robot::operate_BallCollector);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);
    // Task auto_doubleShot(Robot::assistShooting); //added for double shot

    RobotStates::is_Collecting_Ball = true;
    RobotStates::is_Flywheel_Running = true;
    RobotStates::is_assistant_Shooting_back = true;

    Robot::hoodController->reset();
    Robot::hoodController->tarePosition();
    Robot::hoodController->flipDisable(false);
    Robot::hoodController->setTarget(0); //60 //40

    Robot::profileController->removePath("A");
    Robot::profileController->removePath("B");
    Robot::profileController->removePath("short");

    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{1.1_ft, 0_ft, 0_deg}}, "backshort");
    // Robot::profileController->moveTo({Point{0_ft, 0_ft, 0_deg}, Point{1.1_ft, 0_ft, 0_deg}});
    Robot::profileController->setTarget("backshort");
    Task collectPlat(collectFromPlat);
    //-1 -> -1.1
    
    Robot::profileController->waitUntilSettled();
    
    Robot::profileController->setTarget("backshort", true);
    //0.3 not enough to the right
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.375_ft, 0_ft, 0_deg}}, "turn2MidFlag");
    } else {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.375_ft, 0_ft, 0_deg}}, "turn2MidFlag");
    }
    
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("backshort");

    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2MidFlag");
    } else {
        Robot::turnController->setTarget("turn2MidFlag", true);
    }
    
    //0.675
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.5_ft, 0_ft, 0_deg}}, "turn2Cap");
    } else {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.5_ft, 0_ft, 0_deg}}, "turn2Cap");
    }
    
    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("turn2MidFlag");
    prepareToDrive();

    delay(750);
    
    RobotStates::is_Collecting_Ball = false;
    Robot::getInstance()->oneShot2Mid_withAssistant(); //auto collect after shot
    
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.5_ft, 0_ft, 0_deg}}, "backABit");
    Robot::profileController->setTarget("backABit", true);
    RobotStates::is_Collecting_Ball = true;
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("backABit");

    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2Cap", true);
    } else {
        Robot::turnController->setTarget("turn2Cap");
    }
    
    //2.75
    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3.75_ft, 0_ft, 0_deg}}, "A");
    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("turn2Cap");
    prepareToDrive();

    Robot::base->forward(-100);
    delay(400); //300
    Robot::base->stop();
    delay(500);

    Robot::profileController->setTarget("A");
    //0.65 too right //0.5 a bit too left //0.575
    //0.9191 to 90 deg turn 
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.9191_ft, 0_ft, 0_deg}}, "turn2Enemy");
    } else {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.9191_ft, 0_ft, 0_deg}}, "turn2Enemy");
    }
    
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("A");

    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{-.45_ft, 0_ft, 0_deg}}, "back2Plat");
    Robot::profileController->setTarget("back2Plat", true);
    Robot::profileController->waitUntilSettled();

    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2Enemy");
    } else {
        Robot::turnController->setTarget("turn2Enemy", true);
    }
    
    //0.4 //0.325
    // Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.325_ft, 0_ft, 0_deg}}, "turn2Plat");
    Robot::turnController->waitUntilSettled();
    prepareToDrive();
    Robot::turnController->removePath("turn2Enemy");
    
    // RobotStates::is_Collecting_Ball = false;
    // Robot::getInstance()->oneShot2Enemy_withAssistant(); //auto collect after shot
    
    // prepareToTurn();
    // Robot::turnController->setTarget("turn2Plat");
    // Robot::turnController->waitUntilSettled();
    // Robot::turnController->removePath("turn2Plat");
    // prepareToDrive();

    Robot::base->forward(200); //200
    Task knockPlat_task(knockPLat);
    Task resetHood_task(resetHood);
    pros::delay(1250); //1500
    Robot::base->stop();

}

void easyBack() {
    Task auto_collectorTask(Robot::operate_BallCollector);

    RobotStates::is_Collecting_Ball = true;
    Robot::profileController->setTarget("A");
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("A");

    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{-.45_ft, 0_ft, 0_deg}}, "back2Plat");
    Robot::profileController->setTarget("back2Plat", true);
    Robot::profileController->waitUntilSettled();

    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("90deg");
    } else {
        Robot::turnController->setTarget("90deg", true);
    }

    Robot::turnController->waitUntilSettled();
    prepareToDrive();

    //go on platform 
    // Robot::base->forward(200); //200
    // // Task knockPlat_task(knockPLat);
    // pros::delay(1250); //1500
    // Robot::base->stop();
}

void elimBack() {
    // Robot::nuc->hood_Motor->setReversed(true);
    //enable flywheel and collector control tasks
    Task auto_flyWheelTask(Robot::operate_Flywheel);
    Task auto_collectorTask(Robot::operate_BallCollector);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);
    Task auto_movePot(Robot::hoodWithPot);

    Robot::profileController->removePath("B");
    Robot::profileController->removePath("short");
    Robot::turnController->removePath("90deg");

    RobotStates::is_Flywheel_Running = true;

    Robot::profileController->removePath("A");
    
    Robot::mediumSpeedController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
    RobotStates::is_Collecting_Ball = true;
    Robot::mediumSpeedController->setTarget("A");
    Robot::mediumSpeedController->waitUntilSettled();
    Robot::mediumSpeedController->removePath("A");

    Robot::mediumSpeedController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{-.45_ft, 0_ft, 0_deg}}, "back2Plat");
    Robot::mediumSpeedController->setTarget("back2Plat", true);
    Robot::mediumSpeedController->waitUntilSettled();
    Robot::mediumSpeedController->removePath("back2Plat");
    RobotStates::is_Collecting_Ball = false;

    // RobotStates::is_Collecting_Ball = true;
    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        //0.525
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.525_ft, 0_ft, 0_deg}}, "turn2MidFlag");
    } else {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.535_ft, 0_ft, 0_deg}}, "turn2MidFlag");
    }
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2MidFlag");
    } else {
        Robot::turnController->setTarget("turn2MidFlag", true);
    }
    
    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("turn2MidFlag");
    prepareToDrive();
    // RobotStates::is_Collecting_Ball = false;

    RobotStates::hortizontal_correction = 0.0; //-5
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;
    
    pros::delay(200);
    while(!RobotStates::is_Aligned) {
        pros::delay(50);
    }

    // Robot::nuc->hood_Motor->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    delay(2000);

    Robot::nuc->pot->calibrate();
    delay(500);
    RobotStates::potTarget = 70; //50 //70
    RobotStates::is_pot = true;
    delay(2000);

    RobotStates::is_Collecting_Ball = false;
    RobotStates::is_Shooting_Ball = true;
    delay(100);
    RobotStates::is_Shooting_Ball = false;
    
    delay(200);
    RobotStates::is_Collecting_Ball = true;

    Robot::nuc->hoodDown();
    delay(300);
    Robot::nuc->hoodStop();
    
    Robot::nuc->pot->calibrate();
    delay(500);
    RobotStates::potTarget = 250;
    RobotStates::is_at_pot = false;
    RobotStates::is_pot = true;
    delay(1000);

    RobotStates::is_Collecting_Ball = false;
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.3_ft, 0_ft, 0_deg}}, "turn2Plat");
    } else {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.3_ft, 0_ft, 0_deg}}, "turn2Plat");
    }
    // delay(200);

    RobotStates::is_Shooting_Ball = true;
    delay(500);

    RobotStates::is_Shooting_Ball = false; 

    // Robot::nuc->hood_Motor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

    
    prepareToTurn();
    
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2Plat");
    } else {
        Robot::turnController->setTarget("turn2Plat", true);
    }
    RobotStates::is_Collecting_Ball = true;
    Robot::turnController->waitUntilSettled();
    prepareToDrive();

    Robot::base->setBrakeMode(AbstractMotor::brakeMode::hold);
    Robot::base->forward(200); //200

    // disable for descorer
    Task knockPlat_task(knockPLat);

    pros::delay(1300); //1500 //1200
    Robot::base->stop();
    

    Robot::hoodController->flipDisable(true);
    // Robot::nuc->hood_Motor->setReversed(false);
    Robot::nuc->hoodDown();
    delay(500);
    Robot::nuc->hoodStop();
    Robot::base->setBrakeMode(AbstractMotor::brakeMode::coast);
}

void back_2flag_2caps() {
    // Robot::nuc->hood_Motor->setReversed(true);
    //enable flywheel and collector control tasks
    Task auto_flyWheelTask(Robot::operate_Flywheel);
    Task auto_collectorTask(Robot::operate_BallCollector);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);
    Task auto_movePot(Robot::hoodWithPot);

    Robot::profileController->removePath("B");
    Robot::profileController->removePath("short");
    Robot::turnController->removePath("90deg");

    RobotStates::is_Collecting_Ball = true;
    RobotStates::is_Flywheel_Running = true;

    // Robot::profileController->removePath("A");
    
    // Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{2.75_ft, 0_ft, 0_deg}}, "A");
    Robot::profileController->setTarget("A");
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("A");

    Robot::profileController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{-.45_ft, 0_ft, 0_deg}}, "back2Plat");
    Robot::profileController->setTarget("back2Plat", true);
    Robot::profileController->waitUntilSettled();
    Robot::profileController->removePath("back2Plat");
    RobotStates::is_Collecting_Ball = false;

    prepareToTurn();
    Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.6_ft, 0_ft, 0_deg}}, "2cap");
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("2cap", true);
    } else {
        Robot::turnController->setTarget("2cap");
    }
    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("2cap");
    prepareToDrive();

    Task auto_down(capDownBit);

    Robot::mediumSpeedController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{.6_ft, 0_ft, 0_deg}}, "f2cap");
    Robot::mediumSpeedController->setTarget("f2cap");
    Robot::mediumSpeedController->waitUntilSettled();
    Robot::mediumSpeedController->removePath("f2cap");
    Task auto_up(capUpBit);

    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        //0.525
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{1.2_ft, 0_ft, 0_deg}}, "turn2MidFlag");
    } else {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.535_ft, 0_ft, 0_deg}}, "turn2MidFlag");
    }
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2MidFlag");
    } else {
        Robot::turnController->setTarget("turn2MidFlag", true);
    }
    
    Robot::turnController->waitUntilSettled();
    Robot::turnController->removePath("turn2MidFlag");
    prepareToDrive();

    RobotStates::hortizontal_correction = 0.0; //-5
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;
    
    pros::delay(200);
    while(!RobotStates::is_Aligned) {
        pros::delay(50);
    }

    // Robot::nuc->hood_Motor->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    
    Robot::nuc->pot->calibrate();
    delay(500);
    RobotStates::potTarget = 50; //100 //30 //60 //40
    RobotStates::is_pot = true;
    delay(2000);

    RobotStates::is_Collecting_Ball = false;
    RobotStates::is_Shooting_Ball = true;
    delay(100);
    RobotStates::is_Shooting_Ball = false;
    
    delay(200);
    RobotStates::is_Collecting_Ball = true;

    Robot::nuc->hoodDown();
    delay(200);
    Robot::nuc->hoodStop();
    
    Robot::nuc->pot->calibrate();
    delay(1000);
    RobotStates::potTarget = 250;
    RobotStates::is_at_pot = false;
    RobotStates::is_pot = true;
    delay(2000);

    RobotStates::is_Collecting_Ball = false;
    delay(200);

    RobotStates::is_Shooting_Ball = true;
    delay(500);

    RobotStates::is_Shooting_Ball = false; 

    // Robot::nuc->hood_Motor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

    prepareToTurn();
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.3_ft, 0_ft, 0_deg}}, "turn2Plat");
    } else {
        Robot::turnController->generatePath({Point{0_ft, 0_ft, 0_deg}, Point{0.3_ft, 0_ft, 0_deg}}, "turn2Plat");
    }
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        Robot::turnController->setTarget("turn2Plat");
    } else {
        Robot::turnController->setTarget("turn2Plat", true);
    }
    Robot::turnController->waitUntilSettled();
    prepareToDrive();

    Robot::base->setBrakeMode(AbstractMotor::brakeMode::hold);
    Robot::base->forward(200); //200

    // disable for descorer
    Task knockPlat_task(knockPLat);

    pros::delay(1200); //1500
    Robot::base->stop();
    

    Robot::hoodController->flipDisable(true);
    // Robot::nuc->hood_Motor->setReversed(false);
    Robot::nuc->hoodDown();
    delay(500);
    Robot::nuc->hoodStop();
    Robot::base->setBrakeMode(AbstractMotor::brakeMode::coast);
}

void testing() {
    Task pot(Robot::hoodWithPot);
    Task auto_visionTask(Robot::testTracking);
    Task auto_alignTask(Robot::alignTheBot);

    RobotStates::hortizontal_correction = 0; //-5
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;
    
    pros::delay(200);
    while(!RobotStates::is_Aligned) {
        pros::delay(50);
    }

    Robot::nuc->pot->calibrate();
    delay(500);
    RobotStates::potTarget = 50; //100 //30 //60 //40
    RobotStates::is_pot = true;
    delay(2000);

    Robot::nuc->hoodDown();
    delay(200);
    Robot::nuc->hoodStop();

    Robot::nuc->pot->calibrate();
    delay(500);

    
    RobotStates::potTarget = 250;
    RobotStates::is_at_pot = false;
    RobotStates::is_pot = true;
    
    delay(5000);
    printf("pot: %d", Robot::nuc->pot->get_value_calibrated());
}

void autonomous() {
    switch(RobotStates::autoChoice){
		case (RobotStates::AutoChoice::FOUR_FLAGS):
			autoForState();
			break;
        case (RobotStates::AutoChoice::THREE_FLAGS_PLAT):
			threeFlags_Only_Plat();
			break;
        case (RobotStates::AutoChoice::THREE_FLAGS_CAP):
            threeFlags_twoCaps();
            break;
        case (RobotStates::AutoChoice::BACK_TILE):
            elimBack();
            break;
        case (RobotStates::AutoChoice::THREE_FLAGS_SKILLS):
            skills();
            break;
		default:
			break;
	}

    // elimBack();
    // backTile();
    // easyBack();
    // manyFlagsAuto();
    // threeFlags_plat();
    // threeFlags_Only_Plat();
    // skills();
    // testing();
}
