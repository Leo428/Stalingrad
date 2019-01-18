#include "userincludes/robot.hpp"

okapi::ChassisControllerIntegrated* Robot::base = 0;
Collector* Robot::collector = 0;
NucLauncher* Robot::nuc = 0;
Camera* Robot::cam = 0; 
Robot* Robot::instance = 0;
okapi::Motor * Robot::leftFront_Motor = new okapi::Motor(RobotStates::BASE_LEFT_FRONT);
okapi::Motor * Robot::leftBack_Motor = new okapi::Motor(RobotStates::BASE_LEFT_BACK);
okapi::Motor * Robot::rightFront_Motor = new okapi::Motor(RobotStates::BASE_RIGHT_FRONT);
okapi::Motor * Robot::rightBack_Motor = new okapi::Motor(RobotStates::BASE_RIGHT_BACK);

static auto drive = ChassisControllerFactory::create(
		{RobotStates::BASE_LEFT_FRONT,RobotStates::BASE_LEFT_BACK},
		{-RobotStates::BASE_RIGHT_FRONT,-RobotStates::BASE_RIGHT_BACK}, 
		AbstractMotor::gearset::green,
        {4_in, 12.5_in}); //12.5

Robot::Robot() {
    collector = new Collector();
    nuc = new NucLauncher();
    cam = new Camera();
    
    base = &drive;
    static std::vector<vision_object_s_t> flagsVector;
    Camera::targetVector = &flagsVector;
}

Robot* Robot::getInstance() {
    if(instance == 0) {
        instance = new Robot();
    }
    return instance;
}

void Robot::operate_Flywheel(void * param) {
    while(true) {
		if(RobotStates::is_Flywheel_Running) {
			Robot::nuc->stabilize_the_Flywheel();
		} else {
			Robot::nuc->stop_Flywheel();
		}
		pros::delay(20);
	}
}

void Robot::operate_BallCollector(void * param) {
    while(true) {
        if(RobotStates::is_Shooting_Ball) {
            Robot::collector->shootBall();
        } else if(RobotStates::is_Collecting_Ball) {
            Robot::collector->collectBalls();
        } else {
            Robot::collector->stopCollector();
        }
        pros::delay(50);
    }
}

void Robot::doubleShot(void * param) {
    // Robot::collector->shootBall();
    // pros::delay(100);
    // pros::delay(500);
    // Robot::collector->shootBall();
    // pros::delay(200);
}

void Robot::autoAim_Task(void * param) {
    while(true) {
        if(RobotStates::targetY != 0 && RobotStates::targetFlag_Y != 0 && RobotStates::is_autoAiming) {
            Robot::nuc->autoAim(RobotStates::targetY - RobotStates::targetFlag_Y);
        }
        pros::delay(20);
    }
}

void Robot::testTracking(void* param) {
	while(true) {
		Robot::cam->updateSensor();
		// printf("hood at %f \n", Robot::nuc->hood_Motor->getPosition());
		pros::delay(10);
	}
}

void Robot::alignTheBot(void * param) {
    double err;
    int baseSpeed = 10; 
    while(true) {
        if(RobotStates::targetFlag_X != 0 && RobotStates::is_autoAligning) {
            err = (VISION_FOV_WIDTH / 2.0 + RobotStates::hortizontal_correction) - RobotStates::targetFlag_X;
            if(!RobotStates::is_Aligned) {
                int output = (err > 0) ? (baseSpeed + 0.5 * err) : (-baseSpeed + 0.5 * err);
                if(fabs(err) > 5) {
                    leftFront_Motor->move(-output);
                    leftBack_Motor->move(-output);
                    rightFront_Motor->move(output);
                    rightBack_Motor->move(output);
                } else {
                    leftFront_Motor->move(0);
                    leftBack_Motor->move(0);
                    rightFront_Motor->move(0);
                    rightBack_Motor->move(0);
                    RobotStates::is_Aligned = true;
                    RobotStates::is_autoAligning = false;
                }
                printf("drive base output: %d \n", output);
            }
        }
        pros::delay(20);
    }
}

void Robot::assistShooting(void * param) {
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_Collecting_Ball = false;
    
    if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
        RobotStates::hortizontal_correction = -15.0;
    } else {
        RobotStates::hortizontal_correction = 15.0;
    }
    RobotStates::is_Aligned = false;
    RobotStates::is_autoAligning = true;
    pros::delay(200);
    while(!RobotStates::is_Aligned) {
        pros::delay(50);
    }
    
}

void Robot::rest_before_driver() {
    RobotStates::is_Collecting_Ball = false;
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_autoAiming = false;
    RobotStates::is_Aimed = false;
    RobotStates::is_Flywheel_Running = false;
    RobotStates::is_autoAligning = false;
    RobotStates::is_Aligned = false;

    RobotStates::hortizontal_correction = 0.0;

    Robot::nuc->hood_Motor->setReversed(false);
    Robot::rightFront_Motor->setReversed(true);
    Robot::rightBack_Motor->setReversed(true);
}