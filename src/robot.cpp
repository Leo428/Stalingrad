#include "userincludes/robot.hpp"

// okapi::ChassisControllerIntegrated* Robot::base = 0;
Collector* Robot::collector = 0;
NucLauncher* Robot::nuc = 0;
Camera* Robot::cam = 0; 
Robot* Robot::instance = 0;
okapi::Motor * Robot::leftFront_Motor = new okapi::Motor(RobotStates::BASE_LEFT_FRONT);
okapi::Motor * Robot::leftBack_Motor = new okapi::Motor(RobotStates::BASE_LEFT_BACK);
okapi::Motor * Robot::rightFront_Motor = new okapi::Motor(RobotStates::BASE_RIGHT_FRONT);
okapi::Motor * Robot::rightBack_Motor = new okapi::Motor(RobotStates::BASE_RIGHT_BACK);
okapi::AsyncMotionProfileController * Robot::mediumSpeedController = 0;
okapi::AsyncMotionProfileController * Robot::turnController = 0;
okapi::AsyncMotionProfileController * Robot::profileController = 0;

okapi::AsyncPosIntegratedController * Robot::hoodController = 0;
// okapi::AsyncPosPIDController * Robot::cam_hood_Controller = 0;

okapi::ChassisControllerIntegrated* Robot::base = 0;

Robot::Robot() {
    collector = new Collector();
    nuc = new NucLauncher();
    cam = new Camera();
    
    static std::vector<vision_object_s_t> flagsVector;
    Camera::targetVector = &flagsVector;

    static auto drive = ChassisControllerFactory::create(
		{RobotStates::BASE_LEFT_FRONT,RobotStates::BASE_LEFT_BACK},
		{-RobotStates::BASE_RIGHT_FRONT,-RobotStates::BASE_RIGHT_BACK}, 
		AbstractMotor::gearset::green,
        {4_in, 12.5_in}); //12.5
    
    Robot::base = &drive;

    static auto _hoodController = AsyncControllerFactory::posIntegrated(*(nuc->hood_Motor), 100);
    static auto _turnController = AsyncControllerFactory::motionProfile(
        in2meter(20.0),  // Maximum linear velocity of the Chassis in m/s 20
        in2meter(30.0),  // Maximum linear acceleration of the Chassis in m/s/s 30
        in2meter(300.0), // Maximum linear jerk of the Chassis in m/s/s/s 300
        *base // Chassis Controller
    );

    static auto _profileController = AsyncControllerFactory::motionProfile(
        in2meter(45.0),  // Maximum linear velocity of the Chassis in m/s 40
        in2meter(65.0),  // Maximum linear acceleration of the Chassis in m/s/s 60
        in2meter(500.0), // Maximum linear jerk of the Chassis in m/s/s/s
        *base // Chassis Controller
    );

    static auto _mediumSpeedController = AsyncControllerFactory::motionProfile(
        in2meter(20.0),  // Maximum linear velocity of the Chassis in m/s 40
        in2meter(30.0),  // Maximum linear acceleration of the Chassis in m/s/s 60
        in2meter(300.0), // Maximum linear jerk of the Chassis in m/s/s/s
        *base // Chassis Controller
    );

    // std::shared_ptr<ControllerInput<double>> iinput = std::make_shared<Camera>();
    // std::shared_ptr<ControllerOutput<double>> ioutput (Robot::nuc->hood_Motor);
    // auto hoodTime = TimeUtilFactory::withSettledUtilParams(5, 5, 100_ms);
	// static auto _cam_hood_Controller = AsyncControllerFactory::posPID(iinput, ioutput, 0.002, 0.0, 0.005, 0.0, std::make_unique<okapi::PassthroughFilter>(), hoodTime);

    turnController = &_turnController;
    profileController = &_profileController;
    mediumSpeedController = &_mediumSpeedController;
    hoodController = &_hoodController;
    // cam_hood_Controller = &_cam_hood_Controller;
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
            // delay(100);
            // RobotStates::is_Shooting_Ball = false;
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
        if(RobotStates::is_autoAligning) {
            if(RobotStates::targetFlag_X != 0) {
                err = (VISION_FOV_WIDTH / 2.0 + RobotStates::hortizontal_correction) - RobotStates::targetFlag_X;
            } else {
                err = 0;
            }
            
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

void Robot::alignTheHood(void * param) {
    double err;
    int baseSpeed = 10; 
    while(true) {
        if(RobotStates::is_autoHooding) {
            if(RobotStates::targetFlag_Y != 0) {
                err = (VISION_FOV_HEIGHT / 2.0 + RobotStates::hortizontal_correction) - RobotStates::targetFlag_Y;
            } else {
                err = 0;
            }
            
            if(!RobotStates::is_Hooded) {
                int output = (err > 0) ? (baseSpeed + 0.5 * err) : (-baseSpeed + 0.5 * err);
                if(fabs(err) > 5) {
                    Robot::nuc->hood_Motor->move(output);
                } else {
                    Robot::nuc->hood_Motor->move(0);
                    RobotStates::is_Hooded = true;
                    RobotStates::is_autoHooding = false;
                }
                printf("hood output: %d \n", output);
            }
        }
        pros::delay(10);
    }
}

void Robot::toggle_AssistShooting() {
    if(RobotStates::is_assistant_Shooting) { //disrupt
        RobotStates::is_assistant_Shooting = false;
        Robot::hoodController->flipDisable(true);
        RobotStates::is_Collecting_Ball = false;
        RobotStates::is_Shooting_Ball = false;
        Robot::nuc->hood_Motor->setReversed(false);
    } else {
        RobotStates::is_assistant_Shooting = true; //enable
    }
    
    pros::delay(500);
}

void Robot::assistShooting(void * param) {
    while(true) {
        if(RobotStates::is_assistant_Shooting) {
            RobotStates::is_Shooting_Ball = false;
            RobotStates::is_Collecting_Ball = false;

            Robot::nuc->hood_Motor->setReversed(true);
            Robot::hoodController->setMaxVelocity(200);
            
            if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
                RobotStates::hortizontal_correction = -15.0;
            } else {
                RobotStates::hortizontal_correction = 15.0;
            }
            RobotStates::is_Aligned = false;
            RobotStates::is_autoAligning = true;
            pros::delay(100); //200
            while(!RobotStates::is_Aligned) {
                pros::delay(10); //50
            }

            RobotStates::hortizontal_correction = 0.0;
            Robot::hoodController->reset();
            Robot::hoodController->tarePosition();
            Robot::hoodController->flipDisable(false);
            Robot::hoodController->setTarget(0); //60 //40
            Robot::hoodController->waitUntilSettled();

            //TODO: the new double shot, need to be extremely fast!
            // RobotStates::is_Shooting_Ball = true;
            // delay(200); //150
            // Robot::hoodController->setTarget(130); //130
            // // RobotStates::is_Shooting_Ball = true;

            // // pros::delay(100);
            // // RobotStates::is_Shooting_Ball = false;

            // // RobotStates::is_Collecting_Ball = true;
            // // pros::delay(250);
            // // RobotStates::is_Collecting_Ball = false;

            // // Robot::hoodController->setTarget(130); //130
            // Robot::hoodController->waitUntilSettled();
            // // RobotStates::is_Shooting_Ball = true;
            // // pros::delay(200);
            // RobotStates::is_Shooting_Ball = false;
            
            // Robot::hoodController->flipDisable(true);
            
            // Robot::nuc->hood_Motor->setReversed(false);

            // delay(500);
            // Robot::nuc->hoodDown();
            // pros::delay(500);
            // Robot::nuc->hoodStop();
            // RobotStates::is_assistant_Shooting = false;

            //previous version: slow 
            //TODO: change it to this at Crespi
            RobotStates::is_Shooting_Ball = true;
            pros::delay(100);
            RobotStates::is_Shooting_Ball = false;
            // RobotStates::is_Collecting_Ball = true;
            // pros::delay(250);
            // RobotStates::is_Collecting_Ball = false;

            Robot::hoodController->setTarget(130); //130
            Robot::hoodController->waitUntilSettled();
            RobotStates::is_Shooting_Ball = true;
            pros::delay(200);
            RobotStates::is_Shooting_Ball = false;
            
            Robot::hoodController->flipDisable(true);
            
            Robot::nuc->hood_Motor->setReversed(false);

            // delay(500);
            Robot::nuc->hoodDown();
            pros::delay(500);
            Robot::nuc->hoodStop();
            RobotStates::is_assistant_Shooting = false;
        }
        pros::delay(200);
    }
}

void Robot::oneShot(void * param) {
    while(true) {
        if(RobotStates::is_oneShot) {
            RobotStates::is_Shooting_Ball = true;
            pros::delay(100);
            RobotStates::is_Shooting_Ball = false;
            RobotStates::is_oneShot = false;
        }
        pros::delay(100);
    }
} 

void Robot::toggle_OneShot() {
    RobotStates::is_oneShot = !RobotStates::is_oneShot;
    pros::delay(200);
}

void Robot::rest_before_driver() {
    RobotStates::is_Collecting_Ball = false;
    RobotStates::is_Shooting_Ball = false;
    RobotStates::is_autoAiming = false;
    RobotStates::is_Aimed = false;
    RobotStates::is_Flywheel_Running = false;
    RobotStates::is_autoAligning = false;
    RobotStates::is_Aligned = false;
    RobotStates::is_oneShot = false;

    RobotStates::hortizontal_correction = 0.0;

    Robot::hoodController->flipDisable(true);
    Robot::hoodController->reset();

    Robot::nuc->hood_Motor->setReversed(false);
    Robot::rightFront_Motor->setReversed(true);
    Robot::rightBack_Motor->setReversed(true);
}

double Robot::in2meter(double in) {
    return (in / 39.37);
}