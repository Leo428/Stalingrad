#include "main.h"

#include "userincludes/robotstates.hpp"
#include "userincludes/robot.hpp"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 * 
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

okapi::Controller master;
okapi::ControllerButton * shootButton = new ControllerButton(okapi::ControllerDigital::R2);
bool vibrating = false;
static double hoodDrive = 0;

void vibrate(void* param) {
	vibrating = true;
	master.rumble("-");
	pros::delay(1000);
	vibrating = false;
}

void oneShot(void * param) {
	while(true) {
		if(!RobotStates::is_assistant_Shooting) {
			if(shootButton->isPressed()) {
				
				if(shootButton->changedToPressed()) {
					RobotStates::is_Shooting_Ball = true;
					delay(100);
					RobotStates::is_Shooting_Ball = false;
				} else {
					RobotStates::is_Shooting_Ball = false;
				}
			} else {
				shootButton->changedToPressed();
				RobotStates::is_Shooting_Ball = false;
			}
		}
		delay(50);
	}
}

void opcontrol() {
	Robot::getInstance()->rest_before_driver();
	RobotStates::is_Flywheel_Running = true;
	// static auto flipperController = okapi::AsyncControllerFactory::posIntegrated(*Robot::collector->capCollector);
	// flipperController.setTarget(0);
	// flipperController.flipDisable(false);
	// pros::delay(500);
	
	Task flywheelTask(Robot::operate_Flywheel);
	Task visionTask(Robot::testTracking);
	Task collectorTask(Robot::operate_BallCollector);
	Task oneShotTask(oneShot);

	// Task aimTask(Robot::autoAim_Task);
	Task assistShooting(Robot::assistShooting);
	Task alignTask(Robot::alignTheBot);
	// Task oneShotTask(Robot::oneShot);

	Robot::hoodController->setMaxVelocity(200);

	// pros::delay(500);

	// std::vector<vision_object_s_t> gg;
	// gg.clear();
	
	// nuc->aim();
	
	// std::shared_ptr<ControllerInput<double>> iinput = std::make_shared<Camera>();
	// std::shared_ptr<ControllerOutput<double>> ioutput = std::make_shared<okapi::Motor>(6);
	// std::shared_ptr<ControllerOutput<double>> ioutput (Robot::nuc->hood_Motor);

	// auto hoodTime = TimeUtilFactory::withSettledUtilParams(20.0, 1000.0, 100_ms);
	// auto hoodController = AsyncControllerFactory::posPID(iinput, ioutput, 0.002, 0.0, 0.005, 0.0, std::make_unique<okapi::PassthroughFilter>(), hoodTime);
	// hoodController.flipDisable(true);
	// Robot::nuc->hood_Motor->setReversed(false);
	// hoodController.setOutputLimits(10.0, -10.0);
	// hoodController.setSampleTime(20_ms);
	// hoodController.setTarget(Robot::cam->targetY);
	// hoodController.flipDisable(false);
	// hoodController.waitUntilSettled();
	// hoodController.flipDisable(true);
	
	while (true) {
		Robot::base->tank(master.getAnalog(okapi::ControllerAnalog::leftY), master.getAnalog(okapi::ControllerAnalog::rightY), 0.2);
		// MotorGroup leftDrive({11,20});
		// MotorGroup rightDrive({-1, -10});

		// leftDrive.moveVelocity((int) (200.0 * master.getAnalog(okapi::ControllerAnalog::leftY)));
		// rightDrive.moveVelocity((int) (200.0 * master.getAnalog(okapi::ControllerAnalog::rightY)));
		if(!RobotStates::is_assistant_Shooting) {
			if(master.getDigital(okapi::ControllerDigital::R1)) {
				RobotStates::is_Shooting_Ball = false;
				RobotStates::is_Collecting_Ball = true;
				// Robot::collector->collectBalls();
			} else {
				// Robot::collector->stopCollector();
				RobotStates::is_Collecting_Ball = false;
				// RobotStates::is_Shooting_Ball = false;
			}
		}

		if(master.getDigital(okapi::ControllerDigital::L1)) {
			// flipperController.flipDisable(true);
			Robot::collector->capUp();
		} else if (master.getDigital(okapi::ControllerDigital::L2)) {
			// flipperController.flipDisable(true);
			Robot::collector->capDown();
		} else {
			Robot::collector->capStop();
			// flipperController.flipDisable(false);
		}

		if(!RobotStates::is_assistant_Shooting) {
			if(master.getDigital(okapi::ControllerDigital::up)) {
				Robot::nuc->hoodUp();
			} else if (master.getDigital(okapi::ControllerDigital::down)) {
				Robot::nuc->hoodDown();
			} else {
				Robot::nuc->hoodStop();
			}
		}
		
		if(master.getDigital(okapi::ControllerDigital::X)) {
			Robot::nuc->toggle_Flywheel();
		}

		if(master.getDigital(okapi::ControllerDigital::A)) {
			Robot::getInstance()->toggle_AssistShooting();
			// Robot::cam->selectTarget();
			// Robot::nuc->toggleAutoAim();
		}

		if(master.getDigital(okapi::ControllerDigital::B)) {
			Robot::getInstance()->toggle_OneShot();
		}
		
		// drive.arcade(master.getAnalog(okapi::ControllerAnalog::leftY), master.getAnalog(okapi::ControllerAnalog::leftX));
		// nuc->flywheel_Motor.moveVelocity(600);
		
		
		// printf("PID: target: %f \n", hoodController.getTarget());
		// printf("PID error: %f \n", hoodController.getError());
		// printf("PID output: %f \n", hoodController.getOutput());
		// printf("off by: %f \n", cam->targetY);
		// printf("alive \n");
		// if(master.get_digital(DIGITAL_R2)) {
		// 	if(!vibrating) {
		// 		Task vibeTask(vibrate);
		// 	}
		// }
		pros::delay(1);
	}
}
