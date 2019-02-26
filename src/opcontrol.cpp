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
// okapi::ControllerButton * shootButton = new ControllerButton(okapi::ControllerDigital::R2);

void showRPM(void* param) {
	auto rpmStr = std::to_string(Robot::nuc->flywheel_Motor->getActualVelocity());
	while(true) {
		rpmStr = std::to_string(Robot::nuc->flywheel_Motor->getActualVelocity());
		master.setText(0, 0, rpmStr);
		delay(1000);
	}
}

void testHood(void * param) {
	while(true) {
		if(RobotStates::is_assistant_Shooting) {
			Robot::collector->ballCollector->moveVelocity(-100);
			delay(200);
			Robot::nuc->hood_Motor->moveVelocity(-100);
			
			delay(500);
			Robot::nuc->hood_Motor->moveVelocity(0);
			delay(1000);
			Robot::collector->ballCollector->moveVelocity(0);
			RobotStates::is_assistant_Shooting = false;
		}
		delay(50);
	}
}

void opcontrol() {
	Robot::getInstance()->rest_before_driver();
	RobotStates::is_Flywheel_Running = true; //TODO: enable for comp

	Task flywheelTask(Robot::operate_Flywheel);
	Task visionTask(Robot::testTracking);
	Task collectorTask(Robot::operate_BallCollector);
	Task assistShooting(Robot::assistShooting);
	// Task alignTask(Robot::alignTheBot);

	Task displayRPM(showRPM);
	// Task movePot(Robot::hoodWithPot);
	// Task coolDoubleShot(testHood);

	// auto rpmStr = std::to_string(Robot::nuc->flywheel_Motor->getActualVelocity());
	// Robot::hoodController->setMaxVelocity(200);

	// std::shared_ptr<ControllerInput<double>> iinput = std::make_shared<Camera>();
	// std::shared_ptr<ControllerOutput<double>> ioutput = std::make_shared<okapi::Motor>(6);
	// std::shared_ptr<ControllerOutput<double>> ioutput (Robot::nuc->hood_Motor);

	// RobotStates::potTarget = 150;
	while (true) {
		// printf("pot new value: %d \n", Robot::nuc->pot->get_value_calibrated());

		Robot::base->tank(master.getAnalog(okapi::ControllerAnalog::leftY), master.getAnalog(okapi::ControllerAnalog::rightY), 0.2);

		if(!RobotStates::is_assistant_Shooting) {
			if(master.getDigital(okapi::ControllerDigital::R1)) {
				RobotStates::is_Shooting_Ball = false;
				RobotStates::is_Collecting_Ball = true;
				// Robot::collector->collectBalls();
			} else if (master.getDigital(okapi::ControllerDigital::R2)){
				RobotStates::is_Collecting_Ball = false;
				RobotStates::is_Shooting_Ball = true;
				// Robot::collector->shootBall();
			} else {
				RobotStates::is_Collecting_Ball = false;
				RobotStates::is_Shooting_Ball = false;
				// Robot::collector->stopCollector();
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
			// Robot::collector->capCollector->moveVelocity(0);
			// flipperController.flipDisable(false);
		}

		if(!RobotStates::is_assistant_Shooting && !RobotStates::is_pot) {
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
		} else if (master.getDigital(okapi::ControllerDigital::B)) {
			// Robot::getInstance()->toggle_AssistShooting_back();
		}

		// if(master.getDigital(okapi::ControllerDigital::B)) {
		// 	Robot::getInstance()->toggle_OneShot();
		// }
		
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
		
		pros::delay(10);
	}
}
