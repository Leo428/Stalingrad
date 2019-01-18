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
bool vibrating = false;
static double hoodDrive = 0;

void vibrate(void* param) {
	vibrating = true;
	master.rumble("-");
	pros::delay(1000);
	vibrating = false;
}

void opcontrol() {
	Robot::getInstance()->rest_before_driver();
	pros::delay(500);
	
	Task flywheelTask(Robot::operate_Flywheel);
	Task visionTask(Robot::testTracking);
	Task collectorTask(Robot::operate_BallCollector);

	// Task aimTask(Robot::autoAim_Task);
	// Task alignTask(Robot::alignTheBot);

	pros::delay(500);

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

		if(master.getDigital(okapi::ControllerDigital::R1)) {
			RobotStates::is_Shooting_Ball = false;
			RobotStates::is_Collecting_Ball = true;
			// Robot::collector->collectBalls();
		} else if (master.getDigital(okapi::ControllerDigital::R2)) {
			// Robot::collector->shootBall();
			RobotStates::is_Collecting_Ball = false;
			RobotStates::is_Shooting_Ball = true;
		} else {
			// Robot::collector->stopCollector();
			RobotStates::is_Collecting_Ball = false;
			RobotStates::is_Shooting_Ball = false;
		}

		if(master.getDigital(okapi::ControllerDigital::L1)) {
			Robot::collector->capUp();
		} else if (master.getDigital(okapi::ControllerDigital::L2)) {
			Robot::collector->capDown();
		} else {
			Robot::collector->capStop();
		}

		if(!RobotStates::is_autoAiming) {
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
			RobotStates::is_autoAligning = true;
			RobotStates::is_Aligned = false;
			delay(200);
			// Robot::cam->selectTarget();
			// Robot::nuc->toggleAutoAim();
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
		pros::delay(10);
	}
}
