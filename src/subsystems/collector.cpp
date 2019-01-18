#include "userincludes/subsystems/collector.hpp"

Collector::Collector() {
    ballCollector = new okapi::Motor(RobotStates::BALL_PORT);
    ballCollector->setGearing(okapi::AbstractMotor::gearset::green);
    capCollector = new okapi::Motor(RobotStates::CAP_PORT);
    capCollector->setGearing(okapi::AbstractMotor::gearset::green);
}

void Collector::collectBalls() {
    ballCollector->move(127);
    // ballCollector->moveVoltage(12000);
}

void Collector::shootBall() {
    ballCollector->move(-127);
    // ballCollector->moveVoltage(-12000);
}

void Collector::stopCollector() {
    ballCollector->move(0);
    // ballCollector->moveVoltage(0);
}

void Collector::capUp() {
    capCollector->move(127);
    // capCollector->moveVoltage(1000);
}

void Collector::capDown() {
    capCollector->move(-127);
    // capCollector->moveVoltage(-1000);
}

void Collector::capDownSlowly() {
    capCollector->move(-70);
}

void Collector::capUpSlowly() {
    capCollector->move(70);
}

void Collector::capStop() {
    capCollector->move(0);
    // capCollector->moveVoltage(0);
}

