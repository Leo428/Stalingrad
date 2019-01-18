#ifndef ROBOT
#define ROBOT

#include "userincludes/robotstates.hpp"
#include "userincludes/subsystems/collector.hpp"
#include "userincludes/subsystems/nuclauncher.hpp"
#include "userincludes/subsystems/camera.hpp"

class Robot {
    public:
        Robot();
        static okapi::ChassisControllerIntegrated* base;
        static Collector* collector;
        static NucLauncher* nuc;
        static Camera* cam;
        static Robot* getInstance();

        static void operate_Flywheel(void * param);
        static void operate_BallCollector(void * param);
        static void doubleShot(void * param);
        static void autoAim_Task(void * param);
        static void alignTheBot(void * param);
        static void assistShooting(void * param);
        static void testTracking(void * param);

        static okapi::Motor * leftFront_Motor;
        static okapi::Motor * leftBack_Motor;
        static okapi::Motor * rightFront_Motor;
        static okapi::Motor * rightBack_Motor;

        void rest_before_driver();
        
    private:
        static Robot* instance;
};


#endif