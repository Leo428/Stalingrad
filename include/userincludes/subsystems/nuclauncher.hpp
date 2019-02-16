#ifndef NUC_LAUNCHER
#define NUC_LAUNCHER

#include "main.h"
#include "userincludes/robotstates.hpp"

class NucLauncher {
    public:
        NucLauncher();

        //methods 
        void stabilize_the_Flywheel();
        void stop_Flywheel();
        void toggle_Flywheel();
        void toggleAutoAim();
        void hoodUp();
        void hoodDown();
        void hoodStop();
        void autoAim(double err);
        static void autoAim_Task(void * param);
        static okapi::Motor * hood_Motor;
        static okapi::Motor * flywheel_Motor;
        static ADIPotentiometer * pot;
        // static double hoodPID_Output;
        // static double target_Y;

    private:
        //vars 

        //methods
        void initMotors();
};

#endif