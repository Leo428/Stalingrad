#ifndef COLLECTOR
#define COLLECTOR

#include "main.h"
#include "userincludes/robotstates.hpp"

class Collector {
    public:
        Collector();

        //methods 
        void collectBalls();
        void shootBall();
        void shootGood();
        void stopCollector(); 
        void capUp();
        void capDown();
        void capDownSlowly();
        void capUpSlowly();
        void capStop();
        
        okapi::Motor* ballCollector = 0;
        okapi::Motor* capCollector = 0;

    private:
        
};

#endif