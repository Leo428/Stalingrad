#ifndef DRIVE_BASE
#define DRIVE_BASE

#include "main.h"
#define LEFT_MOTOR 1

class Base {
    public:
        Base();
        static okapi::Motor leftFront_Motor;
        static okapi::Motor rightFront_Motor; 
        
    private:
        
};

#endif