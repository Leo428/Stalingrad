#ifndef CAMERA
#define CAMERA

#include "main.h"
#include "userincludes/robotstates.hpp"

class Camera : public ControllerInput<double> {
    public:
        Camera();

        double controllerGet() override;
        static Vision * scope;
        
        void updateSensor();
        void selectTarget();

        //vars
        
        static std::vector<vision_object_s_t> * targetVector;

    private:
        //methods
        void sortByHeight();
        void hortizontalSort();
        void filterTarget();
        static bool compareHeight(vision_object_s_t i, vision_object_s_t j);

        //objects (flags)
        vision_object_s_t allFlags[6];
        vision_object_s_t targetFlags[3];
        // vision_color_code_t flagCode;
        
};

#endif