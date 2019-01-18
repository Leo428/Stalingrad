#include "userincludes/subsystems/camera.hpp"
#include <cmath>

Vision* Camera::scope = 0;
std::vector<vision_object_s_t> * Camera::targetVector = 0;

Camera::Camera() {
    scope = new Vision(RobotStates::CAMERA_STATIC, E_VISION_ZERO_TOPLEFT);
    scope->clear_led();
    // flagCode = scope->create_color_code(3,1,0,0,0);
}

double Camera::controllerGet() {
    updateSensor();
    // printf("function called: %f \n", this->targetY);
    return RobotStates::targetFlag_Y;
}

void Camera::sortByHeight() {
    std::sort(allFlags, allFlags + 6, compareHeight);
}

void Camera::filterTarget() {
    // int index = 0;
    targetVector->clear();
    for(pros::vision_object_s_t obj : this->allFlags) {
        double ratio1 = fabs((obj.height*1.0) / (obj.width*1.0) - 1.0);
        double ratio2 = fabs((obj.width*1.0) / (obj.height*1.0) - 1.0);
        if (obj.height > 10 && obj.width > 10) {
            if(true) { //ratio1 < 0.8 && ratio2 < 0.8
                if (obj.signature == 1) {
                    targetVector->push_back(obj);
                    // this->targetFlags[index] = obj;
                    // index++;
                }
            }
        }
    }
}

void Camera::hortizontalSort() {
    targetVector->clear();
    for(pros::vision_object_s_t obj : this->allFlags) {
        if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
            if (obj.signature == 1) {
                targetVector->push_back(obj);
            }
        } else if (RobotStates::fieldColor == RobotStates::FieldColor::RED) {
            if (obj.signature == 2) {
                targetVector->push_back(obj);
            }
        }
    }
}

bool Camera::compareHeight(vision_object_s_t i, vision_object_s_t j) {
    return (i.y_middle_coord < j.y_middle_coord);
}

void Camera::selectTarget() {
    if(!targetVector->empty()) {
        double tempY = targetVector->at(0).height * (1.0);
        RobotStates::targetY = -0.0235 * pow(tempY, 3) + 1.1648*pow(tempY,2)-23.192*tempY+324.13;
    } else {
        RobotStates::targetY = 0;
    }
}

void Camera::updateSensor() {
    printf("I see %d objects;\n", scope->get_object_count());
    if(scope->get_object_count() > 0 && scope->get_object_count() != 2147483647) {
        RobotStates::is_Static_Cam_Detecting = true;
    } else {
        RobotStates::is_Static_Cam_Detecting = false;
    }
    printf("I see objects: %d\n", RobotStates::is_Static_Cam_Detecting);
    // vision_object_s_t flag = scope->get_by_code(0, flagCode);
    // printf("code test: %d \n", flag.angle);
    // targetVector().clear();
    scope->read_by_size(0, 6, this->allFlags);
    // sortByHeight();
    // filterTarget();
    hortizontalSort();
    // for(vision_object_s_t obj : this->allFlags)
    // {
    //     // printf("flag color: %d; width: %d; height: %d \n", obj.signature, obj.width, obj.height);
    //     if (obj.signature == 1) {
    //         printf("flag color: %s; width: %d; height: %d; x: %d; y: %d \n", "red", obj.width, obj.height, obj.x_middle_coord, obj.y_middle_coord);
    //     }
    //     if (obj.signature == 2) {
    //         printf("flag color: %s; width: %d; height: %d; x: %d; y: %d \n", "blue", obj.width, obj.height, obj.x_middle_coord, obj.y_middle_coord);
    //     }
    // }
    vision_object_s_t target;
    if(!targetVector->empty()) {
        target = targetVector->at(0);
        RobotStates::targetFlag_Y = target.y_middle_coord * 1.0;
        RobotStates::targetFlag_X = target.x_middle_coord * 1.0;
        // this->targetY = target.height * (1.0);
        // this->targetY = -0.0235 * pow(this->targetY, 3) + 1.1648*pow(this->targetY,2)-23.192*this->targetY+324.13;
    } else {
        RobotStates::targetFlag_Y = 0;
        RobotStates::targetFlag_X = 0;
        // this->targetY = 0;
    }

    // for(vision_object_s_t obj : *targetVector)
    // // for(vision_object_s_t obj : this->targetFlags)
    // {
    //     // printf("flag color: %d; width: %d; height: %d \n", obj.signature, obj.width, obj.height);
    //     if (obj.signature == 1) {
    //         printf("flag color: %s; width: %d; height: %d; x: %d; y: %d \n", "red", obj.width, obj.height, obj.x_middle_coord, obj.y_middle_coord);
            
    //     }
    //     if (obj.signature == 2) {
    //         printf("flag color: %s; width: %d; height: %d; x: %d; y: %d \n", "blue", obj.width, obj.height, obj.x_middle_coord, obj.y_middle_coord);
    //     }
    // }
    // printf("-------------------------------------------------------- \n");
    
    // printf("where it is now: %f \n", RobotStates::targetFlag_Y);
    // printf("target at: %f \n", RobotStates::targetY);
}