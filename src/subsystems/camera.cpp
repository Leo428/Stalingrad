#include "userincludes/subsystems/camera.hpp"
#include <cmath>

Vision * Camera::buttomCam = 0;
Vision * Camera::topCam = 0;
std::vector<vision_object_s_t> * Camera::targetVector = 0;
std::vector<vision_object_s_t> * Camera::hoodVector = 0;

Camera::Camera() {
    buttomCam = new Vision(RobotStates::CAMERA_STATIC, E_VISION_ZERO_TOPLEFT);
    topCam = new Vision(RobotStates::CAMERA_PORT, E_VISION_ZERO_TOPLEFT);
    buttomCam->clear_led();
    topCam->clear_led();
    buttomCam->set_exposure(75);
    // flagCode = buttomCam->create_color_code(3,1,0,0,0);
}

double Camera::controllerGet() {
    updateSensor();
    // printf("function called: %f \n", this->targetY);
    return RobotStates::targetFlag_Y;
}

void Camera::sortByHeight() {
    std::sort(hoodFlags, hoodFlags + 6, compareHeight);
}

void Camera::sortByCenter() {
    std::sort(targetVector->begin(), targetVector->end(), compareCenter);
}

void Camera::filterTarget() {
    // int index = 0;
    hoodVector->clear();
    for(pros::vision_object_s_t obj : this->hoodFlags) {
        // double ratio1 = fabs((obj.height*1.0) / (obj.width*1.0) - 1.0);
        // double ratio2 = fabs((obj.width*1.0) / (obj.height*1.0) - 1.0);
        if (obj.height > 10 && obj.width > 10) {
            if(RobotStates::fieldColor == RobotStates::FieldColor::BLUE) {
                if (obj.signature == 1) {
                    hoodVector->push_back(obj);
                }
            } else if (RobotStates::fieldColor == RobotStates::FieldColor::RED) {
                if (obj.signature == 2) {
                    hoodVector->push_back(obj);
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

bool Camera::compareCenter(vision_object_s_t i, vision_object_s_t j) {
    return ((fabs(i.x_middle_coord * 1.0 - (VISION_FOV_WIDTH * 1.0)/2) < fabs(j.x_middle_coord * 1.0 - (VISION_FOV_WIDTH * 1.0)/2)));
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
    // printf("I see %d objects;\n", buttomCam->get_object_count());
    if(buttomCam->get_object_count() > 0 && buttomCam->get_object_count() != 2147483647) {
        RobotStates::is_Static_Cam_Detecting = true;
    } else {
        RobotStates::is_Static_Cam_Detecting = false;
    }
    // printf("I see objects: %d\n", RobotStates::is_Static_Cam_Detecting);
    // vision_object_s_t flag = buttomCam->get_by_code(0, flagCode);
    // printf("code test: %d \n", flag.angle);
    // targetVector().clear();
    buttomCam->read_by_size(0, 6, this->allFlags);
    // topCam->read_by_size(0, 6, hoodFlags);
    // sortByHeight();
    // filterTarget();
    // printf("before %d \n", sizeof(this->allFlags)/(sizeof(this->allFlags[0])));
    hortizontalSort();
    // printf("after %d \n", targetVector->size());
    sortByCenter();
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
        // RobotStates::targetFlag_Y = target.y_middle_coord * 1.0;
        RobotStates::targetFlag_X = target.x_middle_coord * 1.0;
        // this->targetY = target.height * (1.0);
        // this->targetY = -0.0235 * pow(this->targetY, 3) + 1.1648*pow(this->targetY,2)-23.192*this->targetY+324.13;
    } else {
        // RobotStates::targetFlag_Y = 0;
        RobotStates::targetFlag_X = 0;
        // this->targetY = 0;
    }

    // vision_object_s_t hood_target;
    // if(!hoodVector->empty()) {
    //     hood_target = hoodVector->at(0);
    //     RobotStates::targetFlag_Y = hood_target.y_middle_coord * 1.0;
    //     // RobotStates::targetFlag_X = target.x_middle_coord * 1.0;
    //     // this->targetY = target.height * (1.0);
    //     // this->targetY = -0.0235 * pow(this->targetY, 3) + 1.1648*pow(this->targetY,2)-23.192*this->targetY+324.13;
    // } else {
    //     RobotStates::targetFlag_Y = 0;
    //     // RobotStates::targetFlag_X = 0;
    //     // this->targetY = 0;
    // }
    for(vision_object_s_t obj : *targetVector)
    // // // for(vision_object_s_t obj : this->targetFlags)
    {
        printf("flag color: %d; x: %d; y: %d \n", obj.signature, obj.x_middle_coord, obj.y_middle_coord);
    //     if (obj.signature == 1) {
    //         printf("flag color: %s; width: %d; height: %d; x: %d; y: %d \n", "red", obj.width, obj.height, obj.x_middle_coord, obj.y_middle_coord);
            
    //     }
    //     if (obj.signature == 2) {
    //         printf("flag color: %s; width: %d; height: %d; x: %d; y: %d \n", "blue", obj.width, obj.height, obj.x_middle_coord, obj.y_middle_coord);
    //     }
    }
    // printf("flag color: %d; x: %d; y: %d \n", targetVector->at(0).signature, targetVector->at(0).x_middle_coord, targetVector->at(0).y_middle_coord);
    printf("%d \n", buttomCam->get_exposure());
    printf("%d \n", buttomCam->get_object_count());
    printf("%d \n", targetVector->size());
    printf("%f \n", RobotStates::targetFlag_X);
    printf("-------------------------------------------------------- \n");
    
    // printf("where it is now: %f \n", RobotStates::targetFlag_Y);
    // printf("target at: %f \n", RobotStates::targetY);
}