#ifndef ROBOT_STATES
#define ROBOT_STATES

class RobotStates {
    public: 
        RobotStates();

        static RobotStates* getInstance();
        //ports config
        const static double ROBOT_WIDTH_in;
        const static double ROBOT_LENGTH_in;
        const static double ROBOT_MAX_VELOCITY_in;
        const static double ROBOT_MAX_ACCEL_in;
        const static double ROBOT_MAX_JERK_in;

        const static int CAMERA_PORT = 5;
        const static int CAMERA_STATIC = 15;
        const static int FLYWHEEL_PORT = 13;
        const static int HOOD_PORT = 18;
        const static int CAP_PORT = 3;
        const static int BALL_PORT = 8;
        const static int BASE_RIGHT_BACK = 10;
        const static int BASE_RIGHT_FRONT = 1;
        const static int BASE_LEFT_BACK = 20;
        const static int BASE_LEFT_FRONT = 11;

        static double targetY;
        static double targetFlag_X;
        static double targetFlag_Y;
        static double hortizontal_correction;
        static double vertical_correction;
        
        //in game states
        enum FieldColor {
            UNKNOWN,
			RED,
			BLUE
		}; 

        enum AutoChoice {
            NO_AUTO,
            FOUR_FLAGS,
            THREE_FLAGS_PLAT,
            THREE_FLAGS_CAP,
            THREE_FLAGS_SKILLS
        };

        static FieldColor fieldColor;
        static AutoChoice autoChoice;

        static bool is_Flywheel_Running; 
        static bool is_autoAiming;
        static bool is_Aimed;
        static bool is_Collecting_Ball;
        static bool is_Shooting_Ball;
        static bool is_autoAligning; 
        static bool is_autoHooding;
        static bool is_Hooded;
        static bool is_Aligned;
        static bool is_Static_Cam_Detecting;
        static bool is_assistant_Shooting;
        static bool is_assistant_Shooting_back;
        static bool is_oneShot;
    private: 
        static RobotStates* instance;
};

#endif