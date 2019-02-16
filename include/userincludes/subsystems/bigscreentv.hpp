#ifndef BIG_SCREEN_TV
#define BIG_SCREEN_TV

#include "main.h"
#include "pros/apix.h"
#include "userincludes/robotstates.hpp"

// class BigScreenTV {
//     public:
//         void createUIComponents();
// };
class BigScreenTV {
	public:
		BigScreenTV();
		// variables
		
		static lv_obj_t * label_side_choice;
		static lv_obj_t * label_static_cam_connection;
		static lv_obj_t * ddl1;

		//methods
		void createUIComponents();
		void lv_test_theme_1(lv_theme_t * th);
		static void updateScreen(void* param);

	private:
		//variables
		static lv_res_t selSide(lv_obj_t * btn);
		static lv_res_t selAuto(lv_obj_t * list);
		static lv_res_t selNextAuto(lv_obj_t * btn);
		
		//methods
		static void create_tab1(lv_obj_t * parent);
		static void create_tab2(lv_obj_t * parent);
		static void create_tab3(lv_obj_t * parent);
};

#endif