#include "userincludes/subsystems/bigscreentv.hpp"

BigScreenTV::BigScreenTV(){
	
};

// RobotStates::FieldColor BigScreenTV::fieldColor = RobotStates::FieldColor::UNKNOWN;
lv_obj_t* BigScreenTV::label_side_choice = 0;
lv_obj_t * BigScreenTV::label_static_cam_connection = 0;
lv_obj_t * BigScreenTV::ddl1 = 0;

void BigScreenTV::createUIComponents(){
	lv_theme_t* theme = lv_theme_alien_init(0, &lv_font_dejavu_20);
	lv_theme_set_current(theme);
	lv_test_theme_1(theme);
};

/**
 * Create a test screen with a lot objects and apply the given theme on them
 * @param th pointer to a theme
 */
void BigScreenTV::lv_test_theme_1(lv_theme_t * th)
{
    lv_theme_set_current(th);
    th = lv_theme_get_current();    
	/*If `LV_THEME_LIVE_UPDATE  1` `th` is not used directly so get the real theme after set*/

    lv_obj_t * tv = lv_tabview_create(lv_scr_act(), NULL);
    lv_obj_set_size(tv, LV_HOR_RES, LV_VER_RES);
    lv_obj_t * tab1 = lv_tabview_add_tab(tv, "Tab 1");
    lv_obj_t * tab2 = lv_tabview_add_tab(tv, "Tab 2");
    lv_obj_t * tab3 = lv_tabview_add_tab(tv, "Tab 3");

    create_tab1(tab1);
    // create_tab2(tab2);
    // create_tab3(tab3);
}

void BigScreenTV::create_tab1(lv_obj_t * tab_view)
{
	lv_obj_t * btn_red_side = lv_btn_create(tab_view, NULL);
	lv_obj_set_size(btn_red_side, 80, 40);
	lv_cont_set_fit(btn_red_side, true, true);
	lv_obj_set_free_num(btn_red_side, 1);
	lv_btn_set_action(btn_red_side, LV_BTN_ACTION_CLICK ,selSide);

	label_side_choice = lv_label_create(tab_view, NULL);
	lv_obj_align(label_side_choice, btn_red_side, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
	lv_label_set_text(label_side_choice, "Choose Side");

	lv_obj_t * btn_blue_side = lv_btn_create(tab_view, btn_red_side);
	lv_obj_align(btn_blue_side, label_side_choice, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
	lv_obj_set_free_num(btn_blue_side, 2);
	lv_btn_set_action(btn_blue_side, LV_BTN_ACTION_CLICK ,selSide);

	label_static_cam_connection = lv_label_create(tab_view, NULL);
	lv_obj_align(label_static_cam_connection, btn_blue_side, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
	lv_label_set_text(label_static_cam_connection, "Static CAM");

	lv_obj_t * label_red_side = lv_label_create(tab_view, NULL);
	lv_label_set_text(label_red_side, "RED");
	lv_obj_align(label_red_side, btn_red_side, LV_ALIGN_CENTER, 0, 0);

	lv_obj_t * label_blue_side = lv_label_create(tab_view, NULL);
	lv_label_set_text(label_blue_side, "BLUE");
	lv_obj_align(label_blue_side, btn_blue_side, LV_ALIGN_CENTER, 0, 0);

	ddl1 = lv_ddlist_create(tab_view, NULL);
	lv_obj_align(ddl1, tab_view, LV_ALIGN_OUT_TOP_LEFT, 20, 0);
	lv_ddlist_set_options(ddl1, "No Auto\n Four Flags\n 3 Flags PLat\n 3 Flags Cap\n BackTile\n Skills");
	lv_ddlist_set_action(ddl1, selAuto);
	lv_ddlist_set_sb_mode(ddl1, lv_sb_mode_t::LV_SB_MODE_AUTO);
	// lv_ddlist_set_draw_arrow(ddl1, true)
	// // lv_ddlist_open(ddl1, true);
	// // lv_ddlist_set_selected(ddl1, 0);

	lv_obj_t * btn_next_option = lv_btn_create(tab_view, btn_red_side);
	lv_obj_align(btn_next_option, ddl1, LV_ALIGN_IN_RIGHT_MID, 100, 0);
	lv_obj_set_free_num(btn_next_option, 3);
	lv_btn_set_action(btn_next_option, LV_BTN_ACTION_CLICK ,selNextAuto);

	lv_obj_t * label_next_auto = lv_label_create(tab_view, NULL);
	lv_label_set_text(label_next_auto, "Next");
	lv_obj_align(label_next_auto, btn_next_option, LV_ALIGN_CENTER, 0, 0);
}

lv_res_t BigScreenTV::selAuto(lv_obj_t * list) {
	int index = lv_ddlist_get_selected(list);
	// printf("selected %d \n", index);
	switch(lv_ddlist_get_selected(list)){
		case (1):
			RobotStates::autoChoice = RobotStates::AutoChoice::FOUR_FLAGS;
			break;
		case (2):
			RobotStates::autoChoice = RobotStates::AutoChoice::THREE_FLAGS_PLAT;
			break;
		case (3):
			RobotStates::autoChoice = RobotStates::AutoChoice::THREE_FLAGS_CAP;
			break;
		case (4):
			RobotStates::autoChoice = RobotStates::AutoChoice::BACK_TILE;
			break;
		case (5):
			RobotStates::autoChoice = RobotStates::AutoChoice::THREE_FLAGS_SKILLS;
			break;

		default:
			RobotStates::autoChoice = RobotStates::AutoChoice::NO_AUTO;
			break;
	}
	return LV_RES_OK;
}

lv_res_t BigScreenTV::selSide(lv_obj_t * btn) {
	// char sel_str;
	// lv_ddlist_get_selected_str(ddlist, &sel_str);
	// lv_ddlist_set_selected(ddlist, lv_ddlist_get_selected(ddlist));
	switch(lv_obj_get_free_num(btn)){
		case (1):
			RobotStates::fieldColor = RobotStates::FieldColor::RED;
			break;
		case (2):
			RobotStates::fieldColor = RobotStates::FieldColor::BLUE;
			break;
		default:
			RobotStates::fieldColor = RobotStates::FieldColor::UNKNOWN;
			break;
	}
	RobotStates::FieldColor temp = RobotStates::fieldColor;
	printf("triggered button id: %d, sel: %d \n", lv_obj_get_free_num(btn), temp);
	return LV_RES_OK;
}

lv_res_t BigScreenTV::selNextAuto(lv_obj_t * btn) {
	printf("auto enum before: %d \n", RobotStates::autoChoice);
	if(RobotStates::autoChoice < RobotStates::MAX_AUTO_CHOICE - 1) {
		lv_ddlist_set_selected(ddl1, RobotStates::autoChoice + 1);
	} else {
		lv_ddlist_set_selected(ddl1, 0);
	}
	selAuto(ddl1);
	printf("auto enum after: %d \n", RobotStates::autoChoice);
	return LV_RES_OK;
}

void BigScreenTV::updateScreen(void* param) {
	while(true) {
		//update UIs
		switch(RobotStates::fieldColor){
			case RobotStates::FieldColor::RED:
				lv_label_set_text(label_side_choice, "RED");
				break;
			case RobotStates::FieldColor::BLUE:
				lv_label_set_text(label_side_choice, "BLUE");
				break;
			default:
				lv_label_set_text(label_side_choice, "Choose Side");
				break;
		}

		if(RobotStates::is_Static_Cam_Detecting) {
			lv_label_set_text(label_static_cam_connection, "Detect!");
		} else {
			lv_label_set_text(label_static_cam_connection, "Nothing!");
		}

		pros::delay(100); //20
	}
}