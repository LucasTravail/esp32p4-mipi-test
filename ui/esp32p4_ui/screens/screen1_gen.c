/**
 * @file screen1_gen.c
 * @brief Template source file for LVGL objects
 */

/*********************
 *      INCLUDES
 *********************/

#include "screen1_gen.h"
#include "../esp32p4_ui.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/***********************
 *  STATIC VARIABLES
 **********************/

/***********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_obj_t * screen1_create(void)
{
    LV_TRACE_OBJ_CREATE("begin");


    lv_obj_t * the_root = NULL;

    #if ESP32P4_UI_CHECK_COMPILE_TARGET(ESP32P4_UI_TARGET_ALL)
    if (esp32p4_ui_check_target(ESP32P4_UI_TARGET_ALL)) {
        lv_obj_t * lv_obj_0 = lv_obj_create(NULL);
        lv_obj_set_name_static(lv_obj_0, "screen1_#");
        lv_obj_set_width(lv_obj_0, 466);
        lv_obj_set_height(lv_obj_0, 466);
        lv_obj_set_flag(lv_obj_0, LV_OBJ_FLAG_SCROLLABLE, false);
        lv_obj_set_style_bg_color(lv_obj_0, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(lv_obj_0, (255 * 100 / 100), 0);
        lv_obj_set_style_border_width(lv_obj_0, 0, 0);
        lv_obj_set_style_pad_all(lv_obj_0, 0, 0);
        lv_obj_set_style_radius(lv_obj_0, 0, 0);

        lv_obj_t * lv_arc_0 = lv_arc_create(lv_obj_0);
        lv_obj_set_width(lv_arc_0, 340);
        lv_obj_set_height(lv_arc_0, 340);
        lv_obj_set_align(lv_arc_0, LV_ALIGN_CENTER);
        lv_arc_set_rotation(lv_arc_0, 135);
        lv_arc_set_bg_start_angle(lv_arc_0, 0);
        lv_arc_set_bg_end_angle(lv_arc_0, 270);
        lv_arc_set_min_value(lv_arc_0, 0);
        lv_arc_set_max_value(lv_arc_0, 100);
        lv_arc_set_value(lv_arc_0, 40);
        lv_obj_set_style_arc_width(lv_arc_0, 24, 0);
        lv_obj_set_style_arc_width(lv_arc_0, 24, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(lv_arc_0, lv_color_hex(0x2d2d2d), 0);
        lv_obj_set_style_arc_color(lv_arc_0, lv_color_hex(0x00c8ff), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(lv_arc_0, lv_color_hex(0xffffff), LV_PART_KNOB);

        lv_obj_t * lv_label_0 = lv_label_create(lv_obj_0);
        lv_label_set_text(lv_label_0, "LVGL Pro");
        lv_obj_set_align(lv_label_0, LV_ALIGN_CENTER);
        lv_obj_set_y(lv_label_0, -20);
        lv_obj_set_style_text_color(lv_label_0, lv_color_hex(0xffffff), 0);

        lv_obj_t * lv_label_1 = lv_label_create(lv_obj_0);
        lv_label_set_text(lv_label_1, "ESP32-P4 test");
        lv_obj_set_align(lv_label_1, LV_ALIGN_CENTER);
        lv_obj_set_y(lv_label_1, 20);
        lv_obj_set_style_text_color(lv_label_1, lv_color_hex(0x171515), 0);

        lv_obj_t * lv_obj_1 = lv_obj_create(lv_obj_0);
        lv_obj_set_width(lv_obj_1, 26);
        lv_obj_set_height(lv_obj_1, 26);
        lv_obj_set_align(lv_obj_1, LV_ALIGN_TOP_MID);
        lv_obj_set_y(lv_obj_1, 40);
        lv_obj_set_flag(lv_obj_1, LV_OBJ_FLAG_SCROLLABLE, false);
        lv_obj_set_style_radius(lv_obj_1, 13, 0);
        lv_obj_set_style_bg_color(lv_obj_1, lv_color_hex(0xff0000), 0);
        lv_obj_set_style_bg_opa(lv_obj_1, (255 * 100 / 100), 0);
        lv_obj_set_style_border_width(lv_obj_1, 0, 0);

        lv_obj_t * lv_button_0 = lv_button_create(lv_obj_0);
        lv_obj_set_width(lv_button_0, 150);
        lv_obj_set_height(lv_button_0, 52);
        lv_obj_set_align(lv_button_0, LV_ALIGN_BOTTOM_MID);
        lv_obj_set_y(lv_button_0, -34);
        lv_obj_set_style_radius(lv_button_0, 26, 0);
        lv_obj_set_style_bg_color(lv_button_0, lv_color_hex(0x00a0d0), 0);
        lv_obj_set_style_bg_color(lv_button_0, lv_color_hex(0xff6000), LV_STATE_PRESSED);
        lv_obj_t * lv_label_2 = lv_label_create(lv_button_0);
        lv_label_set_text(lv_label_2, "Touch me");
        lv_obj_set_align(lv_label_2, LV_ALIGN_CENTER);
        lv_obj_set_style_text_color(lv_label_2, lv_color_hex(0xffffff), 0);

        the_root = lv_obj_0;
    }
    #endif

    LV_TRACE_OBJ_CREATE("finished");

    return the_root;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

