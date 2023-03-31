#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "../../lvgl.h"
#endif

#ifdef PIKASCRIPT

#include "pika_lvgl_img.h"
#include "pika_lvgl_cf_t.h"
#include "lvgl.h"

void pika_lvgl_cf_t___init__(PikaObj *self){
    obj_setInt(self, "RAW", LV_IMG_CF_RAW);
    obj_setInt(self, "RAW_ALPHA", LV_IMG_CF_RAW_ALPHA);
    obj_setInt(self, "RAW_CHROMA_KEYED", LV_IMG_CF_RAW_CHROMA_KEYED);
    obj_setInt(self, "TRUE_COLOR", LV_IMG_CF_TRUE_COLOR);
    obj_setInt(self, "TRUE_COLOR_ALPHA", LV_IMG_CF_TRUE_COLOR_ALPHA);
    obj_setInt(self, "TRUE_COLOR_CHROMA_KEYED", LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED);
    obj_setInt(self, "INDEXED_1BIT", LV_IMG_CF_INDEXED_1BIT);
    obj_setInt(self, "INDEXED_2BIT", LV_IMG_CF_INDEXED_2BIT);
    obj_setInt(self, "INDEXED_4BIT", LV_IMG_CF_INDEXED_4BIT);
    obj_setInt(self, "INDEXED_8BIT", LV_IMG_CF_INDEXED_8BIT);
    obj_setInt(self, "ALPHA_1BIT", LV_IMG_CF_ALPHA_1BIT);
    obj_setInt(self, "ALPHA_2BIT", LV_IMG_CF_ALPHA_2BIT);
    obj_setInt(self, "ALPHA_4BIT", LV_IMG_CF_ALPHA_4BIT);
    obj_setInt(self, "ALPHA_8BIT", LV_IMG_CF_ALPHA_8BIT);
    obj_setInt(self, "RGB888", LV_IMG_CF_RGB888);
    obj_setInt(self, "RGBA8888", LV_IMG_CF_RGBA8888);
    obj_setInt(self, "RGBX8888", LV_IMG_CF_RGBX8888);
    obj_setInt(self, "RGB565", LV_IMG_CF_RGB565);
    obj_setInt(self, "RGBA5658", LV_IMG_CF_RGBA5658);
    obj_setInt(self, "RGB565A8", LV_IMG_CF_RGB565A8);
}

#endif
