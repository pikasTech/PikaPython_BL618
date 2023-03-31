#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "../../lvgl.h"
#endif

#ifdef PIKASCRIPT

#include "pika_lvgl_canvas.h"

void pika_lvgl_canvas___init__(PikaObj* self, PikaObj* parent) {
    lv_obj_t* lv_parent = obj_getPtr(parent, "lv_obj");
    lv_obj_t* lv_obj = lv_canvas_create(lv_parent);
    obj_setPtr(self, "lv_obj", lv_obj);
}

void pika_lvgl_canvas_copy_buf(PikaObj* self,
                               uint8_t* to_copy,
                               int x,
                               int y,
                               int w,
                               int h) {
    lv_obj_t* lv_obj = obj_getPtr(self, "lv_obj");
    lv_canvas_copy_buf(lv_obj, to_copy, x, y, w, h);
}

void pika_lvgl_canvas_fill_bg(PikaObj* self, PikaObj* color, int opa) {
    lv_obj_t* lv_obj = obj_getPtr(self, "lv_obj");
    lv_color_t* lv_color = obj_getPtr(color, "lv_color");
    lv_canvas_fill_bg(lv_obj, *lv_color, opa);
}

PikaObj* pika_lvgl_canvas_get_img(PikaObj* self) {
    // lv_obj_t* lv_obj = obj_getPtr(self, "lv_obj");
    // lv_img_dsc_t* img = lv_canvas_get_img(lv_obj);
    // PikaObj* pika_img = new_Obj("PikaImg", NULL);
    // obj_setPtr(pika_img, "lv_img_dsc", img);
    // return pika_img;
    return NULL;
}

void pika_lvgl_canvas_get_px(PikaObj* self,
                             int x,
                             int y,
                             PikaObj* color,
                             int opa) {
    // lv_obj_t* lv_obj = obj_getPtr(self, "lv_obj");
    // lv_color_t lv_color = lv_canvas_get_px(lv_obj, x, y);
    // obj_setInt(color, "full", lv_color.full);
    // obj_setInt(color, "ch", lv_color.ch.alpha);
    // obj_setInt(color, "opa", opa);
}

void pika_lvgl_canvas_set_buffer(PikaObj* self,
                                 uint8_t* buf,
                                 int w,
                                 int h,
                                 int cf) {
    lv_obj_t* lv_obj = obj_getPtr(self, "lv_obj");
    lv_canvas_set_buffer(lv_obj, buf, w, h, cf);
}

void pika_lvgl_canvas_set_palette(PikaObj* self, int id, PikaObj* c) {
    lv_obj_t* lv_obj = obj_getPtr(self, "lv_obj");
    lv_color_t* lv_color = obj_getPtr(c, "lv_color");
    lv_canvas_set_palette(lv_obj, id, *lv_color);
}

void pika_lvgl_canvas_set_px(PikaObj* self,
                             int x,
                             int y,
                             PikaObj* color,
                             int opa) {
    lv_obj_t* lv_obj = obj_getPtr(self, "lv_obj");
    lv_color_t* lv_color = obj_getPtr(color, "lv_color");
    lv_canvas_set_px(lv_obj, x, y, *lv_color);
}

void pika_lvgl_canvas_transform(PikaObj* self,
                                PikaObj* img,
                                int angle,
                                int zoom,
                                int offset_x,
                                int offset_y,
                                int pivot_x,
                                int pivot_y,
                                PIKA_BOOL antialias) {
    lv_obj_t* lv_obj = obj_getPtr(self, "lv_obj");
    lv_img_dsc_t* lv_img = obj_getPtr(img, "lv_img_dsc");
    lv_canvas_transform(lv_obj, lv_img, angle, zoom, offset_x, offset_y,
                        pivot_x, pivot_y, antialias);
}

#endif
