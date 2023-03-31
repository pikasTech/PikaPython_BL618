#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "../../lvgl.h"
#endif

#ifdef PIKASCRIPT

#include "pika_lvgl_canvas.h"

void pika_lvgl_canvas___init__(PikaObj *self, PikaObj* parent){

}

void pika_lvgl_canvas_copy_buf(PikaObj *self, uint8_t* to_copy, int x, int y, int w, int h){

}

void pika_lvgl_canvas_fill_bg(PikaObj *self, PikaObj* color, int opa){

}

PikaObj* pika_lvgl_canvas_get_img(PikaObj *self){

}

void pika_lvgl_canvas_get_px(PikaObj *self, int x, int y, PikaObj* color, int opa){

}

void pika_lvgl_canvas_set_buffer(PikaObj *self, uint8_t* buf, int w, int h, int cf){

}

void pika_lvgl_canvas_set_palette(PikaObj *self, int id, int c){

}

void pika_lvgl_canvas_set_px(PikaObj *self, int x, int y, PikaObj* color, int opa){

}

void pika_lvgl_canvas_transform(PikaObj *self, PikaObj* img, int angle, int zoom, int offset_x, int offset_y, int pivot_x, int pivot_y, PIKA_BOOL antialias){

}


#endif
