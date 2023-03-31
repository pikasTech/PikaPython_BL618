#include "bouffalo_Camera.h"

#include "bflb_i2c.h"
#include "bflb_cam.h"
#include "bflb_gpio.h"
#include "image_sensor.h"
#include "lvgl.h"
#include "board.h"

// Global variables
static struct bflb_device_s *i2c0;
static struct bflb_device_s *cam0;
static struct bflb_cam_config_s cam_config;
static struct image_sensor_config_s *sensor_config;

void init_cam(struct bflb_device_s *gpio);
void bouffalo_Camera___init__(PikaObj *self) {
    init_cam(bflb_device_get_by_name("gpio"));
}

void bouffalo_Camera_start(PikaObj *self) {
    bflb_cam_start(cam0);
}

void bouffalo_Camera_stop(PikaObj *self) {
    bflb_cam_stop(cam0);
}

int bouffalo_Camera_get_frame_count(PikaObj *self) {
    return (int)bflb_cam_get_frame_count(cam0);
}

PikaObj* bouffalo_Camera_get_frame_info(PikaObj *self) {
    uint8_t *pic;
    uint32_t pic_size;
    pic_size = bflb_cam_get_frame_info(cam0, &pic);
    return obj_newTuple(arg_newInt((int32_t)pic), arg_newInt(pic_size));
}

void bouffalo_Camera_pop_one_frame(PikaObj *self) {
    bflb_cam_pop_one_frame(cam0);
}

static lv_obj_t *canvas_cam;

static lv_obj_t *canvas_cam_create(lv_obj_t *parent);

void demo(void) {
    canvas_cam = canvas_cam_create(lv_scr_act());
}

void canvas_cam_update(void *pic_addr) {
    if (!canvas_cam) {
        return;
    }

    lv_obj_t *canvas = canvas_cam;
    lv_canvas_set_buffer(canvas, pic_addr, 320, 240, LV_IMG_CF_TRUE_COLOR);
}

static lv_obj_t *canvas_cam_create(lv_obj_t *parent) {
    lv_obj_t *canvas;
    canvas = lv_canvas_create(parent);
    lv_obj_set_size(canvas, 320, 240);
    lv_obj_align(canvas, LV_ALIGN_TOP_MID, 0, 0);

    return canvas;
}

void bouffalo_Camera_demo(PikaObj *self) {
    demo();
}
