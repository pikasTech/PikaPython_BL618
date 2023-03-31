#include "bouffalo_Camera.h"

#include "bflb_i2c.h"
#include "bflb_cam.h"
#include "image_sensor.h"
#include "board.h"

// Global variables
static struct bflb_device_s *i2c0;
static struct bflb_device_s *cam0;
static struct bflb_cam_config_s cam_config;
static struct image_sensor_config_s *sensor_config;

void bouffalo_Camera___init__(PikaObj *self) {
    board_dvp_gpio_init();

    i2c0 = bflb_device_get_by_name("i2c0");
    cam0 = bflb_device_get_by_name("cam0");

    if (image_sensor_scan(i2c0, &sensor_config)) {
        pika_platform_printf("\r\nSensor name: %s\r\n", sensor_config->name);
    } else {
        pika_platform_printf("\r\nError! Can't identify sensor!\r\n");
        while (1) {
        }
    }

    memcpy(&cam_config, sensor_config, IMAGE_SENSOR_INFO_COPY_SIZE);
    cam_config.with_mjpeg = false;
    cam_config.input_source = CAM_INPUT_SOURCE_DVP;
    cam_config.output_format = CAM_OUTPUT_FORMAT_AUTO;
    cam_config.output_bufaddr = BFLB_PSRAM_BASE;
    cam_config.output_bufsize = cam_config.resolution_x * cam_config.resolution_y * 8;

    bflb_cam_init(cam0, &cam_config);
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
