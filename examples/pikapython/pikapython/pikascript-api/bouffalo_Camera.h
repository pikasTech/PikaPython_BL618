/*
 * [Warning!] This file is auto-generated by pika compiler.
 * Do not edit it manually.
 * The source code is *.pyi file.
 * More details: 
 * English Doc:
 * https://pikadoc.readthedocs.io/en/latest/PikaScript%20%E6%A8%A1%E5%9D%97%E6%A6%82%E8%BF%B0.html
 * Chinese Doc:
 * https://pikadoc.readthedocs.io/zh/latest/PikaScript%20%E6%A8%A1%E5%9D%97%E6%A6%82%E8%BF%B0.html
 */

#ifndef __bouffalo_Camera__H
#define __bouffalo_Camera__H
#include <stdio.h>
#include <stdlib.h>
#include "PikaObj.h"

PikaObj *New_bouffalo_Camera(Args *args);

void bouffalo_Camera___init__(PikaObj *self);
void bouffalo_Camera_demo(PikaObj *self);
int bouffalo_Camera_get_frame_count(PikaObj *self);
PikaObj* bouffalo_Camera_get_frame_info(PikaObj *self);
void bouffalo_Camera_pop_one_frame(PikaObj *self);
void bouffalo_Camera_start(PikaObj *self);
void bouffalo_Camera_stop(PikaObj *self);

#endif
