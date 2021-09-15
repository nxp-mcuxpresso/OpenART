/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       		Notes
 * 2020-04-13     Tony.Zhang(NXP)	
 *
 */
 
#ifndef __DRV_CAMERA_H__
#define __DRV_CAMERA_H__

#include <rtthread.h>
#include "imlib.h"
#ifdef __cplusplus
extern "C" {
#endif

struct rt_camera_device;

struct rt_camera_device_ops
{
	rt_size_t (*get_frame_jpeg)(struct rt_camera_device *cam,
                             void *frame_ptr);
    rt_size_t (*get_frame)(struct rt_camera_device *cam,
                            image_t * image);
    rt_err_t (*camera_control)(struct rt_camera_device *cam, rt_uint32_t, rt_uint32_t);
};

struct rt_camera_device
{
    struct rt_device parent;
    const struct rt_camera_device_ops *ops;
  
    rt_sem_t sem;
    rt_uint32_t  status;
	rt_uint32_t* imx_cam;
	rt_thread_t cam_tid;
	rt_thread_t omv_tid;
};

#define SENSOR_NAME "camera0"

#define RT_DRV_CAM_CMD_RESET				0x0001
#define RT_DRV_CAM_CMD_SET_CONTRAST			0x0002
#define RT_DRV_CAM_CMD_SET_GAINCEILING		0x0003
#define RT_DRV_CAM_CMD_SET_FRAMESIZE		0x0004
#define RT_DRV_CAM_CMD_SET_PIXFORMAT		0x0005
#define RT_DRV_CAM_CMD_SNAPSHOT				0x0006
#define RT_DRV_CAM_CMD_SLEEP				0x0007
#define RT_DRV_CAM_CMD_SETBRIGHTNESS		0x0008
#define RT_DRV_CAM_CMD_SETSATURATION		0x0009
#define RT_DRV_CAM_CMD_SETQUALITY			0x000a
#define RT_DRV_CAM_CMD_SETCOLORBAR			0x000b
#define RT_DRV_CAM_CMD_SETAUTOGAIN			0x000C
#define RT_DRV_CAM_CMD_SETHMIRROR			0x000d
#define RT_DRV_CAM_CMD_SET_VFLIP			0x000e
#define RT_DRV_CAM_CMD_SETSPECIALEFFECT		0x000f
#define RT_DRV_CAM_CMD_SETCOLORPALETTE		0x0010
#define RT_DRV_CAM_CMD_SET_FRAMERATE		0x0011
#define RT_DRV_CAM_CMD_SHUTDOWN				0x0012

struct rt_camera_device * imxrt_camera_device_find(char *name);
int imxrt_camera_width(struct rt_camera_device *sensor);
int imxrt_camera_height(struct rt_camera_device *sensor);
int imxrt_camera_chip_id(struct rt_camera_device *sensor);
int imxrt_camera_pixformat(struct rt_camera_device *sensor);
int imxrt_camera_framesize(struct rt_camera_device *sensor);
int imxrt_camera_set_windowing(struct rt_camera_device *sensor, int x,int y, int w,int h);
int imxrt_camera_set_auto_gain(struct rt_camera_device *sensor,int enable, float gain_db, float gain_db_ceiling);
int imxrt_camera_sensor_get_gain_db(struct rt_camera_device *sensor,float *gain_db);
int imxrt_camera_set_auto_exposure(struct rt_camera_device *,int enable, int exposure_us);
int imxrt_camera_get_exposure_us(struct rt_camera_device *sensor, int *us);
int imxrt_camera_set_auto_whitebal(struct rt_camera_device *sensor,int enable, float r_gain_db, float g_gain_db, float b_gain_db);
int imxrt_camera_get_rgb_gain_db(struct rt_camera_device *sensor,float *r_gain_db, float *g_gain_db, float *b_gain_db);
int imxrt_camera_set_lens_correction(struct rt_camera_device *sensor,int enable, int radi, int coef);
uint16_t * imxrt_camera_get_color_palette(struct rt_camera_device *sensor);
int imxrt_camera_write_reg(struct rt_camera_device *sensor,uint16_t reg_addr, uint16_t reg_data);
int imxrt_camera_read_reg(struct rt_camera_device *sensor,uint16_t reg_addr);
int imxrt_camera_ioctl(struct rt_camera_device *sensor,int request, ... /* arg */);
#endif
