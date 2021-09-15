/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "fsl_common.h"
#include "fsl_lpi2c.h"
#include "fsl_ft5406_rt.h"
#define LOG_TAG             "drv.touch"
#include <drv_log.h>
#include "touch.h"
#define DEVICE_NAME "touch"
static ft5406_rt_handle_t s_touchHandle;
typedef struct _ft5406_rt_touch_point
{
    uint8_t XH;
    uint8_t XL;
    uint8_t YH;
    uint8_t YL;
    uint8_t RESERVED[2];
} ft5406_rt_touch_point_t;

typedef struct _ft5406_rt_touch_data
{
    uint8_t GEST_ID;
    uint8_t TD_STATUS;
    ft5406_rt_touch_point_t TOUCH[FT5406_RT_MAX_TOUCHES];
} ft5406_rt_touch_data_t;


#define I2C_BASE   LPI2C1
#define BUS_NAME	"i2c1"
#define NO_TOUCH kStatus_Fail
#define TOUCHED kStatus_Success
#define PANEL_WIDTH     (480)
#define PANEL_HEIGHT    (272)

#define TOUCH_POINT_GET_EVENT(T) ((touch_event_t)((T).XH >> 6))
#define TOUCH_POINT_GET_ID(T) ((T).YH >> 4)
#define TOUCH_POINT_GET_X(T) ((((T).XH & 0x0f) << 8) | (T).XL)
#define TOUCH_POINT_GET_Y(T) ((((T).YH & 0x0f) << 8) | (T).YL)

int ft5406_write_reg(rt_uint8_t reg, rt_uint8_t data)
{
	struct rt_i2c_bus_device *i2c_bus;
	struct rt_i2c_msg msgs;
	rt_uint8_t buf[3];

	buf[0] = reg; //cmd
	buf[1] = data;

	msgs.addr = FT5406_RT_I2C_ADDRESS;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = 2;

	i2c_bus = rt_i2c_bus_device_find(BUS_NAME);
	
	if (rt_i2c_transfer(i2c_bus, &msgs, 1) != 0)
	{
		return 0;
	}
	else
	{
		LOG_E("[%s] error\r\n",__func__);
		return -1;
	}
}

int ft5406_read_reg(rt_uint8_t reg, rt_uint8_t *data, rt_uint8_t len)
{
	struct rt_i2c_bus_device *i2c_bus;
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3];
	
	buf[0] = reg;
	msgs[0].addr = FT5406_RT_I2C_ADDRESS;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 1;

	msgs[1].addr = FT5406_RT_I2C_ADDRESS;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = (uint8_t *)data;
    msgs[1].len = len;

	i2c_bus = rt_i2c_bus_device_find(BUS_NAME);
	
	if (rt_i2c_transfer(i2c_bus, msgs, 2) != 0)
	{
		return 0;
	}
	else
	{
		LOG_E("[%s] error\r\n",__func__);
		return -1;
	}
}


status_t FT5406_RT_Init(ft5406_rt_handle_t *handle, LPI2C_Type *base)
{
    lpi2c_master_transfer_t *xfer = &(handle->xfer);
    status_t status = kStatus_Success;
    uint8_t mode;

    assert(handle);
    assert(base);

    if (!handle || !base)
    {
        return kStatus_InvalidArgument;
    }

    handle->base = base;

    /* clear transfer structure and buffer */
    memset(xfer, 0, sizeof(*xfer));
    memset(handle->touch_buf, 0, FT5406_RT_TOUCH_DATA_LEN);

    /* set device mode to normal operation */
    mode                 = 0;
	ft5406_write_reg(0,mode);
//    xfer->slaveAddress   = FT5406_RT_I2C_ADDRESS;
//    xfer->direction      = kLPI2C_Write;
//    xfer->subaddress     = 0;
//    xfer->subaddressSize = 1;
//    xfer->data           = &mode;
//    xfer->dataSize       = 1;
//    xfer->flags          = kLPI2C_TransferDefaultFlag;

//    status = LPI2C_MasterTransferBlocking(handle->base, &handle->xfer);

    /* prepare transfer structure for reading touch data */
    xfer->slaveAddress   = FT5406_RT_I2C_ADDRESS;
    xfer->direction      = kLPI2C_Read;
    xfer->subaddress     = 1;
    xfer->subaddressSize = 1;
    xfer->data           = handle->touch_buf;
    xfer->dataSize       = FT5406_RT_TOUCH_DATA_LEN;
    xfer->flags          = kLPI2C_TransferDefaultFlag;

    return status;
}

status_t FT5406_RT_Denit(ft5406_rt_handle_t *handle)
{
    assert(handle);

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    handle->base = NULL;
    return kStatus_Success;
}

status_t FT5406_RT_ReadTouchData(ft5406_rt_handle_t *handle)
{
    assert(handle);

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }
	
	return ft5406_read_reg(1, handle->touch_buf, FT5406_RT_TOUCH_DATA_LEN);
	
}

status_t FT5406_RT_GetSingleTouch(ft5406_rt_handle_t *handle, touch_event_t *touch_event, int *touch_x, int *touch_y)
{
    status_t status;
    touch_event_t touch_event_local;

    status = FT5406_RT_ReadTouchData(handle);

    if (status == kStatus_Success)
    {
        ft5406_rt_touch_data_t *touch_data = (ft5406_rt_touch_data_t *)(void *)(handle->touch_buf);

        if (touch_event == NULL)
        {
            touch_event = &touch_event_local;
        }
        *touch_event = TOUCH_POINT_GET_EVENT(touch_data->TOUCH[0]);

        /* Update coordinates only if there is touch detected */
        if ((*touch_event == kTouch_Down) || (*touch_event == kTouch_Contact))
        {
            if (touch_x)
            {
                *touch_x = TOUCH_POINT_GET_X(touch_data->TOUCH[0]);
            }
            if (touch_y)
            {
                *touch_y = TOUCH_POINT_GET_Y(touch_data->TOUCH[0]);
            }
        }
    }

    return status;
}

status_t FT5406_RT_GetMultiTouch(ft5406_rt_handle_t *handle,
                                 int *touch_count,
                                 touch_point_t touch_array[FT5406_RT_MAX_TOUCHES])
{
    status_t status;

    status = FT5406_RT_ReadTouchData(handle);

    if (status == kStatus_Success)
    {
        ft5406_rt_touch_data_t *touch_data = (ft5406_rt_touch_data_t *)(void *)(handle->touch_buf);
        int i;

        /* Check for valid number of touches - otherwise ignore touch information */
        if (touch_data->TD_STATUS > FT5406_RT_MAX_TOUCHES)
        {
            touch_data->TD_STATUS = 0;
        }

        /* Decode number of touches */
        if (touch_count)
        {
            *touch_count = touch_data->TD_STATUS;
        }

        /* Decode valid touch points */
        for (i = 0; i < touch_data->TD_STATUS; i++)
        {
            touch_array[i].TOUCH_ID    = TOUCH_POINT_GET_ID(touch_data->TOUCH[i]);
            touch_array[i].TOUCH_EVENT = TOUCH_POINT_GET_EVENT(touch_data->TOUCH[i]);
            touch_array[i].TOUCH_X     = TOUCH_POINT_GET_X(touch_data->TOUCH[i]);
            touch_array[i].TOUCH_Y     = TOUCH_POINT_GET_Y(touch_data->TOUCH[i]);
        }

        /* Clear vacant elements of touch_array */
        for (; i < FT5406_RT_MAX_TOUCHES; i++)
        {
            touch_array[i].TOUCH_ID    = 0;
            touch_array[i].TOUCH_EVENT = kTouch_Reserved;
            touch_array[i].TOUCH_X     = 0;
            touch_array[i].TOUCH_Y     = 0;
        }
    }

    return status;
}

static int lvgl_w;
static int lvgl_h;

static rt_size_t ft5406_tp_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	touch_event_t touch_event;
	int x, y;
	FT5406_RT_GetSingleTouch(&s_touchHandle, &touch_event, &y, &x);
	if((touch_event == kTouch_Down) || (touch_event == kTouch_Contact) )
	{
#ifdef LCD_DISPLAY_ROTATE_180	
		((int *)buffer)[0] = lvgl_w - (lvgl_w * x / PANEL_WIDTH);
		((int *)buffer)[1] = lvgl_h - (lvgl_h * y / PANEL_HEIGHT);
#else		
		((int *)buffer)[0] = lvgl_w * x / PANEL_WIDTH;
		((int *)buffer)[1] = lvgl_h * y / PANEL_HEIGHT;
#endif		
		return 8;
	}
	
	return 0;
}

static rt_err_t ft5406_tp_control(rt_device_t dev, int cmd, void *args)
{
	switch(cmd)
	{
		case RT_TOUCH_CTRL_SET_X_RANGE:
			lvgl_w = ((int*)args)[0];
			
			break;
        case RT_TOUCH_CTRL_SET_Y_RANGE:
            lvgl_h = ((int*)args)[0];
            break;
	}
	return RT_EOK;
}

static struct rt_device device =
{
    .type    = RT_Device_Class_Touch,
    .read    = ft5406_tp_read,
    .control = ft5406_tp_control,
};


int ft5406_tp_init(void)
{
    rt_err_t ret = RT_EOK;
	
	FT5406_RT_Init(&s_touchHandle, I2C_BASE);
	
    ret = rt_device_register(&device, DEVICE_NAME, RT_DEVICE_FLAG_RDWR);

    if(ret != RT_EOK)
    {
        LOG_E("rt device register failed %d\n", ret);
        return ret;
    }

    rt_device_open(&device, RT_DEVICE_OFLAG_RDWR);

    return RT_EOK;
}

INIT_DEVICE_EXPORT(ft5406_tp_init);



