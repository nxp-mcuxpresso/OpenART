/*
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_gt911.h"
#include <rtthread.h>
#include <rtdevice.h>
#define LOG_TAG             "drv.touch"
#include <drv_log.h>
#include "touch.h"
#define DEVICE_NAME "touch"
#define BUS_NAME "i2c5"
#define PANEL_WIDTH     (720)
#define PANEL_HEIGHT    (1280)

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief GT911 I2C address. */
#define GT911_I2C_ADDRESS0 (0x5D)
#define GT911_I2C_ADDRESS1 (0x14)

#define GT911_REG_ADDR_SIZE 2

/*! @brief GT911 registers. */
#define GT911_REG_ID             0x8140U
#define GT911_CONFIG_ADDR        0x8047U
#define GT911_REG_XL             0x8048U
#define GT911_REG_XH             0x8049U
#define GT911_REG_YL             0x804AU
#define GT911_REG_YH             0x804BU
#define GT911_REG_TOUCH_NUM      0x804CU
#define GT911_REG_CONFIG_VERSION 0x8047U
#define GT911_REG_MODULE_SWITCH1 0x804DU
#define GT911_REG_STAT           0x814EU
#define GT911_REG_FIRST_POINT    0x814FU

#define GT911_STAT_BUF_MASK          (1U << 7U)
#define GT911_STAT_POINT_NUMBER_MASK (0xFU << 0U)
#define GT911_MODULE_SWITCH_X2Y_MASK (1U << 3U)
#define GT911_MODULE_SWITCH_INT_MASK (3U << 0U)

#define GT911_CONFIG_SIZE (186U)

#define PANEL_TOUCH_RST_GPIO          GPIO9
#define PANEL_TOUCH_RST_PIN           0
#define PANEL_TOUCH_INT_GPIO          GPIO8
#define PANEL_TOUCH_INT_PIN           31

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Verify firmware, return true if pass. */
static bool GT911_VerifyFirmware(const uint8_t *firmware);
static uint8_t GT911_GetFirmwareCheckSum(const uint8_t *firmware);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static gt911_handle_t s_touchHandle;
static int s_touchResolutionX;
static int s_touchResolutionY;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(PANEL_TOUCH_RST_GPIO, PANEL_TOUCH_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(PANEL_TOUCH_RST_GPIO, PANEL_TOUCH_RST_PIN, 0);
    }
}

static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode)
{
    if (mode == kGT911_IntPinInput)
    {
        PANEL_TOUCH_INT_GPIO->GDIR &= ~(1UL << PANEL_TOUCH_INT_PIN);
    }
    else
    {
        if (mode == kGT911_IntPinPullDown)
        {
            GPIO_PinWrite(PANEL_TOUCH_INT_GPIO, PANEL_TOUCH_INT_PIN, 0);
        }
        else
        {
            GPIO_PinWrite(PANEL_TOUCH_INT_GPIO, PANEL_TOUCH_INT_PIN, 1);
        }

        PANEL_TOUCH_INT_GPIO->GDIR |= (1UL << PANEL_TOUCH_INT_PIN);
    }
}

static uint8_t GT911_GetFirmwareCheckSum(const uint8_t *firmware)
{
    uint8_t sum = 0;
    uint16_t i  = 0;

    for (i = 0; i < GT911_CONFIG_SIZE - 2U; i++)
    {
        sum += (*firmware);
        firmware++;
    }

    return (~sum + 1U);
}

static bool GT911_VerifyFirmware(const uint8_t *firmware)
{
    return ((firmware[GT911_REG_CONFIG_VERSION - GT911_CONFIG_ADDR] != 0U) &&
            (GT911_GetFirmwareCheckSum(firmware) == firmware[GT911_CONFIG_SIZE - 2U]));
}

status_t GT911_Init(gt911_handle_t *handle, const gt911_config_t *config)
{
    status_t status;
    uint32_t deviceID;
    uint8_t gt911Config[GT911_CONFIG_SIZE];

    assert(NULL != handle);

    (void)memset(handle, 0, sizeof(*handle));

    handle->I2C_SendFunc     = config->I2C_SendFunc;
    handle->I2C_ReceiveFunc  = config->I2C_ReceiveFunc;
    handle->timeDelayMsFunc  = config->timeDelayMsFunc;
    handle->pullResetPinFunc = config->pullResetPinFunc;

    /* Reset the panel and set the I2C address mode. */
    config->intPinFunc(kGT911_IntPinPullDown);
    config->pullResetPinFunc(false);

    /* >= 10ms. */
    handle->timeDelayMsFunc(20);

    if (kGT911_I2cAddrAny == config->i2cAddrMode)
    {
        config->pullResetPinFunc(true);

        /* >= 55ms */
        handle->timeDelayMsFunc(55);

        config->intPinFunc(kGT911_IntPinInput);

        /* Try address 0 */
        handle->i2cAddr = GT911_I2C_ADDRESS0;
        status = handle->I2C_ReceiveFunc(handle->i2cAddr, GT911_REG_ID, GT911_REG_ADDR_SIZE, (uint8_t *)&deviceID, 4);

        if (kStatus_Success != status)
        {
            /* Try address 1 */
            handle->i2cAddr = GT911_I2C_ADDRESS1;
            status =
                handle->I2C_ReceiveFunc(handle->i2cAddr, GT911_REG_ID, GT911_REG_ADDR_SIZE, (uint8_t *)&deviceID, 4);

            if (kStatus_Success != status)
            {
                return status;
            }
        }
    }
    else
    {
        if (kGT911_I2cAddrMode1 == config->i2cAddrMode)
        {
            config->intPinFunc(kGT911_IntPinPullUp);
            handle->i2cAddr = GT911_I2C_ADDRESS1;
        }
        else
        {
            handle->i2cAddr = GT911_I2C_ADDRESS0;
        }

        /* >= 100us */
        handle->timeDelayMsFunc(1);

        config->pullResetPinFunc(true);

        /* >= 5ms */
        handle->timeDelayMsFunc(5);

        config->intPinFunc(kGT911_IntPinPullDown);

        /* >= 50ms */
        handle->timeDelayMsFunc(50);

        config->intPinFunc(kGT911_IntPinInput);

        status = handle->I2C_ReceiveFunc(handle->i2cAddr, GT911_REG_ID, GT911_REG_ADDR_SIZE, (uint8_t *)&deviceID, 4);
        if (kStatus_Success != status)
        {
            return status;
        }
    }

    /* Verify the device. */
    if (deviceID != 0x00313139U)
    {
        return kStatus_Fail;
    }

    /* Initialize the IC. */
    status = handle->I2C_ReceiveFunc(handle->i2cAddr, GT911_CONFIG_ADDR, GT911_REG_ADDR_SIZE, gt911Config,
                                     GT911_CONFIG_SIZE);
    if (kStatus_Success != status)
    {
        return status;
    }

    /*
     * GT911 driver gets the original firmware from touch panel control IC, modify
     * the configuration, then set it to the IC again. The original firmware
     * read from the touch IC must be correct, otherwise setting wrong firmware
     * to the touch IC will break it.
     */
    if (true != GT911_VerifyFirmware(gt911Config))
    {
        return kStatus_Fail;
    }

    handle->resolutionX = ((uint16_t)gt911Config[GT911_REG_XH - GT911_CONFIG_ADDR]) << 8U;
    handle->resolutionX += gt911Config[GT911_REG_XL - GT911_CONFIG_ADDR];
    handle->resolutionY = ((uint16_t)gt911Config[GT911_REG_YH - GT911_CONFIG_ADDR]) << 8U;
    handle->resolutionY += gt911Config[GT911_REG_YL - GT911_CONFIG_ADDR];

    gt911Config[GT911_REG_TOUCH_NUM - GT911_CONFIG_ADDR] = (config->touchPointNum) & 0x0FU;

    gt911Config[GT911_REG_MODULE_SWITCH1 - GT911_CONFIG_ADDR] &= (uint8_t)(~GT911_MODULE_SWITCH_INT_MASK);
    gt911Config[GT911_REG_MODULE_SWITCH1 - GT911_CONFIG_ADDR] |= (uint8_t)(config->intTrigMode);

    gt911Config[GT911_CONFIG_SIZE - 2U] = GT911_GetFirmwareCheckSum(gt911Config);
    gt911Config[GT911_CONFIG_SIZE - 1U] = 1U; /* Mark the firmware as valid. */

    return handle->I2C_SendFunc(handle->i2cAddr, GT911_CONFIG_ADDR, GT911_REG_ADDR_SIZE, gt911Config,
                                GT911_CONFIG_SIZE);
}

status_t GT911_Deinit(gt911_handle_t *handle)
{
    handle->pullResetPinFunc(false);
    return kStatus_Success;
}

static status_t GT911_ReadRawTouchData(gt911_handle_t *handle, uint8_t *touchPointNum)
{
    status_t status;
    uint8_t gt911Stat;

    status = handle->I2C_ReceiveFunc(handle->i2cAddr, GT911_REG_STAT, GT911_REG_ADDR_SIZE, &gt911Stat, 1);
    if (kStatus_Success != status)
    {
        *touchPointNum = 0;
        return status;
    }

    *touchPointNum = gt911Stat & GT911_STAT_POINT_NUMBER_MASK;

    if (0U != (gt911Stat & GT911_STAT_BUF_MASK))
    {
        if (*touchPointNum > 0U)
        {
            status = handle->I2C_ReceiveFunc(handle->i2cAddr, GT911_REG_FIRST_POINT, GT911_REG_ADDR_SIZE,
                                             (void *)handle->pointReg, (*touchPointNum) * sizeof(gt911_point_reg_t));
        }

        /* Must set the status register to 0 after read. */
        gt911Stat = 0;
        status = handle->I2C_SendFunc(handle->i2cAddr, GT911_REG_STAT, GT911_REG_ADDR_SIZE, &gt911Stat, 1);
    }

    return status;
}

status_t GT911_GetSingleTouch(gt911_handle_t *handle, int *touch_x, int *touch_y)
{
    status_t status;
    uint8_t touchPointNum;

    status = GT911_ReadRawTouchData(handle, &touchPointNum);

    if (kStatus_Success == status)
    {
        if (touchPointNum > 0U)
        {
            *touch_x = (int)(uint16_t)((uint16_t)handle->pointReg[0].lowX + (((uint16_t)handle->pointReg[0].highX) << 8U));
            *touch_y = (int)(uint16_t)((uint16_t)handle->pointReg[0].lowY + (((uint16_t)handle->pointReg[0].highY) << 8U));
        }
        else
        {
            status = (status_t)kStatus_TOUCHPANEL_NotTouched;
        }
    }
    else
    {
        status = kStatus_Fail;
    }

    return status;
}

status_t GT911_GetMultiTouch(gt911_handle_t *handle, uint8_t *touch_count, touch_point_t touch_array[])
{
    status_t status;
    uint32_t i;
    uint8_t desiredTouchPointNum;
    uint8_t actualTouchPointNum;

    status = GT911_ReadRawTouchData(handle, &actualTouchPointNum);

    if (kStatus_Success == status)
    {
        desiredTouchPointNum = *touch_count;

        if (0U == actualTouchPointNum)
        {
            status = (status_t)kStatus_TOUCHPANEL_NotTouched;
        }
        else if (actualTouchPointNum > desiredTouchPointNum)
        {
            actualTouchPointNum = desiredTouchPointNum;
        }
        else
        {
            /* MISRA compatible. */
        }

        for (i = 0; i < actualTouchPointNum; i++)
        {
            touch_array[i].valid   = true;
            touch_array[i].touchID = handle->pointReg[i].id;
            touch_array[i].x       = handle->pointReg[i].lowX + (((uint16_t)handle->pointReg[i].highX) << 8U);
            touch_array[i].y       = handle->pointReg[i].lowY + (((uint16_t)handle->pointReg[i].highY) << 8U);
        }

        for (; i < desiredTouchPointNum; i++)
        {
            touch_array[i].valid = false;
        }
    }
    else
    {
        status = kStatus_Fail;
    }

    *touch_count = actualTouchPointNum;

    return status;
}

status_t GT911_GetResolution(gt911_handle_t *handle, int *resolutionX, int *resolutionY)
{
    *resolutionX = (int)handle->resolutionX;
    *resolutionY = (int)handle->resolutionY;

    return kStatus_Success;
}




int gt911_write_reg(rt_uint8_t dev_addr, rt_uint16_t reg, rt_uint8_t *data, rt_uint8_t size)
{
	struct rt_i2c_bus_device *i2c_bus;
	struct rt_i2c_msg msgs;
	rt_uint8_t *buf = rt_malloc(size + 2);

	buf[0] = reg >> 8; //cmd
	buf[1] = reg & 0xff;
	rt_memcpy(&buf[2], data, size);

	msgs.addr = dev_addr;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = size + 2;

	i2c_bus = rt_i2c_bus_device_find(BUS_NAME);
	
	if (rt_i2c_transfer(i2c_bus, &msgs, 1) != 0)
	{
		rt_free(buf);
		return 0;
	}
	else
	{
		rt_free(buf);
		LOG_E("[%s] error\r\n",__func__);
		return -1;
	}
}

int gt911_read_reg(rt_uint8_t dev_addr, rt_uint16_t reg, rt_uint8_t *data, rt_uint8_t len)
{
	struct rt_i2c_bus_device *i2c_bus;
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3];
	
	buf[0] = reg >> 8; //cmd
	buf[1] = reg & 0xff;
	msgs[0].addr = dev_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 2;

	msgs[1].addr = dev_addr;
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

status_t PanelTouch_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize)
{
	return gt911_write_reg(deviceAddress,subAddress,(uint8_t *)txBuff,txBuffSize);
}

status_t PanelTouch_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
	return gt911_read_reg(deviceAddress,subAddress,(uint8_t *)rxBuff,rxBuffSize);
}

static int lvgl_w;
static int lvgl_h;

static rt_size_t gt911_tp_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	int x, y;
	
	if (GT911_GetSingleTouch(&s_touchHandle, &x, &y) != kStatus_TOUCHPANEL_NotTouched) {
        ((int *)buffer)[0] = lvgl_w * y / PANEL_HEIGHT;
        ((int *)buffer)[1] = (lvgl_h - lvgl_h * x / PANEL_WIDTH);
        
        return 8;
    } else {
        return 0;
    }

}

static rt_err_t gt911_tp_control(rt_device_t dev, int cmd, void *args)
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
    .read    = gt911_tp_read,
    .control = gt911_tp_control,
};

extern void VIDEO_DelayMs(uint32_t ms);
static const gt911_config_t s_touchConfig = {
    .I2C_SendFunc     = PanelTouch_I2C_Send,
    .I2C_ReceiveFunc  = PanelTouch_I2C_Receive,
    .pullResetPinFunc = BOARD_PullMIPIPanelTouchResetPin,
    .intPinFunc       = BOARD_ConfigMIPIPanelTouchIntPin,
    .timeDelayMsFunc  = VIDEO_DelayMs,
    .touchPointNum    = 1,
    .i2cAddrMode      = kGT911_I2cAddrMode0,
    .intTrigMode      = kGT911_IntRisingEdge,
};

int gt911_tp_init(void)
{
    rt_err_t ret = RT_EOK;
	
	 const gpio_pin_config_t resetPinConfig = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0, .interruptMode = kGPIO_NoIntmode};
    GPIO_PinInit(GPIO8, 31, &resetPinConfig);
    GPIO_PinInit(GPIO9, 0, &resetPinConfig);
	int status = GT911_Init(&s_touchHandle, &s_touchConfig);

    if (kStatus_Success != status)
    {
        assert(false);
    }
	
    ret = rt_device_register(&device, DEVICE_NAME, RT_DEVICE_FLAG_RDWR);

    if(ret != RT_EOK)
    {
        LOG_E("rt device register failed %d\n", ret);
        return ret;
    }

    rt_device_open(&device, RT_DEVICE_OFLAG_RDWR);

    return RT_EOK;
}

INIT_DEVICE_EXPORT(gt911_tp_init);
