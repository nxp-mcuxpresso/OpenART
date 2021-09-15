/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <rtthread.h>
#include "rthw.h"
#include <rtdevice.h>
#include "drv_uart.h"
#include "board.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_dmamux.h" 


rt_err_t  s_uart_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

rt_err_t  s_uart_close(rt_device_t dev)
{
	return RT_EOK;
}

rt_err_t  s_uart_control(rt_device_t dev, int cmd, void *args)
{
	return RT_EOK;
}

rt_err_t  s_uart_init(rt_device_t dev)
{
	struct imxrt_uart *uart;
    lpuart_config_t config;
	
	RT_ASSERT(dev != RT_NULL);
	struct serial_configure *cfg = (struct serial_configure *)dev->user_data;
	RT_ASSERT(cfg != RT_NULL);
    
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = cfg->baud_rate;

    switch (cfg->data_bits)
    {
    case DATA_BITS_7:
        config.dataBitsCount = kLPUART_SevenDataBits;
        break;

    default:
        config.dataBitsCount = kLPUART_EightDataBits;
        break;
    }

    switch (cfg->stop_bits)
    {
    case STOP_BITS_2:
        config.stopBitCount = kLPUART_TwoStopBit;
        break;
    default:
        config.stopBitCount = kLPUART_OneStopBit;
        break;
    }

    switch (cfg->parity)
    {
    case PARITY_ODD:
        config.parityMode = kLPUART_ParityOdd;
        break;
    case PARITY_EVEN:
        config.parityMode = kLPUART_ParityEven;
        break;
    default:
        config.parityMode = kLPUART_ParityDisabled;
        break;
    }

    config.enableTx = true;
    config.enableRx = true;

    LPUART_Init(LPUART1, &config, GetUartSrcFreq());
	
	return RT_EOK;
}

rt_size_t s_uart_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t length)
{
	int ch;
    int size;
	rt_uint8_t *data = (rt_uint8_t *)buffer;
    RT_ASSERT(dev != RT_NULL);
    size = length;

    while (length)
    {
		if (LPUART_GetStatusFlags(LPUART1) & kLPUART_RxDataRegFullFlag)
		{
			ch = LPUART_ReadByte(LPUART1);
		}
        if (ch == -1) break;

        *data = ch;
        data ++; length --;

        if (ch == '\n') break;
    }

    return size - length;
}

rt_size_t s_uart_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t length)
{
	int size;
    RT_ASSERT(dev != RT_NULL);
	rt_uint8_t *data = (rt_uint8_t *)buffer;
    size = length;
    while (length)
    {
        if (*data == '\n' && (dev->open_flag & RT_DEVICE_FLAG_STREAM))
        {
			LPUART_WriteByte(LPUART1, '\r');
			while (!(LPUART_GetStatusFlags(LPUART1) & kLPUART_TxDataRegEmptyFlag));
        }
		LPUART_WriteByte(LPUART1, *data);
		while (!(LPUART_GetStatusFlags(LPUART1) & kLPUART_TxDataRegEmptyFlag)){};

        ++ data;
        -- length;
    }

    return size - length;
}
 
int rt_simple_uart_init(void)
{
    int i;
    rt_uint32_t flag;
    rt_err_t ret = RT_EOK;
	struct rt_device *device;
    static struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX;
	device = rt_device_create(RT_Device_Class_Char, 1);
	if (device == NULL)
		return RT_ERROR;
	
	device->init        = s_uart_init;
    device->open        = s_uart_open;
    device->close       = s_uart_close;
    device->read        = s_uart_read;
    device->write       = s_uart_write;
    device->control     = s_uart_control;
	device->user_data = &config;
	
	/* register a character device */
    ret = rt_device_register(device, "s_uart", flag);
	
    return ret;
}
/* board init routines will be called in board_init() function */
INIT_BOARD_EXPORT(rt_simple_uart_init);