/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-03-15     Liuguang     the first version.
 * 2019-04-22     tyustli      add imxrt series support
 *
 */
 
#include <rtthread.h>

#ifdef BSP_USING_74HC595

#define LOG_TAG             "drv.74hc595"
#include <drv_log.h>

#include "drv_74hc595.h"
#include "fsl_snvs_hp.h"
#include <time.h>
#ifndef BSP_74HC595_NUM
#define BSP_74HC595_NUM	 1
#endif
#define GPIO_DATA_WIDTH 8
struct drv_hc595_io
{
	uint32_t shcp_io;
	uint32_t stcp_io;
	uint32_t data_gpio;
	uint8_t ds_array[BSP_74HC595_NUM];
};

struct drv_hc595_io drv_pins = {0};

void delay(uint16_t t)
{
	for (; t != 0; t --);
 
}

void HC595_Data_High()
{
	rt_pin_write(drv_pins.data_gpio,PIN_HIGH);
}

void HC595_Data_Low()
{
	rt_pin_write(drv_pins.data_gpio,PIN_LOW);
}

void HC595_SHCP_Low()
{
	rt_pin_write(drv_pins.shcp_io,PIN_LOW);
}

void HC595_SHCP_High()
{
	rt_pin_write(drv_pins.shcp_io,PIN_HIGH);
}

void HC595_STCP_Low()
{
	rt_pin_write(drv_pins.stcp_io,PIN_LOW);
}

void HC595_STCP_High()
{
	rt_pin_write(drv_pins.stcp_io,PIN_HIGH);
}

void HC595_Send_Byte(uint8_t byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++) 
	{
		if (byte & 0x80)        
			HC595_Data_High();    
		else                    
			HC595_Data_Low();
		

		HC595_SHCP_Low();   
		delay(10);           
		HC595_SHCP_High();  
		delay(10);
		
		byte <<= 1;		
	}
	

}

void HC595_CS(void) 
{
	HC595_STCP_Low(); 
	delay(10);         
	HC595_STCP_High();
	delay(10);
}


void drv_74hc595_send_bytes(uint8_t *data, uint8_t len)
{
	int i;
	for (i = len - 1; i >=0; i -- ) 
	{
		HC595_Send_Byte(data[i]);
	}
	
	HC595_CS();
}
 
void drv_74hc595_init_io(uint32_t shcp_io, uint32_t stcp_io, uint32_t data_gpio)
{
	drv_pins.data_gpio = data_gpio;
	drv_pins.shcp_io = shcp_io;
	drv_pins.stcp_io = stcp_io;
	
	
}

int drv_74hc595_init(void)
{
	//memset(drv_pins.ds_array,0x0,BSP_74HC595_NUM);
	rt_pin_mode(drv_pins.shcp_io, PIN_MODE_OUTPUT);
	rt_pin_mode(drv_pins.stcp_io, PIN_MODE_OUTPUT);
	rt_pin_mode(drv_pins.data_gpio, PIN_MODE_OUTPUT);
	rt_pin_write(drv_pins.stcp_io,PIN_HIGH);
	rt_pin_write(drv_pins.stcp_io,PIN_HIGH);
	rt_pin_write(drv_pins.data_gpio,PIN_HIGH);
	drv_74hc595_send_bytes(drv_pins.ds_array,BSP_74HC595_NUM); 
	return 0;
}
INIT_PREV_EXPORT(drv_74hc595_init);

int drv_hc595_gpio_write(uint8_t gpio, uint8_t value)
{
	if (gpio >= GPIO_DATA_WIDTH*BSP_74HC595_NUM)
		return -1;
	int idx = gpio/GPIO_DATA_WIDTH;
	uint8_t dat = 1 << (gpio%GPIO_DATA_WIDTH);
	if (value)
		drv_pins.ds_array[idx] |= dat;
	else 
		drv_pins.ds_array[idx] &= ~(dat);
	
	drv_74hc595_send_bytes(drv_pins.ds_array,BSP_74HC595_NUM); 
	return 0;
}

int drv_hc595_gpio_read(uint8_t gpio)
{
	if (gpio >= GPIO_DATA_WIDTH*BSP_74HC595_NUM)
		return -1;
	int idx = gpio/GPIO_DATA_WIDTH;
	uint8_t dat = 1 << (gpio%GPIO_DATA_WIDTH);
	
	if((drv_pins.ds_array[idx] & dat) == dat)
		return 1;
	else
		return 0;
}

void hc595_write(uint8_t argc, char **argv)
{
	if (argc != 3)
		rt_kprintf("args count error %d\r\n",argc);
	
	rt_kprintf("Expension IO %d -> %d\r\n",atoi(argv[1]),atoi(argv[2]));
	drv_hc595_gpio_write(atoi(argv[1]),atoi(argv[2]));
}

MSH_CMD_EXPORT(hc595_write, hc595 gpio write: hc595_write gpio value);
#endif
