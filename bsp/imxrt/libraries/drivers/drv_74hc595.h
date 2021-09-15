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
#ifndef __DRV_74HC595_H__
#define __DRV_74HC595_H__
#include <rtdevice.h>

void drv_74hc595_send_bytes(uint8_t *data, uint8_t len);
void drv_74hc595_init_io(uint32_t shcp_io, uint32_t stcp_io, uint32_t data_gpio);
#endif