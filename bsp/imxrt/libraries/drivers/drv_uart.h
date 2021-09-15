/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-10-10     Tanek        the first version
 */

#ifndef DRV_UART_H__
#define DRV_UART_H__

#define RT_DEVICE_UART_ANY 0x0100
#define RT_DEVICE_UART_SBK 0x0101 
int rt_hw_uart_init(void);

#endif
