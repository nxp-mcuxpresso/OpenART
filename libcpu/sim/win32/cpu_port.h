/*
************************************************************************************************************************
* File    : cpu_port.h
* By      : xyou
* Version : V1.00.00
************************************************************************************************************************
*/
/*
 * File      : cpuport.c
 * This file is part of RT-Thread RTOS
 * Copyright (C) 2009 - 2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 * /


#ifndef _CPU_PORT_H_
#define _CPU_PORT_H_


/*
*********************************************************************************************************
*                                             CPU INTERRUPT PRIORITY
*********************************************************************************************************
*/
#define CPU_INTERRUPT_YIELD         0x00
#define CPU_INTERRUPT_TICK          0x01



/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void TriggerSimulateInterrupt(rt_uint32_t IntIndex);

void WinThreadScheduler(void);
#endif /* _CPU_PORT_H_ */
