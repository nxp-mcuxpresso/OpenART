/*
 * File      : context.asm
 * This file is part of RT-Thread RTOS
 * Copyright (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2010-04-09     fify         the first version
 * 2010-04-19     fify         rewrite rt_hw_interrupt_disable/enable fuction
 * 2010-04-20     fify         move peripheral ISR to bsp/interrupts.s34 
 *
 * For       : Renesas M16C
 * Toolchain : IAR's EW for M16C v3.401
 */
#ifndef __CPU_CONFIG_H__
#define __CPU_CONFIG_H__

#define MAX_SYSCALL_INTERRUPT_PRIORITY  10
#define KERNEL_INTERRUPT_PRIORITY       1

#endif
