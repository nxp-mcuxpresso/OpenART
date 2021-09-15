/*
 * File      : context_gcc.S
 * This file is part of RT-Thread RTOS
 * Copyright (C) 2006-2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-02-14     aozima       first implementation for Nios II.
 * 2011-02-20     aozima       fix context&switch bug.
 */

.set noat

.globl .Lexception_exit

.section .exceptions.exit.label
.Lexception_exit:
.section .exceptions.exit, "xa"
        ldw r5, 68(sp)

        /* get exception back */
        ldw ea, 72(sp)

        /* if(rt_thread_switch_interrupt_flag == 0) goto no_need_context */
        ldw r4,%gprel(rt_thread_switch_interrupt_flag)(gp)
        beq r4,zero,no_need_context

need_context:
        movia ea, rt_hw_context_switch_interrupt_do
        /* disable interrupt */
        mov r5, zero

no_need_context:
        ldw ra,  0(sp)

        wrctl estatus, r5

        /*
         * Leave a gap in the stack frame at 4(sp) for the muldiv handler to
         * store zero into.
         */

        ldw   r1,   8(sp)
        ldw   r2,  12(sp)
        ldw   r3,  16(sp)
        ldw   r4,  20(sp)
        ldw   r5,  24(sp)
        ldw   r6,  28(sp)
        ldw   r7,  32(sp)
        ldw   r8,  36(sp)
        ldw   r9,  40(sp)
        ldw   r10, 44(sp)
        ldw   r11, 48(sp)
        ldw   r12, 52(sp)
        ldw   r13, 56(sp)
        ldw   r14, 60(sp)
        ldw   r15, 64(sp)

        addi  sp, sp, 76

        eret

