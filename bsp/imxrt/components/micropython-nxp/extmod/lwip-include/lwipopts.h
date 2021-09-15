/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2016 Damien P. George
 * Copyright (c) 2016 Paul Sokolovsky
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_EXTMOD_LWIP_INCLUDE_LWIPOPTS_H
#define MICROPY_INCLUDED_EXTMOD_LWIP_INCLUDE_LWIPOPTS_H

#include <py/mpconfig.h>
#include <py/misc.h>
#include <py/mphal.h>

// We're running without an OS for this port. We don't provide any services except light protection.
#define NO_SYS 1

#define SYS_LIGHTWEIGHT_PROT 1
#include <stdint.h>
typedef uint32_t sys_prot_t;

#define TCP_LISTEN_BACKLOG 1

// We'll put these into a proper ifdef once somebody implements an ethernet driver
#define LWIP_ARP 0
#define LWIP_ETHERNET 0

#define LWIP_DNS 1

#define LWIP_NETCONN 0
#define LWIP_SOCKET 0

#ifdef MICROPY_PY_LWIP_SLIP
#define LWIP_HAVE_SLIPIF 1
#endif

// For now, we can simply define this as a macro for the timer code. But this function isn't
// universal and other ports will need to do something else. It may be necessary to move
// things like this into a port-provided header file.
#define sys_now mp_hal_ticks_ms

#endif // MICROPY_INCLUDED_EXTMOD_LWIP_INCLUDE_LWIPOPTS_H
