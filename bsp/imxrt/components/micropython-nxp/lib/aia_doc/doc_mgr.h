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
#ifndef _CFG_MUX_MGR_H_
#define _CFG_MUX_MGR_H_
#include "py/mpconfig.h"
#include "py/misc.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "py/objstr.h"
//#include "pin.h"
#include "pin_defs_mcu.h"
typedef struct _CMM_DescItem 
{
    const char *pszPin;
	const char *pszHint; /* point to a location inside the szDesc */
	pin_obj_t *pPinObj;	
    mp_obj_tuple_t *pTuple;
    mp_obj_t objOldOwner;
}CMM_DescItem_t, MuxItem_t;

/* Return a pin to CMM, caller should put the pin to proper state according to the board design */
int Mux_Give(MuxItem_t *pMuxData);

/* Take a pin according to the function name (pszFn), unit (unit), and the signal name (pszSignal), 
 * if the pin is owned by others then preempt it. usually one should avoid this confliction.
 * (userObj) will become the new owner of this pin.
 * E.g., Mux_Take(self, "spi", 3, "SCLK", &muxData); 
 */
int Mux_Take(mp_obj_t userObj, const char *pszFn, int unit, const char *pszSignal, MuxItem_t *pMuxData);

/* Like Mux_Take, but if the pin is owned by others then fail and returns a negative value */
int Mux_TryTake(mp_obj_t userObj, const char *pszFn, int unit, const char *pszSignal, MuxItem_t *pMuxData);

/* Just query the pin, do not take */
int Mux_Query(mp_obj_t userObj, const char *pszFn, int unit, const char *pszSignal, MuxItem_t *pMuxData);

/* below APIs are just for complementary, convenient for getting multiple signals together */

/* Return a group of pins */
int Mux_GiveMany(MuxItem_t *pMuxData, int cnt);

/* take a group of pins, the amount is signalCnt */
int Mux_TakeMany(mp_obj_t userObj, const char *pszFn, int unit, const char **ppszSignals, int signalCnt, MuxItem_t *pMuxData);

/* Try take a group of pins, the amount is signalCnt */
int Mux_TryTakeMany(mp_obj_t userObj, const char *pszFn, int unit, const char **ppszSignals, int signalCnt, MuxItem_t *pMuxData);

/* Query a group of pins*/
int Mux_QueryMany(mp_obj_t userObj, const char *pszFn, int unit, const char **ppszSignals, int signalCnt, MuxItem_t *pMuxData);
#endif
