/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*                                                                       */
/*   Portions COPYRIGHT 2014 STMicroelectronics                          */
/*   Portions Copyright (C) 2012, ChaN, all right reserved               */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/
#include "ff.h"

#if _USE_LFN != 0

#if   _CODE_PAGE == 932 /* Japanese Shift_JIS */
#include "cc932.c"
#elif _CODE_PAGE == 936 /* Simplified Chinese GBK */
#include "cc936.c"
#elif _CODE_PAGE == 949 /* Korean */
#include "cc949.c"
#elif _CODE_PAGE == 950 /* Traditional Chinese Big5 */
#include "cc950.c"
#else                   /* Single Byte Character-Set */
#include "ccsbcs.c"
#endif

#endif
