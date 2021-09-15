/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013-2016 Kwabena W. Agyeman <kwagyeman@openmv.io>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Interface for using extra frame buffer RAM as a stack.
 *
 */
#ifndef __FB_ALLOC_H__
#define __FB_ALLOC_H__
#include <stdint.h>
#define FB_ALLOC_NO_HINT 0
#define FB_ALLOC_PREFER_SPEED 1
#define FB_ALLOC_PREFER_SIZE 2
void fb_alloc_fail(void);
void fb_alloc_init0(void);
uint32_t fb_avail(void);
void fb_alloc_mark(void);
void fb_alloc_free_till_mark(void);
void *fb_alloc(uint32_t size, int hints);
void *fb_alloc0(uint32_t size, int hints);
void fb_alloc_mark_permanent(); // tag memory that should not be popped on exception
void fb_alloc_free_till_mark_past_mark_permanent(); // frees past marked permanent allocations
void *fb_alloc_all(uint32_t *size, int hints); // returns pointer and sets size
void *fb_alloc0_all(uint32_t *size, int hints); // returns pointer and sets size
void fb_free(void);
void fb_free_all(void);
#endif /* __FF_ALLOC_H__ */
