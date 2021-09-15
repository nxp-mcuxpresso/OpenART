#include <stdlib.h>
#include <string.h>
#include "py/obj.h"
#include "py/objint.h"
#include "py/objstr.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "py/objarray.h"

// py_image
#include "framebuffer.h"
typedef struct _py_image_obj_t {
    mp_obj_base_t base;
    image_t _cobj;
} py_image_obj_t;

// mp lvgl 
// all the module is from this lv_stuct
typedef struct mp_lv_struct_t
{
    mp_obj_base_t base;
    void *data;
} mp_lv_struct_t;

/*
 * lvgl includes
 */

#include "../../lv_binding_micropython/lvgl/lvgl.h"
#define COLOR_DEPTH  (LV_COLOR_DEPTH / 8)

#ifdef SOC_IMXRT1170_SERIES
extern void Update_FrameBuffer(void* buf, uint32_t w, uint32_t h, uint32_t bpp, void* handler);
#endif
extern bool DEMO_ReadTouch(lv_indev_drv_t *drv, lv_indev_data_t *data, uint32_t lvgl_w, uint32_t lvgl_h);

typedef struct {
	uint32_t w,h;
}screen_size;
static screen_size resolution;

STATIC mp_obj_t mp_get_ptr_from_img(mp_obj_t self){
	py_image_obj_t *o = (py_image_obj_t*)self;
    image_t *image = &(o->_cobj);
	uint32_t w = image->w, h = image->h, bpp = image->bpp;
	uint32_t len = w * h * bpp;
	void* items = image->pixels; 
	return mp_obj_new_bytearray_by_ref(len, items);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_get_ptr_from_img_obj, mp_get_ptr_from_img);

/*
 * r5 << 11 = gray / 2 ^ 3 * 2 ^ 11 = gray * 2 ^ 8
 * g6 << 5  = gray / 2 ^ 2 * 2 ^ 5  = gray * 2 ^ 3 
 * b5 << 0  = gray / 2 ^ 3 * 2 ^ 0  = gray / 2 ^ 3
 * rgb565 = r5 << 11 | g6 << 5 | b5 << 0 
 * 		  = gray * (2^8 + 2^3 + 1 / 2^3) 
 * 		  = gray * 264 + gray << 3 or 
 *		  = (gray / 2 ^ 3 ) * ( 2 ^ 11 + 2 ^ 6 + 1) = (gray << 3) * (2113)
 */
__attribute__((naked)) __attribute__((section(".ram_code"))) void gray2rgb565(uint16_t* rgb, uint8_t* gray, uint32_t gray_len){
	__asm volatile(
		"	push {r3 - r11, lr} \n"
		"   ldr r11, =2113 \n" // scale
		"loop: \n"
		"	mov r7, #0 \n"
		"	mov r8, #0 \n"
		"	mov r9, #0 \n"
		"	mov r10, #0 \n"
	
		"	ldrd r3, r4, [r1], #8 \n" // load 8B each loop
		"	nop \n"
	
		"	ubfx r5, r3, #3, #5 \n"  // pixel 1, we only extract the high 5bits
		"	ubfx r6, r4, #3, #5 \n" // pixel 5
		
		"	bfi r7, r5, #0, #5 \n"
		"	bfi r9, r6, #0, #5 \n"		
	
		"	ubfx r5, r3, #11, #5 \n"  // pixel 2
		"	ubfx r6, r4, #11, #5 \n" // pixel 6
	
		"	bfi r7, r5, #16, #5 \n"
		"	bfi r9, r6, #16, #5 \n"	
		
		"	ubfx r5, r3, #19, #5 \n"  // pixel 3
		"	ubfx r6, r4, #19, #5 \n" // pixel 7
	
		"	bfi r8, r5, #0, #5 \n"
		"	bfi r10, r6, #0, #5 \n"			

		"	ubfx r5, r3, #27, #5 \n"  // pixel 4
		"	ubfx r6, r4, #27, #5 \n" // pixel 8
	
		"	bfi r8, r5, #16, #5 \n"  
		"	bfi r10, r6, #16, #5 \n"	
		
		"	mul r7, r11 \n"   // gray * 264
		"	mul r8, r11 \n"
		"	mul r9, r11 \n"
		"	mul r10, r11 \n"
		
		"	rev16 r7, r7 \n"
		"	rev16 r8, r8 \n"
		"	rev16 r9, r9 \n"
		"	rev16 r10, r10 \n"		
		
		"	stmia r0!, {r7-r10} \n"
		"	subs r2, #8 \n"
		"	bne loop \n"
		"	pop {r3 - r11, pc} \n"
		);
}

__attribute__((section(".ram_code"))) void gray2rgb565_C(uint16_t* rgb, uint8_t* gray, uint32_t gray_len){
	for(int i=0;i<gray_len;i++){
		uint8_t color = *gray++;
		uint16_t color_16 = ((color >> 3) << 11) | ((color >> 2) << 5) | ((color >> 3));
		uint16_t color_16_2 = (color >> 3) * (1<<11 | 1<<6 | 1);
		*rgb++ = ((color_16 << 8) | (color_16 >> 8));
		color_16_2 = 100;
	}
}


typedef struct {
	union {
		void* buf;
		uint8_t* d8;
		uint16_t* d16;
	};
	uint32_t len;
}mem_ctl;

mem_ctl rgb_mem;
#define alloc_mem(len) \
	/*if(rgb_mem.buf) xfree(rgb_mem.buf);*/ \
	rgb_mem.buf = fb_alloc(len, FB_ALLOC_NO_HINT); \
	rgb_mem.len = len; 

STATIC mp_obj_t mp_grayscale_to_rgb(mp_obj_t self){
	py_image_obj_t *o = (py_image_obj_t*)self;
	py_image_obj_t *rgb565 = m_new_obj(py_image_obj_t);
	image_t *image = &(o->_cobj);
	uint32_t w = image->w, h = image->h;
	
	rgb565->base.type = o->base.type;
	rgb565->_cobj.w = w;
	rgb565->_cobj.h = h;
	rgb565->_cobj.bpp = 2;
	
	uint32_t len = w * h * 2; // rgb565 has 2 Bytes
	if(len != rgb_mem.len){
		alloc_mem(len);
	}
	rgb565->_cobj.pixels = rgb_mem.d8;
	
	uint16_t *rgb = (uint16_t*)rgb565->_cobj.pixels;
	uint8_t *gray = image->pixels;
	// do the convert with table
	// do the convert with assemble
	// 3ms
	/*
	{
		uint32_t s = rt_tick_get();
		gray2rgb565_C(rgb, gray, len / 2);
		uint32_t e = rt_tick_get();
		rt_kprintf("%d", e - s);
	}
	*/
	// 1ms ofast
	{
		uint32_t s = rt_tick_get();
		gray2rgb565(rgb, gray, len / 2);
		uint32_t e = rt_tick_get();
		rt_kprintf("%d", e - s);
	}
	// 1ms ofast
	/*
	{
		uint32_t s = rt_tick_get();
		gray2rgb565_tab(rgb, gray, len / 2);
		uint32_t e = rt_tick_get();
		rt_kprintf("%d", e - s);
	}	
	*/
	
	return MP_ROM_PTR(rgb565);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_grayscale_to_rgb_obj, mp_grayscale_to_rgb);

static bool first_alloc = true;
STATIC mp_obj_t mp_allocate_buf(mp_obj_t w_obj, mp_obj_t h_obj){
	if(first_alloc){
		fb_alloc_mark();
		first_alloc = false;
	}
	uint32_t w = mp_obj_get_int(w_obj);
	uint32_t h = mp_obj_get_int(h_obj);
	
	// update the resolution of cur screen
	resolution.w = w;
	resolution.h = h;
	
	uint32_t len = w * h * COLOR_DEPTH;
	void* buf = fb_alloc(len, FB_ALLOC_NO_HINT);
	return mp_obj_new_bytearray_by_ref(len, buf);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_allocate_buf_obj, mp_allocate_buf);


STATIC mp_obj_t mp_free_buf(){
	// make sure that the free will only be called after a allocate at least
	if(!first_alloc){
		first_alloc = true;
		fb_alloc_free_till_mark();
	}
	// free the gray_mem pool if valid
	//if(rgb_mem.buf) xfree(rgb_mem.buf);
	memset(&rgb_mem, 0x0, sizeof(rgb_mem));
	
	return mp_const_none;
}
// export a none-static func that can call outside 
// especially for the USED when quit from the ide, and need to sweep
// i think do not need, we will init the fb after quit from omv ide:
// both except or done the script
void mp_free_buf_out(){
	mp_free_buf();
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_free_buf_obj, mp_free_buf);

#define GET_PTR_FROM_OBJ(type, obj) ((__typeof__(type)*)(((mp_lv_struct_t*) MP_OBJ_TO_PTR(obj))->data));

STATIC mp_obj_t mp_flush(mp_obj_t disp_drv, mp_obj_t area, mp_obj_t color){
	lv_disp_drv_t *disp_ptr = GET_PTR_FROM_OBJ(lv_disp_drv_t, disp_drv);
	lv_area_t *area_ptr = GET_PTR_FROM_OBJ(lv_area_t, area);
	lv_color_t *color_ptr = GET_PTR_FROM_OBJ(lv_color_t, color);
	uint16_t w = lv_area_get_width(area_ptr);
	uint16_t h = lv_area_get_height(area_ptr);
	
extern void set_lvgl_running(bool enable);	
	set_lvgl_running(true);
	
	if(JPEG_FB()->enabled){
		MAIN_FB()->w = w;
		MAIN_FB()->h = h;
		MAIN_FB()->bpp = COLOR_DEPTH;
		MAIN_FB()->pixels = (uint8_t*)color_ptr;
		//rt_enter_critical();
		fb_update_jpeg_buffer(); 
		//rt_exit_critical();
		Update_FrameBuffer((void*)color_ptr, w, h, COLOR_DEPTH, NULL);
	}
	else
	{
		Update_FrameBuffer((void*)color_ptr, w, h, COLOR_DEPTH, NULL);
	}
	lv_disp_flush_ready(disp_ptr);
	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(mp_flush_obj, mp_flush);

STATIC mp_obj_t mp_capture(mp_obj_t indev_drv, mp_obj_t data){
	lv_indev_drv_t *indev_ptr = GET_PTR_FROM_OBJ(lv_indev_drv_t, indev_drv);
	lv_indev_data_t* data_ptr = GET_PTR_FROM_OBJ(lv_indev_data_t, data);
	DEMO_ReadTouch(indev_ptr, data_ptr, resolution.w, resolution.h);	
	return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(mp_capture_obj, mp_capture);

STATIC mp_obj_t mp_pause_lvgl(mp_obj_t enable){

	return mp_const_none;
	
}
MP_DEFINE_CONST_FUN_OBJ_1(mp_pause_lvgl_obj, mp_pause_lvgl);

STATIC mp_obj_t mp_get_display_size()
{
	int disp_w, disp_h;
	mp_obj_t items[2];
	
	LCDMonitor_GetDispSize(&disp_w, &disp_h);
	
	items[0] = mp_obj_new_int(disp_w);
	items[1] = mp_obj_new_int(disp_h);
	
	return mp_obj_new_list(2, items);
}

MP_DEFINE_CONST_FUN_OBJ_0(mp_get_display_size_obj, mp_get_display_size);
STATIC const mp_rom_map_elem_t lvgl_helper_globals_table[] = {
	{ MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_lvgl_helper) },
	{ MP_ROM_QSTR(MP_QSTR_get_ptr), MP_ROM_PTR(&mp_get_ptr_from_img_obj)},
	{ MP_ROM_QSTR(MP_QSTR_grayscale_to_rgb), MP_ROM_PTR(&mp_grayscale_to_rgb_obj)},	
	{ MP_ROM_QSTR(MP_QSTR_alloc), MP_ROM_PTR(&mp_allocate_buf_obj)},
	{ MP_ROM_QSTR(MP_QSTR_free), MP_ROM_PTR(&mp_free_buf_obj)},
	{ MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&mp_flush_obj)},
	{ MP_ROM_QSTR(MP_QSTR_capture), MP_ROM_PTR(&mp_capture_obj)},
	{ MP_ROM_QSTR(MP_QSTR_get_display_size), MP_ROM_PTR(&mp_get_display_size_obj)},
	// define a switch that enable the omv preview instead, without inside lvgl
	//{ MP_ROM_QSTR(MP_QSTR_pause_lvgl), MP_ROM_PTR(&mp_pause_lvgl_obj)},
	
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_lvgl_helper_globals,
    lvgl_helper_globals_table
);

const mp_obj_module_t mp_module_lvgl_helper = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_lvgl_helper_globals
};