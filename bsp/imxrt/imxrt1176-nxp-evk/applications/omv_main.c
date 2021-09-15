/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2019 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2019 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * main function.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <rtthread.h>
#include "dfs_fs.h"
#include <py/compile.h>
#include <py/runtime.h>
#include <py/repl.h>
#include <py/gc.h>
#include <py/mperrno.h>
#include <py/stackctrl.h>
#include <py/frozenmod.h>
#include <lib/mp-readline/readline.h>
#include <lib/utils/pyexec.h>
#include "mpgetcharport.h"
#include "mpputsnport.h"
#include "drv_usb_omv.h"
#include "framebuffer.h"
#include "pendsv.h"
#include "ini.h"
#include "omv_boardconfig.h"
#include "mpconfigboard.h"
#include "usbdbg.h"

volatile uint8_t g_isMainDotPyRunning;
#if defined(__CC_ARM) || defined (__CLANG_ARM)
	#define STACK_SIZE	(0x2000)
	extern unsigned int Image$$RTT_MPY_THREAD_STACK$$Base;
	extern unsigned int Image$$MPY_HEAP_START$$Base;
	uint32_t _thread_stack_start = (uint32_t) &Image$$RTT_MPY_THREAD_STACK$$Base;
	uint32_t _heap_start = (uint32_t) &Image$$MPY_HEAP_START$$Base;
#elif defined(__ICCARM__)
	extern unsigned int RTT_MPY_THREAD_STACK$$Limit[];
	uint32_t _thread_stack_start = (uint32_t)RTT_MPY_THREAD_STACK$$Limit;	// we put HEAP at the last of DATA region, so we can assume mpy heap start here
#elif defined(__GNUC__)

#endif

__WEAK bool usbdbg_script_ready()
{
	return false;
}


#ifdef EARLY_INIT_SEC
extern char Image$$ER_EARLY_INIT$$Base[];
extern char Image$$ER_EARLY_INIT$$Limit[];
extern char Load$$ER_EARLY_INIT$$Base[];
#define SEC_START Image$$ER_EARLY_INIT$$Base
#define SEC_END Image$$ER_EARLY_INIT$$Limit
#define SEC_LOAD Load$$ER_EARLY_INIT$$Base
typedef void( *func_ptr)(void*, void*, void*, void*);
typedef struct {
	func_ptr p_func;
	void* p1,*p2,*p3,*p4;
}ptr_block;
#define INIT_EARLY_INIT_LIST(idx, ptr, v1, v2, v3, v4) \
	RT_USED const __attribute__((section(".early_init"))) ptr_block func_ptr_ ##idx = {.p_func = ptr, .p1=v1, .p2=v2, .p3=v3, .p4=v4};
extern void rt_system_heap_init(void *begin_addr, void *end_addr);
extern int Image$$RTT_HEAP$$Base[];
extern int Image$$RTT_HEAP$$ZI$$Limit[];
INIT_EARLY_INIT_LIST(0, rt_system_heap_init, Image$$RTT_HEAP$$Base, Image$$RTT_HEAP$$ZI$$Limit, NULL, NULL);
void temp_memcpy(void* dst, void* src, int length){
	// 4-byte align
	__asm volatile (
		"push {r3, lr}       \n"
		"loop:               \n"
		"	ldr r3, [r1], #4 \n"
		"	str r3, [r0], #4 \n"
		"	subs r2, #4      \n"
		"	bne  loop        \n"
		"	pop {r3, pc}     \n"
	);
}
void SystemInitHook(){
	// memcpy the data from the load addr
	if(SEC_LOAD != SEC_START){
		temp_memcpy((void*)SEC_START, (void*)(SEC_LOAD), (SEC_END - SEC_START));
	}
	for(int func=(int)SEC_START;func<(int)SEC_END;func++){
		ptr_block* ptr = (ptr_block*)(func);
		ptr->p_func(ptr->p1, ptr->p2, ptr->p3, ptr->p4);
	}	
}
#endif
#define THREAD_STACK_NO_SYNC   4096
#define THREAD_STACK_WITH_SYNC 8192

static void *stack_top = RT_NULL;
static char *heap = RT_NULL;
extern void pyb_pin_init(void);
void omv_main_thread_entry(void *parameter)
{
	int stack_dummy;
    int stack_size_check;
    stack_top = (void *)&stack_dummy;

		
#ifdef RT_USING_DFS
    //struct dfs_fdtable *fd_table_bak = NULL;
    //mp_sys_resource_bak(&fd_table_bak);
#endif
	
omv_restart:	
	mp_getchar_init();
    mp_putsn_init();
#if defined(MICROPYTHON_USING_FILE_SYNC_VIA_IDE)
    stack_size_check = THREAD_STACK_WITH_SYNC;
#else
    stack_size_check = THREAD_STACK_NO_SYNC;
#endif	
	
	if (rt_thread_self()->stack_size < stack_size_check) 
    {
        mp_printf(&mp_plat_print, "The stack (%.*s) size for executing MicroPython must be >= %d\n", RT_NAME_MAX, rt_thread_self()->name, stack_size_check);
    }
	
#if MICROPY_PY_THREAD
    mp_thread_init(rt_thread_self()->stack_addr, ((rt_uint32_t)stack_top - (rt_uint32_t)rt_thread_self()->stack_addr) / 4);
#endif

    mp_stack_set_top(stack_top);
    // Make MicroPython's stack limit somewhat smaller than full stack available
    mp_stack_set_limit(rt_thread_self()->stack_size - 1024);
	
	gc_init((void*)_heap_start, (void*)(_heap_start + MICROPY_HEAP_SIZE));
	
	MP_STATE_PORT(omv_ide_irq) = 0;
	/* MicroPython initialization */
    mp_init();

    /* system path initialization */
    mp_obj_list_init(mp_sys_path, 0);
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_)); // current dir (or base dir of the script)
    mp_obj_list_append(mp_sys_path, mp_obj_new_str(MICROPY_PY_PATH_FIRST, strlen(MICROPY_PY_PATH_FIRST)));
    mp_obj_list_append(mp_sys_path, mp_obj_new_str(MICROPY_PY_PATH_SECOND, strlen(MICROPY_PY_PATH_SECOND)));
    mp_obj_list_init(mp_sys_argv, 0);
    readline_init0();
	//pyb_usb_dev_deinit();
	
	//pyb_usb_dev_init(USBD_VID, USBD_PID_CDC_MSC, USBD_MODE_CDC_MSC, NULL);
	
	pyb_pin_init();
	#ifndef OMV_MPY_ONLY
	fb_alloc_init0();
	#endif
    file_buffer_init0();
	
	usbdbg_init();
	
	//run cmm_load.py
	char *cmm_path = "/sd/cmm_load.py";
	mp_import_stat_t stat = mp_import_stat(cmm_path);
	if (stat != MP_IMPORT_STAT_FILE)
	{
		cmm_path = "/flash/cmm_load.py";
		stat = mp_import_stat(cmm_path);
	}
    if (stat == MP_IMPORT_STAT_FILE) {
        nlr_buf_t nlr;
		mp_printf(&mp_plat_print, "Found and execute %s!\r\n",cmm_path);
        if (nlr_push(&nlr) == 0) {
            int ret = pyexec_file(cmm_path);
            if (ret & PYEXEC_FORCED_EXIT) {
				mp_printf(&mp_plat_print, "cmm_load.py error!\r\n");
                ret = 1;
            }
            if (!ret) {
            }
            nlr_pop();
        }
        else {           
        }
    }
	
	// rocky: pyb's main uses different method to access file system from omv
	char *main_path = "/sd/main.py";
	stat = mp_import_stat(main_path);
	if (stat != MP_IMPORT_STAT_FILE)
	{
		main_path = "/flash/main.py";
		stat = mp_import_stat(main_path);
	}
	if (stat == MP_IMPORT_STAT_FILE) {
		nlr_buf_t nlr;
		if (nlr_push(&nlr) == 0) {
			g_isMainDotPyRunning = 1;
			mp_printf(&mp_plat_print, "Found and execute %s!\r\n",main_path);
			int ret = pyexec_file(main_path);
			g_isMainDotPyRunning = 0;
			if (ret & PYEXEC_FORCED_EXIT) {
				ret = 1;
			}
			if (!ret) {
				
			}
			nlr_pop();
		}
		else {
			g_isMainDotPyRunning = 0;
			
			#if 0
			// 2019.03.27 19:52 rocky: if main.py is interrupted by running another script, 
			// we have to do soft reset, otherwise fb alloc logic may fail and led to hard fault
			// In this case, it makes user have to press start button twice to start the script in OpenMV IDE
			goto cleanup;
			#else
			fb_free_all();
			#endif                    
		}
		
		mp_printf(&mp_plat_print, "Exit from main.py!\r\n");
	}
			
RunREPL:
#if 1	
    while (!usbdbg_script_ready()) {
        nlr_buf_t nlr;

        if (nlr_push(&nlr) == 0) {
		
            // enable IDE interrupt
            usbdbg_set_irq_enabled(true);

            // run REPL
            if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
                if (pyexec_raw_repl() != 0) {
                    break;
                }
            } else {
                if (pyexec_friendly_repl() != 0) {
					
                    break;
                }
            }

            nlr_pop();
        }
    }
	mp_printf(&mp_plat_print, "Exit from repy!\r\n");
    if (usbdbg_script_ready()) {
        nlr_buf_t nlr;
		mp_printf(&mp_plat_print, "script ready!\r\n");
        // execute the script
        if (nlr_push(&nlr) == 0) {
	
			// rocky: 2019.03.27 19:00 reset fb alloc memory for new script
			#ifndef OMV_MPY_ONLY
			fb_alloc_init0();
			#endif
#if 0
			vstr_t *buf = usbdbg_get_script();
			mp_obj_t code = pyexec_compile_str(buf);	
            // enable IDE interrupt
            usbdbg_set_irq_enabled(true);
            pyexec_exec_code(code);
#else
			usbdbg_set_irq_enabled(true);
			pyexec_str((vstr_t *)usbdbg_get_script());
#endif
            nlr_pop();
        } else {
            mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
            usbdbg_stop_script();
		#if defined(SOC_MIMXRT1176DVMMA) && defined(RT_USING_LCD)
			extern void reset_displaymix();
			 /*
			 * Reset the displaymix, otherwise during debugging, the
			 * debugger may not reset the display, then the behavior
			 * is not right.
			 */
			reset_displaymix();
		#endif
			fb_alloc_init0();
            goto RunREPL;   // rocky: script is stopped, waiting for next script
        }

    }

	usbdbg_set_irq_enabled(true);
#endif	

	
	gc_sweep_all();

    mp_deinit();

#if MICROPY_PY_THREAD
    mp_thread_deinit();
#endif

    rt_free(heap);

    mp_putsn_deinit();
    mp_getchar_deinit();
    
#ifdef RT_USING_DFS
    //mp_sys_resource_gc(fd_table_bak);
#endif

	rt_thread_mdelay(500);
	goto omv_restart;
}
static struct rt_thread omv_thread;
void omv_main()
{
    pendsv_init();

	rt_err_t result;
	

    result = rt_thread_init(&omv_thread, "omv_main", omv_main_thread_entry, RT_NULL,
                            (void*)_thread_stack_start, 0x8000, RT_MAIN_THREAD_PRIORITY-5, 20);
    RT_ASSERT(result == RT_EOK);
	rt_thread_startup(&omv_thread);
}

static void omv(uint8_t argc, char **argv) {
    omv_main();
}
//INIT_APP_EXPORT(omv_main);
MSH_CMD_EXPORT(omv, OpenMV: `execute python script);
