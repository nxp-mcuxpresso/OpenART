/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-04-29     tyustli      first version
 */

//#include "MIMXRT1062.h"
#include <rtdevice.h>
#include "drv_gpio.h"
#include "core_cm7.h"
#include <rtthread.h>


#include "dfs_fs.h"
#include "dfs_ramfs.h"
#ifdef RT_USING_DFS_MNTTABLE
const struct dfs_mount_tbl mount_table[] ={//auto mount 
    {"sd0", "/sd", "elm", 0, 0},
    {0}
};
#endif
/* defined the LED pin: GPIO1_IO9 */
#define LED0_PIN               GET_PIN(1, 9)

int main(void)
{
	
	
    while (1)
    {
        //rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        //rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}

void reboot(void)
{
	//LPUART_Deinit(LPUART1);
    NVIC_SystemReset();
}
MSH_CMD_EXPORT(reboot, reset system)

void remount_sd(void)
{
    dfs_unmount("/sd");
	dfs_mount("sd0","/sd","elm",0,0);
}
MSH_CMD_EXPORT(remount_sd, remount sd card)

void remount_flash(void)
{
    dfs_unmount("/flash");
	dfs_mount("flash0","/flash","elm",0,0);
}
MSH_CMD_EXPORT(remount_flash, remount flash)
extern void rt_norflash_mount_fs();
int mnt_init(void)
{
#ifdef RT_USING_DFS	
    if (dfs_mount(RT_NULL, "/", "ram", 0, (const void*)dfs_ramfs_create(rt_malloc(1024),1024)) == 0)
    {
        rt_kprintf("RAM file system initializated!\n");
    }
    else
    {
        rt_kprintf("RAM file system initializate failed!\n");
    }
#endif	
#ifdef BSP_USING_SPIFLASH_PARTITION		
	rt_norflash_mount_fs();
#endif	
    return 0;
}
INIT_ENV_EXPORT(mnt_init);