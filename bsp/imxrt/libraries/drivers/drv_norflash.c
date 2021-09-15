/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       		Notes
 * 2020-04-13     Tony.Zhang(NXP)	
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "dfs_fs.h"
#include "fsl_flash.h"
//#include "ff.h"
#include "dfs_file.h"
#define LOG_TAG             "drv.falsh"
#include <drv_log.h>

#ifdef BSP_USING_SPIFLASH_PARTITION
#define FLASH_DEVICE_NAME "flash0"

#define PAGE_SIZE (256)
#define FLASH_BLOCK_SIZE 512
#define FLASH_SECTOR_SIZE 4096

#ifdef SOC_MIMXRT1176DVMMA
#define FLASH_MEM_BASE FlexSPI1_AMBA_BASE
#define MAX_FLASH_SIZE (16 * 1024 * 1024)
#undef EXAMPLE_FLEXSPI
#define EXAMPLE_FLEXSPI FLEXSPI1
#elif defined (SOC_IMXRT1064_SERIES)
#define FLASH_MEM_BASE 0x70000000U
#define MAX_FLASH_SIZE (4 * 1024 * 1024)
#define EXAMPLE_FLEXSPI FLEXSPI2
#else
#define FLASH_MEM_BASE FlexSPI_AMBA_BASE
#define MAX_FLASH_SIZE (8 * 1024 * 1024)
#define EXAMPLE_FLEXSPI FLEXSPI
#endif
#define TOTAL_FLASH_SIZE (FLASH_BLOCK_SIZE * 4096)
#define SECTOR_OFFSET (MAX_FLASH_SIZE - BSP_SPIFLASH_PARTITION_SIZE*FLASH_BLOCK_SIZE)    /* 16K */
#define FLASH_ADDR_OFFSET SECTOR_OFFSET
#define PARTITION_OFFSET  (SECTOR_OFFSET/FLASH_BLOCK_SIZE) 	 
#define RT_DEVICE_CTRL_BLK_GETOFFSET 0x100

static const char fresh_main_py[] __ALIGNED(4) =
"# main.py -- put your code here!\r\n"
;

typedef struct {
	uint8_t pd;	/* Physical drive number */
	uint8_t pt;	/* Partition: 0:Auto detect, 1-4:Forced partition) */
} PARTITION;

PARTITION VolToPart[RT_DFS_ELM_DRIVES];
static rt_err_t rt_norflash_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t rt_norflash_open(rt_device_t dev, rt_uint16_t oflag)
{

    return RT_EOK;
}

static rt_err_t rt_norflash_close(rt_device_t dev)
{

    return RT_EOK;
}

static void build_partition(uint8_t *buf, int boot, int type, uint32_t start_block, uint32_t num_blocks) {
    buf[0] = boot;

    if (num_blocks == 0) {
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
    } else {
        buf[1] = 0xff;
        buf[2] = 0xff;
        buf[3] = 0xff;
    }

    buf[4] = type;

    if (num_blocks == 0) {
        buf[5] = 0;
        buf[6] = 0;
        buf[7] = 0;
    } else {
        buf[5] = 0xff;
        buf[6] = 0xff;
        buf[7] = 0xff;
    }

    buf[8] = start_block;
    buf[9] = start_block >> 8;
    buf[10] = start_block >> 16;
    buf[11] = start_block >> 24;

    buf[12] = num_blocks;
    buf[13] = num_blocks >> 8;
    buf[14] = num_blocks >> 16;
    buf[15] = num_blocks >> 24;
}

static rt_size_t rt_norflash_read_block(void* buffer, rt_off_t pos)
{
	uint8_t *dest = (uint8_t *)buffer;
	if (pos == 0) {
        // fake the MBR so we can decide on our own partition table

        for (int i = 0; i < 446; i++) {
            dest[i] = 0;
        }
		// must > 0, for 0 is a fake one, return directly. and fatfs.patr=1, means part1, because we have 4 part below, means has 4 part for file-os mounting
        // then will go to the next loop, when second step in
		build_partition(dest + 446, 0, 0x01 /* FAT12 */, PARTITION_OFFSET, BSP_SPIFLASH_PARTITION_SIZE);
        build_partition(dest + 462, 0, 0, 0, 0);
        build_partition(dest + 478, 0, 0, 0, 0);
        build_partition(dest + 494, 0, 0, 0, 0);

        dest[510] = 0x55;
        dest[511] = 0xaa;

        return RT_EOK;

    } else {
		uint32_t primask = __get_PRIMASK();
		__set_PRIMASK(1);
		
		if (pos < PARTITION_OFFSET || pos > MAX_FLASH_SIZE/FLASH_BLOCK_SIZE) {
			// bad block number
			__set_PRIMASK(primask);
			return RT_EINVAL;
		}
		//pos += BSP_SPIFLASH_PARTITION_OFFSET;
		
		uint32_t addr = pos * FLASH_BLOCK_SIZE;
		uint32_t offset = addr & 0xfff;
		addr = (addr >> 12) << 12;
		#ifdef SDRAM_DISK_TEST
		uint32_t destAdrss = (uint32_t)(&fake_fs[0]) + addr + offset;
		if(destAdrss > (uint32_t)(&fake_fs[0])){
			memcpy(dest, (char*)destAdrss, FLASH_BLOCK_SIZE);
		}else{
		#endif
			uint32_t destAdrss = FLASH_MEM_BASE  + addr + offset;
			DCACHE_InvalidateByRange(destAdrss, FLASH_BLOCK_SIZE);
			// flexspi_nor_reset();
			memcpy(dest, (char*)destAdrss, FLASH_BLOCK_SIZE);
			__DSB();		
		#ifdef SDRAM_DISK_TEST
		}
		#endif
		__set_PRIMASK(primask);
		
	}
    return RT_EOK;
}

static rt_size_t rt_norflash_read(rt_device_t dev, rt_off_t pos, void* buffer,rt_size_t num_blocks)
{
	rt_size_t i = 0;
	SCB_CleanInvalidateDCache();
	//SCB_DisableDCache();
    for (i = 0; i < num_blocks; i++) {
        if (rt_norflash_read_block(buffer + i * FLASH_BLOCK_SIZE, pos + i) != RT_EOK) {
			//SCB_EnableDCache();
            return i; // error
        }
    }
	//SCB_EnableDCache();
	
	return i;
}
#undef SECTOR_SIZE
#define SECTOR_SIZE 0x1000
extern void BOARD_Config_FLASH_Excute_MPU(uint8_t dis_excute);
__attribute__((aligned(4))) static char buf[SECTOR_SIZE] = {0};
rt_size_t rt_norflash_write_block(const void* buffer, rt_off_t pos) __attribute__((section(".ram_code"))); /*the static function will not at rom_code with ofast */
rt_size_t rt_norflash_write_block(const void* buffer, rt_off_t pos)
{
	if (pos == 0) {
        // can't write MBR, but pretend we did
        return RT_EOK;

    } else {
		uint32_t primask = __get_PRIMASK();
		__set_PRIMASK(1);
		// non-MBR block, write to SPI flash
		//block += FLASH_PART1_START_BLOCK;
		if (pos < PARTITION_OFFSET || pos > MAX_FLASH_SIZE/FLASH_BLOCK_SIZE) {
			// bad block number
			__set_PRIMASK(primask);
			return RT_EINVAL;
		}
		//pos += BSP_SPIFLASH_PARTITION_OFFSET;
		// align to 4096 sector
		uint32_t addr = pos * FLASH_BLOCK_SIZE;
		uint32_t offset = addr & 0xfff;
		addr = (addr >> 12) << 12;
		// for sdram test, a fake sdram disk
        
                
		#ifdef SDRAM_DISK_TEST
		uint32_t destAdrss = (uint32_t)(&fake_fs[0])+ addr;
		if(destAdrss > (uint32_t)(&fake_fs[0])){
			memcpy((char*)destAdrss + offset, src, FLASH_BLOCK_SIZE);
		}
		else{
		#endif
			
			
			flexspi_nor_reset();
			
			uint32_t destAdrss = FLASH_MEM_BASE  + addr;
		#ifndef SOC_IMXRT1170_SERIES
			DCACHE_InvalidateByRange(destAdrss, SECTOR_SIZE);
		#endif	
			memcpy(buf, (char*)destAdrss, SECTOR_SIZE);
			
			// disable execute
			BOARD_Config_FLASH_Excute_MPU(1);
			// a deadly error,,,, erase a wrong area, cause the flash only be init once
			// when write someone, then disapear, the falsh not erase before program
			uint32_t erase_addr = ((addr ) >> 12) << 12;
			flexspi_nor_flash_erase_sector(EXAMPLE_FLEXSPI, erase_addr);
			// copy new block into buffer
			memcpy(buf + offset, buffer, FLASH_BLOCK_SIZE);
			__DSB();
			
				// write sector in pages of 256 bytes
			int result = -1;
			for (int i = 0; i < SECTOR_SIZE; i += PAGE_SIZE) {
				result = flexspi_nor_flash_page_program(EXAMPLE_FLEXSPI, addr + i, (uint32_t*)(buf+i));	
				__DSB();
				if (result != 0) {
					result = false;
                    break;
				}
			}
		#ifdef SDRAM_DISK_TEST
		}
		#endif        
        // enable execute
        BOARD_Config_FLASH_Excute_MPU(0); 
		__set_PRIMASK(primask);
	}
    return RT_EOK;
}

static rt_size_t rt_norflash_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t num_blocks)
{
	rt_size_t i;
	SCB_CleanInvalidateDCache();
	#ifdef SOC_IMXRT1170_SERIES
	SCB_DisableDCache(); // will only close dcache and clean
	#else
	DCACHE_InvalidateByRange(buffer, FLASH_BLOCK_SIZE*num_blocks);
	#endif
    for (i = 0; i < num_blocks; i++) {
        if (rt_norflash_write_block(buffer + i * FLASH_BLOCK_SIZE, pos + i) != RT_EOK) {
		#ifdef SOC_IMXRT1170_SERIES	
			SCB_EnableDCache();
		#endif	
            return i; // error
        }
    }
	#ifdef SOC_IMXRT1170_SERIES
	SCB_EnableDCache();
	#endif
    return i; // success
}

static rt_err_t rt_norflash_control(rt_device_t dev, int cmd, void *args)
{
	struct rt_device_blk_geometry geometry;
	switch(cmd)
	{
		case RT_DEVICE_CTRL_BLK_GETGEOME:
		{
			geometry.block_size = FLASH_BLOCK_SIZE;
			geometry.bytes_per_sector = FLASH_BLOCK_SIZE;//4k
			geometry.sector_count = BSP_SPIFLASH_PARTITION_SIZE;
			
			memcpy(args,&geometry,sizeof(geometry));
		}
			break;
		case RT_DEVICE_CTRL_BLK_GETOFFSET:
		{
			uint32_t offset = PARTITION_OFFSET;
			memcpy(args,&offset,sizeof(offset));
			break;
		}
		default:
			break;
	}
	return RT_EOK;
}

static struct rt_device flash_device =
{
	.type = RT_Device_Class_Block,
	.init = rt_norflash_init,
	.open = rt_norflash_open,
	.close = rt_norflash_close,
	.read = rt_norflash_read,
	.write = rt_norflash_write,
	.control = rt_norflash_control,
};

void rt_norflash_mount_fs(void)
{
	rt_norflash_drv_init();
	
	if (rt_device_find(FLASH_DEVICE_NAME) == NULL)
		return ;
	
	if (dfs_mount(FLASH_DEVICE_NAME,"/flash","elm",0,0) == -1)
	{//mount faild,mkfs on flash
		LOG_E("Can't find fs on flash, try to mkfs\r\n");
		rt_enter_critical();
		int st = dfs_mkfs("elm",FLASH_DEVICE_NAME);
		if(st == -1)
		{
			rt_exit_critical();
			LOG_E("mkfs on flash failed\r\n");
			rt_set_errno(-ENOSYS);
			return;
		}
		rt_kprintf("make fat on Flash\r\n");
		rt_exit_critical();
		
		if(dfs_mount(FLASH_DEVICE_NAME,"/flash","elm",0,0) == 0)
		{
			struct dfs_fd fd;
			
			dfs_file_open(&fd,"/flash/main.py",O_WRONLY | O_CREAT);
			dfs_file_write(&fd,fresh_main_py,sizeof(fresh_main_py));
			dfs_file_close(&fd);
		}else
		{
			LOG_E("mount flash failed\r\n");
		}
	}
	LOG_D("mount flash to /flash\r\n");
}
RTM_EXPORT(rt_norflash_mount_fs);
#endif
#ifdef BSP_USING_FLASH
int rt_norflash_drv_init(void)
{
	
	rt_err_t ret = RT_EOK;

	flexspi_nor_init();
#ifdef BSP_USING_SPIFLASH_PARTITION	
    ret = rt_device_register(&flash_device, FLASH_DEVICE_NAME, RT_DEVICE_FLAG_RDWR);

    if(ret != RT_EOK)
    {
        LOG_E("rt device:%s register failed %d\n", FLASH_DEVICE_NAME,ret);
        return ret;
    }
	
	VolToPart[0].pd = 0;
	VolToPart[0].pt = 1;
	
	VolToPart[1].pd = 1;
	VolToPart[1].pt = 0;
#endif	
	return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_norflash_drv_init);
#endif
