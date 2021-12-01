/*
 * This file is part of the Micro Python project, http://micropython.org/
 */

/**
  ******************************************************************************
  * @file    usbd_storage_msd.c
  * @author  MCD application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the disk operations functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  * Heavily modified by dpgeorge for Micro Python.
  *
  ******************************************************************************
  */

#include <stdint.h>
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
//#include "fsl_debug_console.h"

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_msc.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include <stdio.h>
#include <stdlib.h>

#include "composite.h"
#include "usbd_msc_storage.h"
#include "storage.h"
#ifdef NXP_USING_OPENMV
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/misc.h"

//#include "sdcard.h"
#endif
#include <drivers/mmcsd_core.h>
#include <drivers/mmc.h>
/*******************************************************************************
* Definitions
******************************************************************************/
#undef usb_echo
#define usb_echo(...) rt_kprintf(__VA_ARGS__)
#define STORAGE_DEVICE_NAME "sd0"
static rt_device_t mss_device = NULL;
/*******************************************************************************
* Variables
******************************************************************************/

static usb_device_composite_struct_t *g_deviceComposite;

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) usb_device_inquiry_data_fromat_struct_t g_InquiryInfo = {
    (USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER << USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER_SHIFT) |
        USB_DEVICE_MSC_UFI_PERIPHERAL_DEVICE_TYPE,
    (uint8_t)(USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT << USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT_SHIFT),
    USB_DEVICE_MSC_UFI_VERSIONS,
    0x02,
    USB_DEVICE_MSC_UFI_ADDITIONAL_LENGTH,
    {0x00, 0x00, 0x00},
    {'N', 'X', 'P', ' ', 'S', 'E', 'M', 'I'},
    {'N', 'X', 'P', ' ', 'M', 'A', 'S', 'S', ' ', 'S', 'T', 'O', 'R', 'A', 'G', 'E'},
    {'0', '0', '0', '1'}};
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) usb_device_mode_parameters_header_struct_t g_ModeParametersHeader = {
    /*refer to ufi spec mode parameter header*/
    0x0000, /*!< Mode Data Length*/
    0x00,   /*!<Default medium type (current mounted medium type)*/
    0x00,   /*!MODE SENSE command, a Write Protected bit of zero indicates the medium is write enabled*/
    {0x00, 0x00, 0x00, 0x00} /*!<This bit should be set to zero*/
};

uint32_t g_mscReadRequestBuffer[USB_DEVICE_MSC_READ_BUFF_SIZE >> 2] __attribute__((section(".usb_buf")));

uint32_t g_mscWriteRequestBuffer[USB_DEVICE_MSC_WRITE_BUFF_SIZE >> 2]  __attribute__((section(".usb_buf")));


/*******************************************************************************
* Code
******************************************************************************/
/*!
 * @brief device msc callback function.
 *
 * This function handle the disk class specified event.
 * @param handle          The USB class  handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the class specific event.
 * @return kStatus_USB_Success or error.
 */
#if FLASH_BLOCK_SIZE != LENGTH_OF_EACH_LBA
#error "FLASH_BLOCK_SIZE "
#endif
#if FLASH_BLOCK_SIZE != 512
#error "only 512B block is supported!"
#else
#define USB_MSC_BLOCK_SIZE_LOG2	9
#endif

bool sdcard_is_present(void)
{
	rt_device_t mss_device = rt_device_find(STORAGE_DEVICE_NAME);
	if(mss_device == RT_NULL)
		return false;
	else
		return true;
	
}

void sdcard_device_close(void)
{
	if(mss_device == RT_NULL)
		return;
	
	rt_device_close(mss_device);
}

uint32_t sdcard_get_lba_count(void)
{
	struct rt_device_blk_geometry geometry;
	
	mss_device = rt_device_find(STORAGE_DEVICE_NAME);
	if(mss_device == RT_NULL)
		return 0;
	
	rt_err_t st = rt_device_open(mss_device, RT_DEVICE_OFLAG_RDWR);
	if( st != RT_EOK)
    {
        rt_kprintf("disk open error\n");
        return 0;
    }
	
	if(rt_device_control(mss_device, RT_DEVICE_CTRL_BLK_GETGEOME, 
        (void*)&geometry) != RT_EOK)
    {
        rt_kprintf("get disk info error\n");
        return 0;
    }
	
	rt_device_close(mss_device);
	
	return geometry.sector_count;
}

uint32_t sdcard_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks)
{
    mss_device = rt_device_find(STORAGE_DEVICE_NAME);
	if(mss_device == RT_NULL)
		return 0;

    rt_err_t st = rt_device_open(mss_device, RT_DEVICE_OFLAG_RDWR);
	if( st != RT_EOK)
    {
        rt_kprintf("disk open error\n");
        return 0;
    }

	int count = rt_device_read(mss_device,block_num,dest,num_blocks);

    rt_device_close(mss_device);
    return count;
}

uint32_t sdcard_write_blocks(const uint8_t *src, uint32_t block_num, uint32_t num_blocks)
{
    mss_device = rt_device_find(STORAGE_DEVICE_NAME);
	if(mss_device == RT_NULL)
		return 0;

    rt_err_t st = rt_device_open(mss_device, RT_DEVICE_OFLAG_RDWR);
	if( st != RT_EOK)
    {
        rt_kprintf("disk open error\n");
        return 0;
    }

	int count = rt_device_write(mss_device,block_num,src,num_blocks);

    rt_device_close(mss_device);
    return count;
}

#ifdef BSP_USING_SPIFLASH_PARTITION
uint32_t flash_start_sector = 0;
uint32_t flash_get_lba_count(void)
{
	struct rt_device_blk_geometry geometry;
	
	mss_device = rt_device_find("flash0");
	if(mss_device == RT_NULL)
		return 0;
	
	rt_err_t st = rt_device_open(mss_device, RT_DEVICE_OFLAG_RDWR);
	if( st != RT_EOK)
    {
        rt_kprintf("flash disk open error\n");
        return 0;
    }
	
	if(rt_device_control(mss_device, RT_DEVICE_CTRL_BLK_GETGEOME, 
        (void*)&geometry) != RT_EOK)
    {
        rt_kprintf("get flash disk info error\n");
        return 0;
    }
	
	if(rt_device_control(mss_device, 0x100, //get start sector count 
        (void*)&flash_start_sector) != RT_EOK)
    {
        rt_kprintf("get flash disk info error\n");
        return 0;
    }
	
    rt_device_close(mss_device);
	return geometry.sector_count;
}

uint32_t flash_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks)
{
    mss_device = rt_device_find("flash0");
	if(mss_device == RT_NULL)
		return 0;

    rt_err_t st = rt_device_open(mss_device, RT_DEVICE_OFLAG_RDWR);
	if( st != RT_EOK)
    {
        rt_kprintf("flash disk open error\n");
        return 0;
    }

	int count = rt_device_read(mss_device,block_num,dest,num_blocks);
    rt_device_close(mss_device);
    
    return count;
}

uint32_t flash_write_blocks(const uint8_t *src, uint32_t block_num, uint32_t num_blocks)
{
	mss_device = rt_device_find("flash0");
	if(mss_device == RT_NULL)
		return 0;

    rt_err_t st = rt_device_open(mss_device, RT_DEVICE_OFLAG_RDWR);
	if( st != RT_EOK)
    {
        rt_kprintf("flash disk open error\n");
        return 0;
    }

	int count = rt_device_write(mss_device,block_num,src,num_blocks);
    rt_device_close(mss_device);

    return count;
}

#endif
uint8_t s_isUseSDCard;
uint8_t s_usb_w_en = 1;//0;
void usb_write_open()
{
	s_usb_w_en = 1;
}
MSH_CMD_EXPORT(usb_write_open, USB:open usb write)
#ifdef RT_USING_DFS
#include <dfs_posix.h>
#endif
void usb_write_close()
{	
	s_usb_w_en = 0;
#ifdef RT_USING_DFS
	if (s_isUseSDCard)
	{
		dfs_unmount("/sd");
		dfs_mount("sd0","/sd","elm",0,0);
	}
	else
	{
		dfs_unmount("/flash");
		dfs_mount("flash0","/flash","elm",0,0);
	}
#endif	
}
MSH_CMD_EXPORT(usb_write_close, USB:close usb write)

void _ConfigLBA(usb_device_lba_information_struct_t *lbaInf)
{
	mmcsd_wait_cd_changed(200);
	if (sdcard_is_present()) 
	{
		usb_echo("USB MSC Connected with SD\r\n");
		s_isUseSDCard = 1; //1;
		lbaInf->logicalUnitInformations[0].totalLbaNumberSupports = sdcard_get_lba_count();
	}
	else {
	#ifdef BSP_USING_SPIFLASH_PARTITION	
		usb_echo("USB MSC Connected with Flash\r\n");
		lbaInf->logicalUnitInformations[0].totalLbaNumberSupports = flash_get_lba_count();
	#endif	
		
	}
	lbaInf->logicalUnitNumberSupported = LOGICAL_UNIT_SUPPORTED;
	lbaInf->logicalUnitInformations[0].lengthOfEachLba = 512;
	lbaInf->logicalUnitInformations[0].bulkInBufferSize = sizeof(g_mscReadRequestBuffer);
	lbaInf->logicalUnitInformations[0].bulkOutBufferSize = sizeof(g_mscWriteRequestBuffer);
	
}



usb_status_t USB_DeviceMscCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Success;
    status_t errorCode = kStatus_Success;
    usb_device_lba_information_struct_t *lbaInformation;
    usb_device_lba_app_struct_t *lba;
    usb_device_ufi_app_struct_t *ufi;
	uint32_t t1 = kStatus_Fail;
	usb_device_capacity_information_struct_t *capacityInformation;
#if (defined(USB_DEVICE_CONFIG_USE_TASK) && (USB_DEVICE_CONFIG_USE_TASK > 0)) && \
    (defined(USB_DEVICE_MSC_USE_WRITE_TASK) && (USB_DEVICE_MSC_USE_WRITE_TASK > 0))
    usb_msc_buffer_struct_t *tempbuffer;
#endif
    switch (event)
    {
        case kUSB_DeviceMscEventReadResponse:
            lba = (usb_device_lba_app_struct_t *)param;
            break;
        case kUSB_DeviceMscEventWriteResponse:
            lba = (usb_device_lba_app_struct_t *)param;
#if (defined(USB_DEVICE_CONFIG_USE_TASK) && (USB_DEVICE_CONFIG_USE_TASK > 0)) && \
    (defined(USB_DEVICE_MSC_USE_WRITE_TASK) && (USB_DEVICE_MSC_USE_WRITE_TASK > 0))
            if (NULL != currentTrasfer)
            {
                currentTrasfer->offset = lba->offset;
                currentTrasfer->size = lba->size;
                if (0 == lba->size)
                {
                    USB_DeviceMscAddBufferToHead(currentTrasfer);
                }
                else
                {
                    USB_DeviceMscAddBufferToTail(currentTrasfer);
                }
            }
#else
            /*write the data to sd card*/
            if (0 != lba->size)
            {
				if (s_usb_w_en)
				{
					if (s_isUseSDCard) {
						if (sdcard_is_present())
							t1 = sdcard_write_blocks(lba->buffer,lba->offset, lba->size >> USB_MSC_BLOCK_SIZE_LOG2);
					} else {
						#ifdef BSP_USING_SPIFLASH_PARTITION
							t1 = flash_write_blocks(lba->buffer,lba->offset + flash_start_sector, lba->size >> USB_MSC_BLOCK_SIZE_LOG2);
						#endif
					}
				}
				else
				{
					g_deviceComposite->mscDisk.readWriteError = 1;
                    usb_echo(
                        "\r\nUSB Write disable, please run 'usb_write_open' to open usb write and run 'usb_write_close' after write done \r\n"
                        );
                    error = kStatus_USB_Error;
					break;
				}
				
				if ((lba->size >> USB_MSC_BLOCK_SIZE_LOG2) != t1)
                {
                    g_deviceComposite->mscDisk.readWriteError = 1;
                    usb_echo(
                        "Write error, error = 0xx%x \t Please check write request buffer size(must be less than 128 "
                        "sectors)\r\n",
                        error);
                    error = kStatus_USB_Error;
                }
            }	

#endif
            break;
        case kUSB_DeviceMscEventWriteRequest:
            lba = (usb_device_lba_app_struct_t *)param;
/*get a buffer to store the data from host*/
#if (defined(USB_DEVICE_CONFIG_USE_TASK) && (USB_DEVICE_CONFIG_USE_TASK > 0)) && \
    (defined(USB_DEVICE_MSC_USE_WRITE_TASK) && (USB_DEVICE_MSC_USE_WRITE_TASK > 0))

            USB_DeviceMscGetBufferFromHead(&tempbuffer);
            while (NULL == tempbuffer)
            {
                usb_echo("No buffer available");
                USB_DeviceMscWriteTask();
                USB_DeviceMscGetBufferFromHead(&tempbuffer);
            }
            lba->buffer = tempbuffer->buffer;
            currentTrasfer = tempbuffer;
#else
            lba->buffer = (uint8_t *)&g_mscWriteRequestBuffer[0];
#endif
            break;
        case kUSB_DeviceMscEventReadRequest:
            lba = (usb_device_lba_app_struct_t *)param;
            lba->buffer = (uint8_t *)&g_mscReadRequestBuffer[0];

			if (s_isUseSDCard) {
				if (sdcard_is_present())
					t1 = sdcard_read_blocks(lba->buffer,lba->offset, lba->size >> USB_MSC_BLOCK_SIZE_LOG2);
			} else {
				#ifdef BSP_USING_SPIFLASH_PARTITION
					t1 = flash_read_blocks(lba->buffer,lba->offset + flash_start_sector, lba->size >> USB_MSC_BLOCK_SIZE_LOG2);	
				#endif
			}

			if ((lba->size >> USB_MSC_BLOCK_SIZE_LOG2) != t1)
            {
                g_deviceComposite->mscDisk.readWriteError = 1;
                usb_echo(
                    "Read error, error = 0xx%x \t Please check read request buffer size(must be less than 128 "
                    "sectors)\r\n",
                    t1);
                error = kStatus_USB_Error;
            }

            break;
        case kUSB_DeviceMscEventGetLbaInformation:
            _ConfigLBA((usb_device_lba_information_struct_t *)param);
            break;
        case kUSB_DeviceMscEventTestUnitReady:
            /*change the test unit ready command's sense data if need, be careful to modify*/
            ufi = (usb_device_ufi_app_struct_t *)param;
            break;
        case kUSB_DeviceMscEventInquiry:
            ufi = (usb_device_ufi_app_struct_t *)param;
            ufi->size = sizeof(usb_device_inquiry_data_fromat_struct_t);
            ufi->buffer = (uint8_t *)&g_InquiryInfo;
            break;
        case kUSB_DeviceMscEventModeSense:
            ufi = (usb_device_ufi_app_struct_t *)param;
            ufi->size = sizeof(usb_device_mode_parameters_header_struct_t);
            ufi->buffer = (uint8_t *)&g_ModeParametersHeader;
            break;
        case kUSB_DeviceMscEventModeSelect:
            break;
        case kUSB_DeviceMscEventModeSelectResponse:
            ufi = (usb_device_ufi_app_struct_t *)param;
            break;
        case kUSB_DeviceMscEventFormatComplete:
            break;
        case kUSB_DeviceMscEventRemovalRequest:
            break;
		case kUSB_DeviceMscEventReadCapacity:
            capacityInformation                         = (usb_device_capacity_information_struct_t *)param;
            capacityInformation->lengthOfEachLba        = 512;
			if (s_isUseSDCard) 
			{
				//usb_echo("USB MSC kUSB_DeviceMscEventReadCapacity SD\r\n");
				capacityInformation->totalLbaNumberSupports = sdcard_get_lba_count();
			}
			else {
				#ifdef BSP_USING_SPIFLASH_PARTITION	
				//usb_echo("USB MSC Connected with Flash\r\n");
				capacityInformation->totalLbaNumberSupports = flash_get_lba_count();
				#endif	
			}

            break;
        case kUSB_DeviceMscEventReadFormatCapacity:
            capacityInformation                         = (usb_device_capacity_information_struct_t *)param;
            capacityInformation->lengthOfEachLba        = 512;
			if (s_isUseSDCard) 
			{
				usb_echo("USB MSC kUSB_DeviceMscEventReadCapacity SD\r\n");
				capacityInformation->totalLbaNumberSupports = sdcard_get_lba_count();
			}
			else {
				#ifdef BSP_USING_SPIFLASH_PARTITION	
				usb_echo("USB MSC Connected with Flash\r\n");
				capacityInformation->totalLbaNumberSupports = flash_get_lba_count();
				#endif	
			}
            break;
        default:
            break;
    }
    return error;
}



/*!
 * @brief msc device set configuration function.
 *
 * This function sets configuration for msc class.
 *
 * @param handle The msc class handle.
 * @param configure The msc class configure index.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceMscDiskSetConfigure(class_handle_t handle, uint8_t configure)
{
    return kStatus_USB_Error;
}
/*!
 * @brief device msc init function.
 *
 * This function initializes the device with the composite device class information.
 *
 * @param deviceComposite          The pointer to the composite device structure.
 * @return kStatus_USB_Success .
 */
usb_status_t USB_DeviceMscDiskInit(usb_device_composite_struct_t *deviceComposite)
{
    g_deviceComposite = deviceComposite;
    return kStatus_USB_Success;
}
