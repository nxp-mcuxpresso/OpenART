/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <stdlib.h>
#include <rtthread.h>
#ifdef NXP_USING_MICROPYTHON
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/misc.h"
#include "irq.h"
#endif

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_msc.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
//#include "fsl_debug_console.h"
#include "composite.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#if ((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
#include "usb_phy.h"
#endif

#include "pin_mux.h"
#include <stdbool.h>
#ifdef NXP_USING_OPENMV
#include "drv_usb_omv.h"
#endif
#ifndef IRQ_PRI_USB_OTG1
#define IRQ_PRI_USB_OTG1        12
#endif
/*******************************************************************************
* Definitions
******************************************************************************/
#undef usb_echo
#ifdef NXP_USING_MICROPYTHON
#define usb_echo(...) mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)
#else
#define usb_echo(...) rt_kprintf(__VA_ARGS__)
#endif
/* USB clock source and frequency*/
#define USB_FS_CLK_SRC kCLOCK_UsbSrcFro
#define USB_FS_CLK_FREQ CLOCK_GetFreq(kCLOCK_FroHf)
#define USB_HS_CLK_SRC kCLOCK_UsbSrcUsbPll
#define USB_HS_CLK_FREQ 0U
#define USB_NXP_STACK_MSC_ENABLE USB_DEVICE_CONFIG_MSC
/* USB PHY condfiguration */
#define BOARD_USB_PHY_D_CAL (0x0CU)
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#define BOARD_USB_PHY_TXCAL45DM (0x06U)

#ifndef USBD_VID
#define USBD_VID         (0x1209)
// nxp is (0x1FC9)
// (0x1400 | USBD_MODE_CDC | USBD_MODE_MSC)
#define USBD_PID_CDC_MSC (0xABD1)
#endif

#define NXP_USB_STACK_THREAD 1
/*******************************************************************************
* Prototypes
******************************************************************************/
void BOARD_InitHardware(void);
void USBAPP_DestroyThread();
void USBAPP_ResumeThread();
void USBAPP_StartThread();
extern void sdcard_device_close(void);
/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

/*******************************************************************************
* Variables
******************************************************************************/
/* Composite device structure. */
usb_device_composite_struct_t g_composite;
extern usb_device_class_struct_t g_usbdCdcVcomConfig[];
extern usb_device_class_struct_t g_usbdMscConfig;
extern usb_device_class_struct_t g_usbdHidKeyboardConfig;
extern usb_device_class_struct_t g_usbdHidMouseConfig;
extern usb_device_class_struct_t g_usbdHidGenericConfig;
usb_status_t USB_DeviceMscCallback_NULL(class_handle_t handle, uint32_t event, void *param);

/* USB device class information, support at most 5 classes */
usb_device_class_config_struct_t g_classes[class_ndx_end] = {
	// IMPORTANT! Class orders must be corresponding to "class_ndx_enum"
#ifdef USB_NXP_STACK_MSC_ENABLE	
    {USB_DeviceMscCallback, (class_handle_t)NULL, &g_usbdMscConfig},
#endif
    {USB_DeviceCdcVcomCallback, (class_handle_t)NULL, &g_usbdCdcVcomConfig[0]},
#if USB_DEVICE_CONFIG_CDC_ACM > 1		
    {USB_DeviceCdcVcomCallback, (class_handle_t)NULL, &g_usbdCdcVcomConfig[1]},
#endif	
};

/* USB device class configuration information */
usb_device_class_config_list_struct_t g_classesConfigList = {
    g_classes, USB_DeviceCallback, USB_DEVICE_CONFIG_CDC_ACM + USB_DEVICE_CONFIG_MSC,
};

/*******************************************************************************
* Code
******************************************************************************/

usb_status_t USB_DeviceMscCallback_NULL(class_handle_t handle, uint32_t event, void *param)
{
	return kStatus_USB_Success;
}

/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint16_t *temp16   = (uint16_t *)param;
    uint8_t *temp8     = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            g_composite.attach               = 0;
            g_composite.currentConfiguration = 0U;
            error                            = kStatus_USB_Success;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &g_composite.speed))
            {
                USB_DeviceSetSpeed(handle, g_composite.speed);
            }
#endif
        }
        break;
        case kUSB_DeviceEventSetConfiguration:
            if (0U == (*temp8))
            {
                g_composite.attach               = 0;
                g_composite.currentConfiguration = 0U;
            }
            else if (USB_COMPOSITE_CONFIGURE_INDEX == (*temp8))
            {
                g_composite.attach               = 1;
                g_composite.currentConfiguration = *temp8;
                USB_DeviceCdcVcomSetConfigure(g_composite.cdcVcom[0].cdcAcmHandle, *temp8);
			#ifdef USB_NXP_STACK_MSC_ENABLE		
                USB_DeviceMscDiskSetConfigure(g_composite.mscDisk.mscHandle, *temp8);
			#endif	
                error = kStatus_USB_Success;
            }
            else
            {
                error = kStatus_USB_InvalidRequest;
            }
            break;
        case kUSB_DeviceEventSetInterface:
            if (g_composite.attach)
            {
                uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if (interface < USB_INTERFACE_COUNT)
                {
                    g_composite.currentInterfaceAlternateSetting[interface] = alternateSetting;
                    error                                                   = kStatus_USB_Success;
                }
            }
            break;
        case kUSB_DeviceEventGetConfiguration:
            if (param)
            {
                *temp8 = g_composite.currentConfiguration;
                error  = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceEventGetInterface:
            if (param)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                if (interface < USB_INTERFACE_COUNT)
                {
                    *temp16 = (*temp16 & 0xFF00U) | g_composite.currentInterfaceAlternateSetting[interface];
                    error   = kStatus_USB_Success;
                }
                else
                {
                    error = kStatus_USB_InvalidRequest;
                }
            }
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param)
            {
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param)
            {
                error = USB_DeviceGetConfigurationDescriptor(handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
#if (defined(USB_DEVICE_CONFIG_CV_TEST) && (USB_DEVICE_CONFIG_CV_TEST > 0U))
        case kUSB_DeviceEventGetDeviceQualifierDescriptor:
            if (param)
            {
                /* Get device descriptor request */
                error = USB_DeviceGetDeviceQualifierDescriptor(
                    handle, (usb_device_get_device_qualifier_descriptor_struct_t *)param);
            }
            break;
#endif
        case kUSB_DeviceEventGetStringDescriptor:
            if (param)
            {
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
        default:
            break;
    }

    return error;
}


/*!
 * @brief USB Interrupt service routine.
 *
 * This function serves as the USB interrupt service routine.
 *
 * @return None.
 */

#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U))
#if NXP_USB_STACK_THREAD
void USBAPP_IRQHandler()
{
	USB_DeviceEhciIsrFunction(g_composite.deviceHandle);
}
#endif

void USB_OTG1_IRQHandler(void)
{
	rt_interrupt_enter();

#if NXP_USB_STACK_THREAD	
	USBAPP_ResumeThread();
#else	
    USB_DeviceEhciIsrFunction(g_composite.deviceHandle);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
    // __DSB();
#endif	

	rt_interrupt_leave();
}
#endif
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U))
void USB_OTG2_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(g_composite.deviceHandle);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
    // __DSB();
}
#endif

void USB_DeviceClockInit(void)
{
	uint32_t usbClockFreq;
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
    };
	
#ifdef SOC_IMXRT1170_SERIES	
	usbClockFreq = 24000000;
#else
	usbClockFreq = 48000000;
#endif	
    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);

}
void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
#endif
/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, IRQ_PRI_USB_OTG1);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, IRQ_PRI_USB_OTG1);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

#if USB_DEVICE_CONFIG_USE_TASK
	void USB_DeviceTaskFn(void *deviceHandle)
	{
		#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
		    USB_DeviceEhciTaskFunction(deviceHandle);
		#endif
	}
#endif


/*!
 * @brief Application initialization function.
 *
 * This function initializes the application.
 *
 * @return None.
 */
void USBAPP_Init(void)
{
	USB_DeviceClockInit();
	#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
		SYSMPU_Enable(SYSMPU, 0);
	#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */



#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
    uint8_t usbDeviceIP3511Irq[] = USB_IRQS;
    irqNumber = usbDeviceIP3511Irq[CONTROLLER_ID - kUSB_ControllerLpcIp3511Fs0];

    // enable USB IP clock
    CLOCK_EnableUsbfs0DeviceClock(USB_FS_CLK_SRC, USB_FS_CLK_FREQ);
#endif

    g_composite.speed = USB_SPEED_FULL;
    g_composite.attach = 0;
    g_composite.cdcVcom[0].cdcAcmHandle = (class_handle_t)NULL;
    g_composite.mscDisk.mscHandle = (class_handle_t)NULL;
    g_composite.deviceHandle = NULL;

    if (kStatus_USB_Success !=
        USB_DeviceClassInit(CONTROLLER_ID, &g_classesConfigList, &g_composite.deviceHandle))
    {
        usb_echo("USB device composite demo init failed\r\n");
        return;
    }
    else
    {
        usb_echo("USB device composite demo\r\n");
		#ifdef USB_NXP_STACK_MSC_ENABLE	
			g_composite.mscDisk.mscHandle = g_classesConfigList.config[0].classHandle;
			g_composite.cdcVcom[0].cdcAcmHandle = g_classesConfigList.config[1].classHandle;
		#if USB_DEVICE_CONFIG_CDC_ACM > 1
			g_composite.cdcVcom[1].cdcAcmHandle = g_classesConfigList.config[2].classHandle;
		#endif
		#else
			g_composite.cdcVcom[0].cdcAcmHandle = g_classesConfigList.config[0].classHandle;
		#endif

        USB_DeviceCdcVcomInit(&g_composite);
	#if USB_NXP_STACK_MSC_ENABLE		
        USB_DeviceMscDiskInit(&g_composite);
	#endif	
    }
#if NXP_USB_STACK_THREAD	
	USBAPP_StartThread();
#else
	USB_DeviceIsrEnable();
#endif	
	
#ifdef SOC_IMXRT1170_SERIES	
	SDK_DelayAtLeastUs(5000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
#endif	
    USB_DeviceRun(g_composite.deviceHandle);
		
}

int32_t USBAPP_Deinit(void)
{
	usb_status_t st = 0;
	if (g_composite.deviceHandle) {
		if (USB_DeviceStop(g_composite.deviceHandle)== kStatus_USB_Success)
		{
			USB_DeviceClassDeinit(CONTROLLER_ID);
			
			st = USB_DeviceDeinit(g_composite.deviceHandle);
			assert(st == kStatus_USB_Success);
			
			USBAPP_DestroyThread();
			sdcard_device_close();
			return 0;
		} else {
			return -1;
		}
	}
	return -1;
}

#define USE_EVT 1
rt_thread_t usb_tid;
rt_sem_t usb_sem = RT_NULL;

struct rt_event usb_isr_event;
#define EVENT_USB_ISR 1
void usb_thread_entry(void *parameter)
{
	#if !USE_EVT
	usb_sem = rt_sem_create("usb", 0, RT_IPC_FLAG_FIFO);
	#endif
	USB_DeviceIsrEnable();
	while(1)
	{
		//disable usb int
		#if USE_EVT
		if (rt_event_recv(&usb_isr_event, EVENT_USB_ISR,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER, NULL) == RT_EOK)
		#else
		rt_sem_take(usb_sem, RT_WAITING_FOREVER);
		#endif
		{
			NVIC_DisableIRQ(USB_OTG1_IRQn);
			__DSB(); __ISB();
			USBAPP_IRQHandler();
			//NVIC_ClearPendingIRQ(USB_OTG1_IRQn);
			NVIC_EnableIRQ(USB_OTG1_IRQn);
        }
		__DSB(); __ISB();
	}
}

void USBAPP_ResumeThread()
{
	#if USE_EVT
	rt_event_send(&usb_isr_event, EVENT_USB_ISR);
	#else
	rt_sem_release(usb_sem);
	#endif

	NVIC_DisableIRQ(USB_OTG1_IRQn);
	__DSB(); __ISB();
}

void USBAPP_StartThread()
{
	rt_err_t result;
	
	NVIC_DisableIRQ(USB_OTG1_IRQn);
	#if USE_EVT
	rt_event_init(&usb_isr_event, "event", RT_IPC_FLAG_FIFO);
	#endif
    usb_tid = rt_thread_create("usb", usb_thread_entry, RT_NULL,
                            RT_MAIN_THREAD_STACK_SIZE, 4, 20);
    RT_ASSERT(usb_tid != RT_NULL);
	rt_thread_startup(usb_tid);
	rt_thread_suspend(usb_tid);
}

void USBAPP_DestroyThread()
{
	#if USE_EVT
	rt_event_control(&usb_isr_event,RT_IPC_CMD_RESET,0);
	rt_event_detach(&usb_isr_event);
	#else
	rt_sem_delete(usb_sem);
	#endif
	rt_thread_delete(usb_tid);
}


int usb_msc(void)
{
	uint32_t usbd_mode = 0;
	usbd_mode = USBD_MODE_CDC_MSC;
#ifdef USB_CONSOLE_CDC_EN
	usbd_mode |= USBD_MODE_DBG_CDC;
#endif	
	
#ifdef NXP_USING_OPENMV	
	pyb_usb_dev_init(USBD_VID, USBD_PID_CDC_MSC, usbd_mode, NULL);
#else
	USBD_SetVIDPIDRelease(USBD_VID, USBD_PID_CDC_MSC, 0x0200, 0);
	if (USBD_SelectMode(usbd_mode, NULL) < 0) {
		return 0;
	}
	USBAPP_Init();
#endif	
	return 0;
}
INIT_APP_EXPORT(usb_msc);

usb_osa_status_t OSA_MutexCreate(usb_osa_mutex_handle handle)
{
	return kStatus_USB_OSA_Success;
}

usb_osa_status_t OsaMutexDestroy(usb_osa_mutex_handle handle)
{
    return kStatus_USB_OSA_Success;
}
usb_osa_status_t OsaMutexLock(usb_osa_mutex_handle handle)
{
    return kStatus_USB_OSA_Success;
}
usb_osa_status_t OsaMutexUnlock(usb_osa_mutex_handle handle)
{
    return kStatus_USB_OSA_Success;
}
