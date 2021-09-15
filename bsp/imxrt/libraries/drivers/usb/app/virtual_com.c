/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
//#include "hal_wrapper.h"
#include <stdio.h>
#include <stdlib.h>

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"
//#include "fsl_debug_console.h"
#include "ring_buffer.h"
#include "usb_device_descriptor.h"
#include "composite.h"
#ifdef NXP_USING_MICROPYTHON
#include "py/mpconfig.h"
//#include "mpconfigport.h"
#include "lib/utils/interrupt_char.h"
#ifdef NXP_USING_OPENMV
#include "pendsv.h"
#endif
#include "py/misc.h"
#endif
#include "dfs_file.h"
#include <dfs_posix.h>
#include <dfs_poll.h>
#ifdef RT_USING_POSIX_TERMIOS
#include <posix_termios.h>
#endif

/* it's possible the 'getc/putc' is defined by stdio.h in gcc/newlib. */
#ifdef getc
#undef getc
#endif

#ifdef putc
#undef putc
#endif
// Most variables here are shared with USB DMA, strongly recommend to put ALL data/bss
// in this file to NON-CACHABLE regions!!!

/*******************************************************************************
* Definitions
******************************************************************************/

/*******************************************************************************
* Variables
******************************************************************************/
extern usb_device_endpoint_struct_t g_cdcVcomDicEndpoints[];
uint8_t g_isVcpOpen;
volatile uint8_t g_isUsbHostOpen;
#undef usb_echo
#define usb_echo(...) rt_kprintf(__VA_ARGS__)
/* Line codinig of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_lineCoding[USB_DEVICE_CONFIG_CDC_ACM][LINE_CODING_SIZE] = {
	{
		/* E.g. 0x00,0xC2,0x01,0x00 : 0x0001C200 is 115200 bits per second */
		(LINE_CODING_DTERATE >> 0U) & 0x000000FFU,
		(LINE_CODING_DTERATE >> 8U) & 0x000000FFU,
		(LINE_CODING_DTERATE >> 16U) & 0x000000FFU,
		(LINE_CODING_DTERATE >> 24U) & 0x000000FFU,
		LINE_CODING_CHARFORMAT,
		LINE_CODING_PARITYTYPE,
		LINE_CODING_DATABITS
	},
#if USB_DEVICE_CONFIG_CDC_ACM > 1	
	{
		/* E.g. 0x00,0xC2,0x01,0x00 : 0x0001C200 is 115200 bits per second */
		(LINE_CODING_DTERATE >> 0U) & 0x000000FFU,
		(LINE_CODING_DTERATE >> 8U) & 0x000000FFU,
		(LINE_CODING_DTERATE >> 16U) & 0x000000FFU,
		(LINE_CODING_DTERATE >> 24U) & 0x000000FFU,
		LINE_CODING_CHARFORMAT,
		LINE_CODING_PARITYTYPE,
		LINE_CODING_DATABITS
	},
#endif	
};

/* Abstract state of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_abstractState[USB_DEVICE_CONFIG_CDC_ACM][COMM_FEATURE_DATA_SIZE] = {
    {(STATUS_ABSTRACT_STATE >> 0U) & 0x00FFU, (STATUS_ABSTRACT_STATE >> 8U) & 0x00FFU},
#if USB_DEVICE_CONFIG_CDC_ACM > 1
	{(STATUS_ABSTRACT_STATE >> 0U) & 0x00FFU, (STATUS_ABSTRACT_STATE >> 8U) & 0x00FFU},
#endif	
};
/* Country code of cdc device */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_countryCode[USB_DEVICE_CONFIG_CDC_ACM][COMM_FEATURE_DATA_SIZE] = {
    {(COUNTRY_SETTING >> 0U) & 0x00FFU, (COUNTRY_SETTING >> 8U) & 0x00FFU},
#if USB_DEVICE_CONFIG_CDC_ACM > 1	
    {(COUNTRY_SETTING >> 0U) & 0x00FFU, (COUNTRY_SETTING >> 8U) & 0x00FFU},
#endif	
};
/* CDC ACM information */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static usb_cdc_acm_info_t s_usbCdcAcmInfo[USB_DEVICE_CONFIG_CDC_ACM] = {
    {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0, 0}, 
#if USB_DEVICE_CONFIG_CDC_ACM > 1
	{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0, 0},
#endif	
};
/* Data buffer for receiving and sending*/

#if VCP_RINGBLK_SIZE % 32 != 0
#error "buffer size must be multiples of 32!"
#endif

#if USB_DATA_ALIGN_SIZE < 32
#error "wrong USB align size"
#endif

#define USB_ALIGN	USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
#if VCP_RINGBLK_SIZE > HS_CDC_VCOM_BULK_OUT_PACKET_SIZE || VCP_RING_BLK_SIZE > HS_CDC_VCOM_BULK_IN_PACKET_SIZE
#error Ring block size must NOT exceed endpoint's max packet size!
#endif
#ifdef USB_CONSOLE_CDC_EN
#define UEBUG_CDC_IDX	(USB_DEVICE_CDC_COUNT-1)
rt_device_t	old_debug_device = NULL;
#endif

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)  static uint8_t s_RecvBuf[VCP_INEPBUF_CNT][VCP_RINGBLK_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)  static uint8_t s_SendBuf[VCP_OUTEPBUF_CNT][VCP_RINGBLK_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)  static uint8_t s_RecvBuf_2[VCP_INEPBUF_CNT][VCP_RINGBLK_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)  static uint8_t s_SendBuf_2[VCP_OUTEPBUF_CNT][VCP_RINGBLK_SIZE];
ring_block_t s_txRB[USB_DEVICE_CONFIG_CDC_ACM], s_rxRB[USB_DEVICE_CONFIG_CDC_ACM];
uint8_t *s_pCurTxBuf[USB_DEVICE_CONFIG_CDC_ACM], *s_pCurRxBuf[USB_DEVICE_CONFIG_CDC_ACM];
uint8_t s_isTxIdle;
volatile uint8_t s_isRxOverrun;
volatile static usb_device_composite_struct_t *g_deviceComposite;
#ifdef NXP_USING_OPENMV	
// >>> openMV IDE related 
#include "usbdbg.h"
#endif

#ifndef NXP_USING_MICROPYTHON
static int mp_interrupt_char = -1;
#endif
/*static*/ __IO uint8_t dev_is_connected = 0; // indicates if we are connected
/*static*/ __IO uint8_t debug_mode = 0; 
/*static*/ __IO uint32_t baudrate = 0;
/*static*/ uint32_t dbg_xfer_length=0;
__IO uint8_t omv_com_idx = 0;
__IO uint8_t omv_start = 0;
#define ALIGN32 __ALIGNED(32)

ring_block_t s_omvRB;
USB_ALIGN static uint8_t s_omvTxBuf[4][VCP_RINGBLK_SIZE];

#define IDE_BAUDRATE_SLOW    (921600)
#define IDE_BAUDRATE_FAST    (12000000)
// <<<


/*******************************************************************************
* Code
******************************************************************************/

usb_status_t _Start_USB_VCOM_Write(class_handle_t handle)
{
	usb_status_t error = kStatus_USB_Error;
	uint32_t len;
	uint8_t txIdleBkup;
	uint32_t primask = __get_PRIMASK();
	__set_PRIMASK(1);	// must disable IRQ, otherwise a buffer can be sent 2 times
	/* User: add your own code for send complete event */
	int i = 0;
	for (i = 0; i < USB_DEVICE_CONFIG_CDC_ACM; i++)
    {
        if (handle == g_deviceComposite->cdcVcom[i].cdcAcmHandle)
        {
            break;
        }
    }
    if (i >= USB_DEVICE_CONFIG_CDC_ACM)
    {
        return error;
    }
	
	len = RingBlk_GetOldestBlk(&s_txRB[i], &s_pCurTxBuf[i]); 
	if (len > 0) {
		txIdleBkup = s_isTxIdle;
		s_isTxIdle = 0;
		error = USB_DeviceCdcAcmSend(handle, g_deviceComposite->cdcVcom[i].DicbulkInEndpoint, s_pCurTxBuf[i], len);
		if (error != kStatus_USB_Success) {
			s_isTxIdle = txIdleBkup;
		} else
			RingBlk_FreeOldestBlk(&s_txRB[i], 0);	// no longer allow to continue to append data on this block
	}
	__set_PRIMASK(primask);
	return error;
}

static uint32_t last_packet = 0;

#define DBG_MAX_PACKET  (VCP_RINGBLK_SIZE)
__attribute__((aligned(4))) static uint8_t dbg_xfer_buffer[DBG_MAX_PACKET] ;


__WEAK void usbdbg_data_in(void *buffer, int length){}
__WEAK void usbdbg_data_out(void * buffer, int length){}
__WEAK void usbdbg_control(void *buffer, uint8_t request, uint32_t length){}



static void send_packet(void) {
    int bytes = MIN(dbg_xfer_length, VCP_RINGBLK_SIZE);
    last_packet = bytes;
    usbdbg_data_in(dbg_xfer_buffer, bytes);
    dbg_xfer_length -= bytes;
	VCOM_Write(dbg_xfer_buffer, bytes);
}


uint32_t usbd_cdc_tx_buf_len(void) {
	return 0;
	// return RingBlk_GetFreeBytes(&s_txRB);
}
uint8_t *usbd_cdc_tx_buf(uint32_t bytes)
{
	return (uint8_t*) &s_rxRB;
}


#ifdef USB_CONSOLE_CDC_EN
static uint8_t vcom_attach = false;

rt_size_t usbd_cdc_debug_vcom_read(struct rt_device *dev,
                                rt_off_t          pos,
                                void             *buffer,
                                rt_size_t         size)
{
	
	if (vcom_attach == false)
		return 0;

	return RingBlk_Read1Blk(&s_rxRB[UEBUG_CDC_IDX], buffer, VCP_RINGBLK_SIZE);
}

rt_size_t usbd_cdc_debug_vcom_write(struct rt_device *dev,
                                 rt_off_t          pos,
                                 const void       *buffer,
                                 rt_size_t         size)
{
	char char_buffer[256] = {0};
	if(size > 0)
	{
		int i;
		int retry = 0;
		int str_len=0;
		for(i=0;i<size;i++)
		{
			if(((uint8_t*)buffer)[i] == 0x0a)
			{
				char_buffer[str_len++] = '\r';
				char_buffer[str_len++] = '\n';
			}
			else
			{
				char_buffer[str_len++] = ((uint8_t*)buffer)[i];
			}
		}
		
		for (i = 0; i < str_len; ) {
			while (RingBlk_GetFreeBytes(&s_txRB[UEBUG_CDC_IDX]) == 0) {
				//HAL_WFI();
				if (retry++ >= 10/*100*/) {
					goto cleanup;
				}
			}
			
			i += RingBlk_Write(&s_txRB[UEBUG_CDC_IDX], (const uint8_t*)char_buffer + i, str_len - i);
		}
cleanup:		
		if (vcom_attach && RingBlk_GetUsedBytes(&s_txRB[UEBUG_CDC_IDX]) && g_deviceComposite->cdcVcom[UEBUG_CDC_IDX].attach) {
			_Start_USB_VCOM_Write(g_deviceComposite->cdcVcom[UEBUG_CDC_IDX].cdcAcmHandle);
		}
	}
	
	return size;
}

static rt_err_t usbd_cdc_debug_vcom_open(struct rt_device *dev, rt_uint16_t oflag)
{
	RingBlk_Init(&s_txRB[UEBUG_CDC_IDX], s_SendBuf_2[0], VCP_RINGBLK_SIZE, VCP_OUTEPBUF_CNT);
	RingBlk_Init(&s_rxRB[UEBUG_CDC_IDX], s_RecvBuf_2[0], VCP_RINGBLK_SIZE, VCP_INEPBUF_CNT);
	return RT_EOK;
}

static rt_err_t vcom_serial_fops_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_wqueue_wakeup(&(dev->wait_queue), (void*)POLLIN);

    return RT_EOK;
}

/* fops for serial */
static int vcom_serial_fops_open(struct dfs_fd *fd)
{
    rt_err_t ret = 0;
    rt_uint16_t flags = 0;
    rt_device_t device;

    device = (rt_device_t)fd->data;
    RT_ASSERT(device != RT_NULL);

    switch (fd->flags & O_ACCMODE)
    {
    case O_RDONLY:
        usb_echo("fops open: O_RDONLY!");
        flags = RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_RDONLY;
        break;
    case O_WRONLY:
        usb_echo("fops open: O_WRONLY!");
        flags = RT_DEVICE_FLAG_WRONLY;
        break;
    case O_RDWR:
        usb_echo("fops open: O_RDWR!");
        flags = RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_RDWR;
        break;
    default:
        usb_echo("fops open: unknown mode - %d!", fd->flags & O_ACCMODE);
        break;
    }

    if ((fd->flags & O_ACCMODE) != O_WRONLY)
        rt_device_set_rx_indicate(device, vcom_serial_fops_rx_ind);
    ret = rt_device_open(device, flags);
    if (ret == RT_EOK) return 0;

    return ret;
}

static int vcom_serial_fops_close(struct dfs_fd *fd)
{
    rt_device_t device;

    device = (rt_device_t)fd->data;

    rt_device_set_rx_indicate(device, RT_NULL);
    rt_device_close(device);

    return 0;
}

static int vcom_serial_fops_ioctl(struct dfs_fd *fd, int cmd, void *args)
{
    return 0;
}

static int vcom_serial_fops_read(struct dfs_fd *fd, void *buf, size_t count)
{
    int size = 0;
    rt_device_t device;

    device = (rt_device_t)fd->data;

    do
    {
        size = rt_device_read(device, -1,  buf, count);
        if (size <= 0)
        {
            if (fd->flags & O_NONBLOCK)
            {
                size = -EAGAIN;
                break;
            }

            rt_wqueue_wait(&(device->wait_queue), 0, RT_WAITING_FOREVER);
        }
    }while (size <= 0);

    return size;
}

static int vcom_serial_fops_write(struct dfs_fd *fd, const void *buf, size_t count)
{
    rt_device_t device;

    device = (rt_device_t)fd->data;
    return rt_device_write(device, -1, buf, count);
}

static int vcom_serial_fops_poll(struct dfs_fd *fd, struct rt_pollreq *req)
{
    int mask = 0;
    int flags = 0;
    rt_device_t device;
    struct rt_serial_device *serial;

    device = (rt_device_t)fd->data;
    RT_ASSERT(device != RT_NULL);

    serial = (struct rt_serial_device *)device;

    /* only support POLLIN */
    flags = fd->flags & O_ACCMODE;
    if (flags == O_RDONLY || flags == O_RDWR)
    {
        rt_base_t level;
        struct rt_serial_rx_fifo* rx_fifo;

        rt_poll_add(&(device->wait_queue), req);

        rx_fifo = (struct rt_serial_rx_fifo*) serial->serial_rx;

        level = rt_hw_interrupt_disable();
        if ((rx_fifo->get_index != rx_fifo->put_index) || (rx_fifo->get_index == rx_fifo->put_index && rx_fifo->is_full == RT_TRUE))
            mask |= POLLIN;
        rt_hw_interrupt_enable(level);
    }

    return mask;
}

const static struct dfs_file_ops _vcom_serial_fops =
{
    vcom_serial_fops_open,
    vcom_serial_fops_close,
    vcom_serial_fops_ioctl,
    vcom_serial_fops_read,
    vcom_serial_fops_write,
    RT_NULL, /* flush */
    RT_NULL, /* lseek */
    RT_NULL, /* getdents */
    vcom_serial_fops_poll,
};


struct rt_device dbg_vcom_dev = {
	.parent = NULL,
	.type = RT_Device_Class_Char,
	
	.open = usbd_cdc_debug_vcom_open,
	.read = usbd_cdc_debug_vcom_read,
	.write = usbd_cdc_debug_vcom_write,
	
	.fops = &_vcom_serial_fops,
};

void usbd_cdc_vcom_rx_ind()
{
	dbg_vcom_dev.rx_indicate(&dbg_vcom_dev,1);
}

int usbd_cdc_vcom_device_init(void)
{
	rt_device_register(&dbg_vcom_dev,RT_CONSOLE_CDC_DEVICE_NAME,0);
	dbg_vcom_dev.fops = &_vcom_serial_fops;
	return 0;
}

INIT_BOARD_EXPORT(usbd_cdc_vcom_device_init);

void usbd_create_cdc_debug_device(uint32_t rate)
{
	if(!vcom_attach)
	{
		vcom_attach = true;
		//usb_echo("connect to vcom debug port\r\n");
		if ((RingBlk_GetUsedBytes(&s_txRB[UEBUG_CDC_IDX]) > 0) && (g_deviceComposite->cdcVcom[UEBUG_CDC_IDX].attach))
		{
			_Start_USB_VCOM_Write(g_deviceComposite->cdcVcom[UEBUG_CDC_IDX].cdcAcmHandle);
		}
	}
}

void usbd_debug_cdc_remove()
{
	vcom_attach = false;
	
	usb_echo("connect to uart debug port\r\n");
}

#endif
void CheckVCOMConnect(int idx) {
#ifdef USB_CONSOLE_CDC_EN	
	if(idx == UEBUG_CDC_IDX)
	{
		baudrate = *((uint32_t*)s_lineCoding[idx]);
		usbd_create_cdc_debug_device(baudrate);
		return;
	}
	
#endif	
	if((debug_mode == 1) && (idx != omv_com_idx))
		return;
	baudrate = *((uint32_t*)s_lineCoding[idx]);
	// The slow baudrate can be used on OSs that don't support custom baudrates
	if (baudrate == IDE_BAUDRATE_SLOW || baudrate == IDE_BAUDRATE_FAST) {
		debug_mode = 1;
		RingBlk_Init(&s_omvRB, s_omvTxBuf[0], VCP_RINGBLK_SIZE, sizeof(s_omvTxBuf) / VCP_RINGBLK_SIZE);
		g_isUsbHostOpen = 1;
		dbg_xfer_length = 0;
		omv_com_idx = idx;
	#ifdef NXP_USING_OPENMV	
		
		usbdbg_connect();
	#endif	
		// UserTxBufPtrIn = UserTxBufPtrOut = UserTxBufPtrOutShadow = 0;
	} else {
		debug_mode = 0;
		// UserTxBufPtrIn = UserTxBufPtrOut = UserTxBufPtrOutShadow = 0;
	}	
}

/*!
 * @brief CDC class specific callback function.
 *
 * This function handles the CDC class specific requests.
 *
 * @param handle          The CDC ACM class handle.
 * @param event           The CDC ACM class event type.
 * @param param           The parameter of the class specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
 /*static*/ uint8_t s_omvSendIsToContinue;
// #define HSRX
#ifdef HSRX

USB_ALIGN static uint8_t s_hsRx[VCP_RINGBLK_SIZE];
#endif

usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint32_t len;
    uint8_t *uartBitmap;
    usb_cdc_acm_info_t *acmInfo;
    usb_device_cdc_acm_request_param_struct_t *acmReqParam;
    usb_device_endpoint_callback_message_struct_t *epCbParam;
    acmReqParam = (usb_device_cdc_acm_request_param_struct_t *)param;
    epCbParam = (usb_device_endpoint_callback_message_struct_t *)param;
	volatile usb_cdc_vcom_struct_t *vcomInstance;
	int i;
	for (i = 0; i < USB_DEVICE_CONFIG_CDC_ACM; i++)
    {
        if (handle == g_deviceComposite->cdcVcom[i].cdcAcmHandle)
        {
            break;
        }
    }
    if (i >= USB_DEVICE_CONFIG_CDC_ACM)
    {
        return error;
    }
    vcomInstance = &g_deviceComposite->cdcVcom[i];
    acmInfo = vcomInstance->usbCdcAcmInfo;
	
	/* test irq latency impact to other IRQs, shows it has no effect to higher priority irqs
	volatile uint32_t delay;
	for (delay=0; delay<10000; delay++) {}
	*/
    switch (event)
    {
        case kUSB_DeviceCdcEventSendResponse:
        {
            if ((epCbParam->length != 0) && (!(epCbParam->length % g_cdcVcomDicEndpoints[0].maxPacketSize)))
            {
                /* If the last packet is the size of endpoint, then send also zero-ended packet,
                 ** meaning that we want to inform the host that we do not have any additional
                 ** data, so it can flush the output.
                 */
                error = USB_DeviceCdcAcmSend(handle, g_deviceComposite->cdcVcom[i].DicbulkInEndpoint, NULL, 0);
            }
            else if ((1 == vcomInstance->attach) && (1 == vcomInstance->startTransactions))
            {
                if ((epCbParam->buffer != NULL) || ((epCbParam->buffer == NULL) && (epCbParam->length == 0)))
                {
                    len = RingBlk_GetOldestBlk(&s_txRB[i], &s_pCurTxBuf[i]); // now that oldest block has been sent, free it and get next oldest block
                    if (len > 0) {
						error = USB_DeviceCdcAcmSend(handle, g_deviceComposite->cdcVcom[i].DicbulkInEndpoint, s_pCurTxBuf[i], len);
						if (error == kStatus_USB_Success)
							RingBlk_FreeOldestBlk(&s_txRB[i], 0);
						else {
							usb_echo("Failed to send in RingBlk_GetOldestBlk()\r\n");
						}
					} else
						s_isTxIdle = 1;
				} else {
                	s_isTxIdle = 1;	// all TX data is sent in this round
				}
            }
			// >>> openMV data resume
			if (s_omvSendIsToContinue) {
				if (dbg_xfer_length) {
					send_packet(); //prime tx buffer
				} else {
					s_omvSendIsToContinue = 0;
				}
			}
			// <<<
        }
        break;
        case kUSB_DeviceCdcEventRecvResponse:
        {
            if ((1 == vcomInstance->attach) && (1 == vcomInstance->startTransactions))
            {	g_isUsbHostOpen = 1;
				if ((i != omv_com_idx) && (epCbParam->length != (uint32_t)-1L))
				{
					RingBlk_FixBlkFillCnt(&s_rxRB[i], epCbParam->length, &s_pCurRxBuf[i]);
					if (s_pCurRxBuf[i]){
						error = USB_DeviceCdcAcmRecv(handle, vcomInstance->DicbulkOutEndpoint, s_pCurRxBuf[i], VCP_RINGBLK_SIZE);
					}
					#ifdef USB_CONSOLE_CDC_EN
					usbd_cdc_vcom_rx_ind();
					#endif
				}
				else if (epCbParam->length != (uint32_t)-1L) {
					if (debug_mode == 0) 
					{
						if (mp_interrupt_char != -1 && epCbParam->length == 1 && 
						s_rxRB[i].pBlks[s_rxRB[i].wNdx * s_rxRB[i].blkSize] == mp_interrupt_char)
						{
						#ifdef NXP_USING_OPENMV	
							pendsv_kbd_intr();
						#endif	
							RingBlk_ReuseTakenBlk(&s_rxRB[i], &s_pCurRxBuf[i]);
						} else if (epCbParam->length == 0) {
                            RingBlk_ReuseTakenBlk(&s_rxRB[i], &s_pCurRxBuf[i]);
                        }
                        else
                        {
							RingBlk_FixBlkFillCnt(&s_rxRB[i], epCbParam->length, &s_pCurRxBuf[i]);
						}
						// provide USBD IP to receive next buffer
						if (s_pCurRxBuf[i])
							error = USB_DeviceCdcAcmRecv(handle, vcomInstance->DicbulkOutEndpoint, s_pCurRxBuf[i], VCP_RINGBLK_SIZE);
						else {
                            s_isRxOverrun = 1;
							// usb_echo("VCOM receive buffer is overrun!\r\n");
						}
					} else {
						uint8_t Buf[VCP_RINGBLK_SIZE];
						uint32_t bytes;
					
					#ifdef HSRX
						bytes = epCbParam->length;
						memcpy(Buf, s_hsRx, bytes);
						
					#else
						// vcom conected to openMV IDE
						RingBlk_FixBlkFillCnt(&s_rxRB[i], epCbParam->length, &s_pCurRxBuf[i]);
						// check if there is keyboard IRQ
						// provide USBD IP to receive next buffer
						bytes = RingBlk_Read1Blk(&s_rxRB[i], Buf, sizeof(Buf));
					#endif
						
						if((i == omv_com_idx) && omv_start)
						{//omv ide com
							if (0 == bytes)
								printf("no data read!\r\n");
							if (dbg_xfer_length) {
								usbdbg_data_out(Buf, bytes);
								dbg_xfer_length -= bytes;
							} else if (Buf[0] == '\x30') { // command
								uint8_t request = Buf[1];
								dbg_xfer_length = *((uint32_t*)(Buf+2));
								usbdbg_control(Buf+6, request, dbg_xfer_length);
								if (dbg_xfer_length && (request & 0x80)) { //request has a device-to-host data phase
									send_packet(); //prime tx buffer
									if (dbg_xfer_length)
										s_omvSendIsToContinue = 1;
								}
							}
						}
						
						
					#ifdef HSRX
						error = USB_DeviceCdcAcmRecv(handle, g_cfgFix.roCdcDicEpOutNdx, s_hsRx, VCP_RINGBLK_SIZE);	
					#else 
						if (s_pCurRxBuf[i])
							error = USB_DeviceCdcAcmRecv(handle, vcomInstance->DicbulkOutEndpoint, s_pCurRxBuf[i], VCP_RINGBLK_SIZE);		
					#endif	
					}

				}
            }
        }
        break;
        case kUSB_DeviceCdcEventSerialStateNotif:
            ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 0;
            error = kStatus_USB_Success;
			if (s_isTxIdle) {
				_Start_USB_VCOM_Write(vcomInstance->cdcAcmHandle);
//				uint32_t cbFill;
//				cbFill = RingBlk_GetOldestBlk(&s_txRB, &s_pCurTxBuf);
//				if (cbFill) {
//					USB_DeviceCdcAcmSend(g_deviceComposite->cdcVcom.cdcAcmHandle, g_cfgFix.roCdcDicEpInNdx, s_pCurTxBuf, cbFill);
//				}
			}
            break;
        case kUSB_DeviceCdcEventSendEncapsulatedCommand:
            break;
        case kUSB_DeviceCdcEventGetEncapsulatedResponse:
            break;
        case kUSB_DeviceCdcEventSetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                if (1 == acmReqParam->isSetup)
                {
                    *(acmReqParam->buffer) = s_abstractState[i];
                }
                else
                {
                    *(acmReqParam->length) = 0;
                }
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                if (1 == acmReqParam->isSetup)
                {
                    *(acmReqParam->buffer) = s_countryCode[i];
                }
                else
                {
                    *(acmReqParam->length) = 0;
                }
            }
            else
            {
            }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventGetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_abstractState[i];
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_countryCode[i];
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
            }
            else
            {
            }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventClearCommFeature:
            break;
        case kUSB_DeviceCdcEventGetLineCoding:
			g_isUsbHostOpen = 1;
            *(acmReqParam->buffer) = s_lineCoding[i];
            *(acmReqParam->length) = LINE_CODING_SIZE;
			CheckVCOMConnect(i);	
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSetLineCoding:
        {   
			g_isUsbHostOpen = 1;
            if (1 == acmReqParam->isSetup)
            {
                *(acmReqParam->buffer) = s_lineCoding[i];
				CheckVCOMConnect(i);			
            }
            else
            {
                *(acmReqParam->length) = 0;
            }
        }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSetControlLineState:
        {
			g_isUsbHostOpen = 0;
            vcomInstance->usbCdcAcmInfo->dteStatus = acmReqParam->setupValue;
            /* activate/deactivate Tx carrier */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }

            /* activate carrier and DTE */
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
				g_isUsbHostOpen = 1;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
				g_isUsbHostOpen = 0;
            }

            /* Indicates to DCE if DTE is present or not */
            acmInfo->dtePresent = (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) ? true : false;

            /* Initialize the serial state buffer */
            acmInfo->serialStateBuf[0] = NOTIF_REQUEST_TYPE;                /* bmRequestType */
            acmInfo->serialStateBuf[1] = USB_DEVICE_CDC_NOTIF_SERIAL_STATE; /* bNotification */
            acmInfo->serialStateBuf[2] = 0x00;                              /* wValue */
            acmInfo->serialStateBuf[3] = 0x00;
            acmInfo->serialStateBuf[4] = 0x00; /* wIndex */
            acmInfo->serialStateBuf[5] = 0x00;
            acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE; /* wLength */
            acmInfo->serialStateBuf[7] = 0x00;
            /* Notifiy to host the line state */
            acmInfo->serialStateBuf[4] = acmReqParam->interfaceIndex;
            /* Lower byte of UART BITMAP */
            uartBitmap    = (uint8_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
            uartBitmap[0] = acmInfo->uartState & 0xFFu;
            uartBitmap[1] = (acmInfo->uartState >> 8) & 0xFFu;
            len           = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
            if (0 == ((usb_device_cdc_acm_struct_t *)handle)->hasSentState)
            {
                error = USB_DeviceCdcAcmSend(handle, vcomInstance->CicbulkInEndpoint, acmInfo->serialStateBuf, len);
                if (kStatus_USB_Success != error)
                {
                    usb_echo("kUSB_DeviceCdcEventSetControlLineState error!");
                }
                ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 1;
            }

            /* Update status */
			volatile uint32_t abc;
            if ((acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION))
            {
				if(i == omv_com_idx)
				{
					if (omv_start)
					{
						#ifdef NXP_USING_OPENMV	
						/*  To do: CARRIER_ACTIVATED */
						usbdbg_disconnect();
						#endif
						g_isUsbHostOpen = 0;
						debug_mode = 0;
					}
				}
				else
				{
				#ifdef USB_CONSOLE_CDC_EN	
					usbd_debug_cdc_remove();
				#endif	
				}
            }
            else
            {
                /* To do: CARRIER_DEACTIVATED */
				abc = 0;
            }

			vcomInstance->startTransactions = vcomInstance->attach;
			#if 0
			// >>> currently we don't care DTE
			if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
            {
                /* DTE_ACTIVATED */
                if (1 == g_deviceComposite->cdcVcom.attach)
                {
                    g_deviceComposite->cdcVcom.startTransactions = 1;
                }
            }
            else
            {
                /* DTE_DEACTIVATED */
                if (1 == g_deviceComposite->cdcVcom.attach)
                {
                    g_deviceComposite->cdcVcom.startTransactions = 0;
                }
            }
			// <<<
			#endif
        }
        break;
        case kUSB_DeviceCdcEventSendBreak:
            break;
        default:
            break;
    }

    return error;
}

int StartOrResumeRecv(int idx)
{
    #ifdef HSRX
    USB_DeviceCdcAcmRecv(g_deviceComposite->cdcVcom.cdcAcmHandle, g_cfgFix.roCdcDicEpOutNdx, s_hsRx, 64);
    #else
    /* Schedule buffer to receive */
    s_pCurRxBuf[idx] = RingBlk_GetTakenBlk(&s_rxRB[idx]);
    if (0 == s_pCurRxBuf[idx])
        s_pCurRxBuf[idx] = RingBlk_TakeNextFreeBlk(&s_rxRB[idx]);
    USB_DeviceCdcAcmRecv(g_deviceComposite->cdcVcom[idx].cdcAcmHandle,g_deviceComposite->cdcVcom[idx].DicbulkOutEndpoint,s_pCurRxBuf[idx], VCP_RINGBLK_SIZE);
    #endif
	return 0;
}

/*!
 * @brief Virtual COM device set configuration function.
 *
 * This function sets configuration for CDC class.
 *
 * @param handle The CDC ACM class handle.
 * @param configure The CDC ACM class configure index.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcVcomSetConfigure(class_handle_t handle, uint8_t configure)
{
    if (USB_COMPOSITE_CONFIGURE_INDEX == configure)
    {
        g_deviceComposite->cdcVcom[0].attach = 1;
		g_deviceComposite->cdcVcom[0].DicbulkInEndpoint = g_cfgFix.roCdcDicEpInNdx;
		g_deviceComposite->cdcVcom[0].DicbulkOutEndpoint = g_cfgFix.roCdcDicEpOutNdx;
		g_deviceComposite->cdcVcom[0].CicbulkInEndpoint = g_cfgFix.roCdcCicEpNdx;
		StartOrResumeRecv(0);
#if USB_DEVICE_CONFIG_CDC_ACM > 1			
		g_deviceComposite->cdcVcom[1].attach = 1;
		g_deviceComposite->cdcVcom[1].DicbulkInEndpoint = g_cfgFix.roCdcDicEpInNdx_2;
		g_deviceComposite->cdcVcom[1].DicbulkOutEndpoint = g_cfgFix.roCdcDicEpOutNdx_2;
		g_deviceComposite->cdcVcom[1].CicbulkInEndpoint = g_cfgFix.roCdcCicEpNdx_2;
		StartOrResumeRecv(1);
#endif		
		/* Schedule buffer to send */
//		if (s_isTxIdle) {
//		uint32_t cbFill;
//			cbFill = RingBlk_GetOldestBlk(&s_txRB, &s_pCurTxBuf);
//			if (cbFill) {
//				USB_DeviceCdcAcmSend(g_deviceComposite->cdcVcom.cdcAcmHandle, g_cfgFix.roCdcDicEpOutNdx, s_pCurTxBuf, cbFill);
//			}
//		}
		
    }
    return kStatus_USB_Success;
}

/*!
 * @brief Virtual COM device initialization function.
 *
 * This function initializes the device with the composite device class information.
 *
 * @param deviceComposite The pointer to the composite device structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCdcVcomInit(usb_device_composite_struct_t *deviceComposite)
{
    g_deviceComposite = deviceComposite;
	RingBlk_Init(&s_txRB[0], s_SendBuf[0], VCP_RINGBLK_SIZE, VCP_OUTEPBUF_CNT);
	RingBlk_Init(&s_rxRB[0], s_RecvBuf[0], VCP_RINGBLK_SIZE, VCP_INEPBUF_CNT);
#if USB_DEVICE_CONFIG_CDC_ACM >1	
	RingBlk_Init(&s_txRB[1], s_SendBuf_2[0], VCP_RINGBLK_SIZE, VCP_OUTEPBUF_CNT);
	RingBlk_Init(&s_rxRB[1], s_RecvBuf_2[0], VCP_RINGBLK_SIZE, VCP_INEPBUF_CNT);
#endif	
	for (uint8_t i = 0; i < USB_DEVICE_CONFIG_CDC_ACM; i++)
    {
        g_deviceComposite->cdcVcom[i].usbCdcAcmInfo = &s_usbCdcAcmInfo[i];
 
    }
	s_isTxIdle = 1;
    return kStatus_USB_Success;
}

extern uint8_t g_isVcpOpen;
uint32_t VCOM_RxBufGetFilledBytes(void) {
	return RingBlk_GetUsedBytes(&s_rxRB[omv_com_idx]);
}

int USBD_CDC_TxHalfEmpty(void) 
{
	return (RingBlk_GetFreeBlks(&s_txRB[omv_com_idx]) >= RingBlk_GetUsedBlks(&s_txRB[omv_com_idx]));
}


void VCOM_Open(void) {
	g_isVcpOpen = 1;
}

void VCOM_Close(void) {
	g_isVcpOpen = 0;
}

int VCOM_Read(uint8_t *buf, uint32_t len, uint32_t timeout) 
{
    // loop to read bytes
    int cbRead = 0;
	if (!g_isVcpOpen)
		return 0;
    while (cbRead < len) 
	{
        // Wait until we have at least 1 byte to read
        uint32_t start = rt_tick_get();
        while (RingBlk_GetUsedBytes(&s_rxRB[omv_com_idx]) == 0) {
            // Wraparound of tick is taken care of by 2's complement arithmetic.
            if (0 == timeout || rt_tick_get() - start >= timeout) {
                // timeout
                return cbRead;
            }
            if (__get_PRIMASK()) {
                // IRQs disabled so buffer will never be filled; return immediately
                return cbRead;
            }
			//HAL_WFI();
        }

        // Copy byte from device to user buffer
        cbRead += RingBlk_Read(&s_rxRB[omv_com_idx], buf, len - cbRead);
	}
 		if (s_isRxOverrun) {
        if (RingBlk_GetFreeBlks(&s_rxRB[omv_com_idx]) != 0) {
                s_isRxOverrun = 0;
                StartOrResumeRecv(omv_com_idx);            
        }
    }
	return cbRead;
}

volatile uint8_t s_isByPassWriteAlways;
void VCOM_WriteAlways(const uint8_t *buf, uint32_t len) {
	if (s_isByPassWriteAlways) {
		VCOM_Write(buf, len);
		return;
	}
	if (0 == len)
		goto cleanup;	
	int i;
	int retry = 0;
	// while (!g_isUsbHostOpen) {}
	if (!g_isUsbHostOpen)
		return;
    for (i = 0; i < len; ) {
		while (RingBlk_GetFreeBytes(&s_txRB[omv_com_idx]) == 0) {
			//HAL_WFI();
			if (retry++ >= 10) {
				s_isTxIdle = 1; // it seems some bug prevent from restoring s_isTxIdle
				goto cleanup;
			}
		}
		i += RingBlk_Write(&s_txRB[omv_com_idx], buf + i, len - i);
    }
	if (s_isTxIdle && g_deviceComposite->cdcVcom[omv_com_idx].attach) {
		_Start_USB_VCOM_Write(g_deviceComposite->cdcVcom[omv_com_idx].cdcAcmHandle);
	}
cleanup:
	return;
}

int VCOM_Write(const uint8_t *buf, uint32_t len) {
	int ret = 0;
	if (!g_isUsbHostOpen || len == 0)
		goto cleanup;
	ret = RingBlk_Write(&s_txRB[omv_com_idx], buf, len);
	if (ret && s_isTxIdle && g_deviceComposite->cdcVcom[omv_com_idx].attach) {
		_Start_USB_VCOM_Write(g_deviceComposite->cdcVcom[omv_com_idx].cdcAcmHandle);
	}
cleanup:
	return ret;
}

bool VCOM_IsTxIdle(void) {
	return s_isTxIdle ? 1 : 0;
}

#ifdef NXP_USING_OPENMV
extern volatile uint8_t g_omvIdeConnecting;

void VCOM_Omv_Start(bool bStart)
{
	omv_start = bStart;
}

bool VCOM_OmvIsIdeConnecting(void) {
	return g_omvIdeConnecting;
}
bool VCOM_OmvIsIdeConnected(void)
{
	if (debug_mode)
		return 1;
	return 0;
}
#endif
void VCOM_OmvWriteAlways(const uint8_t *buf, uint32_t len) {
	int i;
	int retry = 0;
    for (i = 0; i < len; ) {
		while (RingBlk_GetFreeBytes(&s_omvRB) == 0) {
			//HAL_WFI();
			if (retry++ >= 10/*100*/) {
				goto cleanup;
			}
		}
		i += RingBlk_Write(&s_omvRB, buf + i, len - i);
    }
cleanup:
	return;
}

void VCOM_FlushTxBuffer(void) {
	RingBlk_Init(&s_txRB[omv_com_idx], s_SendBuf[0], VCP_RINGBLK_SIZE, VCP_OUTEPBUF_CNT);
}

uint32_t VCOM_OmvGetLogTxLen(void)
{
	return RingBlk_GetUsedBytes(&s_omvRB);
}
int VCOM_OmvReadLogTxBlk(uint8_t *pBuf, uint32_t bufSize) {
	int ret;
	ring_block_t rbBkup = s_omvRB;
tryread:
	ret = RingBlk_Read(&s_omvRB, pBuf, bufSize);
	if (ret < bufSize && rbBkup.cbTotUsed > ret) {
		s_omvRB = rbBkup;
		goto tryread;
	}
	return ret;
}

