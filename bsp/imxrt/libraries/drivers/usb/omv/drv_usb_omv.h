/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014, 2015 Damien P. George
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
#ifndef DRV_USB_OMV_H
#define DRV_USB_OMV_H
#include "usb_device_descriptor.h"
#define PYB_USB_FLAG_DEV_ENABLED        (0x0001)
#define PYB_USB_FLAG_USB_MODE_CALLED    (0x0002)

// Windows needs a different PID to distinguish different device configurations
// must use hex number directly to let python script that generate inf driver work normally
#define USBD_VID         (0x1209)
// nxp is (0x1FC9)
// (0x1400 | USBD_MODE_CDC | USBD_MODE_MSC)
#define USBD_PID_CDC_MSC (0xABD1)
// (0x1400 | USBD_MODE_CDC | USBD_MODE_HIDK | USBD_MODE_HIDM)
#define USBD_PID_CDC_HID (0x140d)
// (0x1400 | USBD_MODE_CDC)
#define USBD_PID_CDC     (0x1401)

typedef enum {
    PYB_USB_STORAGE_MEDIUM_NONE = 0,
    PYB_USB_STORAGE_MEDIUM_FLASH,
    PYB_USB_STORAGE_MEDIUM_SDCARD,
} pyb_usb_storage_medium_t;

typedef enum {
	USB_PHY_FS_ID = 0,
	USB_PHY_HS_ID = 1,
} USB_PHY_ID;



extern const mp_obj_type_t pyb_usb_vcp_type;
extern const mp_obj_type_t pyb_usb_hid_type;
void pyb_usb_dev_deinit(void);
bool pyb_usb_dev_init(uint16_t vid, uint16_t pid, usb_device_mode_t mode, USBD_HID_ModeInfoTypeDef *hid_info);
void USBAPP_IRQHandler(void);
void USBAPP_StartThread(void);
void USBAPP_ResumeThread(void);
bool usb_vcp_is_enabled(void);
void usb_vcp_send_strn(const char *str, int len);
void usb_vcp_send_strn_cooked(const char *str, int len);

 #endif
