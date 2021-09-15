/*
 * Copyright  2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "framebuffer.h"
#ifdef RT_USING_LCD
#include "elcdif_support.h"
#include "fsl_mipi_dsi.h"
#include "fsl_elcdif.h"
#include "fsl_pxp.h"
#include "fsl_soc_src.h"
#include "fsl_cache.h"
#include "sensor.h"
#ifndef LCD_HANDSHAKE
#define LCD_BUFFER s_framebuffer
__attribute__((section(".lcd_buffer"))) uint16_t s_framebuffer[2][APP_IMG_HEIGHT][APP_IMG_WIDTH] = {0};
#else
#define LCD_BUFFER s_handshake
__attribute__((section(".lcd_handshake_buffer"))) uint16_t s_handshake[16*2][APP_IMG_WIDTH] = {0};
#endif
__attribute__((section(".lcd_buffer"))) uint16_t UV_BUFFER[1280*720];
void LCDMonitor_SetPIXFORMAT(int format);

void LCDMonitor_GetDispSize(int *disp_w, int *disp_h)
{
	*disp_w = APP_IMG_WIDTH;
	*disp_h = APP_IMG_HEIGHT;
}

int LCDMonitor_Init(void)
{
	memset(UV_BUFFER, 0x80, sizeof(UV_BUFFER));
	
    BOARD_InitLcdifClock();
    const elcdif_rgb_mode_config_t config = {
        .panelWidth    = APP_IMG_WIDTH,
        .panelHeight   = APP_IMG_HEIGHT,
        .hsw           = APP_HSW,
        .hfp           = APP_HFP,
        .hbp           = APP_HBP,
        .vsw           = APP_VSW,
        .vfp           = APP_VFP,
        .vbp           = APP_VBP,
        .polarityFlags = APP_POL_FLAGS,
		.bufferAddr    = (uint32_t)LCD_BUFFER,
        .pixelFormat   = kELCDIF_PixelFormatRGB565,
        .dataBus       = APP_LCDIF_DATA_BUS,
    };
	
    BOARD_InitDisplayInterface();
    ELCDIF_RgbModeInit(APP_ELCDIF, &config);
#ifdef LCD_HANDSHAKE
	// use handshake mode to save mem
	ELCDIF_EnablePxpHandShake(APP_ELCDIF, true);
#endif
	ELCDIF_RgbModeStart(APP_ELCDIF);
	
	/*
     * Configure the PXP for rotate and scale.
     */
	// if use pxp both for lcd & lvgl, need to call below code
	// each flush loop
	// we only have one pxp, so need to re-configure it at diverse
	// function.
	
    PXP_Init(PXP);

    PXP_SetProcessSurfaceBackGroundColor(PXP, 0U);

    PXP_SetProcessSurfacePosition(PXP, 0U, 0U, APP_IMG_HEIGHT - 1U, APP_IMG_WIDTH - 1U);
	
	PXP_SetRotateConfig(PXP, kPXP_RotateOutputBuffer, kPXP_Rotate90, kPXP_FlipDisable);

    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);

    PXP_EnableCsc1(PXP, false);

#ifdef LCD_HANDSHAKE
	// enable the pxp handshake
	PXP_SetProcessBlockSize(PXP, kPXP_BlockSize16);

    PXP_EnableLcdHandShake(PXP, true);
	
	PXP_EnableContinousRun(PXP, true);
#endif

	LCDMonitor_SetPIXFORMAT(PIXFORMAT_RGB565);

	return 0;
}

INIT_DEVICE_EXPORT(LCDMonitor_Init);

#ifdef RT_USING_LCD

	/* Output config. */
pxp_output_buffer_config_t outputBufferConfig = {
		.pixelFormat    = kPXP_OutputPixelFormatRGB565,
		.interlacedMode = kPXP_OutputProgressive,
		.buffer0Addr    = (uint32_t)LCD_BUFFER,
		.buffer1Addr    = 0U,
		.pitchBytes     = APP_IMG_WIDTH * 2,
		.width  = APP_IMG_HEIGHT, // LCD's W/H: 1280/720
		.height = APP_IMG_WIDTH,

	};
pxp_ps_buffer_config_t psBufferConfig = {
		.swapByte    = true,
		.bufferAddrU = 0U,
		.bufferAddrV = 0U,
	};
#endif

void LCDMonitor_SetPIXFORMAT(int format)
{
	switch(format){
			case PIXFORMAT_RGB565:
				psBufferConfig.pixelFormat = kPXP_PsPixelFormatRGB565;
				break;
			case PIXFORMAT_YUV422:
				psBufferConfig.pixelFormat = kPXP_PsPixelFormatYUV1P444;
				break;
			case PIXFORMAT_GRAYSCALE:
				psBufferConfig.pixelFormat = kPXP_PsPixelFormatYUV2P422;
				psBufferConfig.bufferAddrU = (uint32_t)UV_BUFFER;
				break;
			default:
				break;
		}
}

// indicate that an ide signal occur: script done or ide quit
bool ide_signal = false;
void Update_FrameBuffer(void* buf, uint32_t w, uint32_t h, uint32_t bpp, void* handler){	
	// un-lock cur frame
	{
	#ifdef RT_USING_LCD
		
		PXP_SetProcessSurfaceScaler(PXP, w, h, APP_IMG_HEIGHT, APP_IMG_WIDTH);
		psBufferConfig.bufferAddr = (uint32_t)buf;
		psBufferConfig.pitchBytes  = w * bpp;
		PXP_SetProcessSurfaceBufferConfig(PXP, &psBufferConfig);
#ifndef LCD_HANDSHAKE
		static int idx = 0;
		ELCDIF_ClearInterruptStatus(APP_ELCDIF, kELCDIF_CurFrameDone);
		/* Wait the inactive buffer be active. */
		while (!(kELCDIF_CurFrameDone & ELCDIF_GetInterruptStatus(APP_ELCDIF)))
		{
		}
		outputBufferConfig.buffer0Addr = (uint32_t)LCD_BUFFER[idx]; 
#endif
		PXP_SetOutputBufferConfig(PXP, &outputBufferConfig);	
		// the pxp will use the mem directly, clean the cache
		uint32_t byteToCopy = w * h * bpp;
		DCACHE_CleanInvalidateByRange((uint32_t)(buf), byteToCopy);
		PXP_Start(PXP);

		while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(PXP)))
		{
		}
		PXP_ClearStatusFlags(PXP, kPXP_CompleteFlag);
#ifndef LCD_HANDSHAKE
		ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)LCD_BUFFER[idx]);
		idx ^= 1;
#endif			
	#endif
	}
}

#endif