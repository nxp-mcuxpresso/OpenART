/*
 * Copyright  2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#include "fsl_elcdif.h"
#include "framebuffer.h"
#define APP_ELCDIF LCDIF

#define APP_LCD_HEIGHT 272
#define APP_LCD_WIDTH 480
#define APP_HSW 41
#define APP_HFP 4
#define APP_HBP 8
#define APP_VSW 10
#define APP_VFP 4
#define APP_VBP 2
#define APP_LCD_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)

#define APP_LCDIF_DATA_BUS kELCDIF_DataBus16Bit

/* Display. */
#define LCD_DISP_GPIO GPIO1
#define LCD_DISP_GPIO_PIN 2
/* Back light. */
#define LCD_BL_GPIO GPIO2
#define LCD_BL_GPIO_PIN 31

#define APP_BPP 2
#define LCD_FB __attribute__((section(".lcd_fb")))
/*static*/ LCD_FB uint16_t s_frameBuffer[2][272][480] ;

__attribute__((naked)) static void LCDMonitor_UpdateLineRGB565(uint16_t *pLcdFB, uint16_t *pCamFB, uint32_t u64Cnt){
	__asm volatile(
		"	push {r3, r4} \n"
		"loop: \n"
		"	ldrd r3, r4, [r1], #8 \n"
		"	rev16 r3, r3 \n"
		"	rev16 r4, r4 \n"
		"	strd r3, r4, [r0], #8 \n"
		"	subs r2, #1 \n"
		"	bne loop \n"
		"	pop {r3, r4} \n"
		"	bx lr ");
	
}

__attribute__((naked)) static void LCDMonitor_UpdateLineRGB565_rotate180(uint16_t *pLcdFB, uint16_t *pCamFB, uint32_t u64Cnt){
	__asm volatile(
		"	push {r3, r4} \n"
		"loop1: \n"
		"	ldrd r3, r4, [r1] \n"
	    " 	subs r1, #8 \n"
		"	rev r3, r3 \n"
		"	rev r4, r4 \n"
		"	strd r4, r3, [r0], #8 \n"
		"	subs r2, #1 \n"
		"	bne loop1 \n"
		"	pop {r3, r4} \n"
		"	bx lr ");
	
}

void Update_FrameBuffer(void* buf, uint32_t w, uint32_t h, uint32_t bpp, void* handler){

	uint32_t t1 = w * 2 / 8;
	extern uint16_t s_frameBuffer[2][272][480];
	static int idx = 0;
	uint16_t* pLcd = (uint16_t*)s_frameBuffer[idx];
#ifdef LCD_DISPLAY_ROTATE_180		
	uint16_t* pFrame = (uint16_t*)(buf + w*h*bpp-8);
	// we only need to swap the pixel with rev16
	
	for (uint32_t y=0; y< h; y++) {
		LCDMonitor_UpdateLineRGB565_rotate180(pLcd, pFrame, t1);
		
		pLcd += w;
		pFrame -= w;
	}
#else		
	uint16_t*  pFrame = (uint16_t*)buf;
	for (uint32_t y=0; y< h; y++) {
		LCDMonitor_UpdateLineRGB565(pLcd, pFrame, t1);
		
		pLcd += w;
		pFrame += w;
	}		
#endif		
	ELCDIF_ClearInterruptStatus(LCDIF, kELCDIF_CurFrameDone);
	/* Wait the inactive buffer be active. */
	while (!(kELCDIF_CurFrameDone & ELCDIF_GetInterruptStatus(LCDIF)))
	{
	}
	ELCDIF_SetNextBufferAddr(LCDIF, (uint32_t) s_frameBuffer[idx]);	

	idx ^= 1;
	
}

uint32_t activeFrameAddr;
uint32_t inactiveFrameAddr;

__attribute__((naked))
uint16_t* LCDMonitor_UpdateLineGray(uint16_t *pLcdFB, uint16_t *pCamFB, uint32_t quadPixCnt) {
	__asm volatile(
		"   push   {r0-r6, ip, lr} \n"  // we found GCC caller does not save these for naked callee!
		"	mov 	r5, #0	  \n "
		"	mov 	r6, #0	  \n "
		"10:	\n "
		"	subs	r2, r2, #1	  \n "
		"	ldr 	r3, [r1], #4	\n "
		"	ldr 	ip, =0xFCFCFCFC    \n "
		"	and 	r3, r3, ip	  \n "
		"	lsr 	r3, r3, #2	  \n "
		"	lsr 	r4, r3, #16    \n "
		"	bfi 	r5, r3, #5, #6	  \n "
		"	bfi 	r6, r4, #5, #6	  \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4	  \n "
		"	bfi 	r5, r3, #21, #6    \n "
		"	bfi 	r6, r4, #21, #6    \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4	  \n "
		"	ldr 	ip, =0xFEFE    \n "
		"	and 	r3, r3, ip	  \n "
		"	and 	r4, r4, ip	  \n "
		"	lsr 	r3, r3, #1	  \n "
		"	lsr 	r4, r4, #1	  \n "	
		"	bfi 	r5, r3, #0, #5	  \n "
		"	bfi 	r6, r4, #0, #5	  \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4		\n "
		"	bfi 	r5, r3, #16,	#5	  \n "
		"	bfi 	r6, r4, #16,	#5	  \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4	  \n "
		"	bfi 	r5, r3, #11,	#5	  \n "
		"	bfi 	r6, r4, #11,	#5	  \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4	  \n "	
		"	bfi 	r5, r3, #27,	#5	  \n "
		"	bfi 	r6, r4, #27,	#5	  \n "	
		"	strd	r5, r6, [r0], #8	\n "
		"	bne 	10b    \n "
		"	pop 	{r0-r6, ip, pc}    \n "

	);
}


void LCDMonitor_InitFB(void)
{
	int i, x,y;
	for (i=0; i<2; i++) {
		for (x=0;x<480;x++) {
			for (y=0;y<272;y++) {
				if (x % 10 < 8 && y % 10 < 8)
					s_frameBuffer[i][y][x] = 0;
				else
					s_frameBuffer[i][y][x] = (4 | 8<<6 | 4<<11);
			}
		}
	}
}

void BOARD_InitLcd(void)
{
    volatile uint32_t i = 0x100U;

    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Reset the LCD. */
    GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &config);

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);

    while (i--)
    {
    }

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);

    /* Backlight. */
    config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);
}

void BOARD_InitLcdifPixClock(void)
{
    /*
     * The desired output frame rate is 60Hz. So the pixel clock frequency is:
     * (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2) * 60 = 9.2M.
     * Here set the LCDIF pixel clock to 9.3M.
     */

    /*
     * Initialize the Video PLL.
     * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 93MHz.
     */
    clock_video_pll_config_t config = {
        .loopDivider = 31, .postDivider = 8, .numerator = 0, .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 4);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);

    /*
     * 000 derive clock from divided pre-muxed lcdif1 clock
     * 001 derive clock from ipp_di0_clk
     * 010 derive clock from ipp_di1_clk
     * 011 derive clock from ldb_di0_clk
     * 100 derive clock from ldb_di1_clk
     */
    //CLOCK_SetMux(kCLOCK_Lcdif1Mux, 0);
}



void LCDMonitor_Update(uint32_t fbNdx, uint8_t isGray,uint32_t wndH, uint32_t wndW, uint32_t pixels_addr)
{
	uint32_t y, t1;
	uint16_t *pFB = (uint16_t*) pixels_addr;
	uint8_t *pFBGray = (uint8_t*) pixels_addr;
	uint16_t *pLcd = (uint16_t*) (s_frameBuffer[fbNdx & 1]);
	uint16_t *pLcdBkup;
	uint32_t h = wndH > 272 ? 272 : wndH;
	pLcdBkup = pLcd;
	
	pLcd += (480 - wndW) >> 1;
	pLcd += ((272 - h) >> 1) * 480;
	
	t1 = wndW * 2 / 8;
	if (isGray) {
		pFBGray += (h - 1) * wndW;
		for (y=0; y< h; y++, pFBGray -= wndW) {
			LCDMonitor_UpdateLineGray(pLcd, (uint16_t *)pFBGray, t1);
			pLcd += 480;
		}
		ELCDIF_SetNextBufferAddr(LCDIF, (uint32_t) pLcdBkup);		
	}
	else {
		pFB += (h - 1) * wndW;
		for (y=0; y< h; y++, pFB -= wndW) {
			LCDMonitor_UpdateLineRGB565(pLcd, pFB, t1);
			pLcd += 480;
		}		
	}

	ELCDIF_SetNextBufferAddr(LCDIF, (uint32_t) pLcdBkup);
}

void LCDMonitor_GetDispSize(int *disp_w, int *disp_h)
{
	*disp_w = APP_LCD_WIDTH;
	*disp_h = APP_LCD_HEIGHT;
}

void LCDMonitor_Init(void)
{
	static uint8_t isInited;
	if (isInited)
		return;
	isInited = 1;
    // Initialize the camera bus.
    BOARD_InitLcdifPixClock();
   // BOARD_InitDebugConsole();
    BOARD_InitLcd();	
    elcdif_rgb_mode_config_t lcdConfig = {
        .panelWidth = APP_LCD_WIDTH,
        .panelHeight = APP_LCD_HEIGHT,
        .hsw = APP_HSW,
        .hfp = APP_HFP,
        .hbp = APP_HBP,
        .vsw = APP_VSW,
        .vfp = APP_VFP,
        .vbp = APP_VBP,
        .polarityFlags = APP_LCD_POL_FLAGS,
        .pixelFormat = kELCDIF_PixelFormatRGB565,
        .dataBus = APP_LCDIF_DATA_BUS,
    };	
	LCDMonitor_InitFB();

    lcdConfig.bufferAddr = (uint32_t)activeFrameAddr;

    ELCDIF_RgbModeInit(APP_ELCDIF, &lcdConfig);

    ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)s_frameBuffer);
    ELCDIF_RgbModeStart(APP_ELCDIF);  	

}

INIT_DEVICE_EXPORT(LCDMonitor_Init);