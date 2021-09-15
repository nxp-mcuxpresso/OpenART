/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "fsl_gpio.h"
#include "fsl_mipi_dsi.h"
#include "elcdif_support.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 void VIDEO_DelayMs(uint32_t ms)
{
#if defined(FSL_RTOS_FREE_RTOS)
    TickType_t tick;

    tick = ms * configTICK_RATE_HZ / 1000;

    tick = (0 == tick) ? 1 : tick;

    vTaskDelay(tick);
#else
    while (ms--)
    {
        SDK_DelayAtLeastUs(1000, SystemCoreClock);
    }
#endif
}
#define RM68200_DelayMs VIDEO_DelayMs

/*! @brief Pixel format FOURCC. */
#define FSL_VIDEO_FOURCC(a, b, c, d) \
    ((uint32_t)(a) | ((uint32_t)(b) << 8U) | ((uint32_t)(c) << 16U) | ((uint32_t)(d) << 24U))

/*! @brief Macro to define resolution. */
#define FSL_VIDEO_RESOLUTION(width, height) ((uint32_t)(width) | ((uint32_t)(height) << 16U))

#define FSL_VIDEO_EXTRACT_WIDTH(resolution)  ((uint16_t)((resolution)&0xFFFFU))
#define FSL_VIDEO_EXTRACT_HEIGHT(resolution) ((uint16_t)((resolution) >> 16U))

/*! @brief Pixel format definition. */
typedef enum _video_pixel_format
{
    /* RGB */
    kVIDEO_PixelFormatXRGB8888 = FSL_VIDEO_FOURCC('X', 'R', '2', '4'), /*!< 32-bit XRGB8888. */
    kVIDEO_PixelFormatRGBX8888 = FSL_VIDEO_FOURCC('R', 'X', '2', '4'), /*!< 32-bit RGBX8888. */
    kVIDEO_PixelFormatXBGR8888 = FSL_VIDEO_FOURCC('X', 'B', '2', '4'), /*!< 32-bit XBGR8888. */
    kVIDEO_PixelFormatBGRX8888 = FSL_VIDEO_FOURCC('B', 'X', '2', '4'), /*!< 32-bit BGRX8888. */

    kVIDEO_PixelFormatRGB888 = FSL_VIDEO_FOURCC('R', 'G', '2', '4'), /*!< 24-bit RGB888. */
    kVIDEO_PixelFormatBGR888 = FSL_VIDEO_FOURCC('B', 'G', '2', '4'), /*!< 24-bit BGR888. */

    kVIDEO_PixelFormatRGB565 = FSL_VIDEO_FOURCC('R', 'G', '1', '6'), /*!< 16-bit RGB565. */
    kVIDEO_PixelFormatBGR565 = FSL_VIDEO_FOURCC('B', 'G', '1', '6'), /*!< 16-bit BGR565. */

    kVIDEO_PixelFormatXRGB1555 = FSL_VIDEO_FOURCC('X', 'R', '1', '5'), /*!< 16-bit XRGB1555. */
    kVIDEO_PixelFormatRGBX5551 = FSL_VIDEO_FOURCC('R', 'X', '1', '5'), /*!< 16-bit RGBX5551. */
    kVIDEO_PixelFormatXBGR1555 = FSL_VIDEO_FOURCC('X', 'B', '1', '5'), /*!< 16-bit XBGR1555. */
    kVIDEO_PixelFormatBGRX5551 = FSL_VIDEO_FOURCC('B', 'X', '1', '5'), /*!< 16-bit BGRX5551. */

    kVIDEO_PixelFormatXRGB4444 = FSL_VIDEO_FOURCC('X', 'R', '1', '2'), /*!< 16-bit XRGB4444. */
    kVIDEO_PixelFormatRGBX4444 = FSL_VIDEO_FOURCC('R', 'X', '1', '2'), /*!< 16-bit RGBX4444. */
    kVIDEO_PixelFormatXBGR4444 = FSL_VIDEO_FOURCC('X', 'B', '1', '2'), /*!< 16-bit XBGR4444. */
    kVIDEO_PixelFormatBGRX4444 = FSL_VIDEO_FOURCC('B', 'X', '1', '2'), /*!< 16-bit BGRX4444. */

    /* YUV. */
    kVIDEO_PixelFormatYUYV = FSL_VIDEO_FOURCC('Y', 'U', 'Y', 'V'), /*!< YUV422, Y-U-Y-V. */
    kVIDEO_PixelFormatYVYU = FSL_VIDEO_FOURCC('Y', 'V', 'Y', 'U'), /*!< YUV422, Y-V-Y-U. */
    kVIDEO_PixelFormatUYVY = FSL_VIDEO_FOURCC('U', 'Y', 'V', 'Y'), /*!< YUV422, U-Y-V-Y. */
    kVIDEO_PixelFormatVYUY = FSL_VIDEO_FOURCC('V', 'Y', 'U', 'Y'), /*!< YUV422, V-Y-U-Y. */

    kVIDEO_PixelFormatXYUV = FSL_VIDEO_FOURCC('X', 'Y', 'U', 'V'), /*!< YUV444, X-Y-U-V. */
    kVIDEO_PixelFormatXYVU = FSL_VIDEO_FOURCC('X', 'Y', 'V', 'U'), /*!< YUV444, X-Y-V-U. */
} video_pixel_format_t;

/*! @brief Resolution definition. */
typedef enum _video_resolution
{
    kVIDEO_ResolutionVGA   = FSL_VIDEO_RESOLUTION(640, 480),   /*!< VGA, 640 * 480 */
    kVIDEO_ResolutionQVGA  = FSL_VIDEO_RESOLUTION(320, 240),   /*!< QVGA, 320 * 240 */
    kVIDEO_ResolutionQQVGA = FSL_VIDEO_RESOLUTION(160, 120),   /*!< QQVGA, 160 * 120 */
    kVIDEO_ResolutionCIF   = FSL_VIDEO_RESOLUTION(352, 288),   /*!< CIF, 352 * 288 */
    kVIDEO_ResolutionQCIF  = FSL_VIDEO_RESOLUTION(176, 144),   /*!< QCIF, 176 * 144 */
    kVIDEO_ResolutionQQCIF = FSL_VIDEO_RESOLUTION(88, 72),     /*!< QQCIF, 88 * 72 */
    kVIDEO_Resolution720P  = FSL_VIDEO_RESOLUTION(1280, 720),  /*!< 720P, 1280 * 720 */
    kVIDEO_Resolution1080P = FSL_VIDEO_RESOLUTION(1920, 1080), /*!< 1080P, 1920 * 1280*/
    kVIDEO_ResolutionWXGA  = FSL_VIDEO_RESOLUTION(1280, 800),  /*!< WXGA, 1280 * 800 */
} video_resolution_t;
/*! @brief Display control flags. */
enum _display_flags
{
    kDISPLAY_VsyncActiveLow         = 0U,         /*!< VSYNC active low. */
    kDISPLAY_VsyncActiveHigh        = (1U << 0U), /*!< VSYNC active high. */
    kDISPLAY_HsyncActiveLow         = 0U,         /*!< HSYNC active low. */
    kDISPLAY_HsyncActiveHigh        = (1U << 1U), /*!< HSYNC active high. */
    kDISPLAY_DataEnableActiveHigh   = 0U,         /*!< Data enable line active high. */
    kDISPLAY_DataEnableActiveLow    = (1U << 2U), /*!< Data enable line active low. */
    kDISPLAY_DataLatchOnRisingEdge  = 0U,         /*!< Latch data on rising clock edge. */
    kDISPLAY_DataLatchOnFallingEdge = (1U << 3U), /*!< Latch data on falling clock edge. */
};

/*! @brief Display configuration. */
typedef struct _display_config
{
    uint32_t resolution;              /*!< Resolution, see @ref video_resolution_t and @ref FSL_VIDEO_RESOLUTION. */
    uint16_t hsw;                     /*!< HSYNC pulse width. */
    uint16_t hfp;                     /*!< Horizontal front porch. */
    uint16_t hbp;                     /*!< Horizontal back porch. */
    uint16_t vsw;                     /*!< VSYNC pulse width. */
    uint16_t vfp;                     /*!< Vrtical front porch. */
    uint16_t vbp;                     /*!< Vertical back porch. */
    uint32_t controlFlags;            /*!< Control flags, OR'ed value of @ref _display_flags. */
    uint8_t dsiLanes;                 /*!< MIPI DSI data lanes number. */
    uint32_t pixelClock_Hz;           /*!< Pixel clock in Hz. */
    video_pixel_format_t pixelFormat; /*!< Pixel format. */
} display_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const uint8_t lcmInitSetting[][2] = {
    {0xFE, 0x01},
    {0x24, 0xC0},
    {0x25, 0x53},
    {0x26, 0x00},
    {0x2B, 0xE5},
    {0x27, 0x0A},
    {0x29, 0x0A},
    {0x16, 0x52},
    {0x2F, 0x53},
    {0x34, 0x5A},
    {0x1B, 0x00},
    {0x12, 0x0A},
    {0x1A, 0x06},
    {0x46, 0x56},
    {0x52, 0xA0},
    {0x53, 0x00},
    {0x54, 0xA0},
    {0x55, 0x00},
    /* 2 data lanes */
    {0x5F, 0x11},

    {0xFE, 0x03},
    {0x00, 0x05},
    {0x02, 0x0B},
    {0x03, 0x0F},
    {0x04, 0x7D},
    {0x05, 0x00},
    {0x06, 0x50},
    {0x07, 0x05},
    {0x08, 0x16},
    {0x09, 0x0D},
    {0x0A, 0x11},
    {0x0B, 0x7D},
    {0x0C, 0x00},
    {0x0D, 0x50},
    {0x0E, 0x07},
    {0x0F, 0x08},
    {0x10, 0x01},
    {0x11, 0x02},
    {0x12, 0x00},
    {0x13, 0x7D},
    {0x14, 0x00},
    {0x15, 0x85},
    {0x16, 0x08},
    {0x17, 0x03},
    {0x18, 0x04},
    {0x19, 0x05},
    {0x1A, 0x06},
    {0x1B, 0x00},
    {0x1C, 0x7D},
    {0x1D, 0x00},
    {0x1E, 0x85},
    {0x1F, 0x08},
    {0x20, 0x00},
    {0x21, 0x00},
    {0x22, 0x00},
    {0x23, 0x00},
    {0x24, 0x00},
    {0x25, 0x00},
    {0x26, 0x00},
    {0x27, 0x00},
    {0x28, 0x00},
    {0x29, 0x00},
    {0x2A, 0x07},
    {0x2B, 0x08},
    {0x2D, 0x01},
    {0x2F, 0x02},
    {0x30, 0x00},
    {0x31, 0x40},
    {0x32, 0x05},
    {0x33, 0x08},
    {0x34, 0x54},
    {0x35, 0x7D},
    {0x36, 0x00},
    {0x37, 0x03},
    {0x38, 0x04},
    {0x39, 0x05},
    {0x3A, 0x06},
    {0x3B, 0x00},
    {0x3D, 0x40},
    {0x3F, 0x05},
    {0x40, 0x08},
    {0x41, 0x54},
    {0x42, 0x7D},
    {0x43, 0x00},
    {0x44, 0x00},
    {0x45, 0x00},
    {0x46, 0x00},
    {0x47, 0x00},
    {0x48, 0x00},
    {0x49, 0x00},
    {0x4A, 0x00},
    {0x4B, 0x00},
    {0x4C, 0x00},
    {0x4D, 0x00},
    {0x4E, 0x00},
    {0x4F, 0x00},
    {0x50, 0x00},
    {0x51, 0x00},
    {0x52, 0x00},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x55, 0x00},
    {0x56, 0x00},
    {0x58, 0x00},
    {0x59, 0x00},
    {0x5A, 0x00},
    {0x5B, 0x00},
    {0x5C, 0x00},
    {0x5D, 0x00},
    {0x5E, 0x00},
    {0x5F, 0x00},
    {0x60, 0x00},
    {0x61, 0x00},
    {0x62, 0x00},
    {0x63, 0x00},
    {0x64, 0x00},
    {0x65, 0x00},
    {0x66, 0x00},
    {0x67, 0x00},
    {0x68, 0x00},
    {0x69, 0x00},
    {0x6A, 0x00},
    {0x6B, 0x00},
    {0x6C, 0x00},
    {0x6D, 0x00},
    {0x6E, 0x00},
    {0x6F, 0x00},
    {0x70, 0x00},
    {0x71, 0x00},
    {0x72, 0x20},
    {0x73, 0x00},
    {0x74, 0x08},
    {0x75, 0x08},
    {0x76, 0x08},
    {0x77, 0x08},
    {0x78, 0x08},
    {0x79, 0x08},
    {0x7A, 0x00},
    {0x7B, 0x00},
    {0x7C, 0x00},
    {0x7D, 0x00},
    {0x7E, 0xBF},
    {0x7F, 0x02},
    {0x80, 0x06},
    {0x81, 0x14},
    {0x82, 0x10},
    {0x83, 0x16},
    {0x84, 0x12},
    {0x85, 0x08},
    {0x86, 0x3F},
    {0x87, 0x3F},
    {0x88, 0x3F},
    {0x89, 0x3F},
    {0x8A, 0x3F},
    {0x8B, 0x0C},
    {0x8C, 0x0A},
    {0x8D, 0x0E},
    {0x8E, 0x3F},
    {0x8F, 0x3F},
    {0x90, 0x00},
    {0x91, 0x04},
    {0x92, 0x3F},
    {0x93, 0x3F},
    {0x94, 0x3F},
    {0x95, 0x3F},
    {0x96, 0x05},
    {0x97, 0x01},
    {0x98, 0x3F},
    {0x99, 0x3F},
    {0x9A, 0x0F},
    {0x9B, 0x0B},
    {0x9C, 0x0D},
    {0x9D, 0x3F},
    {0x9E, 0x3F},
    {0x9F, 0x3F},
    {0xA0, 0x3F},
    {0xA2, 0x3F},
    {0xA3, 0x09},
    {0xA4, 0x13},
    {0xA5, 0x17},
    {0xA6, 0x11},
    {0xA7, 0x15},
    {0xA9, 0x07},
    {0xAA, 0x03},
    {0xAB, 0x3F},
    {0xAC, 0x3F},
    {0xAD, 0x05},
    {0xAE, 0x01},
    {0xAF, 0x17},
    {0xB0, 0x13},
    {0xB1, 0x15},
    {0xB2, 0x11},
    {0xB3, 0x0F},
    {0xB4, 0x3F},
    {0xB5, 0x3F},
    {0xB6, 0x3F},
    {0xB7, 0x3F},
    {0xB8, 0x3F},
    {0xB9, 0x0B},
    {0xBA, 0x0D},
    {0xBB, 0x09},
    {0xBC, 0x3F},
    {0xBD, 0x3F},
    {0xBE, 0x07},
    {0xBF, 0x03},
    {0xC0, 0x3F},
    {0xC1, 0x3F},
    {0xC2, 0x3F},
    {0xC3, 0x3F},
    {0xC4, 0x02},
    {0xC5, 0x06},
    {0xC6, 0x3F},
    {0xC7, 0x3F},
    {0xC8, 0x08},
    {0xC9, 0x0C},
    {0xCA, 0x0A},
    {0xCB, 0x3F},
    {0xCC, 0x3F},
    {0xCD, 0x3F},
    {0xCE, 0x3F},
    {0xCF, 0x3F},
    {0xD0, 0x0E},
    {0xD1, 0x10},
    {0xD2, 0x14},
    {0xD3, 0x12},
    {0xD4, 0x16},
    {0xD5, 0x00},
    {0xD6, 0x04},
    {0xD7, 0x3F},
    {0xDC, 0x02},
    {0xDE, 0x12},
    {0xFE, 0x0E},
    {0x01, 0x75},

    /* Gamma Settings */
    {0xFE, 0x04},
    {0x60, 0x00},
    {0x61, 0x0C},
    {0x62, 0x12},
    {0x63, 0x0E},
    {0x64, 0x06},
    {0x65, 0x12},
    {0x66, 0x0E},
    {0x67, 0x0B},
    {0x68, 0x15},
    {0x69, 0x0B},
    {0x6A, 0x10},
    {0x6B, 0x07},
    {0x6C, 0x0F},
    {0x6D, 0x12},
    {0x6E, 0x0C},
    {0x6F, 0x00},
    {0x70, 0x00},
    {0x71, 0x0C},
    {0x72, 0x12},
    {0x73, 0x0E},
    {0x74, 0x06},
    {0x75, 0x12},
    {0x76, 0x0E},
    {0x77, 0x0B},
    {0x78, 0x15},
    {0x79, 0x0B},
    {0x7A, 0x10},
    {0x7B, 0x07},
    {0x7C, 0x0F},
    {0x7D, 0x12},
    {0x7E, 0x0C},
    {0x7F, 0x00},

    /* Page 0. */
    {0xFE, 0x00},
    {0x11, 0x00},
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*! @brief The MIPI panel pins. */
#define BOARD_MIPI_PANEL_RST_GPIO   GPIO9
#define BOARD_MIPI_PANEL_RST_PIN    1
#define BOARD_MIPI_PANEL_POWER_GPIO GPIO11
#define BOARD_MIPI_PANEL_POWER_PIN  16
/* Back light pin. */
#define BOARD_MIPI_PANEL_BL_GPIO GPIO9
#define BOARD_MIPI_PANEL_BL_PIN  29
static void PANEL_PullResetPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, 0);
    }
}

/* From the schematic, the power pin is pinned to high. */
static void PANEL_PullPowerPin(bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, 0);
    }
}

status_t MIPI_DSI_GenericWrite(const uint8_t *txData, int32_t txDataSize)
{
    dsi_transfer_t dsiXfer = {0};

    dsiXfer.virtualChannel = 0;
    dsiXfer.txDataSize     = txDataSize;
    dsiXfer.txData         = txData;

    if (0 == txDataSize)
    {
        dsiXfer.txDataType = kDSI_TxDataGenShortWrNoParam;
    }
    else if (1 == txDataSize)
    {
        dsiXfer.txDataType = kDSI_TxDataGenShortWrOneParam;
    }
    else if (2 == txDataSize)
    {
        dsiXfer.txDataType = kDSI_TxDataGenShortWrTwoParam;
    }
    else
    {
        dsiXfer.txDataType = kDSI_TxDataGenLongWr;
    }

    return DSI_TransferBlocking(MIPI_DSI_HOST, &dsiXfer);
}
status_t RM68200_Init(const display_config_t *config)
{
    uint32_t i;
    uint8_t param[2];
    status_t status              = kStatus_Success;

    /* Only support 720 * 1280 */
    if (config->resolution != FSL_VIDEO_RESOLUTION(720, 1280))
    {
        return kStatus_InvalidArgument;
    }

    /* Power on. */
    PANEL_PullPowerPin(true);
    RM68200_DelayMs(1);

    /* Perform reset. */
    PANEL_PullResetPin(false);
    RM68200_DelayMs(1);
    PANEL_PullResetPin(true);
    RM68200_DelayMs(5);

    /* Set the LCM init settings. */
    for (i = 0; i < ARRAY_SIZE(lcmInitSetting); i++)
    {
        status = MIPI_DSI_GenericWrite(lcmInitSetting[i], 2);

        if (kStatus_Success != status)
        {
            return status;
        }
    }

    RM68200_DelayMs(200);

    param[0] = 0x29;
    param[1] = 0x00;
    MIPI_DSI_GenericWrite(param, 2);

    RM68200_DelayMs(100);

    param[0] = 0x2c;
    MIPI_DSI_GenericWrite(param, 1);
    param[0] = 0x35;
    param[1] = 0x00;
    MIPI_DSI_GenericWrite(param, 2);

    RM68200_DelayMs(200);

    return kStatus_Success;
}


uint32_t mipiDsiTxEscClkFreq_Hz;
uint32_t mipiDsiDphyBitClkFreq_Hz;
uint32_t mipiDsiDphyRefClkFreq_Hz;
uint32_t mipiDsiDpiClkFreq_Hz;
extern void APP_LCDIF_IRQHandler(void);


void BOARD_InitLcdifClock(void)
{
    /*
     * The pixel clock is (height + VSW + VFP + VBP) * (width + HSW + HFP + HBP) * frame rate.
     *
     * Use PLL_528 as clock source.
     *
     * For 60Hz frame rate, the RK055IQH091 pixel clock should be 36MHz.
     * the RK055AHD091 pixel clock should be 62MHz.
     */
    const clock_root_config_t lcdifClockConfig = {
        .clockOff = false,
        .mfn      = 0,
        .mfd      = 0,
        .mux      = 4, /*!< PLL_528. */
        .div 	  = 15,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Lcdif, &lcdifClockConfig);

    mipiDsiDpiClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Lcdif);
}

static status_t BOARD_InitLcdPanel(void)
{
    status_t status;

    const gpio_pin_config_t pinConfig = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    const display_config_t displayConfig = {
        .resolution   = FSL_VIDEO_RESOLUTION(APP_PANEL_WIDTH, APP_PANEL_HEIGHT),
        .hsw          = APP_HSW,
        .hfp          = APP_HFP,
        .hbp          = APP_HBP,
        .vsw          = APP_VSW,
        .vfp          = APP_VFP,
        .vbp          = APP_VBP,
        .controlFlags = 0,
        .dsiLanes     = APP_MIPI_DSI_LANE_NUM,
    };

    GPIO_PinInit(BOARD_MIPI_PANEL_POWER_GPIO, BOARD_MIPI_PANEL_POWER_PIN, &pinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, &pinConfig);
    GPIO_PinInit(BOARD_MIPI_PANEL_RST_GPIO, BOARD_MIPI_PANEL_RST_PIN, &pinConfig);

    status = RM68200_Init(&displayConfig);


    if (status == kStatus_Success)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_BL_GPIO, BOARD_MIPI_PANEL_BL_PIN, 1);
    }

    return status;
}

static void BOARD_InitMipiDsiClock(void)
{
    uint32_t mipiDsiEscClkFreq_Hz;

    /* RxClkEsc max 60MHz, TxClkEsc 12 to 20MHz. */
    /* RxClkEsc = 528MHz / 11 = 48MHz. */
    /* TxClkEsc = 528MHz / 11 / 4 = 16MHz. */
    const clock_root_config_t mipiEscClockConfig = {
        .clockOff = false,
        .mfn      = 0,
        .mfd      = 0,
        .mux      = 4, /*!< PLL_528. */
        .div      = 11,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Esc, &mipiEscClockConfig);

    mipiDsiEscClkFreq_Hz = CLOCK_GetRootClockFreq(kCLOCK_Root_Mipi_Esc);

    const clock_group_config_t mipiEscClockGroupConfig = {
        .clockOff = false,
        .resetDiv = 2,
        .div0     = 2, /* TX esc clock. */
        .div1     = 0, /* RX esc clock. */
    };

    CLOCK_SetGroupConfig(kCLOCK_Group_MipiDsi, &mipiEscClockGroupConfig);

    mipiDsiTxEscClkFreq_Hz = mipiDsiEscClkFreq_Hz / 3;

    /* DPHY reference clock, use OSC 24MHz clock. */
    const clock_root_config_t mipiDphyRefClockConfig = {
        .clockOff = false,
        .mfn      = 0,
        .mfd      = 0,
        .mux      = 1, /*!< OSC_24M. */
        .div      = 1,
    };

    CLOCK_SetRootClock(kCLOCK_Root_Mipi_Ref, &mipiDphyRefClockConfig);

    mipiDsiDphyRefClkFreq_Hz = BOARD_XTAL0_CLK_HZ;
}

static void BOARD_SetMipiDsiConfig(void)
{
    dsi_config_t dsiConfig;
    dsi_dphy_config_t dphyConfig;

    const dsi_dpi_config_t dpiConfig = {.pixelPayloadSize = APP_PANEL_WIDTH,
                                        .dpiColorCoding   = kDSI_Dpi24Bit,
                                        .pixelPacket      = kDSI_PixelPacket24Bit,
                                        .videoMode        = kDSI_DpiBurst,
                                        .bllpMode         = kDSI_DpiBllpLowPower,
                                        .polarityFlags    = kDSI_DpiVsyncActiveLow | kDSI_DpiHsyncActiveLow,
                                        .hfp              = APP_HFP,
                                        .hbp              = APP_HBP,
                                        .hsw              = APP_HSW,
                                        .vfp              = APP_VFP,
                                        .vbp              = APP_VBP,
                                        .panelHeight      = APP_PANEL_HEIGHT,
                                        .virtualChannel   = 0};

    /*
     * dsiConfig.numLanes = 4;
     * dsiConfig.enableNonContinuousHsClk = false;
     * dsiConfig.autoInsertEoTp = true;
     * dsiConfig.numExtraEoTp = 0;
     * dsiConfig.htxTo_ByteClk = 0;
     * dsiConfig.lrxHostTo_ByteClk = 0;
     * dsiConfig.btaTo_ByteClk = 0;
     */
    DSI_GetDefaultConfig(&dsiConfig);
    dsiConfig.numLanes       = APP_MIPI_DSI_LANE_NUM;
    dsiConfig.autoInsertEoTp = true;

    /* Init the DSI module. */
    DSI_Init(APP_MIPI_DSI, &dsiConfig);

    /* Init DPHY.
     *
     * The DPHY bit clock must be fast enough to send out the pixels, it should be
     * larger than:
     *
     *         (Pixel clock * bit per output pixel) / number of MIPI data lane
     *
     * Here the desired DPHY bit clock multiplied by ( 9 / 8 = 1.125) to ensure
     * it is fast enough.
     *
     * Note that the DSI output pixel is 24bit per pixel.
     */
    mipiDsiDphyBitClkFreq_Hz = mipiDsiDpiClkFreq_Hz * (24 / APP_MIPI_DSI_LANE_NUM);

    mipiDsiDphyBitClkFreq_Hz = APP_MIPI_DPHY_BIT_CLK_ENLARGE(mipiDsiDphyBitClkFreq_Hz);

    DSI_GetDphyDefaultConfig(&dphyConfig, mipiDsiDphyBitClkFreq_Hz, mipiDsiTxEscClkFreq_Hz);

    mipiDsiDphyBitClkFreq_Hz = DSI_InitDphy(APP_MIPI_DSI, &dphyConfig, mipiDsiDphyRefClkFreq_Hz);

    /* Init DPI interface. */
    DSI_SetDpiConfig(APP_MIPI_DSI, &dpiConfig, APP_MIPI_DSI_LANE_NUM, mipiDsiDpiClkFreq_Hz, mipiDsiDphyBitClkFreq_Hz);
}

status_t BOARD_InitDisplayInterface(void)
{
    /* LCDIF v2 output to MIPI DSI. */
    CLOCK_EnableClock(kCLOCK_Video_Mux);
    VIDEO_MUX->VID_MUX_CTRL.CLR = VIDEO_MUX_VID_MUX_CTRL_MIPI_DSI_SEL_MASK;

    /* 1. Power on and isolation off. */
    PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

    /* 2. Assert MIPI reset. */
    IOMUXC_GPR->GPR62 &=
        ~(IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK |
          IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 3. Setup clock. */
    BOARD_InitMipiDsiClock();

    /* 4. Deassert PCLK and ESC reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_PCLK_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_ESC_SOFT_RESET_N_MASK);

    /* 5. Configures peripheral. */
    BOARD_SetMipiDsiConfig();

    /* 6. Deassert BYTE and DBI reset. */
    IOMUXC_GPR->GPR62 |=
        (IOMUXC_GPR_GPR62_MIPI_DSI_BYTE_SOFT_RESET_N_MASK | IOMUXC_GPR_GPR62_MIPI_DSI_DPI_SOFT_RESET_N_MASK);

    /* 7. Configure the panel. */
    return BOARD_InitLcdPanel();
}

void BOARD_EnableLcdInterrupt(void)
{
	NVIC_ClearPendingIRQ(APP_ELCDIF_IRQn);
    EnableIRQ(APP_ELCDIF_IRQn);
}
