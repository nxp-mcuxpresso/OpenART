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
#ifdef SOC_IMXRT1170_SERIES
#else
#include "fsl_csi_camera_adapter.h"
#include "fsl_camera.h"
#include "fsl_camera_receiver.h"
#include "fsl_camera_device.h"
#include "fsl_ov7725.h"
#endif
#include "fsl_elcdif.h"
#include "fsl_dmamux.h"
#include "omv_boardconfig.h"
#include "fsl_cache.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_csi.h"
#include "py/mpprint.h"
#include "irq.h"
#include "sensor.h"
#include "drv_camera.h"
#include "ov9650.h"
#include "ov2640.h"
#include "ov5640.h"
#include "ov5640_regs.h"
#include "ov7725.h"
#include "ov7725_regs.h"
#include "mt9v034.h"
#include "framebuffer.h"

#ifdef RT_USING_LCD
#include "elcdif_support.h"
#include "fsl_mipi_dsi.h"
#include "fsl_elcdif.h"
#include "fsl_pxp.h"
#include "fsl_soc_src.h"
#endif
#define cam_echo(...) //mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)
#define cam_err(...) mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)
#define BUFFER_SIZE	(100000)
#if defined(__CC_ARM) || defined(__CLANG_ARM)
	extern unsigned int Image$$MPY_SENSOR_BUFER_START$$Base;
#elif defined(__ICCARM__)
	extern unsigned int MPY_SENSOR_BUFER_START$$Limit[];
#elif defined(__GNUC__)

#endif

//#define GPIO_DEBUGING
#ifdef GPIO_DEBUGING
#include "gpio_debuging.h"
#endif

enum{
	RT_CAMERA_DEVICE_INIT = 0,
	RT_CAMERA_DEVICE_SUSPEND,
	RT_CAMERA_DEVICE_RUNING,
}RT_CAMERA_DEVICE_ST;

//				 			8bit | PixRisEdge | gatedClk  | SyncClrFifo| HSyncActHigh|SofOnVsyncRis|ExtVSync
#define CSICR1_INIT_VAL 	0<<0 | 1<<1	      | 1<<4	  | 1<<8	   | 1<<11		 | 1<<17	   |1<<30
#define FB_MEM_HEAD_SIZE   (2*sizeof(uint32_t))
struct fb_mem
{
	uint32_t tick;
	uint32_t lock;
	uint8_t ptr[];
};

struct fb_mem_list
{
	uint16_t bpp;
	uint16_t w;
	uint16_t h;
	
	uint16_t count;
	uint32_t idx;
	uint32_t frame_size;
	uint32_t *buffer_start;
	uint32_t total_frame_count;
};

typedef struct _CSIIrq_t
{
	uint8_t isStarted;
	uint8_t isGray;
	uint32_t base0;
	uint32_t linePerFrag;
	uint32_t cnt;
	uint32_t dmaBytePerLine;
	uint32_t dmaBytePerFrag;
	uint32_t dmaFragNdx;

	uint32_t datBytePerLine;
	uint32_t datBytePerFrag;
	uint32_t datFragNdx;

	uint32_t datCurBase;

	uint32_t fragCnt;
	// in color mode, dmaFragNdx should == datLineNdx
	// in gray mode, to save memory, move backword nextDmaBulk every 4 lines
	
}CSIIrq_t;
;

struct imxrt_camera
{
    char *name;
    CSI_Type *	csi_base;
    IRQn_Type irqn;
	struct rt_camera_device *rtt_device;
	volatile CSIIrq_t s_irq;
	struct fb_mem_list fb_list;
	sensor_t	sensor;
	
	uint8_t		sensor_id;
	uint8_t     sensor_addr;
	GPIO_Type * sensor_pwn_io;
	uint8_t     sensor_pwn_io_pin;
	GPIO_Type *  sensor_rst_io;
	uint8_t sensor_rst_io_pin;
	char 		*sensor_bus_name;
	uint32_t	fb_buffer_start;
	uint32_t	fb_buffer_end;
};

static struct imxrt_camera *pCam = NULL;
#define CAM_NUM		1
#ifndef BSP_SENSOR_BUS_NAME
#define BSP_SENSOR_BUS_NAME "i2c1"
#endif
static struct imxrt_camera cams[1] = {
	{
		.name = "camera0",
		.csi_base = CSI,
		
		.sensor_id = 0,
		.sensor_addr = 0x42U,
		.sensor_pwn_io = GPIO9,
		.sensor_pwn_io_pin = 25,
		.sensor_rst_io = GPIO11,
		.sensor_rst_io_pin = 15,		
		.sensor_bus_name = BSP_SENSOR_BUS_NAME,
#if defined(__CC_ARM) || defined(__CLANG_ARM)		
		.fb_buffer_start = (uint32_t) &Image$$MPY_SENSOR_BUFER_START$$Base,
		.fb_buffer_end = (uint32_t) &Image$$MPY_SENSOR_BUFER_START$$Base + BUFFER_SIZE,
#elif defined(__ICCARM__)	
		.fb_buffer_start = (uint32_t) MPY_SENSOR_BUFER_START$$Limit,
		.fb_buffer_end = (uint32_t) MPY_SENSOR_BUFER_START$$Limit + BUFFER_SIZE,
#endif		
	},
};


const int resolution[][2] = {
    {0,    0   },
    // C/SIF Resolutions
    {88,   72  },    /* QQCIF     */
    {176,  144 },    /* QCIF      */
    {352,  288 },    /* CIF       */
    {88,   60  },    /* QQSIF     */
    {176,  120 },    /* QSIF      */
    {352,  240 },    /* SIF       */
    // VGA Resolutions
    {40,   30  },    /* QQQQVGA   */
    {80,   60  },    /* QQQVGA    */
    {160,  120 },    /* QQVGA     */
    {320,  240 },    /* QVGA      */
    {640,  480 },    /* VGA       */
    {60,   40  },    /* HQQQVGA   */
    {120,  80  },    /* HQQVGA    */
    {240,  160 },    /* HQVGA     */
    // FFT Resolutions
    {64,   32  },    /* 64x32     */
    {64,   64  },    /* 64x64     */
    {128,  64  },    /* 128x64    */
    {128,  128 },    /* 128x64    */
    // Other
    {128,  160 },    /* LCD       */
    {128,  160 },    /* QQVGA2    */
    {720,  480 },    /* WVGA      */
    {752,  480 },    /* WVGA2     */
    {800,  600 },    /* SVGA      */
    {1024, 768 },    /* XGA       */
    {1280, 1024},    /* SXGA      */
    {1600, 1200},    /* UXGA      */
	{1280, 720 },    /* HD        */ 
    {1920, 1080},    /* FHD       */ 
    {2560, 1440},    /* QHD       */
    {2048, 1536},    /* QXGA      */
    {2560, 1600},    /* WQXGA     */
    {2592, 1944},    /* WQXGA2    */
};
#define FRAMEBUFFER_SIZE (1280*720*2)
#define FRAMEBUFFER_COUNT 3
#define FRAGBUF_LOC  __attribute__((section(".dmaFramebuffer")))
FRAGBUF_LOC static uint64_t s_dmaFragBufs[2][1280 * 4 / 8];	// max supported line length, XRGB8888(*4)

static struct rt_event frame_event;
#define EVENT_CSI (1<<0)
static struct fb_mem *g_framePtr = NULL;
static uint32_t g_csi_int_en = 0;
static uint32_t isToStopCSICommand = 0;
//==================================================================================================



void imx_cam_csi_start_frame(struct imxrt_camera *cam);

void CsiFragModeInit(CSI_Type *s_pCSI ) {

	CLOCK_EnableClock(kCLOCK_Csi);
	CSI_Reset(s_pCSI);
	
	s_pCSI->CSICR1 = CSICR1_INIT_VAL;
	s_pCSI->CSICR2 = 3U << 30;	// INCR16 for RxFIFO DMA
	s_pCSI->CSICR3 = 2U << 4;	// 16 double words to trigger DMA request
	s_pCSI->CSIFBUF_PARA = 0;	// no stride


	s_pCSI->CSICR18 = 13<<12 | 1<<18 | 1<<3;	// HProt AHB bus protocol, write to memory when CSI_ENABLE is 1, enbale 888/yuv input

	NVIC_SetPriority(CSI_IRQn, IRQ_PRI_CSI);
}

void imx_cam_sensor_io_init(GPIO_Type *base, uint32_t pin)
{
    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };
	
	GPIO_PinInit(base, pin, &config);	
}

static void imx_cam_sensor_io_set(GPIO_Type *base, uint32_t pin,bool pullUp)
{
    if (pullUp)
    {
        GPIO_PinWrite(base, pin, 1);
    }
    else
    {
        GPIO_PinWrite(base, pin, 0);
    }
}

uint8_t imx_cam_sensor_scan(struct rt_i2c_bus_device *i2c_bus)
{
	struct rt_i2c_msg msgs;
	rt_uint8_t buf[3] = {0};
	for (uint8_t addr=0x08;addr<=0x77;addr++)
	{
		buf[0] = 0x0; //cmd
		msgs.addr = addr;
		msgs.flags = RT_I2C_WR;
		msgs.buf = buf;
		msgs.len = 1;
		
		if (rt_i2c_transfer(i2c_bus, &msgs, 1) != 0)
		{
			return addr ;
		}
	}
	
	return 0;
}

int imx_cam_sensor_read_reg(struct rt_i2c_bus_device *i2c_bus,rt_uint16_t addr, rt_uint8_t reg, rt_uint8_t *data)
{
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3];
	
	
	buf[0] = reg; //cmd
    msgs[0].addr = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 1;
	
	msgs[1].addr = addr;
    msgs[1].flags = RT_I2C_RD|RT_I2C_NO_START;
    msgs[1].buf = data;
    msgs[1].len = 1;
	
//	count = rt_i2c_master_send(i2c_bus,addr,0,buf,1);
//	count = rt_i2c_master_recv(i2c_bus,addr,RT_I2C_NO_START,data,1);
    
	
    if (rt_i2c_transfer(i2c_bus, msgs, 2) != 0)
    {
        return 0 ;
    }
    else
    {
		cam_err("[%s] error\r\n",__func__);
        return -1;
    }
	
}

int imx_cam_sensor_write_reg(struct rt_i2c_bus_device *i2c_bus,rt_uint16_t addr, rt_uint8_t reg, rt_uint8_t data)
{
	struct rt_i2c_msg msgs;
	rt_uint8_t buf[3];

	buf[0] = reg >> 8; //cmd
	buf[1] = reg & 0xff;
	buf[2] = data;

	msgs.addr = addr;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = 3;

	
	if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
	{
		return 0;
	}
	else
	{
		cam_err("[%s] error\r\n",__func__);
		return -1;
	}
}

int imx_cam_sensor_readb2_reg(struct rt_i2c_bus_device *i2c_bus,rt_uint16_t addr, rt_uint16_t reg, rt_uint8_t *data)
{
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3];
	
	
	buf[0] = reg >> 8; //cmd
	buf[1] = reg & 0xff;
    msgs[0].addr = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 2;
	
	msgs[1].addr = addr;
    msgs[1].flags = RT_I2C_RD|RT_I2C_NO_START;
    msgs[1].buf = data;
    msgs[1].len = 1;
		
    if (rt_i2c_transfer(i2c_bus, msgs, 2) != 0)
    {
        return 0 ;
    }
    else
    {
		cam_err("[%s] error\r\n",__func__);
        return -1;
    }
	
}

int imx_cam_sensor_writeb2_reg(struct rt_i2c_bus_device *i2c_bus,rt_uint16_t addr, rt_uint16_t reg, rt_uint8_t data)
{
	struct rt_i2c_msg msgs;
	rt_uint8_t buf[3];

	buf[0] = reg >> 8; //cmd
	buf[1] = reg & 0xff;
	buf[2] = data;

	msgs.addr = addr;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = 3;

	
	if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
	{
		return 0;
	}
	else
	{
		cam_err("[%s] error\r\n",__func__);
		return -1;
	}
}

int imx_cam_sensor_cambus_writeb(sensor_t *sensor, uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data)
{
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	return imx_cam_sensor_write_reg(i2c_bus,slv_addr,reg_addr,reg_data);
}
int imx_cam_sensor_cambus_readb(sensor_t *sensor, uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data)
{
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	return imx_cam_sensor_read_reg(i2c_bus,slv_addr,reg_addr,reg_data);
}

int imx_cam_sensor_cambus_writeb2(sensor_t *sensor, uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data)
{
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	return imx_cam_sensor_writeb2_reg(i2c_bus,slv_addr,reg_addr,reg_data);
}
int imx_cam_sensor_cambus_readb2(sensor_t *sensor, uint8_t slv_addr, uint16_t reg_addr, uint8_t *reg_data)
{
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	return imx_cam_sensor_readb2_reg(i2c_bus,slv_addr,reg_addr,reg_data);
}

int imx_cam_sensor_cambus_readw(sensor_t *sensor, uint8_t slv_addr, uint8_t reg_addr, uint16_t *reg_data)
{
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[3];
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	buf[0] = reg_addr; //cmd
    msgs[0].addr = slv_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 1;
	
	msgs[1].addr = reg_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = (uint8_t *)reg_data;
    msgs[1].len = 1;
	
    
    if (rt_i2c_transfer(i2c_bus, msgs, 2) != 0)
    {
        return 0 ;
    }
    else
    {
		cam_err("[%s] error\r\n",__func__);
        return -1;
    }
}

int imx_cam_sensor_cambus_writew(sensor_t *sensor, uint8_t slv_addr, uint8_t reg_addr, uint16_t reg_data)
{
	struct rt_i2c_msg msgs;
	rt_uint8_t buf[3];
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	buf[0] = reg_addr; //cmd
	buf[1] = reg_data & 0x00ff;
	buf[2] = reg_data >> 8;
	
	msgs.addr = slv_addr;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = 3;

	
	if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
	{
		return 0;
	}
	else
	{
		cam_err("[%s] error\r\n",__func__);
		return -1;
	}
}

#ifdef RT_USING_MIPI_CSI
#include "fsl_mipi_csi2rx.h"
#define FSL_VIDEO_RESOLUTION(width, height) ((uint32_t)(width) | ((uint32_t)(height) << 16U))
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

void imx_cam_sensor_mipi_init(){
	    /* This clock should be equal or faster than the receive byte clock,
     * D0_HS_BYTE_CLKD, from the RX DPHY. For this board, there are two
     * data lanes, the MIPI CSI pixel format is 16-bit per pixel, the
     * max resolution supported is 720*1280@30Hz, so the MIPI CSI2 clock
     * should be faster than 720*1280*30 = 27.6MHz, choose 60MHz here.
     */
    const clock_root_config_t csi2ClockConfig = {
        .clockOff = false,
        .mfn      = 0,
        .mfd      = 0,
        .mux      = 5,
        .div      = 4,
    };

    /* ESC clock should be in the range of 60~80 MHz */
    const clock_root_config_t csi2EscClockConfig = {
        .clockOff = false,
        .mfn      = 0,
        .mfd      = 0,
        .mux      = 5,
        .div      = 7,
    };

    /* UI clock should be equal or faster than the input pixel clock.
     * The camera max resolution supported is 720*1280@30Hz, so this clock
     * should be faster than 720*1280*30 = 27.6MHz, choose 60MHz here.
     */
    const clock_root_config_t csi2UiClockConfig = {
        .clockOff = false,
        .mfn      = 0,
        .mfd      = 0,
        .mux      = 5,
        .div      = 8,
    };

    /* MIPI CSI2 connect to CSI. */
    CLOCK_EnableClock(kCLOCK_Video_Mux);
    VIDEO_MUX->VID_MUX_CTRL.SET = (VIDEO_MUX_VID_MUX_CTRL_CSI_SEL_MASK);

    CLOCK_SetRootClock(kCLOCK_Root_Csi2, &csi2ClockConfig);
    CLOCK_SetRootClock(kCLOCK_Root_Csi2_Esc, &csi2EscClockConfig);
    CLOCK_SetRootClock(kCLOCK_Root_Csi2_Ui, &csi2UiClockConfig);
}
void imx_cam_sensor_mipi_cfg(struct imxrt_camera *cam){
	csi2rx_config_t csi2rxConfig = {0};
    /*
     * Initialize the MIPI CSI2
     *
     * From D-PHY specification, the T-HSSETTLE should in the range of 85ns+6*UI to 145ns+10*UI
     * UI is Unit Interval, equal to the duration of any HS state on the Clock Lane
     *
     * T-HSSETTLE = csi2rxConfig.tHsSettle_EscClk * (Tperiod of RxClkInEsc)
     *
     * csi2rxConfig.tHsSettle_EscClk setting for camera:
     *
     *    Resolution  |  frame rate  |  T_HS_SETTLE
     *  =============================================
     *     720P       |     30       |     0x12
     *  ---------------------------------------------
     *     720P       |     15       |     0x17
     *  ---------------------------------------------
     *      VGA       |     30       |     0x1F
     *  ---------------------------------------------
     *      VGA       |     15       |     0x24
     *  ---------------------------------------------
     *     QVGA       |     30       |     0x1F
     *  ---------------------------------------------
     *     QVGA       |     15       |     0x24
     *  ---------------------------------------------
     */
    static const uint32_t csi2rxHsSettle[][3] = {
        {
            kVIDEO_Resolution720P,
            FRAMERATE_30FPS,
            0x12,
        },
        {
            kVIDEO_Resolution720P,
            FRAMERATE_15FPS,
            0x17,
        },
        {
            kVIDEO_ResolutionVGA,
            FRAMERATE_30FPS,
            0x1F,
        },
        {
            kVIDEO_ResolutionVGA,
            FRAMERATE_15FPS,
            0x24,
        },
        {
            kVIDEO_ResolutionQVGA,
            FRAMERATE_30FPS,
            0x1F,
        },
        {
            kVIDEO_ResolutionQVGA,
            FRAMERATE_15FPS,
            0x24,
        },
    };

    csi2rxConfig.laneNum          = 2;
    csi2rxConfig.tHsSettle_EscClk = 0x12;

    for (uint8_t i = 0; i < ARRAY_SIZE(csi2rxHsSettle); i++)
    {
        if ((FSL_VIDEO_RESOLUTION(cam->sensor.fb_w, cam->sensor.fb_h) == csi2rxHsSettle[i][0]) &&
            (csi2rxHsSettle[i][1] == cam->sensor.framerate))
        {
            csi2rxConfig.tHsSettle_EscClk = csi2rxHsSettle[i][2];
			CSI2RX_Init(MIPI_CSI2RX, &csi2rxConfig);
			return;
        }
    }
	cam_err("Not support FrameSize & FrameRate, only support QVGA/VGA/720P and (15/30FPS) now\r\n");
}
#endif

extern void BOARD_Camera_pwd_io_set(int value);
extern void BORAD_Camera_rst_io_set(int value);
void imx_cam_sensor_init(struct imxrt_camera *cam)
{
	struct rt_i2c_bus_device *i2c_bus;
	
	
    
	imx_cam_sensor_io_init(cam->sensor_pwn_io,cam->sensor_pwn_io_pin);
	imx_cam_sensor_io_init(cam->sensor_rst_io, cam->sensor_rst_io_pin);
	
	i2c_bus = rt_i2c_bus_device_find(cam->sensor_bus_name);
	if(i2c_bus == RT_NULL)
	{
		cam_err("[%s]driver can not find %s bus\r\n",__func__,cam->sensor_bus_name);
		return ;
	}
	cam->sensor.i2c_bus = (uint32_t *)i2c_bus;
	cam->sensor.cambus_readb = imx_cam_sensor_cambus_readb;
	cam->sensor.cambus_writeb = imx_cam_sensor_cambus_writeb;
	cam->sensor.cambus_readw = imx_cam_sensor_cambus_readw;
	cam->sensor.cambus_writew = imx_cam_sensor_cambus_writew;
	cam->sensor.cambus_readb2 = imx_cam_sensor_cambus_readb2;
	cam->sensor.cambus_writeb2 = imx_cam_sensor_cambus_writeb2;
	cam->sensor_addr = OV5640_SLV_ADDR >> 1;
	cam->sensor.slv_addr = cam->sensor_addr;

#ifdef 	SOC_IMXRT1170_SERIES
#else	
	CLOCK_SetMux(kCLOCK_CsiMux, 0);
    CLOCK_SetDiv(kCLOCK_CsiDiv, 0);
#endif	

	//reset camera
	//power down to low
	BOARD_Camera_pwd_io_set(0);
	//reset 
	BORAD_Camera_rst_io_set(0);
	rt_thread_delay(10);
	BORAD_Camera_rst_io_set(1);
	rt_thread_delay(10);
#if 0	
	cam->sensor_addr = imx_cam_sensor_scan(i2c_bus);
	if (cam->sensor_addr == 0)
	{
		cam->sensor.reset_pol = ACTIVE_HIGH;
		BORAD_Camera_rst_io_set(0);
		rt_thread_delay(10);
		cam->sensor_addr = imx_cam_sensor_scan(i2c_bus);
		if (cam->sensor_addr == 0)
		{
			cam->sensor.pwdn_pol = ACTIVE_HIGH;
			BOARD_Camera_pwd_io_set(1);
			rt_thread_delay(10);
			cam->sensor_addr = imx_cam_sensor_scan(i2c_bus);
			if (cam->sensor_addr == 0)
			{
				cam->sensor.reset_pol = ACTIVE_LOW;
				BORAD_Camera_rst_io_set(1);
				rt_thread_delay(10);
				cam->sensor_addr = imx_cam_sensor_scan(i2c_bus);
				if (cam->sensor_addr == 0)
				{
					cam_err("[%s] can not find any camera device\r\n",__func__);
					return;
				}
			}
		}
	}
#endif	
	#ifdef SENSOR_OV5640
	imx_cam_sensor_readb2_reg(i2c_bus,cam->sensor_addr,OV5640_CHIP_ID, &cam->sensor_id);
	if(cam->sensor_id == OV5640_ID)
	{
		ov5640_init(&cam->sensor);
		cam->sensor.isMipi = 1;
	}
	#else
	imx_cam_sensor_read_reg(i2c_bus,cam->sensor_addr,ON_CHIP_ID, &cam->sensor_id);
	if(cam->sensor_id == MT9V034_ID)
	{
<<<<<<< HEAD
	#ifdef SENSOR_MT9V034
		mt9v034_init(&cam->sensor);
	#endif	
=======
		#ifdef SENSOR_MT9V034
		mt9v034_init(&cam->sensor);
		#endif
>>>>>>> update the camera & LCD drivers
	}
	else
	{
		imx_cam_sensor_read_reg(i2c_bus,cam->sensor_addr,OV_CHIP_ID, &cam->sensor_id);
		cam_echo("Camera Device id:0x%x\r\n",cam->sensor_id);
		switch(cam->sensor_id)
		{
<<<<<<< HEAD
			case MT9V034_ID:
				#ifdef SENSOR_MT9V034
				mt9v034_init(&cam->sensor);
				#endif
				break;
=======
			#ifdef SENSOR_OV9650
>>>>>>> update the camera & LCD drivers
			case OV9650_ID:
				#ifdef SENSOR_OV9650
				ov9650_init(&cam->sensor);
				#endif
				break;
			#endif
			#ifdef SENSOR_OV2640
			case OV2640_ID:
				#ifdef SENSOR_OV2640
				ov2640_init(&cam->sensor);
				#endif
				break;
			#endif
			#ifdef SENSOR_OV7725
			case OV7725_ID:
				#ifdef SENSOR_OV7725
				ov7725_init(&cam->sensor);
				#endif
				break;
			#endif
			default:
				cam_err("[%s] sensor id:0x%2x not support\r\n",__func__,cam->sensor_id);
				break;
		}
	
	}
	#endif
	#ifdef RT_USING_MIPI_CSI
	imx_cam_sensor_mipi_init();
	#endif

}

int imxrt_camera_set_framerate(struct imxrt_camera *cam,framerate_t framerate)
{
    if (cam->sensor.framerate == framerate) {
       /* no change */
        return 0;
    }

#ifdef 	SOC_IMXRT1170_SERIES
#else		
	if (framerate & 0x80000000)
		CCM->CSCDR3 = framerate & (0x1F<<9);
#endif
    if (cam->sensor.set_framerate == NULL
        || cam->sensor.set_framerate(&cam->sensor, framerate) != 0) {
        /* operation not supported */
        return -1;
    }
	
	/* set the frame rate */
    cam->sensor.framerate = framerate;
	
	#ifdef RT_USING_MIPI_CSI
	imx_cam_sensor_mipi_cfg(cam);
	#endif

    return 0;
}

void imx_cam_reset(struct imxrt_camera *cam)
{
	//sensor init
	imx_cam_sensor_init(cam);
	//csi init
	CsiFragModeInit(cam->csi_base);
	
	cam->sensor.isWindowing = 0;
	cam->sensor.wndH = cam->sensor.fb_h = 0;
	cam->sensor.wndW = cam->sensor.fb_w = 0;
	cam->sensor.wndX = cam->sensor.wndY = 0;	
	cam->sensor.sde          = 0xFF;
    cam->sensor.pixformat    = 0xFF;
    cam->sensor.framesize    = 0xFF;
    cam->sensor.framerate    = FRAMERATE_30FPS;
    cam->sensor.gainceiling  = 0xFF;


    // Call sensor-specific reset function; in the moment,we use our init function and defaults regs
    if (cam->sensor.reset)
		cam->sensor.reset(&cam->sensor);
	
	PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);
}

void imxrt_cam_buffer_list_init(struct imxrt_camera *cam)
{
	uint8_t bpp;
	int16_t w,h;
	uint32_t size;
	
	w = cam->sensor.isWindowing ? cam->fb_list.w : resolution[cam->sensor.framesize][0];
	h = cam->sensor.isWindowing ? cam->fb_list.h : resolution[cam->sensor.framesize][1];
	switch (cam->sensor.pixformat) {
        case PIXFORMAT_GRAYSCALE:
            bpp = 1;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_RGB565:
            bpp = 2;
            break;
        case PIXFORMAT_BAYER:
            bpp = 3;
            break;
        case PIXFORMAT_JPEG:
            // Read the number of data items transferred
            // MAIN_FB()->bpp = (MAX_XFER_SIZE - __HAL_DMA_GET_COUNTER(&DMAHandle))*4;
            break;
		default:
			break;
    }
	
	size = w * h * bpp;
	cam->fb_list.w = w;
	cam->fb_list.h = h;
	cam->fb_list.bpp = bpp;
	cam->fb_list.frame_size = size;
	cam->fb_list.buffer_start = (rt_uint32_t *)cam->fb_buffer_start;
	cam->fb_list.idx = 0;
#if 1
	struct fb_mem *fb_mem = (struct fb_mem *)cam->fb_list.buffer_start;
	
	cam->s_irq.base0 = (uint32_t)fb_mem->ptr;
	cam->fb_list.count = FRAMEBUFFER_COUNT;
	memset(cam->fb_list.buffer_start,0x00,FRAMEBUFFER_COUNT*(FB_MEM_HEAD_SIZE + FRAMEBUFFER_SIZE));
	cam->fb_list.total_frame_count = 0;
	cam_echo("Cam buffer start at 0x%x\r\n",fb_mem->ptr);
#else		
	cam->fb_list.count = (cam->fb_buffer_end - cam->fb_buffer_start) / (size + sizeof(framebuffer_t));
	
	if(cam->fb_list.count <= 1)
		cam_err("FrameBufer too small !!");
	cam_echo("Cam Frame size %d buffer count %d\r\n",size, cam->fb_list.count);
	cam->fb_list.rd_idx = 0;
	cam->fb_list.wr_idx = 0;
	struct fb_mem *fb_mem = (struct fb_mem *)cam->fb_list.buffer_start;
	cam->s_irq.base0 = (uint32_t)&fb_mem->ptr;
#endif	
}

#if defined(__CC_ARM)
#define ARMCC_ASM_FUNC	__asm
ARMCC_ASM_FUNC __attribute__((section(".ram_code"))) uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
	push	{r4-r7, lr}
10
	LDMIA	R0!, {r3-r6}
	// schedule code carefully to allow dual-issue on Cortex-M7
	bfi		r7, r3, #0, #8	// Y0
	bfi		ip, r5, #0, #8	// Y4
	lsr		r3,	r3,	#16
	lsr		r5,	r5,	#16
	bfi		r7, r3, #8, #8	// Y1
	bfi		ip, r5, #8, #8  // Y5
	bfi		r7, r4, #16, #8 // Y2
	bfi		ip, r6, #16, #8 // Y6
	lsr		r4,	r4,	#16
	lsr		r6,	r6,	#16
	bfi		r7, r4, #24, #8 // Y3
	bfi		ip, r6, #24, #8	// Y7
	STMIA	r1!, {r7, ip}
	
	subs	r2,	#1
	bne		%b10
	mov		r0,	r1
	pop		{r4-r7, pc}
}
#elif defined(__CLANG_ARM)
__attribute__((section(".ram_code"))) uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
	__asm (
		"	push	{r1-r7, ip, lr}  \n "
		"10:  \n "
		"	ldmia	r0!, {r3-r6}  \n "
			// schedule code carefully to allow dual-issue on Cortex-M7
		"	bfi		r7, r3, #0, #8  \n "	// Y0
		"	bfi		ip, r5, #0, #8  \n "	// Y4
		"	lsr		r3,	r3,	#16  \n "
		"	lsr		r5,	r5,	#16  \n "
		"	bfi		r7, r3, #8, #8  \n "	// Y1
		"	bfi		ip, r5, #8, #8  \n "  // Y5
		"	bfi		r7, r4, #16, #8  \n " // Y2
		"	bfi		ip, r6, #16, #8  \n " // Y6
		"	lsr		r4,	r4,	#16  \n "
		"	lsr		r6,	r6,	#16  \n "
		"	bfi		r7, r4, #24, #8  \n " // Y3
		"	bfi		ip, r6, #24, #8  \n "	// Y7
		"	stmia	r1!, {r7, ip}  \n "	
		"	subs	r2,	#1  \n "
		"	bne		10b  \n "
		"	mov		r0,	r1  \n "
		"	pop		{r1-r7, ip, pc}  \n "		
	);
	
	return 0;
}
#else
__attribute__((naked))
RAM_CODE uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
	__asm volatile (
		"	push	{r1-r7, ip, lr}  \n "
		"10:  \n "
		"	ldmia	r0!, {r3-r6}  \n "
			// schedule code carefully to allow dual-issue on Cortex-M7
		"	bfi		r7, r3, #0, #8  \n "	// Y0
		"	bfi		ip, r5, #0, #8  \n "	// Y4
		"	lsr		r3,	r3,	#16  \n "
		"	lsr		r5,	r5,	#16  \n "
		"	bfi		r7, r3, #8, #8  \n "	// Y1
		"	bfi		ip, r5, #8, #8  \n "  // Y5
		"	bfi		r7, r4, #16, #8  \n " // Y2
		"	bfi		ip, r6, #16, #8  \n " // Y6
		"	lsr		r4,	r4,	#16  \n "
		"	lsr		r6,	r6,	#16  \n "
		"	bfi		r7, r4, #24, #8  \n " // Y3
		"	bfi		ip, r6, #24, #8  \n "	// Y7
		"	stmia	r1!, {r7, ip}  \n "	
		"	subs	r2,	#1  \n "
		"	bne		10b  \n "
		"	mov		r0,	r1  \n "
		"	pop		{r1-r7, ip, pc}  \n "		
	);
}

#endif
#if defined(__CC_ARM) || defined(__CLANG_ARM)
__attribute__((section(".ram_code"))) __attribute__((naked)) void rgb32Torgb565(uint32_t *src, uint16_t *dst, uint32_t len){
#else 
__attribute__((naked)) void rgb32Torgb565(uint32_t *src, uint16_t *dst, uint32_t len){	
#endif
	__asm volatile(
	"push {r3-r11, lr} \n"	
	// handle the remaining 8bytes first 
	"loop: \n"
	"	ldmia r0!, {r4-r11} \n"
	
	"	lsrs r4, #3 \n"      // pix 1
	"	lsls r5, #8 \n"		// pix 2

	"	ubfx r3,r4,#8+2-3, #6 \n"  //green
	"	ubfx ip,r5,#8+2+8, #6 \n"
	
	"	bfi r4,r3,#0+5,#6 \n"
	"	bfi r5,ip,#16+5,#6 \n"
	
	"	ubfx r3,r4,#16+3-3,#5 \n" //red
	"	ubfx ip,r5,#3+8,#5 \n"   //blue

	"	bfi r4,r3,#5+6,#5 \n"
	"	bfi r5,ip,#16,#5 \n"
	
	"	bfi r5,r4,#0,#16 \n"
	"	nop \n"
	
	"	lsrs r6, #3 \n"      // pix 3
	"	lsls r7, #8 \n"		// pix 4

	"	ubfx r3,r6,#8+2-3, #6 \n"  //green
	"	ubfx ip,r7,#8+2+8, #6 \n"
	
	"	bfi r6,r3,#0+5,#6 \n"
	"	bfi r7,ip,#16+5,#6 \n"
	
	"	ubfx r3,r6,#16+3-3,#5 \n" //red
	"	ubfx ip,r7,#3+8,#5 \n"   //blue

	"	bfi r6,r3,#5+6,#5 \n"
	"	bfi r7,ip,#16,#5 \n"
	
	"	bfi r7,r6,#0,#16 \n"
	"	nop \n"
	
	"	lsrs r8, #3 \n"      // pix 5
	"	lsls r9, #8 \n"		// pix 6

	"	ubfx r3,r8,#8+2-3, #6 \n"  //green
	"	ubfx ip,r9,#8+2+8, #6 \n"
	
	"	bfi r8,r3,#0+5,#6 \n"
	"	bfi r9,ip,#16+5,#6 \n"
	
	"	ubfx r3,r8,#16+3-3,#5 \n" //red
	"	ubfx ip,r9,#3+8,#5 \n"   //blue

	"	bfi r8,r3,#5+6,#5 \n"
	"	bfi r9,ip,#16,#5 \n"
	
	"	bfi r9,r8,#0,#16 \n"
	"	nop \n"
	
	"	lsrs r10, #3 \n"      // pix 7
	"	lsls r11, #8 \n"		// pix 8

	"	ubfx r3,r10,#8+2-3, #6 \n"  //green
	"	ubfx ip,r11,#8+2+8, #6 \n"
	
	"	bfi r10,r3,#0+5,#6 \n"
	"	bfi r11,ip,#16+5,#6 \n"
	
	"	ubfx r3,r10,#16+3-3,#5 \n" //red
	"	ubfx ip,r11,#3+8,#5 \n"   //blue

	"	bfi r10,r3,#5+6,#5 \n"
	"	bfi r11,ip,#16,#5 \n"
	
	"	bfi r11,r10,#0,#16 \n"
	"	nop \n"
		
	"	stmia r1!,{r5,r7,r9,r11} \n"
	"	subs r2, #32 \n"
	"	bne loop \n"
	"	pop {r3-r11, pc} \n"		
		);
}
void rgb32Torgb565_c(uint32_t *src, uint16_t *dst, uint32_t len){
	uint8_t r,g,b;
	for(int i=0;i<len;i+=4){
		uint32_t col = *(src++);
		r = (col>>16) & 0xff;
		g = (col>>8) & 0xff;
		b = col & 0xff;
		*dst++ = ((r >> 3) << 11) | ((g>>2)<<5) | ((b>>3));	
	}
}

__attribute__((naked)) void copy2mem(void* dst, void* src, uint32_t len){
	__asm volatile(
		"push {r3-r4, lr}\n"
		"loop1: \n"
		"ldrd r3, r4, [r1], #8\n"
		"strd r3, r4, [r0], #8\n"
		"subs r2, #8\n"
		"bne loop1\n"
		"pop {r3-r4, lr}\n"
		);
}

uint64_t rt_tick_get_us(){
	uint64_t tick = rt_tick_get();
	uint64_t us = 1000000 * (SysTick->LOAD - SysTick->VAL) / SystemCoreClock;
	us += tick * 1000;
	return us;
}
uint32_t g_start,g_s1,g_e1, g_csi_end;
uint64_t g_csi_start, g_exit;
int32_t irq_time = 0;
static int sync_found = 0;
static struct fb_mem *fb_found=NULL;
bool SOF = true;

volatile int a = 0;
volatile int enter_irq = 0;
#if defined(__CC_ARM) || defined(__CLANG_ARM)
__attribute__((section(".ram_code"))) void CSI_IRQHandler(void) {
#else
void CSI_IRQHandler(void) {
#endif	
	enter_irq ++;
	if (pCam == NULL)
		return;
	rt_interrupt_enter();
	#ifdef GPIO_DEBUGING 
	HIGH(NNCU_RATE);
	#endif
    volatile uint32_t csisr = pCam->csi_base->CSISR;
	if((csisr & (1<<24))){
		a++;
	#ifdef GPIO_DEBUGING 	
		HIGH(NNCU_INVOKE_RATE);
	#endif	
	}
    /* Clear the error flags. */
    pCam->csi_base->CSISR = csisr;

	if ((csisr & (1<<16)) && (SOF)) {
		// indicate the first SOF
		SOF = false;
		//find a buffer
		if (!sync_found)
		{
			struct fb_mem *fb_mem=NULL;
			int count=0;
			do{
				pCam->fb_list.idx = (pCam->fb_list.idx + 1) % pCam->fb_list.count;
				fb_mem = (struct fb_mem *)((uint32_t)pCam->fb_list.buffer_start + pCam->fb_list.idx * (FB_MEM_HEAD_SIZE + FRAMEBUFFER_SIZE));
				if (fb_mem->lock == 0 && g_framePtr != fb_mem)
				{
					fb_found = fb_mem;
					break;
				}
				count ++;
			}while(count < pCam->fb_list.count);
			
			if (fb_found) 
			{
				pCam->s_irq.base0 = (uint32_t)fb_found->ptr;
			}
			else
			{
				g_framePtr = NULL;
				fb_found = g_framePtr;
			}
			imx_cam_csi_start_frame(pCam);
		}
		sync_found = 1;
		//               SOF    | FB1    | FB2    irqEn
		pCam->csi_base->CSICR1 = 1U<<16 | 1U<<19 | 1U<<20 | CSICR1_INIT_VAL;
		//				 16 doubleWords| RxFifoDmaReqEn| ReflashRFF|ResetFrmCnt
		pCam->csi_base->CSICR3 = 2<<4          | 1<<12         | 1<<14     |1<<15;	
		#ifdef GPIO_DEBUGING 
		HIGH(CSI_FRAME_RATE);
		#endif
		
	}else if(csisr & (3<<19))
	{
		uint32_t dmaBase, lineNdx = pCam->s_irq.dmaFragNdx * pCam->s_irq.linePerFrag;
			if (pCam->s_irq.dmaFragNdx & 1)
				dmaBase = pCam->csi_base->CSIDMASA_FB2;
			else
				dmaBase = pCam->csi_base->CSIDMASA_FB1;

		// we use the dma only transfer one frame, update the dma base and flash as soon as possible
		if (csisr & (1<<19) ) {
			if (pCam->s_irq.isGray)
				pCam->csi_base->CSIDMASA_FB1 += 2 * pCam->s_irq.dmaBytePerFrag; // two buffer, need multi 2
		} else {
			if (pCam->s_irq.isGray)
				pCam->csi_base->CSIDMASA_FB2 += 2 * pCam->s_irq.dmaBytePerFrag;
			pCam->csi_base->CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK;	// reflash DMA
			__DSB();
		}
		if (dmaBase >= 0x20200000)
			DCACHE_CleanInvalidateByRange(dmaBase, pCam->s_irq.dmaBytePerFrag);
		if (pCam->s_irq.isGray)
		{
			/* cost too much time
			if(pCam->sensor.isWindowing &&  lineNdx >= pCam->sensor.wndY && lineNdx - pCam->sensor.wndY <= pCam->sensor.wndH)
			{
				dmaBase += pCam->sensor.wndX * pCam->s_irq.linePerFrag;	// apply line window offset, does not begin from the 0,0, need follow
				uint32_t byteToCopy = (pCam->sensor.wndW * pCam->s_irq.linePerFrag);
				copy2mem((void*)pCam->s_irq.datCurBase, (void*)dmaBase, byteToCopy);
				pCam->s_irq.datCurBase += byteToCopy;
			}
			*/
		}
		else{
			uint32_t byteToCopy = pCam->s_irq.dmaBytePerFrag;
			bool copy_window = false;
			if(pCam->sensor.isWindowing &&  lineNdx >= pCam->sensor.wndY && lineNdx - pCam->sensor.wndY <= pCam->sensor.wndH)
			{
				copy_window = true;
				dmaBase += pCam->sensor.wndX * 4 * pCam->s_irq.linePerFrag;	// apply line window offset, does not begin from the 0,0, need follow, 4 means RGB8888
				byteToCopy = (pCam->sensor.wndW * pCam->s_irq.linePerFrag) << 2; // rgb8888 one pixel
			}
			// handler the XRGB8888 to rgb565 line by line directly, seems that costs too much time
			if(!pCam->sensor.isWindowing || copy_window){
				rgb32Torgb565((uint32_t*)dmaBase, (uint16_t*)pCam->s_irq.datCurBase, byteToCopy);
				pCam->s_irq.datCurBase += byteToCopy >> 1; //rgb565 in mem
			}
		}
		
		if (++pCam->s_irq.dmaFragNdx == pCam->s_irq.fragCnt || (csisr & (3<<19)) == 3<<19 )
		{
			SOF = true;
			#ifdef GPIO_DEBUGING 
			LOW(CSI_FRAME_RATE);
			#endif
			//				 16 doubleWords| ReflashRFF
			pCam->csi_base->CSICR3 = 2<<4		   | 1<<14;
			//send evt to cam thread
			rt_event_send(&frame_event, EVENT_CSI);

			sync_found = 0;
			if (fb_found)
				g_framePtr = fb_found;
			
			if(isToStopCSICommand || (pCam->fb_list.count == 1))
			{
				NVIC_DisableIRQ(CSI_IRQn);
				CSI_Stop(CSI);
				isToStopCSICommand = 0;
				g_csi_int_en = 1;
				struct rt_camera_device *sensor = imxrt_camera_device_find(SENSOR_NAME);
				sensor->status = RT_CAMERA_DEVICE_INIT;
			}
			else
			{
				
			}
		}
		
	}
Cleanup:
#ifdef GPIO_DEBUGING 	
	LOW(NNCU_RATE);
	LOW(NNCU_INVOKE_RATE);
#endif	
	rt_interrupt_leave();
	return;
}

void imx_cam_csi_fragmode_calc(struct imxrt_camera *cam) {
	
	cam->s_irq.dmaBytePerLine = cam->sensor.fb_w * 4;
	if (cam->sensor.pixformat == PIXFORMAT_GRAYSCALE) {
		cam->s_irq.dmaBytePerLine /= 4;
		cam->s_irq.isGray = 1;
		cam->sensor.gs_bpp = 1;
	} else {
		cam->s_irq.isGray = 0;
		cam->sensor.gs_bpp = 2;
	}
	if (cam->sensor.fb_w == 0 || cam->sensor.fb_h == 0)
		return;

	// calculate max bytes per DMA frag
	uint32_t dmaBytePerFrag, byteStep, dmaByteTotal;
	uint32_t maxBytePerLine = sizeof(s_dmaFragBufs) / ARRAY_SIZE(s_dmaFragBufs);
	dmaByteTotal = cam->s_irq.isGray?cam->sensor.fb_w * cam->sensor.fb_h:cam->sensor.fb_w * cam->sensor.fb_h * 4; // now only the grayscale did not need to re-do	
	if (cam->sensor.wndX == 0 && cam->sensor.wndY == 0) // (s_irq.isGray)
	{
		dmaBytePerFrag = cam->s_irq.dmaBytePerLine;  // set a minial default value
		for (byteStep = cam->s_irq.dmaBytePerLine; byteStep < maxBytePerLine; byteStep += cam->s_irq.dmaBytePerLine) {
			if (0 == byteStep % 32 )
			{
				// find maximum allowed bytes per frag
				dmaBytePerFrag = (maxBytePerLine / byteStep) * byteStep;
				for (; dmaBytePerFrag >= byteStep; dmaBytePerFrag -= byteStep) {
					if (dmaByteTotal % dmaBytePerFrag == 0)
						break;
				}
				if (dmaBytePerFrag < byteStep) {
					dmaBytePerFrag = byteStep;
					while (1) {}
				}
				break;
			}
		}
	} 
	else {
		// for window mode, we only accept 1 line per frag
		dmaBytePerFrag = cam->s_irq.dmaBytePerLine;
	}
	cam->s_irq.linePerFrag = dmaBytePerFrag / cam->s_irq.dmaBytePerLine;
	cam->s_irq.dmaBytePerFrag = dmaBytePerFrag;

	// >>> calculate how many lines per fragment (DMA xfer unit)
	uint32_t burstBytes;
	if (!(cam->s_irq.dmaBytePerLine % (8 * 16)))
	{
		burstBytes = 128;
		cam->csi_base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(3U);
		cam->csi_base->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((7U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	else if (!(cam->s_irq.dmaBytePerLine % (8 * 8)))
	{
		burstBytes = 64;
		cam->csi_base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(2U);
		cam->csi_base->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((1U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	else
	{
		burstBytes = 32;
		cam->csi_base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(1U);
		cam->csi_base->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((0U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	cam->s_irq.fragCnt = cam->sensor.fb_h / cam->s_irq.linePerFrag;
	// <<<
}

#define enable_camera() \
//	pCam->sensor.sleep(&pCam->sensor, false); \
//	for(int i=0;i<1000000;i++); \
//	imx_cam_sensor_cambus_writeb2(&pCam->sensor, pCam->sensor.slv_addr, 0x3007, 0xff); \
//	imx_cam_sensor_cambus_writeb2(&pCam->sensor, pCam->sensor.slv_addr, 0x3008, 0x02); \
//	rt_thread_mdelay(10);

#define disable_camera() \
//	pCam->sensor.sleep(&pCam->sensor, true); \
//	imx_cam_sensor_cambus_writeb2(&pCam->sensor, pCam->sensor.slv_addr, 0x3007, 0xe7); \
//	imx_cam_sensor_cambus_writeb2(&pCam->sensor, pCam->sensor.slv_addr, 0x3008, 0x42); \
//	rt_thread_mdelay(10);

bool g_first_start;
void imx_cam_csi_start_frame(struct imxrt_camera *cam)
{	
	uint32_t start = rt_tick_get_us();
	if(!g_first_start) {
		imx_cam_csi_fragmode_calc(cam);
		g_first_start = true;
	}
	cam->s_irq.dmaFragNdx = 0;
	cam->s_irq.cnt++;
	// DMA also writes to this cache line, to avoid being invalidated, clean MAIN_FB header.
	DCACHE_CleanByRange((uint32_t)cam->s_irq.base0, 32);
	if (cam->s_irq.isGray){
		cam->csi_base->CSIDMASA_FB1 = (uint32_t) cam->s_irq.base0;
		cam->csi_base->CSIDMASA_FB2 = (uint32_t) cam->s_irq.base0 + cam->s_irq.dmaBytePerFrag;
	} else {
		// re-use the s_dmaFragBufs, handle the RGB8888 to rgb565
		cam->csi_base->CSIDMASA_FB1 = (uint32_t) s_dmaFragBufs[0];
		cam->csi_base->CSIDMASA_FB2 = (uint32_t) s_dmaFragBufs[1];
	}
	cam->s_irq.datCurBase = cam->s_irq.base0;
	cam->csi_base->CSICR1 = CSICR1_INIT_VAL;
	if (cam->s_irq.dmaBytePerFrag & 0xFFFF0000) {
		
		uint32_t l16 = cam->s_irq.linePerFrag , h16 = cam->s_irq.dmaBytePerLine << 16;
		cam->csi_base->CSIIMAG_PARA = l16 | h16;
	} else {
		#if 1
		cam->csi_base->CSIIMAG_PARA = cam->s_irq.linePerFrag | (cam->sensor.fb_w) << 16;	// set xfer cnt
		#else
		cam->csi_base->CSIIMAG_PARA = cam->s_irq.linePerFrag | dmaBytePerFrag << 16;
		#endif
	}
	__set_PRIMASK(1);
	cam->csi_base->CSISR = cam->csi_base->CSISR;
	cam->csi_base->CSICR1 |= 1<<16;	// enable SOF iRQ
	cam->csi_base->CSICR18 |= 1U<<31;	// start CSI
	NVIC_EnableIRQ(CSI_IRQn);
	enable_camera();
	g_csi_int_en = 1;
	__set_PRIMASK(0);
}

void imx_cam_start_snapshot(struct imxrt_camera *cam)
{
	pCam = cam;
	//init buffer list
	//imxrt_cam_buffer_list_init(cam);
	//start csi
	uint8_t bpp;
	int16_t w,h;
	uint32_t size;
	
	w = cam->sensor.isWindowing ? cam->fb_list.w : resolution[cam->sensor.framesize][0];
	h = cam->sensor.isWindowing ? cam->fb_list.h : resolution[cam->sensor.framesize][1];
	switch (cam->sensor.pixformat) {
        case PIXFORMAT_GRAYSCALE:
            bpp = 1;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_RGB565:
            bpp = 2;
            break;
        case PIXFORMAT_BAYER:
            bpp = 3;
            break;
        case PIXFORMAT_JPEG:
            // Read the number of data items transferred
            // MAIN_FB()->bpp = (MAX_XFER_SIZE - __HAL_DMA_GET_COUNTER(&DMAHandle))*4;
            break;
		default:
			break;
    }
	
	size = w * h * bpp;
	cam->fb_list.bpp = bpp;
	cam->fb_list.frame_size = size;
	imx_cam_csi_start_frame(cam);
}

void imx_cam_csi_stop(struct rt_camera_device *cam)
{
	cam->status = RT_CAMERA_DEVICE_SUSPEND;
	CSI_Stop(CSI);
	CSI->CSICR3 = 2<<4		   | 1<<14;
	NVIC_DisableIRQ(CSI_IRQn);
	SOF = true;
	g_csi_int_en = 0;
	g_first_start = 0;
}

void imx_cam_sensor_set_contrast(struct imxrt_camera *cam, uint32_t level)
{
	if (cam->sensor.set_contrast != NULL)
		cam->sensor.set_contrast(&cam->sensor,level);
}

void imx_cam_sensor_set_gainceiling(struct imxrt_camera *cam, gainceiling_t gainceiling)
{
	if (cam->sensor.set_gainceiling != NULL && !cam->sensor.set_gainceiling(&cam->sensor, gainceiling))
		cam->sensor.gainceiling = gainceiling;
}

void imx_cam_sensor_set_framesize(struct imxrt_camera *cam, framesize_t framesize)
{
	if(cam->sensor.set_framesize == NULL || cam->sensor.set_framesize(&cam->sensor,framesize) != 0)
		return;
		
	cam->sensor.framesize = framesize;
	
	cam->fb_list.w = cam->sensor.fb_w = resolution[framesize][0];
	cam->fb_list.h = cam->sensor.fb_h = resolution[framesize][1];
	
	cam->sensor.wndX = 0; cam->sensor.wndY = 0 ; cam->sensor.wndW = cam->sensor.fb_w ; cam->sensor.wndH = cam->sensor.fb_h;
	cam->sensor.isWindowing = 0;

	#ifdef RT_USING_MIPI_CSI
	imx_cam_sensor_mipi_cfg(cam);
	#endif
}

void imx_cam_sensor_set_pixformat(struct imxrt_camera *cam, pixformat_t pixformat)
{
	if (cam->sensor.set_pixformat == NULL || cam->sensor.set_pixformat(&cam->sensor,pixformat) != 0)
		return;
	if (cam->sensor.pixformat == pixformat)
		return;
	#ifdef RT_USING_LCD
	PXP_EnableCsc1(PXP, false);
	extern void LCDMonitor_SetPIXFORMAT(int format);
	LCDMonitor_SetPIXFORMAT(pixformat);
	#endif
	switch(pixformat){
		// this is special that grayscale will also use YCbCr(gray:Y, CbCr: 0x40), so need also
		// the color converter
		case PIXFORMAT_GRAYSCALE:
		// to enable the CSI binary mode
		cam->csi_base->CSICR20 |= 1<<31 | 1<<30 | 1<<12 | 4<<9; 	//QRCODE HISTORAGM ORDER INPUT
		case PIXFORMAT_YUV422:
			// Enale the Csc1 is not so well here, move them to update frambuffer
			// we just define it here
			#define ENABLE_PXP_CSC() \
					/* indicate that the Csc function is disable now, open it */ \
					if((PXP->CSC1_COEF1 & PXP_CSC1_COEF0_BYPASS_MASK)){ \
						/* enable the PXP's color converter */ \
						PXP_SetCsc1Mode(PXP, kPXP_Csc1YCbCr2RGB); \
						PXP_EnableCsc1(PXP, true); \
					}
		break;	
		default:
			break;
	}
	
	cam->sensor.pixformat = pixformat;
}

struct fb_mem* old_frame = NULL;
static rt_err_t imx_cam_camera_control(struct rt_camera_device *cam, rt_uint32_t cmd, rt_uint32_t parameter)
{
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)cam->imx_cam;
	switch(cmd)
	{
		case RT_DRV_CAM_CMD_RESET:
			if(cam->status == RT_CAMERA_DEVICE_RUNING)
			{
				imx_cam_csi_stop(cam);
			}
			imx_cam_reset(imx_cam);
			cam->status = RT_CAMERA_DEVICE_INIT;
			break;
		case RT_DRV_CAM_CMD_SET_CONTRAST:
			imx_cam_sensor_set_contrast(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SET_GAINCEILING:
			imx_cam_sensor_set_gainceiling(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SET_FRAMESIZE:
			imx_cam_sensor_set_framesize(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SET_PIXFORMAT:
			imx_cam_sensor_set_pixformat(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SNAPSHOT:
		{
			if(cam->status != RT_CAMERA_DEVICE_RUNING)
			{
				//rt_sem_init(cam->sem,"cam", 0, RT_IPC_FLAG_FIFO);
				imx_cam_start_snapshot((struct imxrt_camera *)cam->imx_cam);
				cam->status = RT_CAMERA_DEVICE_RUNING;
			}
			//rest total count 
			imx_cam->fb_list.total_frame_count = 0;
		}
			break;
	}
	
	return RT_EOK;
}

static rt_size_t imx_cam_get_frame_jpeg(struct rt_camera_device *cam, void *frame_ptr)
{
	return 0;
}

#define PRODUCT_PRIO   (RT_MAIN_THREAD_PRIORITY-6)
#define CONSUME_PRIO   (RT_MAIN_THREAD_PRIORITY-5)


// when we use the lvgl, the omv is as a plug-in
// but now we provide a way to pause the lvgl, and 
// the omv concour the panel
static bool lvgl_running = false;
void set_lvgl_running(bool enable){
	lvgl_running = enable;
}
static rt_size_t imx_cam_get_frame(struct rt_camera_device *cam, image_t * image)
{
	//mp_printf(&mp_plat_print, "overflow %d\r\n", a);
	#ifdef GPIO_DEBUGING 
	HIGH(GET_FRAME_RATE);
	#endif
	static uint32_t ls_prevTick = 0;
	uint32_t s_minProcessTicks = 5;
	struct fb_mem *fb_mem=NULL,*new_fb_mem=NULL; 
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)cam->imx_cam;
	register rt_ubase_t temp;
	//relase cpu to usb debug to upload jpeg
	uint32_t diff = rt_tick_get() - ls_prevTick;
	if((JPEG_FB()->enabled)&&(diff < s_minProcessTicks)){
			rt_thread_mdelay(s_minProcessTicks - diff);
	}
	if(old_frame){
		if((!lvgl_running) && JPEG_FB()->enabled){
			MAIN_FB()->pixels = old_frame->ptr;
			fb_update_jpeg_buffer();
		}	
		/* disable interrupt */
		temp = rt_hw_interrupt_disable();
		// un-lock cur frame
		old_frame->lock = 0;	
		/* enable interrupt */
		rt_hw_interrupt_enable(temp);
	}
		
	if (!g_csi_int_en)
	{//start csi
		imx_cam_csi_start_frame(imx_cam);
	}
	while(1)
	{
		if (rt_event_recv(&frame_event, EVENT_CSI,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER, NULL) == RT_EOK)
		{

			temp = rt_hw_interrupt_disable();
			if(g_framePtr)
			{
				 new_fb_mem = g_framePtr;
				 new_fb_mem->lock = 1;
				 rt_hw_interrupt_enable(temp);
			 break;
			}
			rt_hw_interrupt_enable(temp);
		}
	}
	
	MAIN_FB()->bpp = imx_cam->fb_list.bpp;
	
	if((pCam->sensor.isWindowing) && (MAIN_FB()->bpp == 1))
	{
		uint32_t win_h = pCam->sensor.wndH;
		uint32_t win_w = pCam->sensor.wndW;
		uint32_t win_x = pCam->sensor.wndX;
		uint32_t win_y = pCam->sensor.wndY;
		// handle the windowing 
		uint8_t* src = new_fb_mem->ptr + win_x + pCam->sensor.fb_w * win_y;
		uint8_t* dst = new_fb_mem->ptr;
		for(int i=0;i<win_h;i++){
			memcpy(dst, src, win_w);
			src += pCam->sensor.fb_w;
			dst += win_w;
		}
		MAIN_FB()->w = win_w;
		MAIN_FB()->h = win_h;
	}else{
		MAIN_FB()->w = imx_cam->fb_list.w;
		MAIN_FB()->h = imx_cam->fb_list.h;		
	}
	
	if(image){
		image->w = MAIN_FB()->w;
		image->h = MAIN_FB()->h;
		image->bpp = MAIN_FB()->bpp;
		image->pixels = new_fb_mem->ptr;						
	}
	#ifdef GPIO_DEBUGING 
	LOW(GET_FRAME_RATE);	
	#endif
	//un-lock the frame
	ls_prevTick = rt_tick_get();
	old_frame = new_fb_mem;
	cam_echo("Cam buf addr:0x%x,tick:%d\r\n",image->pixels,new_fb_mem->tick);
	return 0;
}

static const struct rt_camera_device_ops imxrt_cam_ops =
{
    .get_frame_jpeg = imx_cam_get_frame_jpeg,
	.get_frame = imx_cam_get_frame,
	.camera_control = imx_cam_camera_control,
};

void csi_to_handler(void* paramter)
{
	struct rt_camera_device *cam = (struct rt_camera_device *)paramter;
	imx_cam_csi_stop(cam);
	rt_event_control(&frame_event,RT_IPC_CMD_RESET,0);
}

void camera_clear_by_omv_ide(void)
{
	if (pCam) {
		imx_cam_csi_stop(pCam->rtt_device);
		rt_event_control(&frame_event,RT_IPC_CMD_RESET,0);
		set_lvgl_running(0);
	}
}

static void imxrt_cam_device_init(struct imxrt_camera *cam)
{
	struct rt_camera_device *device = NULL;
	rt_err_t ret;
	
	device = (struct rt_camera_device *)rt_malloc(sizeof(struct rt_camera_device));
	if (device == NULL)
	{
		cam_err("malloc failed in %s\r\n",__func__);
		return;
	}
	cam_echo("Camera device: %s init\r\n",cam->name);
	device->imx_cam = (uint32_t *)cam;
	cam->rtt_device = device;
	device->status = RT_CAMERA_DEVICE_INIT;
	device->ops = &imxrt_cam_ops;
	imxrt_cam_buffer_list_init(cam);

	ret = rt_event_init(&frame_event, "event", RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK)
    {
        rt_kprintf("init event failed.\n");
    }	
	device->status = RT_CAMERA_DEVICE_SUSPEND;
	
}

struct rt_camera_device * imxrt_camera_device_find(char *name)
{
	int i;
	
	for(i = 0; i < sizeof(cams);i++)
	{
		if(strcmp(name,cams[i].name) == 0)
		{
			return cams[i].rtt_device;
		}
	}
	
	return NULL;
}

int imxrt_camera_width(struct rt_camera_device *sensor)
{
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)sensor->imx_cam;
	return imx_cam->sensor.fb_w;
}
int imxrt_camera_height(struct rt_camera_device *sensor)
{
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)sensor->imx_cam;
	return imx_cam->sensor.fb_h;
}
int imxrt_camera_chip_id(struct rt_camera_device *sensor)
{
	return 0;
}
int imxrt_camera_pixformat(struct rt_camera_device *sensor)
{
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)sensor->imx_cam;
	return imx_cam->sensor.pixformat;
}
int imxrt_camera_framesize(struct rt_camera_device *sensor)
{
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)sensor->imx_cam;
	return imx_cam->sensor.framesize;
}
int imxrt_camera_set_windowing(struct rt_camera_device *sensor, int x,int y, int w,int h)
{
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)sensor->imx_cam;
	sensor_t *g_pcur_sensor = &imx_cam->sensor;
	w = (w + 7) & ~7 , x = (x + 7) & ~7;
	if (x >= g_pcur_sensor->fb_w - 8)
		x = g_pcur_sensor->fb_w - 8;
	if (y >= g_pcur_sensor->fb_h - 1)
		y = g_pcur_sensor->fb_h - 1;
	if (x + w > g_pcur_sensor->fb_w)
		w = g_pcur_sensor->fb_w - x;
	if (y + h > g_pcur_sensor->fb_h)
		h = g_pcur_sensor->fb_h - y;

	g_pcur_sensor->isWindowing = (w <= g_pcur_sensor->fb_w && h <= g_pcur_sensor->fb_h) ? 1 : 0;
	if(g_pcur_sensor->isWindowing){
		g_pcur_sensor->wndX = x ; g_pcur_sensor->wndY = y ; g_pcur_sensor->wndW = w ; g_pcur_sensor->wndH = h;
		
		imx_cam->fb_list.w = w;
		imx_cam->fb_list.h = h;
	}
	return 0;
}
int imxrt_camera_set_auto_gain(struct rt_camera_device *sensor,int enable, float gain_db, float gain_db_ceiling)
{
	return 0;
}
int imxrt_camera_sensor_get_gain_db(struct rt_camera_device *sensor,float *gain_db)
{
	return 0;
}
int imxrt_camera_set_auto_exposure(struct rt_camera_device *sensor,int enable, int exposure_us)
{
	return 0;
}
int imxrt_camera_get_exposure_us(struct rt_camera_device *sensor, int *us)
{
	return 0;
}
int imxrt_camera_set_auto_whitebal(struct rt_camera_device *sensor,int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
	return 0;
}
int imxrt_camera_get_rgb_gain_db(struct rt_camera_device *sensor,float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
	return 0;
}
int imxrt_camera_set_lens_correction(struct rt_camera_device *sensor,int enable, int radi, int coef)
{
	return 0;
}
uint16_t * imxrt_camera_get_color_palette(struct rt_camera_device *sensor)
{
	return 0;
}
int imxrt_camera_write_reg(struct rt_camera_device *sensor,uint16_t reg_addr, uint16_t reg_data)
{
	return 0;
}
int imxrt_camera_read_reg(struct rt_camera_device *sensor,uint16_t reg_addr)
{
	return 0;
}

int imxrt_camera_ioctl(struct rt_camera_device *sensor,int request, ... /* arg */)
{
    int ret = -1;
	
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)sensor->imx_cam;
    if (imx_cam->sensor.ioctl != NULL) {
        va_list ap;
        va_start(ap, request);
        /* call the sensor specific function */
        ret = imx_cam->sensor.ioctl(&imx_cam->sensor, request, ap);
        va_end(ap);
    }
    return ret;
}

void reset_displaymix(){
//    SRC_AssertSliceSoftwareReset(SRC, kSRC_DisplaySlice);
//    while (kSRC_SliceResetInProcess == SRC_GetSliceResetState(SRC, kSRC_DisplaySlice))
//    {
//    }
}

#include "fsl_nic301.h" 
rt_thread_t thread;
uint32_t csi_prio = 15;
uint32_t usb_prio = 6;
int rt_camera_init(void)
{	
	#ifdef GPIO_DEBUGING
	INIT();
	#endif
	volatile uint32_t semc = CLOCK_GetFreqFromObs(CCM_OBS_SEMC_CLK_ROOT); 
	volatile uint32_t m7_root = CLOCK_GetFreqFromObs(CCM_OBS_M7_CLK_ROOT); 
	volatile uint32_t bus_root = CLOCK_GetFreqFromObs(CCM_OBS_BUS_CLK_ROOT);
	volatile uint32_t lrsr_bus_root = CLOCK_GetFreqFromObs(CCM_OBS_BUS_LPSR_CLK_ROOT);
	volatile uint32_t flexspi_root = CLOCK_GetFreqFromObs(CCM_OBS_FLEXSPI1_CLK_ROOT);
	
	
	int i;
	JPEG_FB()->quality = 25;
	// solve the issue: lcd can not display, but without the pxp
	SEMC->BMCR0 = SEMC->BMCR1 = 0x80;
	
	for(i = 0; i < CAM_NUM;i++)
	{
		imxrt_cam_device_init(&cams[i]);
	}
	
	return 0;
}
INIT_DEVICE_EXPORT(rt_camera_init);
