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
#include "fsl_edma.h"
#include "fsl_dmamux.h"
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


#define cam_echo(...) //mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)
#define cam_err(...) mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)

#if defined(__CC_ARM) || defined(__CLANG_ARM)
	extern unsigned int Image$$MPY_SENSOR_BUFER_START$$Base;
	extern unsigned int Image$$MPY_SENSOR_BUFER_START$$Limit;
#elif defined(__ICCARM__)
	extern unsigned int MPY_SENSOR_BUFER_START$$Limit[];
#elif defined(__GNUC__)

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
	uint8_t ptr[];//64 byte algin
};

struct fb_mem_list
{
	uint16_t bbp;
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
	
	uint16_t	sensor_id;
	uint8_t     sensor_addr;
	GPIO_Type * sensor_pwn_io;
	uint8_t     sensor_pwn_io_pin;
	GPIO_Type *  sensor_rst_io;
	uint8_t sensor_rst_io_pin;
	char 		*sensor_bus_name;
	uint32_t	fb_buffer_start;
	uint32_t	fb_buffer_end;
};

const uint16_t supported_sensors[3][2] = {
	{0x21,OV_CHIP_ID},//ov7725
	{0x48,0x00},//mt9m114
	{OV5640_SLV_ADDR>>1,OV5640_CHIP_ID}
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
		
		.sensor_bus_name = BSP_SENSOR_BUS_NAME,
#if defined(__CC_ARM) || defined(__CLANG_ARM)		
		.fb_buffer_start = (uint32_t) &Image$$MPY_SENSOR_BUFER_START$$Base,
		.fb_buffer_end = (uint32_t) &Image$$MPY_SENSOR_BUFER_START$$Limit,
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
#define FRAGBUF_LOC  /*__attribute__((section(".dmaFramebuffer")))*/
FRAGBUF_LOC static uint64_t s_dmaFragBufs[2][1280 * 4 / 4];	// max supported line length, XRGB8888(*4)

static struct rt_event frame_event;
#define EVENT_CSI	(1<<0)
static struct fb_mem *g_framePtr = NULL;
static uint32_t g_csi_int_en = 0;
static uint32_t isToStopCSICommand = 0;
static rt_timer_t g_csi_to = NULL;
static uint32_t g_csi_to_ticks = 1500;
bool csi_calc_first = 0;
//==================================================================================================
#ifdef RT_USING_LCD
extern void LCDMonitor_Update(uint32_t fbNdx, uint8_t isGray,uint32_t wndH, uint32_t wndW, uint32_t pixels_addr);
#endif

void imx_cam_csi_start_frame(struct imxrt_camera *cam);
void CsiFragModeInit(CSI_Type *s_pCSI ) {

	CLOCK_EnableClock(kCLOCK_Csi);
	CSI_Reset(s_pCSI);
	
	s_pCSI->CSICR1 = CSICR1_INIT_VAL;
	s_pCSI->CSICR2 = 3U << 30;	// INCR16 for RxFIFO DMA
	s_pCSI->CSICR3 = 2U << 4;	// 16 double words to trigger DMA request
	s_pCSI->CSIFBUF_PARA = 0;	// no stride


	s_pCSI->CSICR18 = 13<<12 | 1<<18;	// HProt AHB bus protocol, write to memory when CSI_ENABLE is 1

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

	buf[0] = reg; //cmd
	buf[1] = data;

	msgs.addr = addr;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = 2;

	
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

int imx_cam_sensor_readb2_reg(struct rt_i2c_bus_device *i2c_bus,rt_uint8_t addr, rt_uint16_t reg, rt_uint8_t *data)
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

int imx_cam_sensor_cambus_writews(sensor_t *sensor, uint8_t slv_addr, uint16_t reg_addr, uint8_t *reg_data, uint8_t size)
{
	struct rt_i2c_msg msgs;
	rt_uint8_t *buf;

	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	buf = (rt_uint8_t*)rt_malloc(size+2);
	buf[0] = reg_addr & 0xff;
	buf[1] = reg_addr >> 8; //cmd
	
	memcpy(buf+2 , reg_data,size);

	msgs.addr = slv_addr;
	msgs.flags = RT_I2C_WR;
	msgs.buf = buf;
	msgs.len = 2+size;

	
	if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1)
	{
		rt_free(buf);
		return 0;
	}
	else
	{
		rt_free(buf);
		cam_err("[%s] error\r\n",__func__);
		return -1;
	}
}

int imx_cam_sensor_cambus_readws(sensor_t *sensor, uint8_t slv_addr, uint16_t reg_addr, uint8_t *reg_data, uint8_t size)
{
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[4];
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	buf[0] = reg_addr & 0x00ff; //cmd
	buf[1] = reg_addr >> 8; //cmd
    msgs[0].addr = slv_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 2;
	
	msgs[1].addr = slv_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = reg_data;
    msgs[1].len = size;
	
    
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

int imx_cam_sensor_cambus_readw2(sensor_t *sensor, uint8_t slv_addr, uint16_t reg_addr, uint16_t *reg_data)
{
	struct rt_i2c_msg msgs[2];
	rt_uint8_t buf[4];
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->i2c_bus;
	
	buf[0] = reg_addr & 0x00ff; //cmd
	buf[1] = reg_addr >> 8; //cmd
    msgs[0].addr = slv_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 2;
	
	msgs[1].addr = slv_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = (uint8_t *)(buf+2);
    msgs[1].len = 2;
	
    
    if (rt_i2c_transfer(i2c_bus, msgs, 2) != 0)
    {
		*reg_data = buf[3] | (buf[2] << 8);
        return 0 ;
    }
    else
    {
		cam_err("[%s] error\r\n",__func__);
        return -1;
    }
}


int imxrt_camera_set_framerate(struct imxrt_camera *cam,int framerate)
{
    if (cam->sensor.framerate == framerate) {
       /* no change */
        return 0;
    }
#ifdef 	SOC_IMXRT1170_SERIES
#else		
	if ((framerate & 0x80000000) && (cam->sensor_id == OV7725_ID))
		CCM->CSCDR3 = framerate & (0x1F<<9);
#endif
    /* call the sensor specific function */
    if (cam->sensor.set_framerate == NULL
        || cam->sensor.set_framerate(&cam->sensor, framerate) != 0) {
        /* operation not supported */
        return -1;
    }

    /* set the frame rate */
    cam->sensor.framerate = framerate;

    return 0;
}

extern void BOARD_Camera_pwd_io_set(int value);
extern void BORAD_Camera_rst_io_set(int value);
void imx_cam_sensor_init(struct imxrt_camera *cam)
{
	struct rt_i2c_bus_device *i2c_bus;
	

	
	i2c_bus = rt_i2c_bus_device_find(cam->sensor_bus_name);
	if(i2c_bus == RT_NULL)
	{
		cam_err("[%s]driver can not find %s bus\r\n",__func__,cam->sensor_bus_name);
		return ;
	}

	for (int i=0; i< 3;i++)
	{
		cam->sensor.i2c_bus = (uint32_t *)i2c_bus;
		cam->sensor.cambus_readb = imx_cam_sensor_cambus_readb;
		cam->sensor.cambus_writeb = imx_cam_sensor_cambus_writeb;
		cam->sensor.cambus_readw = imx_cam_sensor_cambus_readw;
		cam->sensor.cambus_writew = imx_cam_sensor_cambus_writew;
		cam->sensor.cambus_readb2 = imx_cam_sensor_cambus_readb2;
		cam->sensor.cambus_writeb2 = imx_cam_sensor_cambus_writeb2;
		cam->sensor.cambus_writews = imx_cam_sensor_cambus_writews;
		cam->sensor.cambus_readws = imx_cam_sensor_cambus_readws;
		cam->sensor_addr = supported_sensors[i][0];
		cam->sensor.slv_addr = cam->sensor_addr;

		CLOCK_SetMux(kCLOCK_CsiMux, 0);
		CLOCK_SetDiv(kCLOCK_CsiDiv, 0);

		//reset camera
		//power down to low
		BOARD_Camera_pwd_io_set(0);
		//reset 
		BORAD_Camera_rst_io_set(0);
		rt_thread_delay(50);
		BORAD_Camera_rst_io_set(1);
		rt_thread_delay(50);

		//read 2 bytes first for mt9m114
		uint16_t sensor_id_s = 0;
		uint8_t sensor_id = 0;
		if(imx_cam_sensor_cambus_readw2(&cam->sensor,cam->sensor_addr,supported_sensors[i][1], &sensor_id_s) == 0)
		{//two bytes addr, two bytes value
			if(sensor_id_s == MT9M114_CHIP_ID)
			{
				cam_err("Camera Device id:0x%x\r\n",sensor_id_s);
				mt9m114_init(&cam->sensor);
				cam->sensor_id = MT9M114_CHIP_ID;
				return;
			}
		}
		
		if (imx_cam_sensor_readb2_reg(i2c_bus,cam->sensor_addr,supported_sensors[i][1], &sensor_id) == 0)
		{//two byte addr, one bytes value
			if(sensor_id == OV5640_ID)
			{
				cam_err("Camera Device id:0x%x\r\n",sensor_id);
				ov5640_init(&cam->sensor);
				cam->sensor_id = OV5640_ID;
				return;
			}
		}
		
		if (imx_cam_sensor_read_reg(i2c_bus,cam->sensor_addr,OV_CHIP_ID, &sensor_id) == 0)
		{//one byte addr. one byte value
			switch(sensor_id)
			{
				case MT9V034_ID:
					cam_err("Camera Device id:0x%x\r\n",sensor_id);
					#ifdef SENSOR_MT9V034
					mt9v034_init(&cam->sensor);
					#endif
					cam->sensor_id = MT9V034_ID;
					return;
				case OV9650_ID:
					cam_err("Camera Device id:0x%x\r\n",sensor_id);
					#ifdef SENSOR_OV9650
					ov9650_init(&cam->sensor);
					#endif
					cam->sensor_id = OV9650_ID;
					return;
				case OV2640_ID:
					cam_err("Camera Device id:0x%x\r\n",sensor_id);
					#ifdef SENSOR_OV2640
					ov2640_init(&cam->sensor);
					#endif
					cam->sensor_id = OV2640_ID;
					return;
				case OV7725_ID:
					cam_err("Camera Device id:0x%x\r\n",sensor_id);
					#ifdef SENSOR_OV7725
					ov7725_init(&cam->sensor);
					cam->sensor_id = OV7725_ID;
					imxrt_camera_set_framerate(cam,0x80000000 | (2<<9|(8-1)<<11));
					#endif
					
					return;
				default:
					
					break;
			}
		
		}
	}
	cam_err("[%s] sensor id:0x%2x not support\r\n",__func__,cam->sensor_id);
}



void imx_cam_reset(struct imxrt_camera *cam)
{
	mutex_init(&JPEG_FB()->lock);
	//sensor init
	imx_cam_sensor_init(cam);
	//csi init
	CsiFragModeInit(cam->csi_base);
	csi_calc_first = 0;
	cam->sensor.isWindowing = 0;
	cam->sensor.wndH = cam->sensor.fb_h;
	cam->sensor.wndW = cam->sensor.fb_w;
	cam->sensor.wndX = cam->sensor.wndY = 0;	
	
	cam->sensor.sde          = 0xFF;
    cam->sensor.pixformat    = 0xFF;
    cam->sensor.framesize    = 0xFF;
    cam->sensor.framerate    = 0xFF;
    cam->sensor.gainceiling  = 0xFF;


    // Call sensor-specific reset function; in the moment,we use our init function and defaults regs
    if (cam->sensor.reset)
		cam->sensor.reset(&cam->sensor);
}

void imxrt_cam_buffer_list_init(struct imxrt_camera *cam)
{
	uint8_t bpp=2;
	int16_t w,h;
	uint32_t size;
	
	w = resolution[cam->sensor.framesize][0];
	h = resolution[cam->sensor.framesize][1];
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
			bpp = 3;
			break;
    }
	
	size = w * h * bpp;
	cam->fb_list.w = w;
	cam->fb_list.h = h;
	cam->fb_list.bbp = bpp;
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
__attribute__((section(".ram_code")))
__attribute__((naked))  uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
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

__attribute__((naked)) void rgb32Torgb565(uint32_t *src, uint16_t *dst, uint32_t len){
	__asm volatile(
		"push {r3-r8, lr}\n"
		"loop: \n"
		"mov r8, #0\n"
		"ldrd r3,r4, [r0], #8\n"
	
		"ubfx r5, r3, #0, #8\n" //r1
		"lsr r5, #3\n"	
		"ubfx r6, r3, #8, #8\n" //g1
		"lsr r6, #2\n"
		"orr r8, r8, r5\n"
		"ubfx r7, r3, #16, #8\n" //b1
		"orr r8, r8, r6, LSL #5\n"	
		"lsr r7, #3\n"

	
		"ubfx r5, r4, #0, #8\n" //r1
		"orr r8, r8, r7, LSL #11\n"
		"lsr r5, #3\n"	
		"ubfx r6, r4, #8, #8\n" //g1
		"lsr r6, #2\n"
		"orr r8, r8, r5, LSL #16\n"
		"ubfx r7, r4, #16, #8\n" //b1
		"orr r8, r8, r6, LSL #21\n"	
		"lsr r7, #3\n"
		"orr r8, r8, r7, LSL #27\n"
			
		"rev16 r8, r8\n"
		"subs r2, 8\n"
		"str r8, [r1], #4\n"
		"bne loop\n"
		"pop {r3-r8, pc}\n"
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
	uint64_t us = 1000000 * (SysTick->LOAD - SysTick->VAL) / (SystemCoreClock);
	us += tick * 1000;
	return us;
}
uint32_t g_start,g_s1,g_e1, g_csi_end;
uint64_t g_csi_start, g_exit;
static int sync_found = 0;
static struct fb_mem *fb_found=NULL;
#if defined(__CC_ARM) || defined(__CLANG_ARM)
__attribute__((section(".ram_code"))) void CSI_IRQHandler(void) {
#else
void CSI_IRQHandler(void) {
#endif
	
	if (pCam == NULL)
		return;
	rt_interrupt_enter();
	
    volatile uint32_t csisr = pCam->csi_base->CSISR;
    /* Clear the error flags. */
    pCam->csi_base->CSISR = csisr;

	if (csisr & (1<<16)) {
		
			
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
		// VSync
		//               SOF    | FB1    | FB2    irqEn
		pCam->csi_base->CSICR1 = 1U<<16 | 1U<<19 | 1U<<20 | CSICR1_INIT_VAL;
		//				 16 doubleWords| RxFifoDmaReqEn| ReflashRFF|ResetFrmCnt
		pCam->csi_base->CSICR3 = 2<<4          | 1<<12         | 1<<14     |1<<15;
		
	} else if (csisr & (3<<19))
	{
		uint32_t dmaBase, lineNdx = pCam->s_irq.dmaFragNdx * pCam->s_irq.linePerFrag;
			if (pCam->s_irq.dmaFragNdx & 1)
				dmaBase = pCam->csi_base->CSIDMASA_FB2;
			else
				dmaBase = pCam->csi_base->CSIDMASA_FB1;
		if (csisr & (1<<19) ) {
			if (!pCam->s_irq.isGray && !pCam->sensor.isWindowing)
				pCam->csi_base->CSIDMASA_FB1 += 2 * pCam->s_irq.dmaBytePerFrag;
		} else {
			if (!pCam->s_irq.isGray && !pCam->sensor.isWindowing)
				pCam->csi_base->CSIDMASA_FB2 += 2 * pCam->s_irq.dmaBytePerFrag;
			pCam->csi_base->CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK;	// reflash DMA
		}
		
		if (dmaBase >= 0x20200000)
			DCACHE_CleanInvalidateByRange(dmaBase, pCam->s_irq.dmaBytePerFrag);
		if (pCam->s_irq.isGray || 
			(pCam->sensor.isWindowing &&  lineNdx >= pCam->sensor.wndY && lineNdx - pCam->sensor.wndY <= pCam->sensor.wndH) )
		{

			dmaBase += pCam->sensor.wndX * 2 * pCam->s_irq.linePerFrag;	// apply line window offset
			if (pCam->s_irq.isGray) {
				
				pCam->s_irq.datCurBase = ExtractYFromYuv(dmaBase, pCam->s_irq.datCurBase, (pCam->sensor.wndW * pCam->s_irq.linePerFrag) >> 3);
			} else {
				uint32_t byteToCopy = (pCam->sensor.wndW * pCam->s_irq.linePerFrag) << 1;
				memcpy((void*)pCam->s_irq.datCurBase, (void*)dmaBase, byteToCopy);
				pCam->s_irq.datCurBase += byteToCopy;
			}
		}
		
		if (++pCam->s_irq.dmaFragNdx == pCam->s_irq.fragCnt || (csisr & (3<<19)) == 3<<19 )
		{
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
				g_csi_int_en = 0;
			}
			else
			{
				
			}

			goto Cleanup;
		}
	}
Cleanup:
	rt_interrupt_leave();
	__DSB(); //__ISB();
	return;
}

void imx_cam_csi_fragmode_calc(struct imxrt_camera *cam) {
	
	cam->s_irq.datBytePerLine = cam->s_irq.dmaBytePerLine = cam->sensor.fb_w * 2;
	if (cam->sensor.pixformat == PIXFORMAT_GRAYSCALE) {
		cam->s_irq.datBytePerLine /= 2;	// only contain Y
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
	dmaByteTotal = cam->sensor.fb_w * cam->sensor.fb_h * 2;	
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
					//while (1) {}
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
	cam->s_irq.datBytePerLine = cam->s_irq.isGray ? dmaBytePerFrag / 2 : dmaBytePerFrag;

	// >>> calculate how many lines per fragment (DMA xfer unit)
	uint32_t burstBytes;
	if (!(cam->s_irq.dmaBytePerLine % (8 * 16)))
	{
		burstBytes = 128;
		cam->csi_base->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(3U);
		cam->csi_base->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((2U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
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

void imx_cam_csi_start_frame(struct imxrt_camera *cam)
{	
	uint32_t start = rt_tick_get_us();
	if(!csi_calc_first) {
		imx_cam_csi_fragmode_calc(cam);
		csi_calc_first = true;
	}
	cam->s_irq.dmaFragNdx = 0;
	cam->s_irq.cnt++;
	// DMA also writes to this cache line, to avoid being invalidated, clean MAIN_FB header.
	DCACHE_CleanByRange((uint32_t)cam->s_irq.base0, 32);
	if (cam->s_irq.isGray || cam->sensor.isWindowing) {
		cam->csi_base->CSIDMASA_FB1 = (uint32_t) s_dmaFragBufs[0];
		cam->csi_base->CSIDMASA_FB2 = (uint32_t) s_dmaFragBufs[1];
	} else {
		cam->csi_base->CSIDMASA_FB1 = cam->s_irq.base0;
		cam->csi_base->CSIDMASA_FB2 = cam->s_irq.base0 + cam->s_irq.dmaBytePerFrag;
	}
	cam->s_irq.datCurBase = cam->s_irq.base0; // + s_sensor.wndY * s_irq.datBytePerLine + s_sensor.wndX * s_sensor.gs_bpp;
	cam->csi_base->CSICR1 = CSICR1_INIT_VAL;
	if (cam->s_irq.dmaBytePerFrag & 0xFFFF0000) {
		
		uint32_t l16 = cam->s_irq.linePerFrag , h16 = cam->s_irq.dmaBytePerLine << 16;
		cam->csi_base->CSIIMAG_PARA = l16 | h16;
	} else {
		cam->csi_base->CSIIMAG_PARA = 1U | cam->s_irq.dmaBytePerFrag << 16;	// set xfer cnt
	}
	__set_PRIMASK(1);
	cam->csi_base->CSISR = cam->csi_base->CSISR;
	g_start = rt_tick_get();
	cam->csi_base->CSICR18 |= 1U<<31;	// start CSI
	cam->csi_base->CSICR1  |= 1<<16;	// must enable SOF iRQ after all the para is initialized
	NVIC_EnableIRQ(CSI_IRQn);
	g_csi_int_en = 1;
	__set_PRIMASK(0);
}

void imx_cam_start_snapshot(struct imxrt_camera *cam)
{
	pCam = cam;
	//init buffer list
	//imxrt_cam_buffer_list_init(cam);
	//start csi
	uint8_t bpp=2;
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
			bpp = 3;
			break;
    }
	
	size = w * h * bpp;
	cam->fb_list.bbp = bpp;
	cam->fb_list.frame_size = size;
	imx_cam_csi_start_frame(cam);
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

int imx_cam_sensor_set_framesize(struct imxrt_camera *cam, framesize_t framesize)
{
	if(cam->sensor.set_framesize == NULL || cam->sensor.set_framesize(&cam->sensor,framesize) != 0)
		return -1;
	
	cam->sensor.framesize = framesize;
	
	cam->fb_list.w = cam->sensor.fb_w = resolution[framesize][0];
	cam->fb_list.h = cam->sensor.fb_h = resolution[framesize][1];
	
	cam->sensor.wndX = 0; cam->sensor.wndY = 0 ; cam->sensor.wndW = cam->sensor.fb_w ; cam->sensor.wndH = cam->sensor.fb_h;

	return 0;
}

void imx_cam_sensor_set_pixformat(struct imxrt_camera *cam, pixformat_t pixformat)
{
	if (cam->sensor.set_pixformat == NULL || cam->sensor.set_pixformat(&cam->sensor,pixformat) != 0)
		return;
	if (cam->sensor.pixformat == pixformat)
		return;
	
	cam->sensor.pixformat = pixformat;
}

void imx_cam_csi_stop(struct rt_camera_device *cam)
{
	cam->status = RT_CAMERA_DEVICE_SUSPEND;
	CSI_Stop(CSI);
	g_csi_int_en = 0;
	NVIC_DisableIRQ(CSI_IRQn);
}

struct fb_mem* old_frame = NULL;
static rt_err_t imx_cam_camera_control(struct rt_camera_device *cam, rt_uint32_t cmd, rt_uint32_t parameter)
{
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)cam->imx_cam;
	switch(cmd)
	{
		case RT_DRV_CAM_CMD_RESET:
			cam->omv_tid = rt_thread_self();
			if(cam->status == RT_CAMERA_DEVICE_RUNING)
			{
				imx_cam_csi_stop(cam);
			}
			imx_cam_reset(imx_cam);
			pCam = (struct imxrt_camera *)cam->imx_cam;
			cam->status = RT_CAMERA_DEVICE_INIT;
			break;
		case RT_DRV_CAM_CMD_SET_FRAMERATE:
			return imxrt_camera_set_framerate(imx_cam, parameter);
		    break;
		case RT_DRV_CAM_CMD_SET_CONTRAST:
			imx_cam_sensor_set_contrast(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SET_GAINCEILING:
			imx_cam_sensor_set_gainceiling(imx_cam, parameter);
			break;
		case RT_DRV_CAM_CMD_SET_FRAMESIZE:
			return imx_cam_sensor_set_framesize(imx_cam, parameter);
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
			//#warning "Enable this, and can not begin twice"
			break;
			
		}
		case RT_DRV_CAM_CMD_SHUTDOWN:
			//imx_cam_csi_stop(cam);
			//rt_event_control(&frame_event,RT_IPC_CMD_RESET,0);
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
	static uint32_t ls_prevTick = 0;
	uint32_t s_minProcessTicks = 10;
	struct fb_mem *fb_mem=NULL,*new_fb_mem=NULL; 
	struct imxrt_camera *imx_cam = (struct imxrt_camera *)cam->imx_cam;
	register rt_ubase_t temp;
	
	
	if(old_frame){
		// un-lock cur frame
		/* disable interrupt */
		temp = rt_hw_interrupt_disable();
		old_frame->lock = 0;
		/* enable interrupt */
		rt_hw_interrupt_enable(temp);
		
	}
	
	//relase cpu to usb debug to upload jpeg
	if((JPEG_FB()->enabled || lvgl_running )&&(rt_tick_get() - ls_prevTick < s_minProcessTicks)){
			rt_thread_mdelay(s_minProcessTicks);
	}
	
	while (pCam->fb_list.count == 1)
	{//start csi
		if(!g_csi_int_en){
			imx_cam_csi_start_frame(imx_cam);
			break;
		}
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

	//mp_printf(&mp_plat_print, "fameRate schedule: %d\r\n", (int)(1000 / (e2- s +0.000001)));
	if(image){
		image->bpp = MAIN_FB()->bpp = imx_cam->fb_list.bbp;
		image->w = MAIN_FB()->w = imx_cam->fb_list.w;
		image->h = MAIN_FB()->h = imx_cam->fb_list.h;
		#if 0
		image->pixels = MAIN_FB()->pixels;
		#endif
		image->pixels = MAIN_FB()->pixels = new_fb_mem->ptr;	
		
//		SCB_CleanInvalidateDCache_by_Addr((void*)new_fb_mem->ptr, image->w * image->h * image->bpp);
		if(!lvgl_running){
			#ifdef RT_USING_LCD
			static uint8_t fbIdx = 0;
			LCDMonitor_Update(fbIdx++,pCam->s_irq.isGray, pCam->sensor.wndH, pCam->sensor.wndW, (uint32_t)MAIN_FB()->pixels);
			#endif
			
			if(old_frame){
				if(JPEG_FB()->enabled){
					MAIN_FB()->pixels = old_frame->ptr;
					//rt_enter_critical();
					fb_update_jpeg_buffer(); 
					//rt_exit_critical();
				}else{
				
				}
			}
		}
	}
	else
	{

	}
	
	
	
	
	ls_prevTick = rt_tick_get();	
	
	old_frame = new_fb_mem;
	
	return 0;
}

static const struct rt_camera_device_ops imxrt_cam_ops =
{
    .get_frame_jpeg = imx_cam_get_frame_jpeg,
	.get_frame = imx_cam_get_frame,
	.camera_control = imx_cam_camera_control,
};

//csi timeout handler
void csi_to_handler(void* paramter)
{
	return;
	//struct rt_camera_device *cam = (struct rt_camera_device *)paramter;
	//imx_cam_csi_stop(cam);
	//rt_event_control(&frame_event,RT_IPC_CMD_RESET,0);
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
	//g_csi_to = rt_timer_create("cam",csi_to_handler,(void*)cam,g_csi_to_ticks,RT_TIMER_FLAG_ONE_SHOT);
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
	
	for(i = 0; i < CAM_NUM;i++)
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

	g_pcur_sensor->isWindowing = (w < g_pcur_sensor->fb_w && h < g_pcur_sensor->fb_h) ? 1 : 0;
	if(g_pcur_sensor->isWindowing){
		g_pcur_sensor->wndX = x ; g_pcur_sensor->wndY = y ; g_pcur_sensor->wndW = w ; g_pcur_sensor->wndH = h;
		
		imx_cam->fb_list.w = w;
		imx_cam->fb_list.h = h;
		csi_calc_first = 0;
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
   
}
int rt_camera_init(void)
{
	int i;

	for(i = 0; i < CAM_NUM;i++)
	{
		imxrt_cam_device_init(&cams[i]);
	}
		
	return 0;
}


INIT_DEVICE_EXPORT(rt_camera_init);
