#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mt9m114.h"
#include "systick.h"
#include "py/mpprint.h"
#include "fmath.h"
#include "imlib.h"

#define cam_err(...) mp_printf(MP_PYTHON_PRINTER, __VA_ARGS__)

int support_res[][2] = {
    {160,120},//QQVGA
    {320,240},//QVGA
    {640,480},//VGA
    {960,640},//full
    {1280,720},//wxga
};


static const mt9m114_reg_t mt9m114_960p30_regs[] = {
  {MT9M114_REG_LOGICAL_ADDRESS_ACCESS, 2, 0x0000},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_END, 2, 971},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_END, 2, 1291},
  {MT9M114_VAR_CAM_SENSOR_CFG_PIXCLK, 4, 48000000},
  {MT9M114_VAR_CAM_SENSOR_CFG_ROW_SPEED, 2, 0x0001},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN, 2, 219},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX, 2, 1480},
  {MT9M114_VAR_CAM_SENSOR_CFG_FRAME_LENGTH_LINES, 2, 1007},
  {MT9M114_VAR_CAM_SENSOR_CFG_LINE_LENGTH_PCK, 2, 1611},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_CORRECTION, 2, 96},
  {MT9M114_VAR_CAM_SENSOR_CFG_CPIPE_LAST_ROW, 2, 963},
  {MT9M114_VAR_CAM_SENSOR_CFG_REG_0_DATA, 2, 0x0020},
 // {REG_CAM_SENSOR_CONTROL_READ_MODE, 1, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_XOFFSET, 2, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_YOFFSET, 2, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_WIDTH, 2, 1280},
  {MT9M114_VAR_CAM_CROP_WINDOW_HEIGHT, 2, 960},
  {MT9M114_VAR_CAM_CROP_CROPMODE, 1, 0x03},
  {MT9M114_VAR_CAM_OUTPUT_WIDTH, 2, 1280},
  {MT9M114_VAR_CAM_OUTPUT_HEIGHT, 2, 960},
  {MT9M114_VAR_CAM_AET_AEMODE, 1, 0x0},
  {MT9M114_VAR_CAM_AET_MAX_FRAME_RATE, 2, 0x1D97},
  {MT9M114_VAR_CAM_AET_MIN_FRAME_RATE, 2, 0x1D97},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XEND, 2, 1279},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YEND, 2, 959},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XEND, 2, 255},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YEND, 2, 191},
};


static const mt9m114_reg_t mt9m114_720p36_regs[] = {
  {MT9M114_REG_LOGICAL_ADDRESS_ACCESS, 2, 0x0000},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_START, 2, 124},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_END, 2, 851},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_END, 2, 1291},
  {MT9M114_VAR_CAM_SENSOR_CFG_PIXCLK, 4, 48000000},
  {MT9M114_VAR_CAM_SENSOR_CFG_ROW_SPEED, 2, 0x0001},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN, 2, 219},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX, 2, 1558},
  {MT9M114_VAR_CAM_SENSOR_CFG_FRAME_LENGTH_LINES, 2, 778},
  {MT9M114_VAR_CAM_SENSOR_CFG_LINE_LENGTH_PCK, 2, 1689},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_CORRECTION, 2, 96},
  {MT9M114_VAR_CAM_SENSOR_CFG_CPIPE_LAST_ROW, 2, 723},
  {MT9M114_VAR_CAM_SENSOR_CFG_REG_0_DATA, 2, 0x0020},
 // {MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, 1, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_XOFFSET, 2, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_YOFFSET, 2, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_WIDTH, 2, 1280},
  {MT9M114_VAR_CAM_CROP_WINDOW_HEIGHT, 2, 720},
  {MT9M114_VAR_CAM_CROP_CROPMODE, 1, 0x03},
  {MT9M114_VAR_CAM_OUTPUT_WIDTH, 2, 1280},
  {MT9M114_VAR_CAM_OUTPUT_HEIGHT, 2, 720},
  {MT9M114_VAR_CAM_AET_AEMODE, 1, 0x00},
  {MT9M114_VAR_CAM_AET_MAX_FRAME_RATE, 2, 0x24AB},
  {MT9M114_VAR_CAM_AET_MIN_FRAME_RATE, 2, 0x24AB},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XEND, 2, 1279},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YEND, 2, 719},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XEND, 2, 255},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YEND, 2, 143},
};

static const mt9m114_reg_t mt9m114_vga_30_scaling_regs[] = {
  {MT9M114_REG_LOGICAL_ADDRESS_ACCESS, 2, 0x0000},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_END, 2, 971},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_END, 2, 1291},
  {MT9M114_VAR_CAM_SENSOR_CFG_PIXCLK, 4, 48000000},
  {MT9M114_VAR_CAM_SENSOR_CFG_ROW_SPEED, 2, 1},//FIXME according to the documentation this value is unused, however we still set the default. No idea why
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN, 2, 219},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX, 2, 1460},
  {MT9M114_VAR_CAM_SENSOR_CFG_FRAME_LENGTH_LINES, 2, 1006},//FIXME might be a typo. default value is 1007
  {MT9M114_VAR_CAM_SENSOR_CFG_LINE_LENGTH_PCK, 2, 1591},//FIXME might be a typo? default is 1589
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_CORRECTION, 2, 96},
  {MT9M114_VAR_CAM_SENSOR_CFG_CPIPE_LAST_ROW, 2, 963}, 
  {MT9M114_VAR_CAM_SENSOR_CFG_REG_0_DATA, 2, 0x0020},
 // {MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, 1, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_XOFFSET, 2, 0x0000}, 
  {MT9M114_VAR_CAM_CROP_WINDOW_YOFFSET, 2, 0x0000}, 
  {MT9M114_VAR_CAM_CROP_WINDOW_WIDTH, 2, 1280},
  {MT9M114_VAR_CAM_CROP_WINDOW_HEIGHT, 2, 960},
  {MT9M114_VAR_CAM_CROP_CROPMODE, 1, 3},
  {MT9M114_VAR_CAM_OUTPUT_WIDTH, 2, 640},
  {MT9M114_VAR_CAM_OUTPUT_HEIGHT, 2, 480},
  {MT9M114_VAR_CAM_AET_AEMODE, 1, 0x00},
  {MT9M114_VAR_CAM_AET_MAX_FRAME_RATE, 2, 0x1DFD},
  {MT9M114_VAR_CAM_AET_MIN_FRAME_RATE, 2, 0x1DFD},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XEND, 2, 639},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YEND, 2, 479},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XEND, 2, 127},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YEND, 2, 95},
 // {MT9M114_VAR_AUTO_BINNING_MODE,1, 0x00},
};

static const mt9m114_reg_t mt9m114_qvga_30_scaling_regs[] = {
  {MT9M114_REG_LOGICAL_ADDRESS_ACCESS, 2, 0x0000},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_END, 2, 971},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_END, 2, 1291},
  {MT9M114_VAR_CAM_SENSOR_CFG_PIXCLK, 4, 48000000},
  {MT9M114_VAR_CAM_SENSOR_CFG_ROW_SPEED, 2, 1},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN, 2, 219},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX, 2, 1460},
  {MT9M114_VAR_CAM_SENSOR_CFG_FRAME_LENGTH_LINES, 2, 1006},
  {MT9M114_VAR_CAM_SENSOR_CFG_LINE_LENGTH_PCK, 2, 1591},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_CORRECTION, 2, 96},
  {MT9M114_VAR_CAM_SENSOR_CFG_CPIPE_LAST_ROW, 2, 963},
  {MT9M114_VAR_CAM_SENSOR_CFG_REG_0_DATA, 2, 0x0020},
  //{MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, 1, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_XOFFSET, 2, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_YOFFSET, 2, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_WIDTH, 2, 1280},
  {MT9M114_VAR_CAM_CROP_WINDOW_HEIGHT, 2, 960},
  {MT9M114_VAR_CAM_CROP_CROPMODE, 1, 3},
  {MT9M114_VAR_CAM_OUTPUT_WIDTH, 2, 320},
  {MT9M114_VAR_CAM_OUTPUT_HEIGHT, 2, 240},
  {MT9M114_VAR_CAM_AET_AEMODE, 1, 0x00},
  {MT9M114_VAR_CAM_AET_MAX_FRAME_RATE, 2, 0x1DFD},
  {MT9M114_VAR_CAM_AET_MIN_FRAME_RATE, 2, 0x1DFD},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XEND, 2, 319},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YEND, 2, 239},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XEND, 2, 63},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YEND, 2, 47},

//  {MT9M114_VAR_CAM_MODE_SELECT,1,0x2},
//  {MT9M114_VAR_CAM_MODE_TEST_PATTERN_SELECT,1,1},
//  {MT9M114_VAR_CAM_MODE_TEST_PATTERN_RED,0x2,0x00ff},
//  {MT9M114_VAR_CAM_MODE_TEST_PATTERN_GREEN,0x2,0},
//  {MT9M114_VAR_CAM_MODE_TEST_PATTERN_BLUE,0x2,0},
};

static const mt9m114_reg_t mt9m114_160x120_30_scaling_regs[] = {
  {MT9M114_REG_LOGICAL_ADDRESS_ACCESS, 2, 0x0000},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_START, 2, 4},
  {MT9M114_VAR_CAM_SENSOR_CFG_Y_ADDR_END, 2, 971},
  {MT9M114_VAR_CAM_SENSOR_CFG_X_ADDR_END, 2, 1291},
  {MT9M114_VAR_CAM_SENSOR_CFG_PIXCLK, 4, 48000000},
  {MT9M114_VAR_CAM_SENSOR_CFG_ROW_SPEED, 2, 1},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN, 2, 219},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX, 2, 1460},
  {MT9M114_VAR_CAM_SENSOR_CFG_FRAME_LENGTH_LINES, 2, 1006},
  {MT9M114_VAR_CAM_SENSOR_CFG_LINE_LENGTH_PCK, 2, 1591},
  {MT9M114_VAR_CAM_SENSOR_CFG_FINE_CORRECTION, 2, 96},
  {MT9M114_VAR_CAM_SENSOR_CFG_CPIPE_LAST_ROW, 2, 963},
  {MT9M114_VAR_CAM_SENSOR_CFG_REG_0_DATA, 2, 0x0020},
  //{MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, 1, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_XOFFSET, 2, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_YOFFSET, 2, 0x0000},
  {MT9M114_VAR_CAM_CROP_WINDOW_WIDTH, 2, 1280},
  {MT9M114_VAR_CAM_CROP_WINDOW_HEIGHT, 2, 960},
  {MT9M114_VAR_CAM_CROP_CROPMODE, 1, 3},
  {MT9M114_VAR_CAM_OUTPUT_WIDTH, 2, 160},
  {MT9M114_VAR_CAM_OUTPUT_HEIGHT, 2, 120},
  {MT9M114_VAR_CAM_AET_AEMODE, 1, 0x00},
  {MT9M114_VAR_CAM_AET_MAX_FRAME_RATE, 2, 0x1DFD},
  {MT9M114_VAR_CAM_AET_MIN_FRAME_RATE, 2, 0x1DFD},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XEND, 2, 159},
  {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YEND, 2, 119},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YSTART, 2, 0x0000},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XEND, 2, 31},
  {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YEND, 2, 23},
//  {MT9M114_VAR_AUTO_BINNING_MODE,1, 0x00},
};

static const mt9m114_reg_t mt9m114InitConfig[] = {
    {MT9M114_REG_LOGICAL_ADDRESS_ACCESS, 2u, 0x1000},
    /* PLL Fout = (Fin * 2 * m) / ((n + 1) * (p + 1)) */
    {MT9M114_VAR_CAM_SYSCTL_PLL_ENABLE, 1u, 0x01},        /*  cam_sysctl_pll_enable = 1 */
    {MT9M114_VAR_CAM_SYSCTL_PLL_DIVIDER_M_N, 2u, 0x0120}, /*  cam_sysctl_pll_divider_m_n = 288 */
    {MT9M114_VAR_CAM_SYSCTL_PLL_DIVIDER_P, 2u, 0x0700},   /*  cam_sysctl_pll_divider_p = 1792 */
    {MT9M114_VAR_CAM_SENSOR_CFG_PIXCLK, 4u, 0x2DC6C00},   /*  cam_sensor_cfg_pixclk = 48000000 */
    {0x316A, 2, 0x8270}, /*  auto txlo_row for hot pixel and linear full well optimization */
    {0x316C, 2, 0x8270}, /*  auto txlo for hot pixel and linear full well optimization */
    {0x3ED0, 2, 0x2305}, /*  eclipse setting, ecl range=1, ecl value=2, ivln=3 */
    {0x3ED2, 2, 0x77CF}, /*  TX_hi=12 */
    {0x316E, 2, 0x8202}, /*  auto ecl , threshold 2x, ecl=0 at high gain, ecl=2 for low gain */
    {0x3180, 2, 0x87FF}, /*  enable delta dark */
    {0x30D4, 2, 0x6080}, /*  disable column correction due to AE oscillation problem */
    {0xA802, 2, 0x0008}, /*  RESERVED_AE_TRACK_02 */
    {0x3E14, 2, 0xFF39}, /*  Enabling pixout clamping to VAA during ADC streaming to solve column band issue */
    {MT9M114_VAR_CAM_SENSOR_CFG_ROW_SPEED, 2u, 0x0001},           /*  cam_sensor_cfg_row_speed = 1 */
    {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN, 2u, 0x00DB}, /*  cam_sensor_cfg_fine_integ_time_min = 219 */
    {MT9M114_VAR_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX, 2u, 0x07C2}, /*  cam_sensor_cfg_fine_integ_time_max = 1986 */
    {MT9M114_VAR_CAM_SENSOR_CFG_FRAME_LENGTH_LINES, 2u, 0x02FE},  /*  cam_sensor_cfg_frame_length_lines = 766 */
    {MT9M114_VAR_CAM_SENSOR_CFG_LINE_LENGTH_PCK, 2u, 0x0845},     /*  cam_sensor_cfg_line_length_pck = 2117 */
    {MT9M114_VAR_CAM_SENSOR_CFG_FINE_CORRECTION, 2u, 0x0060},     /*  cam_sensor_cfg_fine_correction = 96 */
    {MT9M114_VAR_CAM_SENSOR_CFG_REG_0_DATA, 2u, 0x0020},          /*  cam_sensor_cfg_reg_0_data = 32 */
    {MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, 2u, 0x00},       /*  cam_sensor_control_read_mode = 0 */
    {MT9M114_VAR_CAM_CROP_WINDOW_XOFFSET, 2u, 0x0000},            /*  cam_crop_window_xoffset = 0 */
    {MT9M114_VAR_CAM_CROP_WINDOW_YOFFSET, 2u, 0x0000},            /*  cam_crop_window_yoffset = 0 */
    {MT9M114_VAR_CAM_CROP_CROPMODE, 1u, 0x03},                    /*  cam_crop_cropmode = 3 */
    {MT9M114_VAR_CAM_AET_AEMODE, 1u, 0x00},                       /*  cam_aet_aemode = 0 */
    {MT9M114_VAR_CAM_AET_MAX_FRAME_RATE, 2u, 0x1D9A},             /*  cam_aet_max_frame_rate = 7578 */
    {MT9M114_VAR_CAM_AET_MIN_FRAME_RATE, 2u, 0x1D9A},             /*  cam_aet_min_frame_rate = 7578 */
    {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_XSTART, 2u, 0x0000},    /*  cam_stat_awb_clip_window_xstart = 0 */
    {MT9M114_VAR_CAM_STAT_AWB_CLIP_WINDOW_YSTART, 2u, 0x0000},    /*  cam_stat_awb_clip_window_ystart = 0 */
    {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_XSTART, 2u, 0x0000},  /*  cam_stat_ae_initial_window_xstart = 0 */
    {MT9M114_VAR_CAM_STAT_AE_INITIAL_WINDOW_YSTART, 2u, 0x0000},  /*  cam_stat_ae_initial_window_ystart = 0 */
    {MT9M114_REG_PAD_SLEW, 2u, 0x0777},                           /*  Pad slew rate */
    {MT9M114_VAR_CAM_OUTPUT_FORMAT_YUV, 2u, 0x0038},              /*  Must set cam_output_format_yuv_clip for CSI */
};
static int cambus_writews(sensor_t *sensor, uint16_t reg_addr, uint32_t value, int size)
{
	uint16_t addr;
	uint8_t cmd[8];
	int index=0;

	addr = ((reg_addr &0x00ff) << 8)|(reg_addr >> 8);
	
	if( size == 2) 
    {
        //FIXME this breaks signedness.
        cmd[index++] = value/256;
        cmd[index++] = value%256;
    }
    else if( size == 4)
    {
        cmd[index++] = value>>24 & 0xff;
        cmd[index++] = value>>16 & 0xff;
        cmd[index++] = value>>8   & 0xff;
        cmd[index++] = value>>0   & 0xff;
    }
    else if( size == 1)
    {
        cmd[index++] = value;
    }
		
	sensor->cambus_writews(sensor,sensor->slv_addr,addr,cmd,size);
	return 0;
}

static int cambus_readws(sensor_t *sensor, uint16_t reg_addr, uint32_t *value,int size)
{
	uint16_t addr;
	uint8_t cmd[10] = {0};
	addr = ((reg_addr &0x00ff) << 8)|(reg_addr >> 8);
	
	sensor->cambus_readws(sensor,sensor->slv_addr,addr,cmd,size);
    if( size == 2 )
    {
        *value = (((uint32_t)cmd[0])<<8) + cmd[1];
    }
    else if( size == 4 )
    {
        *value = (((uint32_t)cmd[0])<<24) + (((uint32_t)cmd[1])<<16) +
                (((uint32_t)cmd[2])<<8)  + (((uint32_t)cmd[3])<<0);
    }
    else if( size == 1 )
    {
        *value = cmd[0];
    }

	return 0;
}

static void MT9M114_dump_regs(sensor_t *sensor)
{
    uint32_t value = 0;
    cam_err("\r\nDump all control reg:");
    for (int i=0;i<sizeof(all_control_regs)/sizeof(all_control_regs[0]);i++)
    {
        cambus_readws(sensor,all_control_regs[i],&value,2);
        cam_err("[0x%4x]=0x%4x\r\n",all_control_regs[i],value);
        value = 0;
    }
    cam_err("\r\n\r\n");
}

static status_t MT9M114_SetState(sensor_t *sensor, uint8_t nextState)
{
    status_t status;
    uint32_t value = 0U;

    value = nextState;
    /* Set the desired next state. */
    cambus_writews(sensor,MT9M114_VAR_SYSMGR_NEXT_STATE,value,1);
    /* Check that the FW is ready to accept a new command. */
    while (true)
    {
        cambus_readws(sensor,MT9M114_REG_COMMAND_REGISTER, &value,2);
        
        if (0U == (value & MT9M114_COMMAND_SET_STATE))
        {
            break;
        }
    }

    /* Issue the Set State command. */
    cambus_writews(sensor,MT9M114_REG_COMMAND_REGISTER,MT9M114_COMMAND_SET_STATE | MT9M114_COMMAND_OK,2);
    
    /* Wait for the FW to complete the command. */
    while (true)
    {
        rt_thread_delay(1);
        cambus_readws(sensor,MT9M114_REG_COMMAND_REGISTER, &value,2);
        
        if (0U == (value & MT9M114_COMMAND_SET_STATE))
        {
            break;
        }
    }

    /* Check the 'OK' bit to see if the command was successful. */
    cambus_readws(sensor,MT9M114_REG_COMMAND_REGISTER, &value,2);
    

    if (0U == (value & MT9M114_COMMAND_OK))
    {
        return -1;
    }

    return 0;
}

static int reset(sensor_t *sensor)
{
    uint32_t reg = 0;

    cambus_readws(sensor,MT9M114_REG_RESET_AND_MISC_CONTROL, &reg,2);
    reg = (reg & ~(1)) | (1 & 1);
    cambus_writews(sensor,MT9M114_REG_RESET_AND_MISC_CONTROL, reg,2);

    rt_thread_delay(100);

    cambus_readws(sensor,MT9M114_REG_RESET_AND_MISC_CONTROL, &reg,2);
    reg = (reg & ~(1)) | (0 & 1);
    cambus_writews(sensor,MT9M114_REG_RESET_AND_MISC_CONTROL, reg,2);

    rt_thread_delay(45);


    const mt9m114_reg_t (*regs);
    int i=0;
    uint32_t value=0;
	//cam_err("\r\n");
    for(i=0,regs=&mt9m114InitConfig[i];i<sizeof(mt9m114InitConfig)/sizeof(mt9m114InitConfig[0]);i++)
    {
        cambus_writews(sensor,regs[i].reg, (regs[i].value),regs[i].size);
        //cambus_readws(sensor,regs[i].reg, &(value),regs[i].size);
        //cam_err("0x%x:0x%x at 0x%x\r\n",regs[i].value,value,regs[i].reg);
        //value = 0;
    }

    
	
    //MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	return 0;
}

static int sleep(sensor_t *sensor, int enable)
{
	return 0;
}

static int read_reg(sensor_t *sensor, uint16_t reg_addr)
{
	return 0;
}

static int write_reg(sensor_t *sensor, uint16_t reg_addr, uint16_t reg_data)
{
	return 0;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    uint32_t outputFormat = 0;
    switch (pixformat) {
        case PIXFORMAT_RGB565:
            outputFormat |= ((1U << 8U));
            cambus_writews(sensor,MT9M114_VAR_CAM_OUTPUT_FORMAT, outputFormat,2);
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
            outputFormat = MT9M114_OUTPUT_FORMAT_YUV|MT9M114_OUTPUT_FORMAT_SWAP_BYTES;
            cambus_writews(sensor,MT9M114_VAR_CAM_OUTPUT_FORMAT,outputFormat,2);
            break;
        case PIXFORMAT_BAYER:
            
            break;
        default:
            return -1;
    }

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	return 0;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret=0;
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];
    const mt9m114_reg_t *mt9m114ResConfig;
    const mt9m114_reg_t (*regs);
    uint32_t value = 0;
    int i = 0;

    if(w == 160 && h == 120)
    {//QQVGA
        for(i=0,regs=&mt9m114_160x120_30_scaling_regs[i];i<sizeof(mt9m114_160x120_30_scaling_regs)/sizeof(mt9m114_160x120_30_scaling_regs[0]);i++)
        {
            cambus_writews(sensor,regs[i].reg, (regs[i].value),regs[i].size);
        }
    }
    else if (w == 320 && h == 240)
    {
        for(i=0,regs=&mt9m114_qvga_30_scaling_regs[i];i<sizeof(mt9m114_qvga_30_scaling_regs)/sizeof(mt9m114_qvga_30_scaling_regs[0]);i++)
        {
            cambus_writews(sensor,regs[i].reg, (regs[i].value),regs[i].size);
        }
    }
    else if (w == 640 && h == 480)
    {
        mt9m114ResConfig = mt9m114_vga_30_scaling_regs;
        for(i=0,regs=&mt9m114_vga_30_scaling_regs[i];i<sizeof(mt9m114_vga_30_scaling_regs)/sizeof(mt9m114_vga_30_scaling_regs[0]);i++)
        {
            cambus_writews(sensor,regs[i].reg, (regs[i].value),regs[i].size);
        }
    }
    else if (w == 960 && h == 640)
    {
        mt9m114ResConfig = mt9m114_960p30_regs;
        for(i=0,regs=&mt9m114_960p30_regs[i];i<sizeof(mt9m114_960p30_regs)/sizeof(mt9m114_960p30_regs[0]);i++)
        {
            cambus_writews(sensor,regs[i].reg, (regs[i].value),regs[i].size);
        }
    }
    else if (w == 1280 && h == 720)
    {
        mt9m114ResConfig = mt9m114_720p36_regs;
        for(i=0,regs=&mt9m114_720p36_regs[i];i<sizeof(mt9m114_720p36_regs)/sizeof(mt9m114_720p36_regs[0]);i++)
        {
            cambus_writews(sensor,regs[i].reg, (regs[i].value),regs[i].size);
        }
    }
    else
    {
        return -1;
    }

    cambus_writews(sensor,MT9M114_VAR_CAM_PORT_OUTPUT_CONTROL, 0x8008,2);

    //vertical flip
    cambus_writews(sensor,MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE,0x2,0x2);
    

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);

    rt_thread_delay(80);

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_START_STREAMING);

   
    return 0;
}

static int set_framerate(sensor_t *sensor, framerate_t framerate)
{

#if 0    
    cambus_writews(sensor,MT9M114_VAR_CAM_AET_MAX_FRAME_RATE, framerate * 256,2);

    cambus_writews(sensor,MT9M114_VAR_CAM_AET_MIN_FRAME_RATE, framerate * 128,2);

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
#endif

    return 0;
}


static int set_contrast(sensor_t *sensor, int level)
{
    int new_level = (level > 0) ? (level * 2) : level;

    if ((new_level < -16) || (32 < new_level)) {
        return -1;
    }

    cambus_writews(sensor,MT9M114_VAR_UVC_CONTRAST_CONTROL, new_level + 32,2);
    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);

	return 0;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int new_level = level * 2;

    if ((new_level < -32) || (32 < new_level)) {
        return -1;
    }

    cambus_writews(sensor,MT9M114_VAR_UVC_BRIGHTNESS_CONTROL, new_level + 55,2);

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);

	return 0;
}

static int set_saturation(sensor_t *sensor, int level)
{
    int new_level = level * 8;

    if ((new_level < -128) || (128 < new_level)) {
        return -1;
    }

    new_level = IM_MIN(new_level, 127);

    cambus_writews(sensor,MT9M114_VAR_UVC_SATURATION_CONTROL, new_level + 128,2);
    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);

	return 0;
}

static int set_gainceiling(sensor_t *sensor, gainceiling_t gainceiling)
{
	return 0;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    cambus_writews(sensor,MT9M114_VAR_CAM_MODE_SELECT, enable ? 2 : 0,2);

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	return 0;
}

static int set_auto_gain(sensor_t *sensor, int enable, float gain_db, float gain_db_ceiling)
{
    cambus_writews(sensor,MT9M114_VAR_UVC_AE_MODE_CONTROL, enable ? 0x2 : 0x1,1);

    if ((enable == 0) && (!isnanf(gain_db)) && (!isinff(gain_db))) {
        int gain = IM_MAX(IM_MIN(fast_expf((gain_db / 20.f) * fast_log(10.f)) * 32.f, 0xffff), 0x0000);

        cambus_writews(sensor, MT9M114_VAR_UVC_GAIN_CONTROL, gain,2);
    }

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	return 0;
}

static int get_gain_db(sensor_t *sensor, float *gain_db)
{
    uint16_t gain;

    cambus_readws(sensor, MT9M114_VAR_UVC_GAIN_CONTROL, &gain,2);

    *gain_db = 20.f * (fast_log(gain / 32.f) / fast_log(10.f));

    return 0;

}

static int set_auto_exposure(sensor_t *sensor, int enable, int exposure_us)
{
    cambus_writews(sensor, MT9M114_VAR_UVC_AE_MODE_CONTROL, enable ? 0x2 : 0x1,1);

    if ((enable == 0) && (exposure_us >= 0)) {
        cambus_writews(sensor, MT9M114_VAR_UVC_MANUAL_EXPOSURE_CONFIGURATION, 0x1,1);

        int exposure_100_us = exposure_us / 100;

        cambus_writews(sensor, MT9M114_VAR_UVC_EXPOSURE_TIME_ABSOLUTE_CONTROL,
                exposure_100_us >> 16,2);

        cambus_writews(sensor, MT9M114_VAR_UVC_EXPOSURE_TIME_ABSOLUTE_CONTROL + 2,
                exposure_100_us,2);
    }
    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	return 0;
}

static int get_exposure_us(sensor_t *sensor, int *exposure_us)
{
	uint16_t reg_h, reg_l;

    cambus_readws(sensor, MT9M114_VAR_UVC_EXPOSURE_TIME_ABSOLUTE_CONTROL, &reg_h,2);

    cambus_readws(sensor, MT9M114_VAR_UVC_EXPOSURE_TIME_ABSOLUTE_CONTROL + 2, &reg_l,2);

    *exposure_us = ((reg_h << 16) | reg_l) * 100;

    return 0;
}

static int set_auto_whitebal(sensor_t *sensor, int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    cambus_writews(sensor, MT9M114_VAR_UVC_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL, enable ? 0x1 : 0x0,1);
    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	return 0;
}

static int get_rgb_gain_db(sensor_t *sensor, float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    *r_gain_db = 0;
    *g_gain_db = 0;
    *b_gain_db = 0;

	return 0;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    uint16_t reg_data;

    cambus_readws(sensor, MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, &reg_data,2);

    reg_data = (reg_data & (~MT9M114_SENSOR_CONTROL_READ_MODE_HMIRROR)) |
            (enable ? 0x0 : MT9M114_SENSOR_CONTROL_READ_MODE_HMIRROR); // inverted

    cambus_writews(sensor, MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, reg_data,2);

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	return 0;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    uint16_t reg_data;

    cambus_readws(sensor, MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, &reg_data,2);

    reg_data = (reg_data & (~MT9M114_SENSOR_CONTROL_READ_MODE_VFLIP)) |
            (enable ? 0x0 : MT9M114_SENSOR_CONTROL_READ_MODE_VFLIP); // inverted

    cambus_writews(sensor, MT9M114_VAR_CAM_SENSOR_CONTROL_READ_MODE, reg_data,2);

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);

	return 0;
}

static int set_special_effect(sensor_t *sensor, sde_t sde)
{
    switch (sde) {
        case SDE_NEGATIVE:
            cambus_writews(sensor, MT9M114_VAR_CAM_SFX_CONTROL, 0x3,1);
            break;
        case SDE_NORMAL:
            cambus_writews(sensor, MT9M114_VAR_CAM_SFX_CONTROL, 0x0,1);
            break;
        default:
            return -1;
    }

    MT9M114_SetState(sensor,MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	return 0;
}

static int set_lens_correction(sensor_t *sensor, int enable, int radi, int coef)
{
	return 0;
}

void mt9m114_init(sensor_t *sensor)
{
    // Initialize sensor structure.
    sensor->gs_bpp              = 2;
    sensor->reset               = reset;
    sensor->sleep               = sleep;
    sensor->read_reg            = read_reg;
    sensor->write_reg           = write_reg;
    sensor->set_pixformat       = set_pixformat;
    sensor->set_framesize       = set_framesize;
    sensor->set_framerate       = set_framerate;
    sensor->set_contrast        = set_contrast;
    sensor->set_brightness      = set_brightness;
    sensor->set_saturation      = set_saturation;
    sensor->set_gainceiling     = set_gainceiling;
    sensor->set_colorbar        = set_colorbar;
    sensor->set_auto_gain       = set_auto_gain;
    sensor->get_gain_db         = get_gain_db;
    sensor->set_auto_exposure   = set_auto_exposure;
    sensor->get_exposure_us     = get_exposure_us;
    sensor->set_auto_whitebal   = set_auto_whitebal;
    sensor->get_rgb_gain_db     = get_rgb_gain_db;
    sensor->set_hmirror         = set_hmirror;
    sensor->set_vflip           = set_vflip;
    sensor->set_special_effect  = set_special_effect;
    sensor->set_lens_correction = set_lens_correction;


}