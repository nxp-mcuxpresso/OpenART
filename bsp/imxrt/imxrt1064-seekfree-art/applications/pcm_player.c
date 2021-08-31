#include <rtdevice.h>
#include "drv_gpio.h"
#include "core_cm7.h"
#include <rtthread.h>

#include <py/compile.h>
#include <py/runtime.h>
#include <py/repl.h>
#include <py/gc.h>
#include <py/mperrno.h>
#include <py/stackctrl.h>
#include <py/frozenmod.h>
#include <lib/mp-readline/readline.h>
#include <lib/utils/pyexec.h>
#include "mpgetcharport.h"
#include "mpputsnport.h"
#include "dfs_fs.h"
#include "fsl_sai.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_sai_edma.h"
#include <dfs_posix.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* SAI instance and clock */
#define DEMO_SAI SAI3

/* Select Audio PLL (786.432 MHz) as sai1 clock source */
#define DEMO_SAI_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai clock source */
#define DEMO_SAI_CLOCK_SOURCE_PRE_DIVIDER (3U)
/* Clock divider for sai clock source */
#define DEMO_SAI_CLOCK_SOURCE_DIVIDER (4U)
/* Get frequency of sai clock: SAI3_Clock = 786.432MHz /(3+1)/(4+1) = 39.321MHz */
#define DEMO_SAI_CLK_FREQ (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI_CLOCK_SOURCE_DIVIDER + 1U) / (DEMO_SAI_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* DMA */
#define DMAMUX0 DMAMUX
#define EXAMPLE_DMA DMA0
#define EXAMPLE_CHANNEL (0U)
#define EXAMPLE_SAI_TX_SOURCE kDmaRequestMuxSai3Tx
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void callback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_INIT(static sai_edma_handle_t txHandle) = {0};
edma_handle_t dmaHandle = {0};

static volatile bool isFinished = false;
static uint8_t pcm_buffer[4096] __attribute__((section(".usb_buf")));
static uint8_t play_buffer[2048]__attribute__((section(".usb_buf")));
/*******************************************************************************
 * Code
 ******************************************************************************/

/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 768/1000)
 *                              = 786.432 MHz
 */
static const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,      /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,       /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator = 768,       /* 30 bit numerator of fractional loop divider. */
    .denominator = 1000,    /* 30 bit denominator of fractional loop divider */
};

static void callback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    isFinished = true;
}

void configMQS(void)
{
    CCM->CCGR0 &= (~CCM_CCGR0_CG2_MASK) | CCM_CCGR0_CG2(3);         /* Enable MQS hmclk. */

    IOMUXC_MQSEnterSoftwareReset(IOMUXC_GPR, true);                             /* Reset MQS. */
    IOMUXC_MQSEnterSoftwareReset(IOMUXC_GPR, false);                            /* Release reset MQS. */
    IOMUXC_MQSEnable(IOMUXC_GPR, true);                                         /* Enable MQS. */
    IOMUXC_MQSConfig(IOMUXC_GPR, kIOMUXC_MqsPwmOverSampleRate64, 0u);           /* 78.6432MHz/64/(0+1) = 1.2288MHz
                                                                                Higher frequency PWM involves less low frequency harmonic.*/
}

/*!
 * @brief Main function
 */
void pcm_player(int argc, void **argv)
{
    sai_config_t config;
    sai_transfer_format_t format;
    sai_transfer_t xfer;
    edma_config_t dmaConfig = {0};

    CLOCK_InitAudioPll(&audioPllConfig);

    /*Clock setting for SAI. */
    CLOCK_SetMux(kCLOCK_Sai3Mux, DEMO_SAI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai3PreDiv, DEMO_SAI_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai3Div, DEMO_SAI_CLOCK_SOURCE_DIVIDER);


    /* Create EDMA handle */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(EXAMPLE_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaHandle, EXAMPLE_DMA, EXAMPLE_CHANNEL);

    DMAMUX_Init(DMAMUX0);
    DMAMUX_SetSource(DMAMUX0, EXAMPLE_CHANNEL, EXAMPLE_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(DMAMUX0, EXAMPLE_CHANNEL);

    /* Init SAI module */
	
    SAI_TxGetDefaultConfig(&config);
    SAI_TxInit(DEMO_SAI, &config);

    /* Configure the audio format */
    format.bitWidth = kSAI_WordWidth32bits;
    format.channel = 0U;
    format.sampleRate_Hz = kSAI_SampleRate48KHz;
    
    format.protocol = config.protocol;
    format.stereo = kSAI_Stereo;
    format.isFrameSyncCompact = true;
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;

    SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &txHandle, callback, NULL, &dmaHandle);
    SAI_TransferTxSetFormatEDMA(DEMO_SAI, &txHandle, &format, DEMO_SAI_CLK_FREQ, DEMO_SAI_CLK_FREQ);

    configMQS();

    /*  xfer structure */
    
	int fd = open((char*)argv[1], O_RDONLY);
	int read_len;
	int total_len = 0;
	if (fd < 0)
	{
		rt_kprintf("can not open %s\r\n",argv[1]);
		return;
	}
    while (1)
    {
        isFinished = false;
		read_len = read(fd,pcm_buffer,4096);
		if (read_len <= 0)
		{
			close(fd);
			rt_kprintf("play end\r\n");
			SAI_TransferTerminateSendEDMA(DEMO_SAI, &txHandle);
			IOMUXC_MQSEnable(IOMUXC_GPR, false);
			return;
		}
		total_len += read_len;
		//int32_t *load_ptr = (int32_t*)pcm_buffer;
		//int16_t *write_ptr = (int16_t *)play_buffer;
		//for (int i=0;i<read_len/4;i++)
		//	write_ptr[i] = load_ptr[i] >> 16;
		
		
		xfer.data = (uint8_t *)(uint32_t)pcm_buffer;
		xfer.dataSize = read_len;
		
        SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
		
        /* Wait until finished */
        while (isFinished != true)
        {
        }
    }
}

MSH_CMD_EXPORT(pcm_player, play pcm)