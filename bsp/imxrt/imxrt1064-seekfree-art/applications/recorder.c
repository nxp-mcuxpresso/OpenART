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

#define DEMO_SAI              SAI1
#define DEMO_SAI_CHANNEL      (0)
#define DEMO_SAI_IRQ          SAI1_IRQn
#define DEMO_SAITxIRQHandler  SAI1_IRQHandler
#define DEMO_SAI_RX_SYNC_MODE kSAI_ModeSync
#define DEMO_SAI_TX_SYNC_MODE kSAI_ModeSync
#define DEMO_SAI_MCLK_OUTPUT  true
#define DEMO_SAI_MASTER_SLAVE kSAI_Master

#define DEMO_AUDIO_DATA_CHANNEL (2U)
#define DEMO_AUDIO_BIT_WIDTH    kSAI_WordWidth32bits
#define DEMO_AUDIO_SAMPLE_RATE  (kSAI_SampleRate48KHz)
#define DEMO_AUDIO_MASTER_CLOCK DEMO_SAI_CLK_FREQ


/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (0U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (63U)
/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ                                                        \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI1_CLOCK_SOURCE_DIVIDER + 1U) / \
     (DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* DMA */
#define DEMO_DMA             DMA0
#define DEMO_DMAMUX          DMAMUX
#define DEMO_RX_EDMA_CHANNEL (0U)
#define DEMO_SAI_RX_SOURCE   kDmaRequestMuxSai1Rx

#define BUFFER_SIZE   (1024U)
#define BUFFER_NUMBER (4U)
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t Buffer[BUFFER_NUMBER * BUFFER_SIZE], 4);
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t txHandle);
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t rxHandle);
edma_handle_t dmaTxHandle = {0}, dmaRxHandle = {0};
static uint32_t tx_index = 0U, rx_index = 0U;
volatile uint32_t emptyBlock = BUFFER_NUMBER;

rt_thread_t record_tid;
static struct rt_mailbox  record_mb;
static rt_uint32_t record_mb_pool[4];
static void rx_callback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_RxError == status)
    {
        /* Handle the error. */
    }
    else
    {
		emptyBlock --;
        rt_mb_send(&record_mb, 0);
    }
}

static void tx_callback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_SAI_TxError == status)
    {
        /* Handle the error. */
    }
    else
    {
        emptyBlock++;
    }
}

const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator   = 77,  /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};


static void recard_thread_entry(void *parameter)
{
	int fd = open((char*)parameter, O_WRONLY | O_CREAT);
	if (fd < 0)
	{
		rt_kprintf("can not open:%s\r\n",(char*)parameter);
		rt_thread_delete(record_tid);
		return;
	}
	rt_kprintf("record start...\r\n");
	int total_count = 800*2;
	sai_transfer_t xfer;
	
	xfer.data     = Buffer + rx_index * BUFFER_SIZE;
	xfer.dataSize = BUFFER_SIZE;
	if (kStatus_Success == SAI_TransferReceiveEDMA(DEMO_SAI, &rxHandle, &xfer))
	{
		rx_index++;
	}
	if (rx_index == BUFFER_NUMBER)
	{
		rx_index = 0U;
	}
	rt_uint32_t buffer;		
	while(total_count --)
	{
		if (rt_mb_recv(&record_mb, (rt_ubase_t *)&buffer, RT_WAITING_FOREVER) == RT_EOK)
        {
			if (emptyBlock > 0)
			{
				xfer.data     = Buffer + rx_index * BUFFER_SIZE;
				xfer.dataSize = BUFFER_SIZE;
				if (kStatus_Success == SAI_TransferReceiveEDMA(DEMO_SAI, &rxHandle, &xfer))
				{
					rx_index++;
				}
				if (rx_index == BUFFER_NUMBER)
				{
					rx_index = 0U;
				}
			}
		
			if (emptyBlock < BUFFER_NUMBER)
			{
				xfer.data     = Buffer + tx_index * BUFFER_SIZE;
				xfer.dataSize = BUFFER_SIZE;
				int size = write(fd,Buffer + tx_index * BUFFER_SIZE,BUFFER_SIZE);
				if (size == BUFFER_SIZE)
				{
					tx_index++;
					emptyBlock++;
				}
				if (tx_index == BUFFER_NUMBER)
				{
					tx_index = 0U;
				}
			}
		}
	}
		
	close(fd);
	SAI_TransferTerminateReceiveEDMA(DEMO_SAI, &rxHandle);
	rt_kprintf("record end\r\n");
	rt_mb_detach(&record_mb);
}

void recorder_init()
{
	
	CLOCK_InitAudioPll(&audioPllConfig);
	/*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
	
	IOMUXC_MQSEnable(IOMUXC_GPR,1);
	IOMUXC_MQSConfig(IOMUXC_GPR,kIOMUXC_MqsPwmOverSampleRate32,0);
	
	    /* Init DMAMUX */
    DMAMUX_Init(DEMO_DMAMUX);
    DMAMUX_SetSource(DEMO_DMAMUX, DEMO_RX_EDMA_CHANNEL, (uint8_t)DEMO_SAI_RX_SOURCE);
    DMAMUX_EnableChannel(DEMO_DMAMUX, DEMO_RX_EDMA_CHANNEL);

	edma_config_t dmaConfig = {0};
    /* Init DMA and create handle for DMA */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(DEMO_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaRxHandle, DEMO_DMA, DEMO_RX_EDMA_CHANNEL);
#if defined(FSL_FEATURE_EDMA_HAS_CHANNEL_MUX) && FSL_FEATURE_EDMA_HAS_CHANNEL_MUX
    EDMA_SetChannelMux(DEMO_DMA, DEMO_TX_EDMA_CHANNEL, DEMO_SAI_TX_EDMA_CHANNEL);
    EDMA_SetChannelMux(DEMO_DMA, DEMO_RX_EDMA_CHANNEL, DEMO_SAI_RX_EDMA_CHANNEL);
#endif
	
}

void record_start(int argc, void**argv)
{
	sai_transfer_t xfer;
    sai_transceiver_t config;
	
	rt_mb_init(&record_mb, "record",
        &record_mb_pool[0], sizeof(record_mb_pool) / sizeof(record_mb_pool[0]),
        RT_IPC_FLAG_FIFO);
		
	recorder_init();
	
	/* SAI init */
    SAI_Init(DEMO_SAI);
    SAI_TransferRxCreateHandleEDMA(DEMO_SAI, &rxHandle, rx_callback, NULL, &dmaRxHandle);

    /* I2S mode configurations */
    SAI_GetClassicI2SConfig(&config, DEMO_AUDIO_BIT_WIDTH, kSAI_Stereo, 1); 
		
	config.serialData.dataWord0Length     = (uint8_t)32;
    config.serialData.dataWordLength      = (uint8_t)32;
    config.serialData.dataWordNLength     = (uint8_t)32;
    config.serialData.dataFirstBitShifted = (uint8_t)24;
		
    config.syncMode    = DEMO_SAI_TX_SYNC_MODE;
    config.masterSlave = DEMO_SAI_MASTER_SLAVE;
	SAI_TransferTxSetConfigEDMA(DEMO_SAI, &txHandle, &config);
    config.syncMode 	= DEMO_SAI_RX_SYNC_MODE;
    
	config.frameSync.frameSyncPolarity =  kSAI_PolarityActiveLow;
	config.frameSync.frameSyncEarly = true;
	config.frameSync.frameSyncWidth = 32;
	config.serialData.dataWordNum = 2; 
	
	SAI_TransferRxSetConfigEDMA(DEMO_SAI, &rxHandle, &config);
	
    /* set bit clock divider */
	SAI_TxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, 32,2);
		
    SAI_RxSetBitClockRate(DEMO_SAI, DEMO_AUDIO_MASTER_CLOCK, DEMO_AUDIO_SAMPLE_RATE, 32,
                          2);
	
	char *fname = "/sd/record0.pcm";
	
	if (argc == 2)
		fname = argv[1];
		
	record_tid = rt_thread_create("recorder", recard_thread_entry, (void*)fname,
                            RT_MAIN_THREAD_STACK_SIZE, RT_MAIN_THREAD_PRIORITY-6, 20);
	
	rt_thread_startup(record_tid);
	
	
}

MSH_CMD_EXPORT(record_start, start record)