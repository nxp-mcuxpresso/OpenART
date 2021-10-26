/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-03-27     Liuguang     the first version.
 */

#include <rtthread.h>
#ifdef BSP_USING_SPI

#include "drv_spi.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_edma.h"
#include "fsl_dmamux.h"
#include "drv_gpio.h"

#define LOG_TAG             "drv.spi"
#include <drv_log.h>

#if defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL
#error "Please don't define 'FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL'!"
#endif
#define SPI1_CS               GET_PIN(3,13)
#define SPI2_CS               GET_PIN(3,6)
#define SPI3_CS               GET_PIN(1,3)
#define SPI4_CS               GET_PIN(1,19)
#define EXAMPLE_LPSPI_CLOCK_SOURCE_SELECT (1U)
#define EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER (7U)

#define LPSPI_MASTER_CLK_FREQ (CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER + 1U))
enum
{
#ifdef BSP_USING_SPI1
    SPI1_INDEX,
#endif
#ifdef BSP_USING_SPI2
    SPI2_INDEX,
#endif
#ifdef BSP_USING_SPI3
    SPI3_INDEX,
#endif
#ifdef BSP_USING_SPI4
    SPI4_INDEX,
#endif
};
#define BSP_SPI1_RX_DMA_CHANNEL 0U
#define BSP_SPI1_TX_DMA_CHANNEL 1U


struct dma_config
{
    lpspi_master_edma_handle_t spi_edma;

    edma_handle_t rx_edma;
    dma_request_source_t rx_request;
    rt_uint8_t rx_channel;

    edma_handle_t tx_edma;
    dma_request_source_t tx_request;
    rt_uint8_t tx_channel;
};

struct imxrt_spi
{
    char* bus_name;
    char* device_name;
    LPSPI_Type* base;
    rt_uint32_t cs_pin;
    struct rt_spi_bus spi_bus;
    struct dma_config* dma;
};

static struct imxrt_spi lpspis[] =
{
#ifdef BSP_USING_SPI1
    {
        .bus_name = "spi1",
        .device_name = "spi10",
        .cs_pin = SPI1_CS,
        .base = LPSPI1,
        .dma = RT_NULL,
    },
#endif
#ifdef BSP_USING_SPI2
    {
        .bus_name = "spi2",
        .device_name = "spi20",
        .cs_pin = SPI2_CS,
        .base = LPSPI2,
        .dma = RT_NULL,
    },
#endif
#ifdef BSP_USING_SPI3
    {
        .bus_name = "spi3",
        .device_name = "spi30",
        .cs_pin = SPI3_CS,
        .base = LPSPI3,
        .dma = RT_NULL,
    },
#endif
#ifdef BSP_USING_SPI4
    {
        .bus_name = "spi4",
        .device_name = "spi40",
        .cs_pin = SPI4_CS,
        .base = LPSPI4,
        .dma = RT_NULL,
    },
#endif
};

static void spi_get_dma_config(void)
{
#if (defined(BSP_SPI_USING_DMA) & defined(BSP_USING_SPI1))
    static struct dma_config spi1_dma =
    {
        .rx_request = kDmaRequestMuxspi->baseRx,
        .rx_channel = BSP_SPI1_RX_DMA_CHANNEL,
        .tx_request = kDmaRequestMuxspi->baseTx,
        .tx_channel = BSP_SPI1_TX_DMA_CHANNEL,
    };

    lpspis[SPI1_INDEX].dma = &spi1_dma;
#endif

#if (defined(BSP_SPI_USING_DMA) & defined(BSP_USING_SPI2))
    static struct dma_config spi2_dma =
    {
        .rx_request = kDmaRequestMuxLPSPI2Rx,
        .rx_channel = BSP_SPI2_RX_DMA_CHANNEL,
        .tx_request = kDmaRequestMuxLPSPI2Tx,
        .tx_channel = BSP_SPI2_TX_DMA_CHANNEL,
    };

    lpspis[SPI2_INDEX].dma = &spi2_dma;
#endif

#if (defined(BSP_SPI_USING_DMA) & defined(BSP_USING_SPI3))
    static struct dma_config spi3_dma =
    {
        .rx_request = kDmaRequestMuxLPSPI3Rx,
        .rx_channel = BSP_SPI3_RX_DMA_CHANNEL,
        .tx_request = kDmaRequestMuxLPSPI3Tx,
        .tx_channel = BSP_SPI3_TX_DMA_CHANNEL,
    };

    lpspis[SPI3_INDEX].dma = &spi3_dma;
#endif

#if (defined(BSP_SPI_USING_DMA) & defined(BSP_USING_SPI4))
    static struct dma_config spi4_dma =
    {
        .rx_request = kDmaRequestMuxLPSPI4Rx,
        .rx_channel = BSP_SPI4_RX_DMA_CHANNEL,
        .tx_request = kDmaRequestMuxLPSPI4Tx,
        .tx_channel = BSP_SPI4_TX_DMA_CHANNEL,
    };

    lpspis[SPI4_INDEX].dma = &spi4_dma;
#endif
}

void edma_xfer_callback(LPSPI_Type* base, lpspi_master_edma_handle_t* handle, status_t status, void* userData)
{
    /* xfer complete callback */
}

rt_err_t rt_hw_spi_device_attach(const char* bus_name, const char* device_name, rt_uint32_t pin)
{
    rt_err_t ret = RT_EOK;

    struct rt_spi_device* spi_device = (struct rt_spi_device*)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);

    struct imxrt_sw_spi_cs* cs_pin = (struct imxrt_sw_spi_cs*)rt_malloc(sizeof(struct imxrt_sw_spi_cs));
    RT_ASSERT(cs_pin != RT_NULL);
#if 0//mux set in machine_hw_spi.c,filled in configurate
    cs_pin->pin = pin;
    rt_pin_mode(pin, PIN_MODE_OUTPUT);
    rt_pin_write(pin, PIN_HIGH);
#endif
    ret = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void*)cs_pin);

    return ret;
}

static uint32_t imxrt_get_lpspi_freq(void)
{
    uint32_t freq = 0;

    /* CLOCK_GetMux(kCLOCK_LpspiMux):
       00b: derive clock from PLL3 PFD1 720M
       01b: derive clock from PLL3 PFD0 720M
       10b: derive clock from PLL2      528M
       11b: derive clock from PLL2 PFD2 396M
    */
    switch(CLOCK_GetMux(kCLOCK_LpspiMux))
    {
    case 0:
        freq = CLOCK_GetFreq(kCLOCK_Usb1PllPfd1Clk);
        break;

    case 1:
        freq = CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk);
        break;

    case 2:
        freq = CLOCK_GetFreq(kCLOCK_SysPllClk);
        break;

    case 3:
        freq = CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk);
        break;
    }

    freq /= (CLOCK_GetDiv(kCLOCK_LpspiDiv) + 1U);

    return freq;
}
#ifdef BSP_SPI_USING_DMA
static void lpspi_dma_config(struct imxrt_spi* spi)
{
    RT_ASSERT(spi != RT_NULL);

    DMAMUX_SetSource(DMAMUX, spi->dma->rx_channel, spi->dma->rx_request);
    DMAMUX_EnableChannel(DMAMUX, spi->dma->rx_channel);
    EDMA_CreateHandle(&spi->dma->rx_edma, DMA0, spi->dma->rx_channel);

    DMAMUX_SetSource(DMAMUX, spi->dma->tx_channel, spi->dma->tx_request);
    DMAMUX_EnableChannel(DMAMUX, spi->dma->tx_channel);
    EDMA_CreateHandle(&spi->dma->tx_edma, DMA0, spi->dma->tx_channel);

    LPSPI_MasterTransferCreateHandleEDMA(spi->base,
                                         &spi->dma->spi_edma,
                                         edma_xfer_callback,
                                         spi,
                                         &spi->dma->rx_edma,
                                         &spi->dma->tx_edma);

    LOG_D("%s dma config done\n", spi->bus_name);
}
#endif
static rt_err_t spi_configure(struct rt_spi_device* device, struct rt_spi_configuration* cfg)
{
    lpspi_master_config_t masterConfig;
    struct imxrt_spi* spi = RT_NULL;
    rt_uint32_t srcClock_Hz;
    rt_uint8_t g_masterRxWatermark;
    rt_uint8_t g_masterFifoSize;
    rt_uint32_t whichPcs;
    rt_uint8_t txWatermark;
    rt_bool_t isMasterTransferCompleted = RT_FALSE;
    volatile uint32_t masterTxCount;
    volatile uint32_t masterRxCount;

    RT_ASSERT(cfg != RT_NULL);
    RT_ASSERT(device != RT_NULL);
    CLOCK_SetMux(kCLOCK_LpspiMux, EXAMPLE_LPSPI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER);
    spi = (struct imxrt_spi*)(device->bus->parent.user_data);
    RT_ASSERT(spi != RT_NULL);
	
	//set io
	struct imxrt_sw_spi_cs* cs_pin = (struct imxrt_sw_spi_cs*)device->parent.user_data;
	cs_pin->pin = cfg->cs_pin.pin;
	cs_pin->gpio = cfg->cs_pin.gpio;
	
    if(cfg->data_width != 8 && cfg->data_width != 16 && cfg->data_width != 32)
    {
        return RT_EINVAL;
    }

    LPSPI_MasterGetDefaultConfig(&masterConfig);

    if(cfg->max_hz > 40 * 1000 * 1000)
    {
        cfg->max_hz = 40 * 1000 * 1000;
    }
    masterConfig.baudRate     = cfg->max_hz;
    masterConfig.bitsPerFrame = cfg->data_width;

    if(cfg->mode & RT_SPI_MSB)
    {
        masterConfig.direction = kLPSPI_MsbFirst;
    }
    else
    {
        masterConfig.direction = kLPSPI_LsbFirst;
    }

    if(cfg->mode & RT_SPI_CPHA)
    {
        masterConfig.cpha = kLPSPI_ClockPhaseSecondEdge;
    }
    else
    {
        masterConfig.cpha = kLPSPI_ClockPhaseFirstEdge;
    }

    if(cfg->mode & RT_SPI_CPOL)
    {
        masterConfig.cpol = kLPSPI_ClockPolarityActiveLow;
    }
    else
    {
        masterConfig.cpol = kLPSPI_ClockPolarityActiveHigh;
    }
	masterConfig.pcsToSckDelayInNanoSec        = 1000000000 / masterConfig.baudRate;
    masterConfig.lastSckToPcsDelayInNanoSec    = 1000000000 / masterConfig.baudRate;
    masterConfig.betweenTransferDelayInNanoSec = 1000000000 / masterConfig.baudRate;
	
    masterConfig.whichPcs = kLPSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;
    masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;
    masterConfig.dataOutConfig                 = kLpspiDataOutRetained;

	LPSPI_Deinit(spi->base);
    LPSPI_MasterInit(spi->base, &masterConfig, imxrt_get_lpspi_freq());
	
    isMasterTransferCompleted = false;
    masterTxCount             = 0;
    masterRxCount             = 0;
    whichPcs                  = kLPSPI_Pcs0;

    /*The TX and RX FIFO sizes are always the same*/
    g_masterFifoSize = LPSPI_GetRxFifoSize(spi->base);

    /*Set the RX and TX watermarks to reduce the ISR times.*/
    if(g_masterFifoSize > 1)
    {
        txWatermark         = 1;
        g_masterRxWatermark = g_masterFifoSize - 2;
    }
    else
    {
        txWatermark         = 0;
        g_masterRxWatermark = 0;
    }

    LPSPI_SetFifoWatermarks(spi->base, txWatermark, g_masterRxWatermark);
    spi->base->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);

    /*Flush FIFO , clear status , disable all the inerrupts.*/
    LPSPI_FlushFifo(spi->base, true, true);
    LPSPI_ClearStatusFlags(spi->base, kLPSPI_AllStatusFlag);
    LPSPI_DisableInterrupts(spi->base, kLPSPI_AllInterruptEnable);

    spi->base->TCR =
        (spi->base->TCR &
         ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK)) |
        LPSPI_TCR_CONT(0) | LPSPI_TCR_CONTC(0) | LPSPI_TCR_RXMSK(0) | LPSPI_TCR_TXMSK(0) | LPSPI_TCR_PCS(whichPcs);


    return RT_EOK;
}
static rt_uint32_t spixfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    lpspi_transfer_t transfer;
    status_t status;
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->parent.user_data != RT_NULL);

    struct imxrt_spi* spi = (struct imxrt_spi*)(device->bus->parent.user_data);
    struct imxrt_sw_spi_cs* cs = device->parent.user_data;

    if(message->cs_take)
    {
        //rt_pin_write(cs->pin, PIN_LOW);
		GPIO_PinWrite(cs->gpio, cs->pin, 0);
    }
    

    transfer.dataSize = message->length;
    transfer.rxData   = (uint8_t*)(message->recv_buf);
    transfer.txData   = (uint8_t*)(message->send_buf);
    transfer.configFlags = kLPSPI_MasterPcsContinuous;
#if defined (BSP_SPI_USING_DMA)
    status = LPSPI_MasterTransferEDMA(spi->base, &spi->dma->spi_edma, &transfer);
#else
    status = LPSPI_MasterTransferBlocking(spi->base, &transfer);
#endif
    if(message->cs_release)
    {
        //rt_pin_write(cs->pin, PIN_HIGH);
		GPIO_PinWrite(cs->gpio, cs->pin, 1);
    }

    if(status != kStatus_Success)
    {
        LOG_E("%s transfer error : %d", spi->bus_name, status);
        message->length = 0;
    }

    return message->length;
}

static struct rt_spi_ops imxrt_spi_ops =
{
    .configure = spi_configure,
    .xfer      = spixfer
};

int rt_hw_spi_bus_init(void)
{
    int i;
    rt_err_t ret = RT_EOK;
    rt_err_t error = RT_EOK;
    spi_get_dma_config();

    for(i = 0; i < sizeof(lpspis) / sizeof(lpspis[0]); i++)
    {
        lpspis[i].spi_bus.parent.user_data = &lpspis[i];

        ret = rt_spi_bus_register(&lpspis[i].spi_bus, lpspis[i].bus_name, &imxrt_spi_ops);
        error = rt_hw_spi_device_attach(lpspis[i].bus_name, lpspis[i].device_name, NULL);
        if(error != RT_EOK)
        {
            LOG_E("%s has no external slave", lpspis[i].bus_name);
        }
#ifdef BSP_SPI_USING_DMA
        lpspi_dma_config(&lpspis[i]);
#endif
    }
    return ret;
}
INIT_BOARD_EXPORT(rt_hw_spi_bus_init);

#endif /* BSP_USING_SPI */
