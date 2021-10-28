/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"

#ifdef BSP_USING_DMA
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#endif

#define NVIC_PRIORITYGROUP_0         0x00000007U /*!< 0 bits for pre-emption priority
                                                      4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         0x00000006U /*!< 1 bits for pre-emption priority
                                                      3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         0x00000005U /*!< 2 bits for pre-emption priority
                                                      2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         0x00000004U /*!< 3 bits for pre-emption priority
                                                      1 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         0x00000003U /*!< 4 bits for pre-emption priority

                                                      0 bits for subpriority */
#ifdef BSP_USING_FLASH
#include "fsl_flexspi.h"
#include "flash.h"


__attribute__((section(".ram_code"))) uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
    /* Normal read mode -SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x03, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Fast read mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Fast read quad mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x06, kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04),

    /* Read extend parameters */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Erase Sector  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x20, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

    /* Page Program - single mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Page Program - quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read ID */
    [4 * NOR_CMD_LUT_SEQ_IDX_READID] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enable Quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enter QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_ENTERQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x35, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Exit QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_EXITQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_4PAD, 0xF5, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read status register */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Erase whole chip */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASECHIP] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xc7, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),
};



void BOARD_InitFlexspiPins(){
	  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* iomuxc clock (iomuxc_clk_enable): 0x03u */
                                  /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_05_FLEXSPIA_DQS,      /* GPIO_SD_B1_05 is configured as FLEXSPIA_DQS */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_05 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_06_FLEXSPIA_SS0_B,    /* GPIO_SD_B1_06 is configured as FLEXSPIA_SS0_B */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_06 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_07_FLEXSPIA_SCLK,     /* GPIO_SD_B1_07 is configured as FLEXSPIA_SCLK */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_07 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_08_FLEXSPIA_DATA00,   /* GPIO_SD_B1_08 is configured as FLEXSPIA_DATA00 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_08 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_09_FLEXSPIA_DATA01,   /* GPIO_SD_B1_09 is configured as FLEXSPIA_DATA01 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_09 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_10_FLEXSPIA_DATA02,   /* GPIO_SD_B1_10 is configured as FLEXSPIA_DATA02 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_10 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B1_11_FLEXSPIA_DATA03,   /* GPIO_SD_B1_11 is configured as FLEXSPIA_DATA03 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B1_11 */

  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_05_FLEXSPIA_DQS,      /* GPIO_SD_B1_05 PAD functional properties : */
      0x10F1u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_06_FLEXSPIA_SS0_B,    /* GPIO_SD_B1_06 PAD functional properties : */
      0x10F1u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_07_FLEXSPIA_SCLK,     /* GPIO_SD_B1_07 PAD functional properties : */
      0x10F1u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_08_FLEXSPIA_DATA00,   /* GPIO_SD_B1_08 PAD functional properties : */
      0x10F1u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_09_FLEXSPIA_DATA01,   /* GPIO_SD_B1_09 PAD functional properties : */
      0x10F1u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_10_FLEXSPIA_DATA02,   /* GPIO_SD_B1_10 PAD functional properties : */
      0x10F1u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B1_11_FLEXSPIA_DATA03,   /* GPIO_SD_B1_11 PAD functional properties : */
      0x10F1u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
}
#endif

void BOARD_Config_FLASH_Excute_MPU(uint8_t dis_excute) __attribute__((section(".ram_code")));
void BOARD_Config_FLASH_Excute_MPU(uint8_t dis_excute)
{
	MPU->RBAR = ARM_MPU_RBAR(3, 0x70000000U);
    MPU->RASR = ARM_MPU_RASR(dis_excute, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_512MB);
}


uint32_t BOARD_DebugConsoleSrcFreq(void)
{
    uint32_t freq;

    /* To make it simple, we assume default PLL and divider settings, and the only variable
       from application is use PLL3 source or OSC source */
    if (CLOCK_GetMux(kCLOCK_UartMux) == 0) /* PLL3 div6 80M */
    {
        freq = (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }
    else
    {
        freq = CLOCK_GetOscFreq() / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }

    return freq;
} 


/* MPU configuration. */
void BOARD_ConfigMPU(void)
{
	//uint32_t rgn = 0;
    /* Disable I cache and D cache */ 
    SCB_DisableICache();
    SCB_DisableDCache();
    /* Disable MPU */ 
    ARM_MPU_Disable(); 
    /* Region 0 setting */
    MPU->RBAR = ARM_MPU_RBAR(0, 0x00000000U);	// itcm, max 512kB
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512KB);

    /* Region 1 setting */
	// itcm RO region, catch wild pointers that will corrupt firmware code, region number must be larger to enable nest
    //MPU->RBAR = ARM_MPU_RBAR(1, 0x00000000U);	
    //MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_RO, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_8KB);

    /* Region 2 setting */
    MPU->RBAR = ARM_MPU_RBAR(1, 0x20000000U);	// dtcm, max 512kB
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512KB);   
    
    /* Region 3 setting */
    MPU->RBAR = ARM_MPU_RBAR(2, 0x20200000U);	// ocram
	// rocky: Must NOT set to device or strong ordered types, otherwise, unaligned access leads to fault	// ocram
	// better to disable bufferable ---- write back, so CPU always write through, avoid DMA and CPU write same line, error prone
	MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512KB);
	// MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_256KB);

    MPU->RBAR = ARM_MPU_RBAR(3, 0x70000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_512MB); 

   // MPU->RBAR = ARM_MPU_RBAR(5, 0x60380000U);
    //MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512KB); 

	uint32_t cur_pc = (uint32_t)__builtin_return_address(0);
	/* Region 5 setting, set whole SDRAM can be accessed by cache */
	//if ((cur_pc > 0x80000000U) && (cur_pc < 0x82000000U))
	//{
	//	MPU->RBAR = ARM_MPU_RBAR(4, 0x80000000U);
	//	MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_16MB);
	//	MPU->RBAR = ARM_MPU_RBAR(5, 0x81000000U);
	//	MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 0, 0, ARM_MPU_REGION_SIZE_16MB);
	//}
	//else
	//{
		MPU->RBAR = ARM_MPU_RBAR(4, 0x80000000U);
		MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_32MB);    
	//}
    /* Region 6 setting, set last 2MB of SDRAM can't be accessed by cache, glocal variables which are not expected to be accessed by cache can be put here */
    MPU->RBAR = ARM_MPU_RBAR(6, 0x81E00000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_2MB);   
  

    // MPU->RBAR = ARM_MPU_RBAR(7, 0xC0000000U);
    // MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_512MB);

    /* Enable MPU, enable background region for priviliged access */ 
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
	
    /* Enable I cache and D cache */ 
    SCB_EnableDCache();
    SCB_EnableICache();
}


/* This is the timer interrupt service routine. */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef BSP_USING_DMA
void imxrt_dma_init(void)
{
    edma_config_t config;

    DMAMUX_Init(DMAMUX);
    EDMA_GetDefaultConfig(&config);
    EDMA_Init(DMA0, &config);
}
#endif

#ifdef BSP_USING_74HC595
extern int drv_hc595_gpio_write(uint8_t gpio, uint8_t value);
void BOARD_Camera_pwd_io_set(int value)
{
	drv_hc595_gpio_write(7,value);
}

void BORAD_Camera_rst_io_set(int value)
{
	drv_hc595_gpio_write(8,value);
}
#endif


void imxrt_led_pins_init(void)
{
//	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03,0);
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03, 0x017089u);
	
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09,0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_GPIO1_IO09, 0x017089u);
		 
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10, 0x017089u);
	 
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11,0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_GPIO1_IO11, 0x017089u);
	 
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_11_GPIO2_IO11,0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_11_GPIO2_IO11, 0x017089u);
}

 void imxrt_i2c_pins_init(void)
 {
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL,        /* GPIO_AD_B1_00 is configured as LPI2C1_SCL */
      1U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA,        /* GPIO_AD_B1_01 is configured as LPI2C1_SDA */
      1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL,0xD8B0u); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA,0xD8B0u); 
 }

#ifdef BSP_USING_LPUART
 void imxrt_uart_pins_init(void)
 {
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_B1_12_LPUART5_TX,        	  /* GPIO_B1_12 is configured as LPUART5_TX */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_B1_13_LPUART5_RX,        	  /* GPIO_B1_13 is configured as LPUART5_RX */
		0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	  
	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_B1_12_LPUART5_TX,        	  /* GPIO_B1_12 PAD functional properties : */
		0x10B0u);                               /* Slew Rate Field: Slow Slew Rate*/

	IOMUXC_SetPinConfig(
		IOMUXC_GPIO_B1_13_LPUART5_RX,        	  /* GPIO_B1_13 PAD functional properties : */
		0x10B0u);                               /* Slew Rate Field: Slow Slew Rate*/
	 
 #ifdef BSP_USING_LPUART1

         IOMUXC_SetPinMux(
             IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 is configured as LPUART1_TX */
             0U);                                    /* Software Input On Field: Input Path is determined by functionality */
         IOMUXC_SetPinMux(
             IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 is configured as LPUART1_RX */
             0U);                                    /* Software Input On Field: Input Path is determined by functionality */
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_AD_B0_12_LPUART1_TX,        /* GPIO_AD_B0_12 PAD functional properties : */
             0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                     // Drive Strength Field: R0/6
                                                     // Speed Field: medium(100MHz)
                                                     // Open Drain Enable Field: Open Drain Disabled
                                                     // Pull / Keep Enable Field: Pull/Keeper Enabled
                                                     // Pull / Keep Select Field: Keeper
                                                     // Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                     // Hyst. Enable Field: Hysteresis Disabled */
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_AD_B0_13_LPUART1_RX,        /* GPIO_AD_B0_13 PAD functional properties : */
             0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                     // Drive Strength Field: R0/6
                                                     // Speed Field: medium(100MHz)
                                                     // Open Drain Enable Field: Open Drain Disabled
                                                     // Pull / Keep Enable Field: Pull/Keeper Enabled
                                                     // Pull / Keep Select Field: Keeper
                                                     // Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                     // Hyst. Enable Field: Hysteresis Disabled */
 #endif
 #ifdef BSP_USING_LPUART2

         IOMUXC_SetPinMux(
             IOMUXC_GPIO_AD_B1_02_LPUART2_TX,
            0U);
         IOMUXC_SetPinMux(
             IOMUXC_GPIO_AD_B1_03_LPUART2_RX,
             0U);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_AD_B1_02_LPUART2_TX,
             0x10B0u);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_AD_B1_03_LPUART2_RX,
             0x10B0u);

 #endif
 #ifdef BSP_USING_LPUART3

         IOMUXC_SetPinMux(
             IOMUXC_GPIO_B0_08_LPUART3_TX,
             0U);
         IOMUXC_SetPinMux(
             IOMUXC_GPIO_B0_09_LPUART3_RX,
             0U);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_B0_08_LPUART3_TX,
             0x10B0u);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_B0_09_LPUART3_RX,
             0x10B0u);
#endif
 #ifdef BSP_USING_LPUART4

         IOMUXC_SetPinMux(
             IOMUXC_GPIO_B1_00_LPUART4_TX,
             0U);
         IOMUXC_SetPinMux(
             IOMUXC_GPIO_B1_01_LPUART4_RX,
             0U);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_B1_00_LPUART4_TX,
             0x10B0u);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_B1_01_LPUART4_RX,
             0x10B0u);
#endif
#ifdef BSP_USING_LPUART5

         IOMUXC_SetPinMux(
             IOMUXC_GPIO_B1_12_LPUART5_TX,
             0U);
         IOMUXC_SetPinMux(
             IOMUXC_GPIO_B1_13_LPUART5_RX,
             0U);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_B1_12_LPUART5_TX,
             0x10B0u);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_B1_13_LPUART5_RX,
             0x10B0u);
 #endif
 #ifdef BSP_USING_LPUART6

         IOMUXC_SetPinMux(
             IOMUXC_GPIO_AD_B0_02_LPUART6_TX,
             0U);
         IOMUXC_SetPinMux(
             IOMUXC_GPIO_AD_B0_03_LPUART6_RX,
             0U);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_AD_B0_02_LPUART6_TX,
             0x10B0u);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_AD_B0_03_LPUART6_RX,
             0x10B0u);
 #endif
 #ifdef BSP_USING_LPUART7

         IOMUXC_SetPinMux(
             IOMUXC_GPIO_EMC_31_LPUART7_TX,
             0U);
         IOMUXC_SetPinMux(
             IOMUXC_GPIO_EMC_32_LPUART7_RX,
             0U);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_EMC_31_LPUART7_TX,
             0x10B0u);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_EMC_32_LPUART7_RX,
             0x10B0u);
#endif
#ifdef BSP_USING_LPUART8

         IOMUXC_SetPinMux(
             IOMUXC_GPIO_AD_B1_10_LPUART8_TX,
             0U);
         IOMUXC_SetPinMux(
             IOMUXC_GPIO_AD_B1_11_LPUART8_RX,
             0U);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_AD_B1_10_LPUART8_TX,
             0x10B0u);
         IOMUXC_SetPinConfig(
             IOMUXC_GPIO_AD_B1_11_LPUART8_RX,
             0x10B0u);
#endif
}
#endif /* BSP_USING_LPUART */

#ifdef BSP_USING_ETH
void imxrt_enet_pins_init(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);          /* iomuxc clock (iomuxc_clk_enable): 0x03u */

    IOMUXC_SetPinMux(
	IOMUXC_GPIO_AD_B0_09_GPIO1_IO09,        /* GPIO_AD_B0_09 is configured as GPIO1_IO09 */
	0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,        /* GPIO_AD_B0_10 is configured as GPIO1_IO10 */
        0U);
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_B1_04_ENET_RX_DATA00,       /* GPIO_B1_04 is configured as ENET_RX_DATA00 */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_B1_05_ENET_RX_DATA01,       /* GPIO_B1_05 is configured as ENET_RX_DATA01 */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_B1_06_ENET_RX_EN,           /* GPIO_B1_06 is configured as ENET_RX_EN */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_B1_07_ENET_TX_DATA00,       /* GPIO_B1_07 is configured as ENET_TX_DATA00 */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_B1_08_ENET_TX_DATA01,       /* GPIO_B1_08 is configured as ENET_TX_DATA01 */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_B1_09_ENET_TX_EN,           /* GPIO_B1_09 is configured as ENET_TX_EN */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_B1_10_ENET_REF_CLK,         /* GPIO_B1_10 is configured as ENET_REF_CLK */
        1U);                                    /* Software Input On Field: Force input path of pad GPIO_B1_10 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_B1_11_ENET_RX_ER,           /* GPIO_B1_11 is configured as ENET_RX_ER */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_EMC_40_ENET_MDC,            /* GPIO_EMC_40 is configured as ENET_MDC */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_EMC_41_ENET_MDIO,           /* GPIO_EMC_41 is configured as ENET_MDIO */
        0U);                                    /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_AD_B0_09_GPIO1_IO09,        /* GPIO_AD_B0_09 PAD functional properties : */
        0xB0A9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,        /* GPIO_AD_B0_10 PAD functional properties : */
        0xB0A9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_B1_04_ENET_RX_DATA00,       /* GPIO_B1_04 PAD functional properties : */
        0xB0E9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_B1_05_ENET_RX_DATA01,       /* GPIO_B1_05 PAD functional properties : */
        0xB0E9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_B1_06_ENET_RX_EN,           /* GPIO_B1_06 PAD functional properties : */
        0xB0E9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_B1_07_ENET_TX_DATA00,       /* GPIO_B1_07 PAD functional properties : */
        0xB0E9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_B1_08_ENET_TX_DATA01,       /* GPIO_B1_08 PAD functional properties : */
        0xB0E9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_B1_09_ENET_TX_EN,           /* GPIO_B1_09 PAD functional properties : */
        0xB0E9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_B1_10_ENET_REF_CLK,         /* GPIO_B1_10 PAD functional properties : */
        0x31u);                                 /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: low(50MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Disabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_B1_11_ENET_RX_ER,           /* GPIO_B1_11 PAD functional properties : */
        0xB0E9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_EMC_40_ENET_MDC,            /* GPIO_EMC_40 PAD functional properties : */
        0xB0E9u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: max(200MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
    IOMUXC_SetPinConfig(
        IOMUXC_GPIO_EMC_41_ENET_MDIO,           /* GPIO_EMC_41 PAD functional properties : */
        0xB829u);                               /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/5
                                                 Speed Field: low(50MHz)
                                                 Open Drain Enable Field: Open Drain Enabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
}

void imxrt_enet_phy_reset_by_gpio(void)
{
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    GPIO_PinInit(GPIO1, 9, &gpio_config);
    GPIO_PinInit(GPIO1, 10, &gpio_config);
    /* pull up the ENET_INT before RESET. */
    GPIO_WritePinOutput(GPIO1, 10, 1);
    GPIO_WritePinOutput(GPIO1, 9, 0);
    rt_thread_delay(100);
    GPIO_WritePinOutput(GPIO1, 9, 1);
}
#endif /* BSP_USING_ETH */

#ifdef BSP_USING_AUDIO
void imxrt_sai_pins_init(void)
{                                   /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL,        /* GPIO_AD_B1_00 is configured as LPI2C1_SCL */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_B1_00 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA,        /* GPIO_AD_B1_01 is configured as LPI2C1_SDA */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_B1_01 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_09_SAI1_MCLK,         /* GPIO_AD_B1_09 is configured as SAI1_MCLK */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_B1_09 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_12_SAI1_RX_DATA00,    /* GPIO_AD_B1_12 is configured as SAI1_RX_DATA00 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_B1_12 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_13_SAI1_TX_DATA00,    /* GPIO_AD_B1_13 is configured as SAI1_TX_DATA00 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_B1_13 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_14_SAI1_TX_BCLK,      /* GPIO_AD_B1_14 is configured as SAI1_TX_BCLK */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_B1_14 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_B1_15_SAI1_TX_SYNC,      /* GPIO_AD_B1_15 is configured as SAI1_TX_SYNC */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_AD_B1_15 */

  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_00_LPI2C1_SCL,        /* GPIO_AD_B1_00 PAD functional properties : */
      0xD8B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Enabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 22K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_01_LPI2C1_SDA,        /* GPIO_AD_B1_01 PAD functional properties : */
      0xD8B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Enabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 22K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_09_SAI1_MCLK,         /* GPIO_AD_B1_09 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_12_SAI1_RX_DATA00,    /* GPIO_AD_B1_12 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_13_SAI1_TX_DATA00,    /* GPIO_AD_B1_13 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_14_SAI1_TX_BCLK,      /* GPIO_AD_B1_14 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_B1_15_SAI1_TX_SYNC,      /* GPIO_AD_B1_15 PAD functional properties : */
      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: R0/6
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
                                                 Hyst. Enable Field: Hysteresis Disabled */

}
#endif
/**
 * This function will initial rt1050 board.
 */
#ifdef BSP_USING_SDIO
static void BOARD_USDHCClockConfiguration(void)
{

    CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
    
    CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
    CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);
 CLOCK_EnableClock(kCLOCK_Dma);
} 

void imxrt_SDcard_pins_init(void)
{
//  IOMUXC_SetPinMux(
//      IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,        /* GPIO_AD_B0_05 is configured as GPIO1_IO05 */
//      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
//  IOMUXC_SetPinMux(
//      IOMUXC_GPIO_B1_12_GPIO2_IO28,           /* GPIO_B1_12 is configured as GPIO2_IO28 */
//      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B1_14_USDHC1_VSELECT,       /* GPIO_B1_14 is configured as USDHC1_VSELECT */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,        /* GPIO_SD_B0_00 is configured as USDHC1_CMD */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,        /* GPIO_SD_B0_01 is configured as USDHC1_CLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,      /* GPIO_SD_B0_02 is configured as USDHC1_DATA0 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,      /* GPIO_SD_B0_03 is configured as USDHC1_DATA1 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,      /* GPIO_SD_B0_04 is configured as USDHC1_DATA2 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,      /* GPIO_SD_B0_05 is configured as USDHC1_DATA3 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
//  IOMUXC_SetPinConfig(
//      IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,        /* GPIO_AD_B0_05 PAD functional properties : */
//      0x10B0u);                               /* Slew Rate Field: Slow Slew Rate
//                                                 Drive Strength Field: R0/6
//                                                 Speed Field: medium(100MHz)
//                                                 Open Drain Enable Field: Open Drain Disabled
//                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
//                                                 Pull / Keep Select Field: Keeper
//                                                 Pull Up / Down Config. Field: 100K Ohm Pull Down
//                                                 Hyst. Enable Field: Hysteresis Disabled */
//  IOMUXC_SetPinConfig(
//      IOMUXC_GPIO_B1_12_GPIO2_IO28,           /* GPIO_B1_12 PAD functional properties : */
//      0x017089u);                             /* Slew Rate Field: Fast Slew Rate
//                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
//                                                 Speed Field: medium(100MHz)
//                                                 Open Drain Enable Field: Open Drain Disabled
//                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
//                                                 Pull / Keep Select Field: Pull
//                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
//                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_B1_14_USDHC1_VSELECT,       /* GPIO_B1_14 PAD functional properties : */
      0x0170A1u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0/4
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_00_USDHC1_CMD,        /* GPIO_SD_B0_00 PAD functional properties : */
      0x017089u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_01_USDHC1_CLK,        /* GPIO_SD_B0_01 PAD functional properties : */
      0x014089u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Disabled
                                                 Pull / Keep Select Field: Keeper
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_02_USDHC1_DATA0,      /* GPIO_SD_B0_02 PAD functional properties : */
      0x017089u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_03_USDHC1_DATA1,      /* GPIO_SD_B0_03 PAD functional properties : */
      0x017089u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_04_USDHC1_DATA2,      /* GPIO_SD_B0_04 PAD functional properties : */
      0x017089u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B0_05_USDHC1_DATA3,      /* GPIO_SD_B0_05 PAD functional properties : */
      0x017089u);                             /* Slew Rate Field: Fast Slew Rate
                                                 Drive Strength Field: R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
                                                 Speed Field: medium(100MHz)
                                                 Open Drain Enable Field: Open Drain Disabled
                                                 Pull / Keep Enable Field: Pull/Keeper Enabled
                                                 Pull / Keep Select Field: Pull
                                                 Pull Up / Down Config. Field: 47K Ohm Pull Up
                                                 Hyst. Enable Field: Hysteresis Enabled */	
}
#endif

#ifdef BSP_USING_74HC595
#include "drv_74hc595.h"
#define GET_PIN(PORTx, PIN)      (32 * (PORTx - 1) + (PIN & 31))    /* PORTx:1,2,3,4,5 */
void BOARD_InitExpensionIO()
{
//	//hc595 shcp
//	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_GPIO1_IO31,0);
//	IOMUXC_SetPinConfig(
//      IOMUXC_GPIO_AD_B1_15_GPIO1_IO31,          
//      0x017089u);
//	//hc 595 stcp
//	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_40_GPIO3_IO26,0);
//	IOMUXC_SetPinConfig(
//      IOMUXC_GPIO_EMC_40_GPIO3_IO26,          
//      0x017089u); 
//	//hc595 data
//	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_41_GPIO3_IO27,0);
//	IOMUXC_SetPinConfig(
//      IOMUXC_GPIO_EMC_41_GPIO3_IO27,          
//      0x017089u); 
//	
//	drv_74hc595_init_io(GET_PIN(1,31),GET_PIN(3,26),GET_PIN(3,27));
}
#endif

#ifdef RT_USING_WIFI_M8266
void m8266_pins_init()
{                                   /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_B1_07_LPSPI4_SCK,        /* GPIO_SD_B0_00 is configured as LPSPI1_SCK */
	  0U);                                    /* Software Input On Field: Input Path is determined by functionality */                                   /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_B1_06_LPSPI4_SDO,        /* GPIO_SD_B0_02 is configured as LPSPI1_SDO */
	  0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_B1_05_LPSPI4_SDI,        /* GPIO_SD_B0_03 is configured as LPSPI1_SDI */
	  0U);                                    /* Software Input On Field: Input Path is determined by functionality */
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_07_LPSPI4_SCK,0x3069);  //0x3069=b0| 00|11| 0|000 01|10 1|00|1 -> Hysteresis Disabled, 100KOhm Pulldown, pull enabled, 100MHz, R0/5=52, Fast Slew Rate
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_06_LPSPI4_SDO,0x3069);                                                                              
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_05_LPSPI4_SDI,0x10B0u);
	
}

void M8266HostIf_Set_SPI_nCS_Pin(int value)
{
	if(value!=0)
	{
		drv_hc595_gpio_write(4,1);
	}
	else
	{
		drv_hc595_gpio_write(4,0);
	}
}

void M8266HostIf_Set_nRESET_Pin(int value)
{
	if(value!=0)
	{
		drv_hc595_gpio_write(5,1);
	}
	else
	{
		drv_hc595_gpio_write(5,0);
	}
}


#endif

void BOARD_MIC_Pins()
{
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_B0_14_SAI1_RX_SYNC,         
	  1U);                                    
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_B0_15_SAI1_RX_BCLK,    
	  1U);                                   
	IOMUXC_SetPinMux(
	  IOMUXC_GPIO_B1_00_SAI1_RX_DATA00,      
	  1U); 

	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_B0_15_SAI1_RX_BCLK,    
	  0x10B0u);                              
	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_B1_00_SAI1_RX_DATA00,   
	  0x10B0u);  
	  
	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_B0_14_SAI1_RX_SYNC,     
	  0x10B0u);       

	IOMUXC_SetPinMux(
		IOMUXC_GPIO_B0_00_MQS_RIGHT,
	1);
	IOMUXC_SetPinMux(
		IOMUXC_GPIO_B0_01_MQS_LEFT,
	1);
	
	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_B0_00_MQS_RIGHT,    
	  0x1088u); 
	  
	IOMUXC_SetPinConfig(
	  IOMUXC_GPIO_B0_01_MQS_LEFT,    
	  0x1088u); 
}

void imxrt_lcd_pins_init()
{
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_00_LPSPI3_SCK,1);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_00_LPSPI3_SCK,0x3069);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_01_LPSPI3_SDO,1);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_01_LPSPI3_SDO,0x3069);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_LPSPI3_PCS0,1);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_LPSPI3_PCS0,0x10B0u);
    //rst
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,0);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,0xB0A9u);
    
    //blk
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_02_GPIO2_IO18,0);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_02_GPIO2_IO18,0xB0A9u);
    //dc
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_03_GPIO2_IO19,0);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_03_GPIO2_IO19,0xB0A9u);

     gpio_pin_config_t config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    GPIO_PinInit(GPIO1, 2, &config);
    GPIO_PinInit(GPIO2, 18, &config);
    GPIO_PinInit(GPIO2, 19, &config);
}

void board_lcd_set_reset(int value)
{
    if(value)
	    GPIO_WritePinOutput(GPIO1, 2, 1);
    else
        GPIO_WritePinOutput(GPIO1, 2, 0);
}

void board_lcd_set_dc(int value)
{
    if(value)
	    GPIO_PinWrite(GPIO2, 19, 1);
    else
        GPIO_PinWrite(GPIO2, 19, 0);
}

void board_lcd_set_blk(int value)
{
    if(value)
	    GPIO_PinWrite(GPIO2, 18, 1);
    else
        GPIO_PinWrite(GPIO2, 18, 0);
}


#ifdef NXP_USING_OMV_TFLITE
	extern int SHT$$INIT_ARRAY$$Base[];
	extern int SHT$$INIT_ARRAY$$Limit[];
	#define S_CTOR SHT$$INIT_ARRAY$$Base
	#define E_CTOR SHT$$INIT_ARRAY$$Limit
	void $Sub$$__cpp_initialize__aeabi_(){

	}
#endif
void rt_hw_board_init()
{
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
	BOARD_MIC_Pins();
#ifdef BSP_USING_74HC595
	BOARD_InitExpensionIO();
#endif
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
	
	imxrt_led_pins_init();
	
	imxrt_i2c_pins_init();

    imxrt_lcd_pins_init();


#ifdef BSP_USING_LPUART
    imxrt_uart_pins_init();
#endif

#ifdef BSP_USING_ETH
    imxrt_enet_pins_init();
#endif

#ifdef BSP_USING_DMA
    imxrt_dma_init();
#endif

#ifdef BSP_USING_AUDIO
    imxrt_sai_pins_init();
#endif

#ifdef BSP_USING_SDIO
    imxrt_SDcard_pins_init();
	BOARD_USDHCClockConfiguration();
		
#endif

#ifdef RT_USING_HEAP
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif

#ifdef NXP_USING_OMV_TFLITE
	extern void $Super$$__cpp_initialize__aeabi_();
	$Super$$__cpp_initialize__aeabi_();
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#ifdef RT_USING_CONSOLE
#ifdef USB_CONSOLE_CDC_EN
	rt_console_set_device(RT_CONSOLE_CDC_DEVICE_NAME);
#else
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif	
#endif
}
