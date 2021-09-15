/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_tempsensor.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.tempsensor"
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void TMPSNS_AIWriteAccess(uint32_t address, uint32_t data);
static uint32_t TMPSNS_AIReadAccess(uint32_t address);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static int32_t s_Tp; /*!< Temperature of fuse-programming at 25 celsius degree.*/
static int32_t s_Np; /*!< Clock counts at 25 celsius degree using temperature sensor.*/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * brief Initializes the TMPSNS module.
 *
 * param base TMPSNS base pointer
 * param config Pointer to configuration structure.
 */

void TMPSNS_Init(TMPSNS_Type *base, const tmpsns_config_t *config)
{
    assert(NULL != config);
    uint32_t ControlVal;
    /* bias = 0x8020; */
    /* Bias trim and slopecal will be loaded into TMPSNS_CTRL0 control register from fuse
       by ROM code within B0, but now it will loaded by software within A0 */
    uint32_t iBiasTrim = 0x07;
    uint32_t slopeCal  = 0x1E;

    /* Set parameters value */
    /* Temperature of fuse-programming at 25 celsius degree.*/
    s_Tp = 25;
    /* Clock counts at 25 celsius degree using temperature sensor, which is 1915 + fuse.*/
    s_Np = 1915;

    /* Write bias and slope cal */
    TMPSNS_AIWriteAccess((uint32_t) & (base->CTRL0), (iBiasTrim << 12) + slopeCal);

    /* Power up the temperature sensor */
    ControlVal = TMPSNS_CTRL1_PWD(0x00U);

    if (config->measureMode == (uint8_t)kTEMPSENSOR_SingleMode)
    {
        ControlVal |= TMPSNS_CTRL1_FREQ(0x00U);
    }
    else if (config->measureMode == (uint8_t)kTEMPSENSOR_ContinuousMode)
    {
        ControlVal |= TMPSNS_CTRL1_FREQ(config->frequency);
    }
    else
    {
        ; /* Intentional empty for MISRA C-2012 rule 15.7*/
    }

    /* Enable finish interrupt status */
    ControlVal |= TMPSNS_CTRL1_FINISH_IE_MASK;

    TMPSNS_AIWriteAccess((uint32_t) & (base->CTRL1), ControlVal);

    /* Set alarm temperature */
    TMPSNS_SetTempAlarm(base, config->highAlarmTemp, kTEMPMON_HighAlarmMode);
    TMPSNS_SetTempAlarm(base, config->panicAlarmTemp, kTEMPMON_PanicAlarmMode);
    TMPSNS_SetTempAlarm(base, config->lowAlarmTemp, kTEMPMON_LowAlarmMode);
}

/*!
 * brief Deinitializes the TMPSNS module.
 *
 * param base TMPSNS base pointer
 */
void TMPSNS_Deinit(TMPSNS_Type *base)
{
    TMPSNS_AIWriteAccess((uint32_t) & (base->CTRL1), TMPSNS_CTRL1_PWD_MASK);
}

/*!
 * brief AI interface write access.
 *
 * param base TMPSNS base pointer
 */
static void TMPSNS_AIWriteAccess(uint32_t address, uint32_t data)
{
    /* AI bridge setting: AIRWB and ADDR
       Write: 0x00
       Address: offset from base address
     */
    ANADIG_MISC->VDDLPSR_AI_CTRL = (0x00UL << 16) | (address & 0xFFFFU);
    /* Write data into related register through AI bridge */
    ANADIG_MISC->VDDLPSR_AI_WDATA = data;
    /* AI toggle */
    ANADIG_TEMPSENSOR->TEMPSENSOR ^= ANADIG_TEMPSENSOR_TEMPSENSOR_TEMPSNS_AI_TOGGLE_MASK;
}

/*!
 * brief AI interface read access.
 *
 * param base TMPSNS base pointer
 */
static uint32_t TMPSNS_AIReadAccess(uint32_t address)
{
    uint32_t ret;

    /* AI bridge setting: AIRWB and ADDR
       Read: 0x01
       Address: offset from base address
     */
    ANADIG_MISC->VDDLPSR_AI_CTRL = (0x01UL << 16) | (address & 0xFFFFU);
    /* AI toggle */
    ANADIG_TEMPSENSOR->TEMPSENSOR ^= ANADIG_TEMPSENSOR_TEMPSENSOR_TEMPSNS_AI_TOGGLE_MASK;
    /* Read data from related register through AI bridge */
    ret = ANADIG_MISC->VDDLPSR_AI_RDATA_TMPSNS;

    return ret;
}

/*!
 * brief Gets the default configuration structure.
 *
 * This function initializes the TMPSNS configuration structure to a default value. The default
 * values are:
 *   tempmonConfig->frequency = 0x00U;
 *   tempmonConfig->highAlarmTemp = 25U;
 *   tempmonConfig->panicAlarmTemp = 80U;
 *   tempmonConfig->lowAlarmTemp = 20U;
 *
 * param config Pointer to a configuration structure.
 */
void TMPSNS_GetDefaultConfig(tmpsns_config_t *config)
{
    assert(config);

    /* Initializes the configure structure to zero. */
    (void)memset(config, 0, sizeof(*config));

    /* Default measurement mode */
    config->measureMode = kTEMPSENSOR_SingleMode;
    /* Default measure frequency */
    config->frequency = 0x00U;
    /* Default high alarm temperature */
    config->highAlarmTemp = 25;
    /* Default panic alarm temperature */
    config->panicAlarmTemp = 80;
    /* Default low alarm temperature */
    config->lowAlarmTemp = 20;
}

/*!
 * @brief start the temperature measurement process.
 *
 * @param base TMPSNS base pointer.
 */
void TMPSNS_StartMeasure(TMPSNS_Type *base)
{
    uint32_t controlVal;

    /* Read CTRL1 value*/
    controlVal = TMPSNS_AIReadAccess((uint32_t) & (base->CTRL1));

    /* Start measurement */
    TMPSNS_AIWriteAccess((uint32_t) & (base->CTRL1), controlVal | TMPSNS_CTRL1_SET_START_MASK);
}

/*!
 * @brief stop the measurement process.
 *
 * @param base TMPSNS base pointer
 */
void TMPSNS_StopMeasure(TMPSNS_Type *base)
{
    uint32_t controlVal;

    /* Read CTRL1 value*/
    controlVal = TMPSNS_AIReadAccess((uint32_t) & (base->CTRL1));

    /* Start measurement */
    TMPSNS_AIWriteAccess((uint32_t) & (base->CTRL1), controlVal & (~TMPSNS_CTRL1_SET_START_MASK));
}

/*!
 * brief Get current temperature with the fused temperature calibration data.
 *
 * param base TMPSNS base pointer
 * return current temperature with degrees Celsius.
 */
float TMPSNS_GetCurrentTemperature(TMPSNS_Type *base)
{
    uint32_t measureTempVal;
    uint32_t statusVal;
    float actualTempVal;

    /* Waiting for measurement finished */
    while (0U == (TMPSNS_AIReadAccess((uint32_t) & (base->STATUS0)) & TMPSNS_STATUS0_FINISH_MASK))
    {
    }

    /* Ready to read measured temperature value */
    measureTempVal = (TMPSNS_AIReadAccess((uint32_t) & (base->STATUS0)) & TMPSNS_STATUS0_TEMP_VAL_MASK) >>
                     TMPSNS_STATUS0_TEMP_VAL_SHIFT;

    /* Calculate actual temperature */
    actualTempVal = (float)s_Tp + ((float)s_Np - (float)measureTempVal) / (float)5.1;

    /* Read STATUS0 value */
    statusVal = TMPSNS_AIReadAccess((uint32_t) & (base->STATUS0));

    /* Clear the FINISH flag */
    TMPSNS_AIWriteAccess((uint32_t) & (base->STATUS0), statusVal & (~TMPSNS_STATUS0_FINISH_MASK));

    return actualTempVal;
}

/*!
 * brief Set the temperature count (raw sensor output) that will generate an alarm interrupt.
 *
 * param base TMPSNS base pointer
 * param tempVal The alarm temperature with degrees Celsius
 * param alarmMode The alarm mode.
 */
void TMPSNS_SetTempAlarm(TMPSNS_Type *base, int32_t tempVal, tmpsns_alarm_mode_t alarmMode)
{
    float temp;
    int32_t tempCodeVal;
    uint32_t tempRegVal;

    /* Calculate alarm temperature code value */;
    temp        = (((float)s_Tp - (float)tempVal) * (float)5.1);
    tempCodeVal = (int32_t)temp + s_Np;

    switch (alarmMode)
    {
        case kTEMPMON_HighAlarmMode:
            /* Clear alarm value and set a new high alarm temperature code value */
            tempRegVal = TMPSNS_AIReadAccess((uint32_t) & (base->RANGE0));
            tempRegVal = (tempRegVal & ~TMPSNS_RANGE0_HIGH_TEMP_VAL_MASK) | TMPSNS_RANGE0_HIGH_TEMP_VAL(tempCodeVal);
            TMPSNS_AIWriteAccess((uint32_t) & (base->RANGE0), tempRegVal);
            /* Enable high temperature interrupt */
            TMPSNS_EnableInterrupt(base, kTEMPSENSOR_HighTempInterruptStatusEnable);
            break;

        case kTEMPMON_PanicAlarmMode:
            /* Clear panic alarm value and set a new panic alarm temperature code value */
            tempRegVal = TMPSNS_AIReadAccess((uint32_t) & (base->RANGE1));
            tempRegVal = (tempRegVal & ~TMPSNS_RANGE1_PANIC_TEMP_VAL_MASK) | TMPSNS_RANGE1_PANIC_TEMP_VAL(tempCodeVal);
            TMPSNS_AIWriteAccess((uint32_t) & (base->RANGE1), tempRegVal);
            /* Enable panic temperature interrupt */
            TMPSNS_EnableInterrupt(base, kTEMPSENSOR_PanicTempInterruptStatusEnable);
            break;

        case kTEMPMON_LowAlarmMode:
            /* Clear low alarm value and set a new low alarm temperature code value */
            tempRegVal = TMPSNS_AIReadAccess((uint32_t) & (base->RANGE0));
            tempRegVal = (tempRegVal & ~TMPSNS_RANGE0_LOW_TEMP_VAL_MASK) | TMPSNS_RANGE0_LOW_TEMP_VAL(tempCodeVal);
            TMPSNS_AIWriteAccess((uint32_t) & (base->RANGE0_SET), tempRegVal);
            /* Enable low temperature interrupt */
            TMPSNS_EnableInterrupt(base, kTEMPSENSOR_LowTempInterruptStatusEnable);
            break;

        default:
            assert(false);
            break;
    }
}

/*!
 * brief Enable interrupt status.
 *
 * param base TMPSNS base pointer
 * param mask The interrupts to enable from tmpsns_interrupt_status_enable_t.
 */
void TMPSNS_EnableInterrupt(TMPSNS_Type *base, uint32_t mask)
{
    uint32_t tempRegVal;

    tempRegVal = TMPSNS_AIReadAccess((uint32_t) & (base->CTRL1));
    TMPSNS_AIWriteAccess((uint32_t) & (base->CTRL1), tempRegVal | mask);
}

/*!
 * brief Disable interrupt status.
 *
 * param base TMPSNS base pointer
 * param mask The interrupts to disable from tmpsns_interrupt_status_enable_t.
 */
void TMPSNS_DisableInterrupt(TMPSNS_Type *base, uint32_t mask)
{
    uint32_t tempRegVal;

    tempRegVal = TMPSNS_AIReadAccess((uint32_t) & (base->CTRL1));
    TMPSNS_AIWriteAccess((uint32_t) & (base->CTRL1), tempRegVal & (~mask));
}
