/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-04-28     tyustli      first version
 *
 */

#include <rtthread.h>

#ifdef BSP_USING_PWM

#if !defined(BSP_USING_PWM1) && !defined(BSP_USING_PWM2) && !defined(BSP_USING_PWM3) && !defined(BSP_USING_PWM4)
#error "Please define at least one BSP_USING_PWMx_CHx"
#endif

#define LOG_TAG             "drv.pwm"
#include <drv_log.h>

#include <rtdevice.h>
#include "fsl_pwm.h"
#include "drv_pwm.h"

#define DEFAULT_PRE                   5
#define DEFAULT_DUTY                  50
#define DEFAULT_FRE                   1000
#define PWM_SRC_CLK_FREQ              CLOCK_GetFreq(kCLOCK_IpgClk)
#define DEFAULT_COMPLEMENTARY_PAIR    kPWM_PwmA
#define DEFAULT_POLARITY              kPWM_HighTrue

typedef struct drv_pwm_param
{
	PWM_Type *	base;
	pwm_signal_param_t Pwm_Signal[2];
	pwm_submodule_t pwm_submodule;
	pwm_channels_t channel;
	char name[20];
}drv_pwm_param_t;

drv_pwm_param_t drv_pwms[]=
{
#ifdef BSP_USING_PWM1
	{
		.base = PWM1,
		.pwm_submodule = kPWM_Module_0,
		.channel = kPWM_PwmA,
		.name = "pwm10",
	},
	{
		.base = PWM1,
		.pwm_submodule = kPWM_Module_1,
		.channel = kPWM_PwmA,
		.name = "pwm11",
	},
	{
		.base = PWM1,
		.pwm_submodule = kPWM_Module_2,
		.channel = kPWM_PwmA,
		.name = "pwm12",
	},
	{
		.base = PWM1,
		.pwm_submodule = kPWM_Module_3,
		.channel = kPWM_PwmA,
		.name = "pwm13",
	},
#endif
#ifdef BSP_USING_PWM2	
	{
		.base = PWM2,
		.pwm_submodule = kPWM_Module_0,
		.channel = kPWM_PwmA,
		.name = "pwm20",
	},
	{
		.base = PWM2,
		.pwm_submodule = kPWM_Module_1,
		.channel = kPWM_PwmA,
		.name = "pwm21",
	},
	{
		.base = PWM2,
		.pwm_submodule = kPWM_Module_2,
		.channel = kPWM_PwmA,
		.name = "pwm22",
	},
	{
		.base = PWM2,
		.pwm_submodule = kPWM_Module_3,
		.channel = kPWM_PwmA,
		.name = "pwm23",
	},
#endif	
};

static rt_err_t imxrt_drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg);

static struct rt_pwm_ops imxrt_drv_ops =
{
    .control = imxrt_drv_pwm_control
};

static rt_err_t imxrt_drv_pwm_enable(struct rt_device_pwm *device, struct rt_pwm_configuration *configuration, rt_bool_t enable)
{
    pwm_module_control_t pwm_module_control;

    drv_pwm_param_t *pPwm = (drv_pwm_param_t *)device->parent.user_data;
    pwm_module_control = (pwm_module_control_t)(1 << pPwm->pwm_submodule);

    if (!enable)
    {
        PWM_StopTimer(pPwm->base, pwm_module_control);
    }
    else
    {
        PWM_StartTimer(pPwm->base, pwm_module_control);
    }

    return RT_EOK;
}

static rt_err_t imxrt_drv_pwm_get(struct rt_device_pwm *device, struct rt_pwm_configuration *configuration)
{
    uint8_t get_duty;
    uint16_t pulseCnt = 0, pwmHighPulse = 0;
    uint32_t get_frequence;
    uint32_t pwmClock;

    pwm_submodule_t pwm_submodule;

    drv_pwm_param_t *pPwm = (drv_pwm_param_t *)device->parent.user_data;
    pwm_submodule = (pwm_submodule_t)pPwm->pwm_submodule;

    /* get frequence */
    get_frequence = pPwm->base->SM[pwm_submodule].VAL1;
    pwmClock = (PWM_SRC_CLK_FREQ / (1U << ((pPwm->base->SM[pwm_submodule].CTRL & PWM_CTRL_PRSC_MASK) >> PWM_CTRL_PRSC_SHIFT)));
    get_frequence = pwmClock / get_frequence;

    /* get dutycycle */
    pulseCnt = pPwm->base->SM[pwm_submodule].VAL1;
    pwmHighPulse = pulseCnt - (pPwm->base->SM[pwm_submodule].VAL2) * 2;
    get_duty = pwmHighPulse * 100 / pulseCnt;

    /* conversion */
    configuration->period = 1000000000 / get_frequence;
    configuration->pulse = get_duty * configuration->period / 100;

    return RT_EOK;
}

static rt_err_t imxrt_drv_pwm_set(struct rt_device_pwm *device, struct rt_pwm_configuration *configuration)
{
    RT_ASSERT(configuration->period > 0);
    RT_ASSERT(configuration->pulse <= configuration->period);

    pwm_submodule_t pwm_submodule;
    pwm_module_control_t pwm_module_control;
    uint32_t period = 0;
    uint8_t duty = 0;

    drv_pwm_param_t *pPwm = (drv_pwm_param_t *)device->parent.user_data;
    pwm_submodule = pPwm->pwm_submodule;
    pwm_module_control = (pwm_module_control_t)(1 << pwm_submodule);
	pPwm->Pwm_Signal[configuration->channel].pwmChannel = configuration->channel;
    duty = configuration->pulse * 100 / configuration->period;
    pPwm->Pwm_Signal[configuration->channel].dutyCyclePercent = duty;
    period = (uint32_t)(1000000000 / configuration->period);

    PWM_SetupPwm(pPwm->base, pwm_submodule, pPwm->Pwm_Signal, 2, kPWM_CenterAligned, period, PWM_SRC_CLK_FREQ);
    PWM_UpdatePwmDutycycle(pPwm->base, pwm_submodule, pPwm->Pwm_Signal[configuration->channel].pwmChannel, kPWM_CenterAligned, duty);
    PWM_SetPwmLdok(pPwm->base, pwm_module_control, true);

    return RT_EOK;
}

static rt_err_t imxrt_drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;
	drv_pwm_param_t *pPwm = (drv_pwm_param_t *)device->parent.user_data;
	
    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        return imxrt_drv_pwm_enable(device, configuration, RT_TRUE);
    case PWM_CMD_DISABLE:
		pPwm->Pwm_Signal[configuration->channel].dutyCyclePercent = 0;
        return imxrt_drv_pwm_enable(device, configuration, RT_FALSE);
    case PWM_CMD_SET:
        return imxrt_drv_pwm_set(device, configuration);
    case PWM_CMD_GET:
        return imxrt_drv_pwm_get(device, configuration);
    default:
        return RT_EINVAL;
    }
}

static rt_err_t imxrt_drv_pwm_init(PWM_Type *base, pwm_signal_param_t *Pwm_Signal, pwm_submodule_t pwm_submodule,pwm_channels_t chl, uint16_t psc, uint32_t fre, uint8_t duty)
{
    pwm_config_t PwmConfig;
    uint8_t fault_input;
    pwm_clock_prescale_t pwm_prescale = (pwm_clock_prescale_t)psc;
    fault_input = (uint8_t)pwm_submodule;
    PWM_GetDefaultConfig(&PwmConfig);

    PwmConfig.prescale = pwm_prescale;
    PwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
    PwmConfig.pairOperation = kPWM_Independent;
	
    PwmConfig.enableDebugMode = true;
	
    if (PWM_Init(base, pwm_submodule, &PwmConfig) == kStatus_Fail)
    {
        LOG_E("init pwm failed \n");
        return RT_ERROR;
    }

    base->SM[fault_input].DISMAP[0] = 0x00;
    base->SM[fault_input].DISMAP[1] = 0x00;

    Pwm_Signal->pwmChannel = chl;
    Pwm_Signal->level = DEFAULT_POLARITY;
    Pwm_Signal->dutyCyclePercent = duty;

    PWM_SetupPwm(base, pwm_submodule, Pwm_Signal, 2, kPWM_CenterAligned, fre, PWM_SRC_CLK_FREQ);
    PWM_SetPwmLdok(base, pwm_submodule<<1, true);

    return RT_EOK;
}

static rt_err_t imxrt_pwm1_init(drv_pwm_param_t *p_drv_pwm)
{

    if (imxrt_drv_pwm_init(p_drv_pwm->base,p_drv_pwm->Pwm_Signal, p_drv_pwm->pwm_submodule,p_drv_pwm->channel, DEFAULT_PRE, DEFAULT_FRE, DEFAULT_DUTY) != RT_EOK)
    {
        return RT_ERROR;
    }

	struct rt_device_pwm *pwm_device = (struct rt_device_pwm *)rt_malloc(sizeof(struct rt_device_pwm));
	
	if (pwm_device == NULL)
	{
		return RT_ERROR;
	}
	
	if(rt_device_pwm_register(pwm_device, p_drv_pwm->name, &imxrt_drv_ops, p_drv_pwm->base) != RT_EOK)
	{
		return RT_ERROR;
	}
	pwm_device->parent.user_data = p_drv_pwm;
	
    return RT_EOK;
}


int rt_hw_pwm_init(void)
{
    rt_err_t ret = RT_EOK;

	for (int i=0;i < sizeof(drv_pwms)/sizeof(struct drv_pwm_param);i++)
	{
		imxrt_pwm1_init(&drv_pwms[i]);
	}

    return ret;
}

INIT_DEVICE_EXPORT(rt_hw_pwm_init);

#endif /* BSP_USING_PWM */
