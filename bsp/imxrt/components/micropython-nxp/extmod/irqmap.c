// 枚举各外设的各内部中断源,表示为Irq slot
// 枚举外设的各种使用IRQ的功能，表示为Irq func
// 使用弱函数和支持TBB优化的switch来降低中断延迟
#include <stdio.h>

#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "py/nlr.h"
#include "py/gc.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "fsl_iomuxc.h"
#include "fsl_common.h"
#include "fsl_pwm.h"
#include "fsl_xbara.h"
#include "modservo.h"
#include "lib/utils/interrupt_char.h"
//#include "pendsv.h"
#include "fsl_qtmr.h"
#include "irqmap.h"

#define IRQMAP_MAX_SLOT		64
#define IRQFUNC_INVALID 0
//STATIC mp_obj_t pyb_timer_callback(mp_obj_t self_in, mp_obj_t callback);
//STATIC void timer_handle_irq_channel(pyb_qtimer_obj_t* tim, mp_obj_t callback);
//pyb_qtimer_obj_t pyb_qtimer_obj[16] =
//{
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR1, .ticks = 0, .IRQn = TMR1_IRQn, .idex = kQTMR_Channel_0, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR1, .ticks = 0, .IRQn = TMR1_IRQn, .idex = kQTMR_Channel_1, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR1, .ticks = 0, .IRQn = TMR1_IRQn, .idex = kQTMR_Channel_2, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR1, .ticks = 0, .IRQn = TMR1_IRQn, .idex = kQTMR_Channel_3, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR2, .ticks = 0, .IRQn = TMR2_IRQn, .idex = kQTMR_Channel_0, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR2, .ticks = 0, .IRQn = TMR2_IRQn, .idex = kQTMR_Channel_1, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR2, .ticks = 0, .IRQn = TMR2_IRQn, .idex = kQTMR_Channel_2, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR2, .ticks = 0, .IRQn = TMR2_IRQn, .idex = kQTMR_Channel_3, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR3, .ticks = 0, .IRQn = TMR3_IRQn, .idex = kQTMR_Channel_0, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR3, .ticks = 0, .IRQn = TMR3_IRQn, .idex = kQTMR_Channel_1, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR3, .ticks = 0, .IRQn = TMR3_IRQn, .idex = kQTMR_Channel_2, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR3, .ticks = 0, .IRQn = TMR3_IRQn, .idex = kQTMR_Channel_3, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR4, .ticks = 0, .IRQn = TMR4_IRQn, .idex = kQTMR_Channel_0, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR4, .ticks = 0, .IRQn = TMR4_IRQn, .idex = kQTMR_Channel_1, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR4, .ticks = 0, .IRQn = TMR4_IRQn, .idex = kQTMR_Channel_2, .callback = mp_const_none, .isActive = false},
//    {.base = {&pyb_qtimer_type}, .tmr_base = TMR4, .ticks = 0, .IRQn = TMR4_IRQn, .idex = kQTMR_Channel_3, .callback = mp_const_none, .isActive = false},
//};

extern void servo_timer_irq_callback();
IrqFuncs_e s_irqMap[IRQMAP_MAX_SLOT];

/* >>> APIs */
IrqFuncs_e IRQMAP_Get(IrqSlots_e slot)
{
    IrqFuncs_e oldFunc = s_irqMap[slot];
    return oldFunc;
}

IrqFuncs_e IRQMAP_Set(IrqSlots_e slot, IrqFuncs_e newFunc)
{
    IrqFuncs_e oldFunc = s_irqMap[slot];
    s_irqMap[slot] = newFunc;
    return oldFunc;
}

IrqFuncs_e IRQMAP_Clr(IrqSlots_e slot, IrqFuncs_e current)
{
    IrqFuncs_e oldFunc = s_irqMap[slot];
    if(oldFunc == current)
    {
        s_irqMap[slot] = IRQFUNC_INVALID;
    }
    else
    {

    }
    return oldFunc;
}


/* -------------------------------------------------------*/
/* 提供弱实现。若相关驱动未编译，则会调用弱实现 */
/* 需要和各驱动的作者align好，使用约定的handler名称*/
/* 相比使用函数指针，开销更小. switch可被优化为TBB指令*/

__WEAK void QTmr_DefaultHandler(unsigned ndx, int submod)
{
//   int id=(ndx-1)*4+submod;
//    pyb_qtimer_obj_t* temp = &pyb_qtimer_obj[id];
//    uint16_t reg = temp->tmr_base->CHANNEL[temp->idex].SCTRL;
//    //check if the timer channel is active, and if the interrupt in this channel is occur
//    if((temp->isActive) && ((reg & TMR_SCTRL_TCF_MASK) >> TMR_SCTRL_TCF_SHIFT))
//    {
//        QTMR_ClearStatusFlags(temp->tmr_base, temp->idex, kQTMR_CompareFlag);
//        timer_handle_irq_channel(temp, temp->callback);
//        temp->ticks++;
//    }
}

__WEAK void QTmr_TmrHandler(unsigned ndx, int submod)
{
    QTmr_DefaultHandler(ndx, submod);
}
__WEAK void QTmr_ServoHandler()
{
	#ifdef BSP_USING_SERVO
    servo_timer_irq_callback();
	#endif
}
__WEAK void QTmr_PWMHandler(unsigned ndx, int submod)
{
    QTmr_DefaultHandler(ndx, submod);
}
__WEAK void QTmr_DCMotorHandler(unsigned ndx, int submod)
{
    QTmr_DefaultHandler(ndx, submod);
}
__WEAK void QTmr_QEncHandler(unsigned ndx, int submod)
{
    QTmr_DefaultHandler(ndx, submod);
}
__WEAK void QTmr_RPMHandler(unsigned ndx, int submod)
{
    QTmr_DefaultHandler(ndx, submod);
}
__WEAK void QTmr_SRPMHandler(unsigned ndx, int submod)
{
    QTmr_DefaultHandler(ndx, submod);
}
static int index[4]= {0};
uint32_t QTmrGetSubModule(TMR_Type* base)
{
    for(int i = 0; i < 4; i++)
    {
		#ifdef BSP_USING_SERVO
        if(QTMR_GetEnabledInterrupts(base, i)&QTMR_GetStatus(base, i))
            index[i] = i + 1;
		#endif
    }
	return 0;
}

void QTmrCommonIrqHandler(unsigned ndx)
{
    switch(ndx)
    {
    case 1:
        QTmrGetSubModule(TMR1);
        break;
    case 2:
        QTmrGetSubModule(TMR2);
        break;
#if defined(SOC_IMXRT1060_SERIES)	||	defined(SOC_MIMXRT1050_SERIES)		
    case 3:
        QTmrGetSubModule(TMR3);
        break;
    case 4:
        QTmrGetSubModule(TMR4);
        break;
#endif	
    }
    //QTmr_ServoHandler(ndx, 0);
    // for each SM do:
    for(int i = 0; i < 4; i++)
    {
        if(index[i] != 0)
        {
            unsigned slot = kIrqSlot_QT1_0 + (ndx - 1) * 4 + i;
            IrqFuncs_e func = IRQMAP_Get(slot);

            switch(func)
            {
            case kIrqFunc_QT_Tmr:
                // 调用高精度定时器的中断处理函数
                QTmr_TmrHandler(ndx, i);
                break;
            case kIrqFunc_QT_Servo:
                // 调用舵机中断处理函数
                QTmr_ServoHandler();
                break;
            case kIrqFunc_QT_PWM:
                // 调用通用PWM中断处理函数
                QTmr_PWMHandler(ndx, i);
                break;
            case kIrqFunc_QT_DCMotor:
                // 调用直流电机中断处理函数
                QTmr_DCMotorHandler(ndx, i);
                break;
            case kIrqFunc_QT_QEnc:
                // 调用正交编码器中断处理函数
                QTmr_QEncHandler(ndx, i);
                break;
            case kIrqFunc_QT_RPM:
                // 调用测速中断处理函数
                QTmr_RPMHandler(ndx, i);
                break;
            case kIrqFunc_QT_SRPM:
                // 调用有向测速中断处理函数
                QTmr_SRPMHandler(ndx, i);
                break;
defaut:
                QTmr_DefaultHandler(ndx, i);
                break;
            }
        }
    }
}

void TMR1_IRQHandler(void)
{
    QTmrCommonIrqHandler(1);
}
void TMR2_IRQHandler(void)
{
    QTmrCommonIrqHandler(2);
}
void TMR3_IRQHandler(void)
{
    QTmrCommonIrqHandler(3);
}
void TMR4_IRQHandler(void)
{
    QTmrCommonIrqHandler(4);
}

__WEAK void FlexPWM_DefaultHandler(void)
{
    while(1) {}
}
__WEAK void FlexPWM_PWMHandler(void)
{
    FlexPWM_DefaultHandler();
}
__WEAK void FlexPWM_BLDCHandler(void)
{
    FlexPWM_DefaultHandler();
}
__WEAK void FlexPWM_SRPMHandler(void)
{
    FlexPWM_DefaultHandler();
}

void FlexPWMCommonIrqHandler(unsigned ndx, unsigned subModNdx)
{
    unsigned slot = kIrqSlot_FP1_1 + (ndx - 1) * 4 + subModNdx;
    IrqFuncs_e func = IRQMAP_Get(slot);

    switch(func)
    {
    case kIrqFunc_FP_PWM:
        FlexPWM_PWMHandler();
        break;
    case kIrqFunc_FP_BLDC:
        FlexPWM_BLDCHandler();
        break;
	case kIrqFunc_QT_DCMotor:
	case kIrqFunc_QT_QEnc:
	case kIrqFunc_QT_RPM:
	case kIrqFunc_QT_SRPM:
	case kIrqFunc_QT_PWM:
	defaut:
        FlexPWM_DefaultHandler();
        break;
    }
}

void PWM1_0_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(1, 0);
}
void PWM1_1_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(1, 1);
}
void PWM1_2_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(1, 2);
}
void PWM1_3_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(1, 3);
}

void PWM2_0_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(2, 0);
}
void PWM2_1_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(2, 1);
}
void PWM2_2_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(2, 2);
}
void PWM2_3_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(2, 3);
}

void PWM3_0_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(3, 0);
}
void PWM3_1_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(3, 1);
}
void PWM3_2_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(3, 2);
}
void PWM3_3_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(3, 3);
}

void PWM4_0_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(4, 0);
}
void PWM4_1_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(4, 1);
}
void PWM4_2_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(4, 2);
}
void PWM4_3_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(4, 3);
}
void PWM1_FAULT_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(5, 0);
}
void PWM2_FAULT_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(5, 1);
}
void PWM3_FAULT_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(5, 2);
}
void PWM4_FAULT_IRQHandler(void)
{
    FlexPWMCommonIrqHandler(5, 3);
}


// ... in cmm_cfg.csv
// servo, 3, -, q4.2, <引脚名>, 3号舵机控制通道使用QTimer 4，子模块2

// // ... In servo.c, make_new() function.
// ...
// CMM_Query("servo", channel, "-", &mux);
// // get slot info from hint message qm.n
// unsigned slot = 4 * (mux.szHint[1] - '1') + mux.szHint[3] - '0' ;
// unsigned subModNdx = mux.szHint[3] - '0';
// old = IRQMAP_Set((IRQSlot_e)slot, kIrqFunc_QT_Servo);
// if (old != 255) raise("confliction!");
// ...
