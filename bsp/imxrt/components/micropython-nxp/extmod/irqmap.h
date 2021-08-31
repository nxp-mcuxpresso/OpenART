#include <stdio.h>

#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "py/obj.h"

#include "pin_defs_mcu.h"
typedef enum _IrqSlots_e
{
    kIrqSlot_QT1_0 = 0,
    kIrqSlot_QT1_1,	// QTimer 1, sub module 1
    kIrqSlot_QT1_2,
    kIrqSlot_QT1_3,

    kIrqSlot_QT2_0,
    kIrqSlot_QT2_1,	// QTimer 2, sub module 1
    kIrqSlot_QT2_2,
    kIrqSlot_QT2_3,


    kIrqSlot_QT3_0,
    kIrqSlot_QT3_1,	// QTimer 3, sub module 1
    kIrqSlot_QT3_2,
    kIrqSlot_QT3_3,

    kIrqSlot_QT4_0,
    kIrqSlot_QT4_1,	// QTimer 4, sub module 1
    kIrqSlot_QT4_2,
    kIrqSlot_QT4_3,


    kIrqSlot_FP1_0,
    kIrqSlot_FP1_1,	// FlexPWM1, sub module 1
    kIrqSlot_FP1_2,
    kIrqSlot_FP1_3,

    kIrqSlot_FP2_0,
    kIrqSlot_FP2_1,	// FlexPWM2, sub module 1
    kIrqSlot_FP2_2,
    kIrqSlot_FP2_3,

    kIrqSlot_FP3_0,
    kIrqSlot_FP3_1,	// FlexPWM3, sub module 1
    kIrqSlot_FP3_2,
    kIrqSlot_FP3_3,

    kIrqSlot_FP4_0,
    kIrqSlot_FP4_1,	// FlexPWM3, sub module 1
    kIrqSlot_FP4_2,
    kIrqSlot_FP4_3,


    kIrqSlot_FP1_F,	// FlexPWM1, fault inpu
    kIrqSlot_FP2_F,
    kIrqSlot_FP3_F,
    kIrSlot_FP4_F,
    // can also use this on other peripherals which
    // can have multiple sub modules
} IrqSlots_e;

typedef enum _IrqFuncs_e
{
    // start from 0 so compiler can optimize switch to "TBB" instruction
    kIrqFunc_QT_Tmr = 0,
    kIrqFunc_QT_Servo,
    kIrqFunc_QT_PWM,
    kIrqFunc_QT_DCMotor,
    kIrqFunc_QT_QEnc,
    kIrqFunc_QT_RPM,
    kIrqFunc_QT_SRPM,
    // start from 0 so compiler can optimize switch to "TBB" instruction
    kIrqFunc_FP_PWM = 0,
    kIrqFunc_FP_BLDC,

    // can also use this on other peripherals which/
    // can be used in different functions
    kIrqFunc_ADC_Single = 0,	// 单ADC单次采样
    kIrqFunc_ADC_Sequence,	// 单ADC序列采样
    kIrqFunc_ADC_SyncedPair,// 双ADC同步采样
} IrqFuncs_e;

//typedef struct _qtimer_obj_t{
//	mp_obj_base_t base;
//	TMR_Type* tmr_base;
//	IRQn_Type IRQn;
//	qtmr_channel_selection_t idex;
//	__IO uint32_t ticks;
//	float period;
//	uint16_t prescale;
//	bool isActive;
//	mp_obj_t callback;
//} pyb_qtimer_obj_t;
extern const mp_obj_type_t pyb_qtimer_type;
IrqFuncs_e IRQMAP_Set(IrqSlots_e slot, IrqFuncs_e newFunc);
IrqFuncs_e IRQMAP_Clr(IrqSlots_e slot, IrqFuncs_e current);
IrqFuncs_e IRQMAP_Get(IrqSlots_e slot);