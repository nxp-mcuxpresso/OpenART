#include "fsl_gpio.h"

#ifdef GPIO_DEBUGING 
#define GPIO(n) GPIO ##n
// ch0
#define CSI_FRAME_RATE   GPIO(3), 7 
// ch1
#define GET_FRAME_RATE	 GPIO(3), 8
// ch2
#define REFRESH_FRAME_RATE   GPIO(3), 12
// ch3
#define CLS_FRAME_RATE    GPIO(3), 11
// ch4
#define TF_FRAME_RATE     GPIO(3), 9
// ch5
#define IMG_PROCESS_RATE  GPIO(5), 12
// ch6 
#define NNCU_RATE       GPIO(5), 11
// ch7
#define NNCU_INVOKE_RATE  GPIO(5), 13

#define TOOL(x) x

#define INIT() \
	INIT_GPIO(CSI_FRAME_RATE);  \
	INIT_GPIO(GET_FRAME_RATE); \
	INIT_GPIO(REFRESH_FRAME_RATE); \
	INIT_GPIO(CLS_FRAME_RATE); \
	INIT_GPIO(TF_FRAME_RATE); \
	INIT_GPIO(IMG_PROCESS_RATE); \
	INIT_GPIO(NNCU_RATE);  \
	INIT_GPIO(NNCU_INVOKE_RATE); 
	
static inline void INIT_GPIO(GPIO_Type* n, int pin) 
{ 
	gpio_pin_config_t config = {
		.direction = kGPIO_DigitalOutput,
		.outputLogic = 0,
	};
	GPIO_PinInit(n, pin, &config); \
}
	
static inline void TOGGLE(GPIO_Type*n, int pin)  {n->DR_TOGGLE |= 1 << pin;}
static inline void HIGH(GPIO_Type*n, int pin) {n->DR_SET |= 1 << pin; }
static inline void LOW(GPIO_Type*n, int pin) {n->DR_CLEAR |= 1 << pin; }
#endif