#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include <stdint.h>
#include "stm32f407xx.h"


/*
 *
 * Config struct for GPIOx pin
 *
 */

typedef struct{
	uint8_t GPIO_PinNum; //Pin number, possible configurations @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode; //Pin mode, possible configurations @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed; //Pin speed, possible configurations @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdCntl; //Pull-up/pull-down control, possible configurations @GPIO_PUPD
	uint8_t GPIO_PinOType; //Output type, possible configurations @GPIO_PIN_OUT_TYPES
	uint8_t GPIO_PinAFMode; //Alternate function mode
}GPIO_PinCfg_t;

/*
 *
 * Handle struct for GPIOx pin
 *
 */

typedef struct{
	GPIO_Reg_t *pGPIOx; //Base address of the GPIO port where the pin used belongs
	GPIO_PinCfg_t GPIO_PinCfg; //GPIOx pin configuration
}GPIO_Handle_t;


/*@GPIO_PIN_NUMBERS
 *
 * GPIO pin number macros
 *
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*@GPIO_PIN_MODES
 *
 * GPIO input mode macros
 *
 */
#define GPIO_MODE_IN 		0 //Input mode (reset)
#define GPIO_MODE_OUT 		1 //Output mode
#define GPIO_MODE_AF 		2 //Alternate function mode
#define GPIO_MODE_AN 		3 //Analog mode
#define GPIO_MODE_IT_FT		4 //Interrupt falling edge
#define GPIO_MODE_IT_RT		5 //Interrput rising edge
#define GPIO_MODE_IT_RFT	6 //Interrupt both edges

/*@GPIO_PIN_OUT_TYPES
 *
 * GPIO output type macros
 *
 */
#define GPIO_OUT_TYPE_PP	0 //Push-pull (reset)
#define GPIO_OUT_TYPE_OD 	1 //Open drain

/*@GPIO_PIN_SPEED
 *
 * GPIO output speed macros
 *
 */
#define GPIO_OUT_SPD_LOW	0 //Low speed
#define GPIO_OUT_SPD_MED	1 //Medium speed
#define GPIO_OUT_SPD_HIGH	2 //High speed
#define GPIO_OUT_SPD_VHIGH 	3 //Very high speed

/*@GPIO_PUPD
 *
 * GPIO pull-up/down macros
 *
 */
#define GPIO_PUPD_DI		0 //No pull-up/pull-down
#define GPIO_PUPD_PU		1 //Pull-up
#define GPIO_PUPD_PD		2 //Pull-down

/*
 *
 * Driver API prototypes
 *
 */

//Init, de-init
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_Reg_t* pGPIOx);

//Pclk control
void GPIO_PClkCntl(GPIO_Reg_t* pGPIOx, uint8_t state);

//Read & write
uint8_t GPIO_ReadPinIn(GPIO_Reg_t* pGPIOx, uint8_t pin);
uint16_t GPIO_ReadPortIn(GPIO_Reg_t* pGPIOx);
void GPIO_WriteOutPin(GPIO_Reg_t* pGPIOx, uint8_t pin, uint8_t data);
void GPIO_WriteOutPort(GPIO_Reg_t* pGPIOx, uint16_t data);
void GPIO_ToggleOutPin(GPIO_Reg_t* pGPIOx, uint8_t pin);

//IRQ
void GPIO_IRQCfg(uint8_t IRQ_num, uint8_t IRQ_priority, uint8_t state);
void GPIO_IRQHandle(uint8_t pin);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
