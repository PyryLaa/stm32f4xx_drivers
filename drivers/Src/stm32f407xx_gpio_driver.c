#include <stdint.h>
#include "stm32f407xx_gpio_driver.h"


/*
 *
 * Init, de-init
 *
 */

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the GPIOx pin for use
 *
 * @param[in]         - Address of the handle for the GPIOx pin
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle){
	uint32_t temp = 0;

	//Pin mode
	if(pGPIOHandle -> GPIO_PinCfg.GPIO_PinMode <= GPIO_MODE_AN){
		temp = (pGPIOHandle -> GPIO_PinCfg.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum)); //Multiply with 2 because 2 bits per pin
		pGPIOHandle -> pGPIOx -> MODER &= ~(3 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum);
		pGPIOHandle -> pGPIOx -> MODER |= temp;
	}else{
		if(pGPIOHandle -> GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_IT_FT){ //Interrupt on falling edge
			EXTI -> FTSR |= (1 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum); //Enable the falling edge trigger interrupt for the given line
			EXTI -> RTSR &= ~(1 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum); //Make sure the rising edge trigger is disabled

		}else if(pGPIOHandle -> GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_IT_RT){ //Interrupt on rising edge
			EXTI -> RTSR |= (1 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum); //Enable the rising edge trigger interrupt for the given pin
			EXTI -> FTSR &= ~(1 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum); //Make sure the falling edge trigger is disabled

		}else if(pGPIOHandle -> GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_IT_RFT){ //Interrupt on both edges
			EXTI -> RTSR |= (1 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum); //Enable the rising edge trigger interrupt for the given line
			EXTI -> FTSR |= (1 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum); //Enable the falling edge trigger interrupt for the given line
		}
		//Choose the right EXTI line in SYSCFG
		uint8_t temp1 = pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum / 4; //Used to choose the right EXTICR register from the array
		uint8_t temp2 = pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum % 4; //Used to choose the right bitfield from the chosen EXTICR
		uint8_t port = GPIO_BASE_TO_PORT(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG -> EXTICR[temp1] = port << (temp2 * 4); //One bitfield is 4 bits in EXTICR
		//Enable the exti interrupt delivery using IMR register
		EXTI -> IMR |= (1 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum);
	}

	//Pin speed
	temp = 0;
	temp = (pGPIOHandle -> GPIO_PinCfg.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum));
	pGPIOHandle -> pGPIOx-> OSPEEDR &= ~(3 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum);
	pGPIOHandle -> pGPIOx-> OSPEEDR |= temp;

	//Pupd configuration
	temp = 0;
	temp = (pGPIOHandle -> GPIO_PinCfg.GPIO_PinPuPdCntl << (2 * pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum));
	pGPIOHandle -> pGPIOx-> PUPDR &= ~(3 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum);
	pGPIOHandle -> pGPIOx-> PUPDR |= temp;

	//Output type
	temp = 0;
	temp = (pGPIOHandle -> GPIO_PinCfg.GPIO_PinOType << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum);
	pGPIOHandle -> pGPIOx-> OTYPER &= ~(1 << pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum);
	pGPIOHandle -> pGPIOx-> OTYPER |= temp;

	//Alternate functionality
	temp = 0;
	if(pGPIOHandle -> GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_AF){ //Only if the pin mode is alternate function
		uint8_t afr = pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum / 8; //Variable to choose afr high or afr low
		uint8_t start = pGPIOHandle -> GPIO_PinCfg.GPIO_PinNum % 8; //Variable to hold the starting bit to configure the pin

		pGPIOHandle -> pGPIOx -> AFR[afr] &= ~(0xF << (4 * start));
		pGPIOHandle -> pGPIOx -> AFR[afr] |= (pGPIOHandle -> GPIO_PinCfg.GPIO_PinAFMode << (4 * start)); //Multiply with 4 because 4 bits per pin
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initializes the GPIOx pin
 *
 * @param[in]         - Handle for the GPIOx pin
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_DeInit(GPIO_Reg_t* pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}



/*
 *
 * Pclk control
 *
 */

/*********************************************************************
 * @fn      		  - GPIO_PClkCntl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PClkCntl(GPIO_Reg_t* pGPIOx, uint8_t state){
	if(state == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}else if(state == DISABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}



/*
 *
 * Read & write
 *
 */

/*********************************************************************
 * @fn      		  - GPIO_ReadPinIn
 *
 * @brief             - This function reads input data from the specified GPIO pin
 *
 * @param[in]         - Base address of the GPIO port
 * @param[in]         - Pin number to read from
 * @param[in]         -
 *
 * @return            -  0 or 1
 *
 * @Note              -  none

 */
uint8_t GPIO_ReadPinIn(GPIO_Reg_t* pGPIOx, uint8_t pin){
	uint8_t data = 0;
	data = (uint8_t)((pGPIOx -> IDR >> pin) & 0x00000001);
	return data;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadPortIn
 *
 * @brief             - This function reads data from the whole specified GPIO port
 *
 * @param[in]         - Base address of the GPIO port to read from
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  16 bits, corresponding to the pins in a port
 *
 * @Note              -  none

 */
uint16_t GPIO_ReadPortIn(GPIO_Reg_t* pGPIOx){
	uint16_t data = 0;
	data = (uint16_t)(pGPIOx -> IDR);
	return data;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteOutPin
 *
 * @brief             - This function writes data to the specified output pin
 *
 * @param[in]         - Base address of the GPIO port
 * @param[in]         -	Pin number to write to
 * @param[in]         -	Data to write
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteOutPin(GPIO_Reg_t* pGPIOx, uint8_t pin, uint8_t data){
	if(data == GPIO_PIN_SET){
		pGPIOx -> ODR |= (1 << pin);
	}else if(data == GPIO_PIN_RESET){
		pGPIOx -> ODR &= ~(1 << pin);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteOutPort
 *
 * @brief             - This function writes data to the specified GPIO port
 *
 * @param[in]         - Base address of the GPIO port
 * @param[in]         -	Data to write
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_WriteOutPort(GPIO_Reg_t* pGPIOx, uint16_t data){
	pGPIOx -> ODR = data;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutPin
 *
 * @brief             - This function toggles the state of the specified output pin
 *
 * @param[in]         - Base address of the GPIO port
 * @param[in]         -	Pin to toggle
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_ToggleOutPin(GPIO_Reg_t* pGPIOx, uint8_t pin){
	pGPIOx -> ODR ^= (1 << pin);
}



/*
 *
 * IRQ
 *
 */

/*********************************************************************
 * @fn      		  - GPIO_IRQCfg
 *
 * @brief             - This function enables or disables the given interrupt in the M4 NVIC registers
 *
 * @param[in]         - IRQ number
 * @param[in]         -	ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQCfg(uint8_t IRQ_num, uint8_t state){
	if(state == ENABLE){
		if(IRQ_num <= 31){
			*NVIC_ISER0 = (1 << IRQ_num); //Enable the interrupt in the NVIC

		}else if(IRQ_num >= 32 && IRQ_num < 64){
			*NVIC_ISER1 = (1 << IRQ_num % 32);

		}
		else if(IRQ_num >= 64 && IRQ_num < 96){
			*NVIC_ISER2 = (1 << IRQ_num % 64);
		}
	}else if(state == ENABLE){
		if(IRQ_num <= 31){
			*NVIC_ICER0 = (1 << IRQ_num); //Disable the interrupt in the NVIC

		}else if(IRQ_num >= 32 && IRQ_num < 64){
			*NVIC_ICER1 = (1 << IRQ_num % 32);

		}
		else if(IRQ_num >= 64 && IRQ_num < 96){
			*NVIC_ICER2 = (1 << IRQ_num % 64);
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQ_PriorityCfg
 *
 * @brief             - This function configures the priority of the given interrupt in the M4 NVIC registers
 *
 * @param[in]         - IRQ_priority
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQ_PriorityCfg(uint8_t IRQ_num, uint8_t IRQ_priority){
	uint8_t int_reg = IRQ_num / 4; //Number of the IPR register to be used
	uint8_t int_field = IRQ_num % 4; //Starting position of the bitfield used in the specified IPR register
	uint8_t shift_amount = (8 * int_field) + (8 - PR_BITS_IMPLEMENTED);

	*(NVIC_IPR0 + int_reg) |= (IRQ_priority << (shift_amount));
}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandle
 *
 * @brief             - This function handles the IRQ for the given pin
 *
 * @param[in]         - Pin to handle
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_IRQHandle(uint8_t pin){

}
