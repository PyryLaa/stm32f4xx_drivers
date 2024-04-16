#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/*******************************START: Processor specific definitions*****************************************************/
/*
 * ARM Cortex M4 CPU NVIC ISERx register addresses
 */

//For STM32F407 only 81 interrupts are maskable, so ISER0-ISER3 are the only ones needed
#define NVIC_ISER0				((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*) 0xE000E108)


/*
 * ARM Cortex M4 CPU NVIC ICERx register addresses
 */
#define NVIC_ICER0				((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1				((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2				((volatile uint32_t*) 0xE000E188)

/*
 * ARM Cortex M4 CPU NVIC IPRx register addresses
 */
#define NVIC_IPR0				((volatile uint32_t*) 0xE000E400)

/*******************************END: Processor specific definitions*****************************************************/

/*******************************START: MCU specific definitions*****************************************************/

//
#define PR_BITS_IMPLEMENTED		4 					//Priority bits implemented in the STM32F407

//Memory base addresses
#define FLASH_BASEADDR 			0x08000000U 		//Base address for the flash memory
#define SRAM1_BASEADDR			0x20000000U 		//Base address for the sram1 memory, 112KB
#define SRMA2_BASEADDR			0x2001C000U 		//Base address for the sram2 memory, comes straight after sram1
#define ROM_BASEADDR			0x1FFF0000U 		//Base address for the system memory
#define SRAM 					SRAM1_BASEADDR

//Bus domains' base addresses
#define PERIPH_BASEADDR			0x40000000U 		//Peripheral bus base address (same as APB1)
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR 	//Base address for the APB1 bus peripherals
#define APB2PERIPH_BASEADDR		0x40010000U 		//Base address for the APB2 bus peripherals
#define AHB1PERIPH_BASEADDR		0x40020000U 		//Base address for the AHB1 bus peripherals
#define AHB2PERIPH_BASEADDR		0x50000000U 		//base address for the AHB2 bus peripherals


/*
 * Peripheral bus base addresses
 */

//AHB1 bus, for every peripheral, address is AHB1PERIPH_BASEADDR + offset
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

//APB1 bus, for every peripheral, address is APB1PERIPH_BASEADDR + offset
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define UART7_BASEADDR			(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR			(APB1PERIPH_BASEADDR + 0x7C00)

//APB2 bus, for every peripheral, address is APB2PERIPH_BASEADDR + offset
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI5_BASEADDR			(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR			(APB2PERIPH_BASEADDR + 0x5400)


/*
 * Peripheral register structs
 */

//GPIOx
typedef struct{
	volatile uint32_t MODER;		//Offset 0x00
	volatile uint32_t OTYPER;		//Offset 0x04
	volatile uint32_t OSPEEDR;		//Offset 0x08
	volatile uint32_t PUPDR;		//Offset 0x0C
	volatile uint32_t IDR;			//Offset 0x10
	volatile uint32_t ODR;			//Offset 0x14
	volatile uint32_t BSRR;			//Offset 0x18
	volatile uint32_t LCKR;			//Offset 0x1C
	volatile uint32_t AFR[2];		//Offset 0x20 & 0x24
}GPIO_Reg_t;

//RCC
typedef struct{
	volatile uint32_t CR;			//Offset 0x00
	volatile uint32_t PLLCFGR;		//Offset 0x04
	volatile uint32_t CFGR;			//Offset 0x08
	volatile uint32_t CIR;			//Offset 0x0C
	volatile uint32_t AHB1RSTR;		//Offset 0x10
	volatile uint32_t AHB2RSTR;		//Offset 0x14
	volatile uint32_t AHB3RSTR;		//Offset 0x18
			 uint32_t RESERVED0;	//Offset 0x1C
	volatile uint32_t APB1RSTR;		//Offset 0x20
	volatile uint32_t APB2RSTR;		//Offset 0x24
	 	 	 uint32_t RESERVED1[2];	//Offset 0x28 & 0x2C
    volatile uint32_t AHB1ENR;		//Offset 0x30
    volatile uint32_t AHB2ENR;		//Offset 0x34
    volatile uint32_t AHB3ENR;		//Offset 0x38
    		 uint32_t RESERVED2;	//Offset 0x3C
    volatile uint32_t APB1ENR;		//Offset 0x40
    volatile uint32_t APB2ENR;		//Offset 0x44
    		 uint32_t RESERVED3[2];	//Offset 0x48 & 0x4C
    volatile uint32_t AHB1LPENR;	//Offset 0x50
    volatile uint32_t AHB2LPENR;	//Offset 0x54
    volatile uint32_t AHB3LPENR;	//Offset 0x58
    		 uint32_t RESERVED4;	//Offset 0x5C
    volatile uint32_t APB1LPENR;	//Offset 0x60
    volatile uint32_t APB2LPENR;	//Offset 0x64
    		 uint32_t RESERVED5[2];	//Offset 0x68 & 0x6C
    volatile uint32_t BDCR;			//Offset 0x70
    volatile uint32_t CSR;			//Offset 0x74
    		 uint32_t RESERVED6[2];	//Offset 0x78 & 0x7C
    volatile uint32_t SSCGR;		//Offset 0x80
    volatile uint32_t PLLI2SCFGR;	//Offset 0x84
}RCC_Reg_t;

//EXTI
typedef struct{
	volatile uint32_t IMR;			//Offset 0x00
	volatile uint32_t EMR;			//Offset 0x04
	volatile uint32_t RTSR;			//Offset 0x08
	volatile uint32_t FTSR;			//Offset 0x0C
	volatile uint32_t SWIER;		//Offset 0x10
	volatile uint32_t PR;			//Offset 0x14
}EXTI_Reg_t;

//SYSCFG
typedef struct{
	volatile uint32_t MEMRMP;		//Offset 0x00
	volatile uint32_t PMC;			//Offset 0x04
	volatile uint32_t EXTICR[4];	//Offset 0x08 - 0x14
			 uint32_t RESERVED[2];	//Offset 0x18 & 0x1C
	volatile uint32_t CMPCR;		//Offset 0x20
}SYSCFG_Reg_t;

/*
 * Peripheral definitions
 */

//GPIOx
#define GPIOA					 ((GPIO_Reg_t*)GPIOA_BASEADDR)
#define GPIOB					 ((GPIO_Reg_t*)GPIOB_BASEADDR)
#define GPIOC					 ((GPIO_Reg_t*)GPIOC_BASEADDR)
#define GPIOD					 ((GPIO_Reg_t*)GPIOD_BASEADDR)
#define GPIOE					 ((GPIO_Reg_t*)GPIOE_BASEADDR)
#define GPIOF					 ((GPIO_Reg_t*)GPIOF_BASEADDR)
#define GPIOG					 ((GPIO_Reg_t*)GPIOG_BASEADDR)
#define GPIOH					 ((GPIO_Reg_t*)GPIOH_BASEADDR)
#define GPIOI					 ((GPIO_Reg_t*)GPIOI_BASEADDR)

//RCC
#define RCC						 ((RCC_Reg_t*)RCC_BASEADDR)

//EXTI
#define EXTI					 ((EXTI_Reg_t*)EXTI_BASEADDR)

//SYSCFG
#define SYSCFG					 ((SYSCFG_Reg_t*)SYSCFG_BASEADDR)

/*
 * Clock enable macros
 */

//Clock enable macros for GPIOx
#define GPIOA_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 8))

//Clock enable macros for I2C
#define I2C1_PCLK_EN()			 (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			 (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			 (RCC->APB1ENR |= (1 << 23))

//Clock enable macros for SPI
#define SPI1_PCLK_EN()			 (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			 (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			 (RCC->APB1ENR |= (1 << 15))

//Clock enable macros for USART
#define USART1_PCLK_EN()		 (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		 (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		 (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		 	 (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		 	 (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()		 (RCC->APB2ENR |= (1 << 5))

//Clock enable macro for SYSCFG
#define SYSCFG_PCLK_EN()		 (RCC->APB2ENR |= (1 << 14))

/*
 * Clock disable macros
 */

//Clock disable macros for GPIOx
#define GPIOA_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()			 (RCC->AHB1ENR &= ~(1 << 8))

//Clock disable macros for I2C
#define I2C1_PCLK_DI()			 (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			 (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			 (RCC->APB1ENR &= ~(1 << 23))

//Clock disable macros for SPI
#define SPI1_PCLK_DI()			 (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			 (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			 (RCC->APB1ENR &= ~(1 << 15))

//Clock disable macros for USART
#define USART1_PCLK_DI()		 (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		 (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		 (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		 	 (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		 	 (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()		 (RCC->APB2ENR &= ~(1 << 5))

//Clock disable macro for SYSCFG
#define SYSCFG_PCLK_DI()		 (RCC->APB2ENR &= ~(1 << 14))

/*
 * GPIOx reset macros
 */
#define GPIOA_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()		 do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * Generics
 */
#define ENABLE 						 1
#define DISABLE 					 0
#define SET 						 ENABLE
#define RESET 						 DISABLE
#define GPIO_PIN_SET				 SET
#define GPIO_PIN_RESET				 RESET

#define GPIO_BASE_TO_PORT(x)		 ((x == GPIOA) ? 0:\
									  (x == GPIOB) ? 1:\
									  (x == GPIOC) ? 2:\
									  (x == GPIOD) ? 3:\
									  (x == GPIOE) ? 4:\
									  (x == GPIOF) ? 5:\
									  (x == GPIOG) ? 6:\
									  (x == GPIOH) ? 7:\
									  (x == GPIOI) ? 8:0)

//IRQ numbers for the STM32F407
#define IRQ_NUM_EXTI0				 6
#define IRQ_NUM_EXTI1				 7
#define IRQ_NUM_EXTI2				 8
#define IRQ_NUM_EXTI3				 9
#define IRQ_NUM_EXTI4				 10
#define IRQ_NUM_EXTI9_5				 23
#define IRQ_NUM_EXTI15_10			 40

/*******************************END: MCU specific definitions*****************************************************/
#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
