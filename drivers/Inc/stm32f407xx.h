#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

//Memory base addresses
#define DRV_FLASH_BASEADDR 			0x08000000U 		//Base address for the flash memory
#define DRV_SRAM1_BASEADDR			0x20000000U 		//Base address for the sram1 memory, 112KB
#define DRV_SRMA2_BASEADDR			0x2001C000U 		//Base address for the sram2 memory, comes straight after sram1
#define DRV_ROM_BASEADDR			0x1FFF0000U 		//Base address for the system memory
#define DRV_SRAM 					SRAM1_BASEADDR

//Bus domains' base addresses
#define DRV_PERIPH_BASEADDR			0x40000000U 		//Peripheral bus base address (same as APB1)
#define DRV_APB1PERIPH_BASEADDR		PERIPH_BASEADDR 	//Base address for the APB1 bus peripherals
#define DRV_APB2PERIPH_BASEADDR		0x40010000U 		//Base address for the APB2 bus peripherals
#define DRV_AHB1PERIPH_BASEADDR		0x40020000U 		//Base address for the AHB1 bus peripherals
#define DRV_AHB2PERIPH_BASEADDR		0x50000000U 		//base address for the AHB2 bus peripherals

//Peripheral base addresses in AHB1 bus, for every peripheral, address is AHB1PERIPH_BASEADDR + offset
#define DRV_GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define DRV_GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define DRV_GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define DRV_GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define DRV_GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define DRV_GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define DRV_GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define DRV_GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define DRV_GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define DRV_GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define DRV_GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)

//Peripheral base addresses in APB1 bus, for every peripheral, address is APB1PERIPH_BASEADDR + offset
#define DRV_SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define DRV_SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define DRV_USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define DRV_USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)

#define DRV_UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define DRV_UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

#define DRV_I2C1_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5400)
#define DRV_I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define DRV_I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define DRV_UART7_BASEADDR			(APB1PERIPH_BASEADDR + 0x7800)
#define DRV_UART8_BASEADDR			(APB1PERIPH_BASEADDR + 0x7C00)

//Peripheral base addresses in APB2 bus, for every peripheral, address is APB2PERIPH_BASEADDR + offset
#define DRV_USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define DRV_USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

#define DRV_SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define DRV_SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define DRV_SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define DRV_EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define DRV_SPI5_BASEADDR			(APB2PERIPH_BASEADDR + 0x5000)
#define DRV_SPI6_BASEADDR			(APB2PERIPH_BASEADDR + 0x5400)


//Register structures for peripherals

typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_Reg_t;

#define DRV_GPIOA					 ((GPIO_Reg_t*)DRV_GPIOA_BASEADDR)
#define DRV_GPIOB					 ((GPIO_Reg_t*)DRV_GPIOB_BASEADDR)
#define DRV_GPIOC					 ((GPIO_Reg_t*)DRV_GPIOC_BASEADDR)
#define DRV_GPIOD					 ((GPIO_Reg_t*)DRV_GPIOD_BASEADDR)
#define DRV_GPIOE					 ((GPIO_Reg_t*)DRV_GPIOE_BASEADDR)
#define DRV_GPIOF					 ((GPIO_Reg_t*)DRV_GPIOF_BASEADDR)
#define DRV_GPIOG					 ((GPIO_Reg_t*)DRV_GPIOG_BASEADDR)
#define DRV_GPIOH					 ((GPIO_Reg_t*)DRV_GPIOH_BASEADDR)
#define DRV_GPIOI					 ((GPIO_Reg_t*)DRV_GPIOI_BASEADDR)
#define DRV_GPIOJ					 ((GPIO_Reg_t*)DRV_GPIOJ_BASEADDR)
#define DRV_GPIOK					 ((GPIO_Reg_t*)DRV_GPIOK_BASEADDR)

#endif /* INC_STM32F407XX_H_ */
