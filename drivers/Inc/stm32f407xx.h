#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

//Memory base addresses
#define DRV_FLASH_BASEADDR 			0x08000000U 		//Base address for the flash memory
#define DRV_SRAM1_BASEADDR			0x20000000U 		//Base address for the sram1 memory, 112KB
#define DRV_SRMA2_BASEADDR			0x2001C000U 		//Base address for the sram2 memory, comes straight after sram1
#define DRV_ROM_BASEADDR			0x1FFF0000U 		//Base address for the system memory
#define DRV_SRAM 					DRV_SRAM1_BASEADDR

//Bus domains' base addresses
#define DRV_PERIPH_BASEADDR			0x40000000U 		//Peripheral bus base address (same as APB1)
#define DRV_APB1PERIPH_BASEADDR		DRV_PERIPH_BASEADDR 	//Base address for the APB1 bus peripherals
#define DRV_APB2PERIPH_BASEADDR		0x40010000U 		//Base address for the APB2 bus peripherals
#define DRV_AHB1PERIPH_BASEADDR		0x40020000U 		//Base address for the AHB1 bus peripherals
#define DRV_AHB2PERIPH_BASEADDR		0x50000000U 		//base address for the AHB2 bus peripherals

//Peripheral base addresses in AHB1 bus, for every peripheral, address is AHB1PERIPH_BASEADDR + offset
#define DRV_GPIOA_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x0000)
#define DRV_GPIOB_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x0400)
#define DRV_GPIOC_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x0800)
#define DRV_GPIOD_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x0C00)
#define DRV_GPIOE_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x1000)
#define DRV_GPIOF_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x1400)
#define DRV_GPIOG_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x1800)
#define DRV_GPIOH_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x1C00)
#define DRV_GPIOI_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x2000)

#define DRV_RCC_BASEADDR			(DRV_AHB1PERIPH_BASEADDR + 0x3800)

//Peripheral base addresses in APB1 bus, for every peripheral, address is APB1PERIPH_BASEADDR + offset
#define DRV_SPI2_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x3800)
#define DRV_SPI3_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x3C00)

#define DRV_USART2_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x4400)
#define DRV_USART3_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x4800)

#define DRV_UART4_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x4C00)
#define DRV_UART5_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x5000)

#define DRV_I2C1_BASEADDR 			(DRV_APB1PERIPH_BASEADDR + 0x5400)
#define DRV_I2C2_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x5800)
#define DRV_I2C3_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x5C00)

#define DRV_UART7_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x7800)
#define DRV_UART8_BASEADDR			(DRV_APB1PERIPH_BASEADDR + 0x7C00)

//Peripheral base addresses in APB2 bus, for every peripheral, address is APB2PERIPH_BASEADDR + offset
#define DRV_USART1_BASEADDR			(DRV_APB2PERIPH_BASEADDR + 0x1000)
#define DRV_USART6_BASEADDR			(DRV_APB2PERIPH_BASEADDR + 0x1400)

#define DRV_SPI1_BASEADDR			(DRV_APB2PERIPH_BASEADDR + 0x3000)
#define DRV_SPI4_BASEADDR			(DRV_APB2PERIPH_BASEADDR + 0x3400)

#define DRV_SYSCFG_BASEADDR			(DRV_APB2PERIPH_BASEADDR + 0x3800)

#define DRV_EXTI_BASEADDR			(DRV_APB2PERIPH_BASEADDR + 0x3C00)

#define DRV_SPI5_BASEADDR			(DRV_APB2PERIPH_BASEADDR + 0x5000)
#define DRV_SPI6_BASEADDR			(DRV_APB2PERIPH_BASEADDR + 0x5400)


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

//Peripheral definitions

#define DRV_GPIOA					 ((GPIO_Reg_t*)DRV_GPIOA_BASEADDR)
#define DRV_GPIOB					 ((GPIO_Reg_t*)DRV_GPIOB_BASEADDR)
#define DRV_GPIOC					 ((GPIO_Reg_t*)DRV_GPIOC_BASEADDR)
#define DRV_GPIOD					 ((GPIO_Reg_t*)DRV_GPIOD_BASEADDR)
#define DRV_GPIOE					 ((GPIO_Reg_t*)DRV_GPIOE_BASEADDR)
#define DRV_GPIOF					 ((GPIO_Reg_t*)DRV_GPIOF_BASEADDR)
#define DRV_GPIOG					 ((GPIO_Reg_t*)DRV_GPIOG_BASEADDR)
#define DRV_GPIOH					 ((GPIO_Reg_t*)DRV_GPIOH_BASEADDR)
#define DRV_GPIOI					 ((GPIO_Reg_t*)DRV_GPIOI_BASEADDR)

#define DRV_RCC						 ((RCC_Reg_t*)DRV_RCC_BASEADDR)

//Clock enable macros for GPIOx
#define DRV_GPIOA_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 0))
#define DRV_GPIOB_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 1))
#define DRV_GPIOC_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 2))
#define DRV_GPIOD_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 3))
#define DRV_GPIOE_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 4))
#define DRV_GPIOF_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 5))
#define DRV_GPIOG_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 6))
#define DRV_GPIOH_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 7))
#define DRV_GPIOI_PCLK_EN()			 (RCC->AHB1ENR |= (1 << 8))

//Clock enable macros for I2C
#define DRV_I2C1_PCLK_EN()			 (RCC->APB1ENR |= (1 << 21))
#define DRV_I2C2_PCLK_EN()			 (RCC->APB1ENR |= (1 << 22))
#define DRV_I2C3_PCLK_EN()			 (RCC->APB1ENR |= (1 << 23))

//Clock enable macros for SPI
#define DRV_SPI1_PCLK_EN()			 (RCC->APB2ENR |= (1 << 12))
#define DRV_SPI2_PCLK_EN()			 (RCC->APB1ENR |= (1 << 14))
#define DRV_SPI3_PCLK_EN()			 (RCC->APB1ENR |= (1 << 15))

//Clock enable macros for USART
#define DRV_USART1_PCLK_EN()		 (RCC->APB2ENR |= (1 << 4))
#define DRV_USART2_PCLK_EN()		 (RCC->APB1ENR |= (1 << 17))
#define DRV_USART3_PCLK_EN()		 (RCC->APB1ENR |= (1 << 18))
#define DRV_UART4_PCLK_EN()		 	 (RCC->APB1ENR |= (1 << 19))
#define DRV_UART5_PCLK_EN()		 	 (RCC->APB1ENR |= (1 << 20))
#define DRV_USART6_PCLK_EN()		 (RCC->APB2ENR |= (1 << 5))

//Clock enable macros for SYSCFG
#define DRV_SYSCFG_PCLK_EN()		 (RCC->APB2ENR |= (1 << 14))

//TODO Write clock disable macros


#endif /* INC_STM32F407XX_H_ */
