#This repo holds my drivers for the STM32F407G-DISC1 board, following [this Udemy course](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development).

##GPIO
Usage of GPIO drivers:
1. Create a GPIO_Handle_t struct
2. `GPIO_Handle_t.pGPIOx` is a struct holding the base address for the GPIO port used (GPIOA, GPIOB, etc)
3. Configure `GPIO_Handle_t.GPIO_PinCfg` struct with the appropriate values (macros found in the GPIO driver header [file](https://github.com/PyryLaa/stm32f4xx_drivers/blob/main/drivers/Inc/stm32f407xx_gpio_driver.h))
4. Use `GPIO_PClkCntl()` to enable the peripheral clock
5. Use `GPIO_Init()` to initialize the GPIO port register with the values configured to `GPIO_Handle_t.GPIO_PinCfg`
