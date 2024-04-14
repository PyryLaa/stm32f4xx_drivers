//Toggles on board user led4 (PD12) using own developed drivers, pin in push-pull mode

#include "stm32f407xx.h"

int main(void){

	//Create and initialize handle for GPIOD pin 12
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOD; //Base address of GPIO port D
	GpioLed.GPIO_PinCfg.GPIO_PinNum = GPIO_PIN_12;
	GpioLed.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinCfg.GPIO_PinSpeed = GPIO_OUT_SPD_HIGH;
	GpioLed.GPIO_PinCfg.GPIO_PinOType = GPIO_OUT_TYPE_PP; //Push pull
	GpioLed.GPIO_PinCfg.GPIO_PinPuPdCntl = GPIO_PUPD_DI; //No pull-up / pull-down

	//Enable clock and initialize registers
	GPIO_PClkCntl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutPin(GPIOD, GPIO_PIN_12);
		for(int i = 0; i < 0xFFFFF; i++); //Crude delay
	}

	return 0;
}



