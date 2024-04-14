//Toggles on board user led4 (PD12) using own developed drivers when the user button is pressed on board

#include "stm32f407xx.h"

int main(void){

	//Create and initialize GPIO handles for pd12(led) and pa0(button)
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	GpioLed.pGPIOx = GPIOD; //Base address of GPIO port D
	GpioLed.GPIO_PinCfg.GPIO_PinNum = GPIO_PIN_12;
	GpioLed.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinCfg.GPIO_PinSpeed = GPIO_OUT_SPD_HIGH;
	GpioLed.GPIO_PinCfg.GPIO_PinOType = GPIO_OUT_TYPE_PP; //Push pull
	GpioLed.GPIO_PinCfg.GPIO_PinPuPdCntl = GPIO_PUPD_DI; //No pull-up / pull-down

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinCfg.GPIO_PinNum = GPIO_PIN_0;
	GpioButton.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinCfg.GPIO_PinPuPdCntl = GPIO_PUPD_DI; //No pull-up/pull-down

	//Enable clock and initialize registers
	GPIO_PClkCntl(GPIOD, ENABLE); //Led
	GPIO_PClkCntl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	while(1){
		//Read button and toggle led if pressed
		if(GPIO_ReadPinIn(GPIOA, 0) == 1){
			for(int i = 0; i < 250000; i++); //Crude debounce delay
			GPIO_ToggleOutPin(GPIOD, GPIO_PIN_12);
		}
	}

	return 0;
}

