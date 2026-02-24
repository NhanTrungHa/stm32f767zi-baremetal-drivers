/*
 * 001led_toggle.c
 *
 *  Created on: Feb 22, 2026
 *      Author: nhant
 */

#include "stm32f767xx.h"

 void delay(void)
 {
     for (int i = 0; i < 500000; i++);
 }

int main(void)
{
    GPIO_Handle_t GPIOLED;
    GPIOLED.pGPIOx = GPIOB;
    GPIOLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIOLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIOLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_Init(&GPIOLED);

    while (1)
    {
        GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
        delay();
    }
    
    return 0;
}
