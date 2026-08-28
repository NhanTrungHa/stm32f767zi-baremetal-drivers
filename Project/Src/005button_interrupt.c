#include "stm32f767xx.h"
#include <string.h>

void delay(void)
{
    for (int i = 0; i < 1000000; i++);
}

int main(void)
{
    GPIO_Handle_t GPIOBtn;
    GPIO_Handle_t GPIOLED;

    memset(&GPIOLED, 0, sizeof(GPIOLED));
    memset(&GPIOBtn, 0, sizeof(GPIOBtn));    // <-- fix
    GPIOLED.pGPIOx = GPIOB;
    GPIOLED.GPIO_PinConfig.GPIO_PinNumber    = GPIO_PIN_NO_7;
    GPIOLED.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_OUT;
    GPIOLED.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST;
    GPIOLED.GPIO_PinConfig.GPIO_PinOPType    = GPIO_OP_TYPE_PP;
    GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


    GPIOBtn.pGPIOx = GPIOD;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_5;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_IT_FT;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&GPIOLED);
    GPIO_Init(&GPIOBtn);

    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, GPIO_PIN_RESET);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PR15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

    while (1);
}

void EXTI9_5_IRQHandler(void)
{
    delay();  // debounce
    GPIO_IRQHandling(GPIO_PIN_NO_5);
    GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
}
