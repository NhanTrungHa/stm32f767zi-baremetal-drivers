#include "stm32f767xx.h"

int main(void)
{

    return 0;
}

void EXTI0_IRQHandler(void)
{

	// Clear Exti PR Register corresponding to pin #
    GPIO_IRQHandling(0);
}
