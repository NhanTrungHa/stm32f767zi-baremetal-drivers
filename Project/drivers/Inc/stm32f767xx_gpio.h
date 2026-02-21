/*
 * st32f767xx_gpio.h
 *
 *  Created on: Feb 20, 2026
 *      Author: nhant
 */

#ifndef INC_STM32F767XX_GPIO_H_
#define INC_STM32F767XX_GPIO_H_

#include "stm32f767xx.h"

/****************** GPIO function prototypes ******************/
void GPIO_Init(void);
void GPIO_DeInit(void);
void GPIO_PeriClockControl(void);
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);

#endif /* INC_STM32F767XX_GPIO_H_ */
