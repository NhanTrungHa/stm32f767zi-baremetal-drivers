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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber);
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint16_t PinNumber);





#endif /* INC_STM32F767XX_GPIO_H_ */
