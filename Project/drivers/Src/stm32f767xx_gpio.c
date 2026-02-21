/*
 * stm32f767xx_gpio.c
 *
 *  Created on: Feb 20, 2026
 *      Author: nhant
 */

#include "stm32f767xx_gpio.h"

typedef struct
{
    uint8_t GPIO_PinNumber;		/* Possible values from @GPIO_PIN_NUMBERS */
    uint8_t GPIO_PinMode;		/* Possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;		/* Possible values from @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl;/* Possible values from @GPIO_PIN_PUPD */
    uint8_t GPIO_PinOPType;		/* Possible values from @GPIO_PIN_OP_TYPE */
    uint8_t GPIO_PinAltFunMode; /* Alternate function mode configuration, possible values from @GPIO_ALT_FUN_MODES */
}GPIO_PinConfig_t;

typedef struct 
{
    GPIO_RegDef_t *pGPIOx;		/* This holds the base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig;	/* This holds GPIO pin configuration settings */
}GPIO_Handle_t;