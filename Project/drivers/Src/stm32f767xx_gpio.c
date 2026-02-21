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

/****************** GPIO Initialization and de-initialization functions ******************/

/*******************************************
    * @fn				- GPIO_Init
    * @brief			- This function initializes the GPIO pin based on the GPIO_PinConfig structure
    * @param[in]		- pGPIOHandle: GPIO handle structure that contains the
    * base address of the GPIO port and the pin configuration settings
    * @return			- None
    * Note			- None
    *******************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
}

/*******************************************
    * @fn				- GPIO_DeInit
    * @brief			- This function de-initializes the GPIO port registers to their default reset values
    * @param[in]		- pGPIOx: Base address of the GPIO port to be de-initialized
    * @return			- None
    * Note			- None
    *******************************************/    
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
}

/****************** GPIO Peripheral Clock setup ******************/

/*******************************************
    * @fn				- GPIO_PeriClockControl
    * @brief			- This function enables or disables the peripheral clock for the given GPIO port
    * @param[in]		- pGPIOx: Base address of the GPIO port
    * @param[in]		- EnorDi: ENABLE or DISABLE macros
    * @return			- None
    * Note			- None
    *******************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
}

/****************** GPIO Read and Write functions ******************/

/*******************************************
    * @fn				- GPIO_ReadFromInputPin
    * @brief			- This function reads the value from the specified GPIO input pin
    * @param[in]		- pGPIOx: Base address of the GPIO port
    * @param[in]		- PinNumber: GPIO pin number (0 to 15)
    * @return			- The value read from the specified GPIO input pin (0 or 1)
    * Note			- None
    *******************************************/
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber)
{
}

/*******************************************
    * @fn				- GPIO_ReadFromInputPort
    * @brief			- This function reads the value from the entire GPIO input port
    * @param[in]		- pGPIOx: Base address of the GPIO port
    * @return			- The value read from the entire GPIO input port (16-bit value)
    * Note			- None
    *******************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
}

/*******************************************
    * @fn				- GPIO_WriteToOutputPin
    * @brief			- This function writes a value to the specified GPIO output pin
    * @param[in]		- pGPIOx: Base address of the GPIO port
    * @param[in]		- PinNumber: GPIO pin number (0 to 15)
    * @param[in]		- Value: Value to be written to the specified GPIO output pin (0 or 1)
    * @return			- None
    * Note			- None
    *******************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber, uint8_t Value)
{
}

/*******************************************
    * @fn				- GPIO_WriteToOutputPort
    * @brief			- This function writes a value to the entire GPIO output port
    * @param[in]		- pGPIOx: Base address of the GPIO port
    * @param[in]		- Value: Value to be written to the entire GPIO output port (16-bit value)
    * @return			- None
    * Note			- None
    *******************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
}

/*******************************************
    * @fn				- GPIO_ToggleOutputPin
    * @brief			- This function toggles the value of the specified GPIO output pin
    * @param[in]		- pGPIOx: Base address of the GPIO port
    * @param[in]		- PinNumber: GPIO pin number (0 to 15)
    * @return			- None
    * Note			- None
    *******************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint16_t PinNumber)
{
}

/****************** GPIO IRQ Configuration and ISR handling functions ******************/

/*******************************************
    * @fn				- GPIO_IRQConfig
    * @brief			- This function configures the IRQ for the specified GPIO pin
    * @param[in]		- IRQNumber: IRQ number for the GPIO pin
    * @param[in]		- IRQPriority: Priority of the IRQ
    * @param[in]		- EnorDi: ENABLE or DISABLE macros
    * @return			- None
    * Note			- None
    *******************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{
}

/*******************************************
    * @fn				- GPIO_IRQHandling
    * @brief			- This function handles the IRQ for the specified GPIO pin
    * @param[in]		- PinNumber: GPIO pin number (0 to 15) for which the IRQ occurred
    * @return			- None
    * Note			- None
    *******************************************/
void GPIO_IRQHandling(uint16_t PinNumber)
{
}

