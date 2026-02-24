/*
 * stm32f767xx_gpio.c
 *
 *  Created on: Feb 20, 2026
 *      Author: nhant
 */

#include "stm32f767xx_gpio.h"



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
uint32_t temp = 0; 
   if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
   {
        // Non-interrupt mode
        // 1. Configure the mode of the GPIO pin
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER = temp; 
        
   }
   else
   {
       // Interrupt mode
   }

   temp = 0;
    // 2. Configure the speed of the GPIO pin
   temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
   pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 
   pGPIOHandle->pGPIOx->OSPEEDR |= temp;

   temp = 0;
   // 3. Configure the pull-up/pull-down settings of the GPIO pin
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 
    pGPIOHandle->pGPIOx->PUPDR |= temp;

   // 4. Configure the output type of the GPIO pin
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    // 5. Configure the alternate function mode of the GPIO pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        // Temp1 and Temp2 are used to determine which AFR register (AFR[0] or AFR[1]) and which position within that register to configure for the alternate function mode
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); 
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
    
   temp = 0;
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
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
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
    if(EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DIS();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DIS();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DIS();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DIS();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DIS();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DIS();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DIS();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DIS();
        }
    }
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
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
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
    uint16_t value;
    value = (uint16_t)(pGPIOx->IDR);
    return value;
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
    if(Value == GPIO_PIN_SET)
    {
        // Write 1 to the output data register at the bit position corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // Write 0 to the output data register at the bit position corresponding to the pin number
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
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
    pGPIOx->ODR = Value;
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
    pGPIOx->ODR ^= (1 << PinNumber);
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

