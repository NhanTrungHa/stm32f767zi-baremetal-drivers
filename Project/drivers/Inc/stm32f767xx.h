/*
 * stm32f767xx.h
 *
 *  Created on: Feb 20, 2026
 *      Author: nhant
 */

#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

/* Includes */
#include "stdint.h"

/* Base addresses of Flash and SRAM memories */

#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20020000U
#define SRAM2_BASEADDR		0x2007C000U
#define SRAM 				SRAM1_BASEADDR
#define ROM_BASEADDR		0x1FF00000U
#define ROM					ROM_BASEADDR


/* Base addresses of AHBx and APBx bus peripheral */

#define PERIPH_BASEADDR	    0x40000000U
#define	APB1PERIPH_BASE     (PERIPH_BASEADDR)
#define AHB1PERIPH_BASE     (PERIPH_BASEADDR + 0x00020000U)
#define APB2PERIPH_BASE     (PERIPH_BASEADDR + 0x00010000U)
#define AHB2PERIPH_BASE     (PERIPH_BASEADDR + 0x10000000U)

/* Base addresses of AHB1 peripherals */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASEADDR		(AHB1PERIPH_BASE + 0x2800U)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800U)
/* Base addresses of APB1 peripherals */
#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00U)
#define I2C4_BASEADDR		(APB1PERIPH_BASE + 0x6000U)
#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASEADDR     (APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR     (APB1PERIPH_BASE + 0x4800U)
#define USART4_BASEADDR     (APB1PERIPH_BASE + 0x4C00U)
#define USART5_BASEADDR     (APB1PERIPH_BASE + 0x5000U)
#define UART7_BASEADDR		(APB1PERIPH_BASE + 0x7800U)
#define UART8_BASEADDR		(APB1PERIPH_BASE + 0x7C00U)

/* Base addresses of APB2 peripherals */
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00U)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASEADDR		(APB2PERIPH_BASE + 0x3400U)
#define USART1_BASEADDR     (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR     (APB2PERIPH_BASE + 0x1400U)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800U)

typedef struct 
{
    volatile uint32_t MODER;    /* GPIO port mode register,               Address offset: 0x00      */
    volatile uint32_t OTYPER;   /* GPIO port output type register,        Address offset: 0x04      */
    volatile uint32_t OSPEEDR;  /* GPIO port output speed register,       Address offset: 0x08      */
    volatile uint32_t PUPDR;    /* GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    volatile uint32_t IDR;      /* GPIO port input data register,         Address offset: 0x10      */
    volatile uint32_t ODR;      /* GPIO port output data register,        Address offset: 0x14      */
    volatile uint32_t BSRR;     /* GPIO port bit set/reset register,      Address offset: 0x18      */
    volatile uint32_t LCKR;     /* GPIO port configuration lock register, Address offset: 0x1C      */
    volatile uint32_t AFR[2];   /* AFR[0]: GPIO alternate function low register,  Address offset: 0x20
                                    AFR[1]: GPIO alternate function high register, Address offset: 0x24 */
}GPIO_RegDef_t;

typedef struct 
{
    volatile uint32_t CR;        /* RCC clock control register,                      Address offset: 0x00 */
    volatile uint32_t PLLCFGR;   /* RCC PLL configuration register,                  Address offset: 0x04 */
    volatile uint32_t CFGR;      /* RCC clock configuration register,                Address offset: 0x08 */
    volatile uint32_t CIR;       /* RCC clock interrupt register,                    Address offset: 0x0C */
    volatile uint32_t AHB1RSTR;  /* RCC AHB1 peripheral reset register,              Address offset: 0x10 */
    volatile uint32_t AHB2RSTR;  /* RCC AHB2 peripheral reset register,              Address offset: 0x14 */
    volatile uint32_t AHB3RSTR;  /* RCC AHB3 peripheral reset register,              Address offset: 0x18 */
    uint32_t      RESERVED0;    /* Reserved,                                        Address offset: 0x1C */
    volatile uint32_t APB1RSTR;  /* RCC APB1 peripheral reset register,              Address offset: 0x20 */
    volatile uint32_t APB2RSTR;  /* RCC APB2 peripheral reset register,              Address offset: 0x24 */
    uint32_t      RESERVED1[2]; /* Reserved,                                        Address offset: 0x28-0x2C */
    volatile uint32_t AHB1ENR;   /* RCC AHB1 peripheral clock enable register,       Address offset: 0x30 */
    volatile uint32_t AHB2ENR;   /* RCC AHB2 peripheral clock enable register,       Address offset: 0x34 */
    volatile uint32_t AHB3ENR;   /* RCC AHB3 peripheral clock enable register,       Address offset: 0x38 */
    uint32_t      RESERVED2;    /* Reserved,                                        Address offset: 0x3C */
    volatile uint32_t APB1ENR;   /* RCC APB1 peripheral clock enable register,       Address offset: 0x40 */
    volatile uint32_t APB2ENR;   /* RCC APB2 peripheral clock enable register,       Address offset: 0x44 */
    uint32_t      RESERVED3[2]; /* Reserved,                                        Address offset: 0x48-0x4C */
    volatile uint32_t AHB1LPENR; /* RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    volatile uint32_t AHB2LPENR; /* RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    volatile uint32_t AHB3LPENR; /* RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
    uint32_t      RESERVED4;    /* Reserved,                                        Address offset: 0x5C */
    volatile uint32_t APB1LPENR; /* RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    volatile uint32_t APB2LPENR; /* RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
    uint32_t      RESERVED5[2]; /* Reserved,                                        Address offset: 0x68-0x6C */
    volatile uint32_t BDCR;      /* RCC backup domain control register,              Address offset: 0x70 */
    volatile uint32_t CSR;       /* RCC clock control & status register,             Address offset: 0x74 */
    uint32_t      RESERVED6[2]; /* Reserved,                                        Address offset: 0x78-0x7C */
    volatile uint32_t SSCGR;     /* RCC spread spectrum clock generation register,   Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR;/* RCC PLLI2S configuration register,                  Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR;/* RCC PLLSAI configuration register,                  Address offset: 0x88 */
    volatile uint32_t DCKCFGR1;   /* RCC Dedicated Clocks Configuration Register,     Address offset: 0x8C */
    volatile uint32_t DCKCFGR2;   /* RCC Dedicated Clocks Configuration Register,     Address offset: 0x90 */

}RCC_RegDef_t;


/* Peripheral definitions */
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ		((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK		((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)


/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()   (RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()   (RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()   (RCC->AHB1ENR |= (1 << 10))

/* Clock Enable Macros for I2C peripherals */
#define I2C1_PCLK_EN()   (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |= (1 << 23))

/* Clock Enable Macros for SPI peripherals */
#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= (1 << 15))

/* Clock Enable Macros for USART peripherals */
#define USART1_PCLK_EN()   (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()   (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()   (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()    (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()    (RCC->APB1ENR |= (1 << 20))

/* Clock Enable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |= (1 << 14))

/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DIS()   (RCC->AHB1ENR &= ~(1 << 10))

/* Clock Disable Macros for I2C peripherals */
#define I2C1_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 23))

/* Clock Disable Macros for SPI peripherals */
#define SPI1_PCLK_DIS()   (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 15))

/****************** Generic Macros ******************/
#define ENABLE                1
#define DISABLE               0
#define SET                   ENABLE
#define RESET                 DISABLE
#define GPIO_PIN_SET           SET
#define GPIO_PIN_RESET         RESET

#endif /* INC_STM32F767XX_H_ */
