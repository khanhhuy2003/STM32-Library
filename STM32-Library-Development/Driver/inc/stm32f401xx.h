/*
 * stm32f401xx.h
 *
 *  Created on: Feb 5, 2025
 *      Author: ASUS
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses, for enable interupt
 * NVIC is controled by the CPU itself, so refer more to Cortex M4 user guide, not STM32 reference manual
 */

#define NVIC_ISER0      ( (__vo uint32_t*)0xE000E100 ) //each register control 32 IRQ number => only use 0 1 2 in stm32f4
#define NVIC_ISER1      ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2      ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3      ( (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4      ( (__vo uint32_t*)0xE000E110 )
#define NVIC_ISER5      ( (__vo uint32_t*)0xE000E114 )
#define NVIC_ISER6      ( (__vo uint32_t*)0xE000E118 )
#define NVIC_ISER7      ( (__vo uint32_t*)0xE000E11C )

/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses, for clear/disable interupt
 */
#define NVIC_ICER0      ( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1      ( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2      ( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3      ( (__vo uint32_t*)0XE000E18C )
#define NVIC_ICER4      ( (__vo uint32_t*)0XE000E190 )
#define NVIC_ICER5      ( (__vo uint32_t*)0XE000E194 )
#define NVIC_ICER6      ( (__vo uint32_t*)0XE000E198 )
#define NVIC_ICER7      ( (__vo uint32_t*)0XE000E19C )

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR   ( (__vo uint32_t*)0XE000E400 )

#define NO_PR_BITS_IMPLEMENTED    4

/*
 * Base addresses of FLASH and SRAM memories
 */
#define FLASH_BASEADDR      0x08000000U     /* Base address of FLASH memory */
#define SRAM1_BASEADDR      0x20000000U     /* Base address of SRAM1 memory */
#define SRAM2_BASEADDR      0x2001C000U     /* Base address of SRAM2 memory */
#define ROM_BASEADDR        0x1FFF0000U     /* Base address of ROM memory   */
#define SRAM_BASEADDR       SRAM1_BASEADDR  /* Base address of SRAM memory  */

/*
 * AHBx and APBx Bus peripheral base addresses
 */
#define PERIPH_BASEADDR         0x40000000U     /* Base address of PERIPHERALS memory     */
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR /* Base address of APB1 peripheral memory */
#define APB2PERIPH_BASEADDR     0x40010000U     /* Base address of APB2 peripheral memory */
#define AHB1PERIPH_BASEADDR     0x40020000U     /* Base address of AHB1 peripheral memory */
#define AHB2PERIPH_BASEADDR     0x50000000U     /* Base address of AHB2 peripheral memory */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0000) /* GPIOA base address                    */
#define GPIOB_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0400) /* GPIOB base address                    */
#define GPIOC_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0800) /* GPIOC base address                    */
#define GPIOD_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0C00) /* GPIOD base address                    */
#define GPIOE_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1000) /* GPIOE base address                    */
#define GPIOF_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1400) /* GPIOF base address                    */
#define GPIOG_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1800) /* GPIOG base address                    */
#define GPIOH_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1C00) /* GPIOH base address                    */
#define RCC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x3800) /* RCC base address                      */
#define CRC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x3000) /* CRC base address                      */
#define FIR_BASEADDR        (AHB1PERIPH_BASEADDR + 0x3C00) /* Flash interface register base address */
#define DMA1_BASEADDR       (AHB1PERIPH_BASEADDR + 0x6000) /* DMA1 base address                     */
#define DMA2_BASEADDR       (AHB1PERIPH_BASEADDR + 0x6400) /* DMA2 base address                     */

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR       (APB1PERIPH_BASEADDR + 0x5400) /* I2C1 base address   */
#define I2C2_BASEADDR       (APB1PERIPH_BASEADDR + 0x5800) /* I2C2 base address   */
#define I2C3_BASEADDR       (APB1PERIPH_BASEADDR + 0x5C00) /* I2C3 base address   */
#define SPI2_BASEADDR       (APB1PERIPH_BASEADDR + 0x3800) /* SPI2 base address   */
#define SPI3_BASEADDR       (APB1PERIPH_BASEADDR + 0x3C00) /* SPI3 base address   */
#define USART2_BASEADDR     (APB1PERIPH_BASEADDR + 0x4400) /* USART2 base address */
#define USART3_BASEADDR     (APB1PERIPH_BASEADDR + 0x4800) /* USART3 base address */
#define UART4_BASEADDR      (APB1PERIPH_BASEADDR + 0x4C00) /* UART4 base address  */
#define UART5_BASEADDR      (APB1PERIPH_BASEADDR + 0x5000) /* UART5 base address  */

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASEADDR       (APB2PERIPH_BASEADDR + 0x3000) /* SPI1 base address   */
#define SPI4_BASEADDR       (APB2PERIPH_BASEADDR + 0x3400) /* SPI4 base address   */
#define USART1_BASEADDR     (APB2PERIPH_BASEADDR + 0x1000) /* USART1 base address */
#define USART6_BASEADDR     (APB2PERIPH_BASEADDR + 0x1400) /* USART6 base address */
#define SYSCFG_BASEADDR     (APB2PERIPH_BASEADDR + 0x3800) /* SYSCFG base address */
#define EXTI_BASEADDR       (APB2PERIPH_BASEADDR + 0x3C00) /* EXTI base address   */

/**********************************************************************************************
 *			Peripheral register definition structures
 **********************************************************************************************/
/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
    __vo uint32_t MODER;        /* GPIO port mode register,                         Address offset: 0x00    */
    __vo uint32_t OTYPER;       /* GPIO port output type register,                  Address offset: 0x04    */
    __vo uint32_t OSPEEDR;      /* GPIO port output speed register,                 Address offset: 0x08    */
    __vo uint32_t PUPDR;        /* GPIO port pull-up/down register,                 Address offset: 0x0C    */
    __vo uint32_t IDR;          /* GPIO port input data register,                   Address offset: 0x10    */
    __vo uint32_t ODR;          /* GPIO port output data register,                  Address offset: 0x14    */
    __vo uint32_t BSRR;         /* GPIO port bit set/reset register,                Address offset: 0x18    */
    __vo uint32_t LCKR;         /* GPIO port configuration lock register,           Address offset: 0x1C    */
    __vo uint32_t AFR[2];       /* AFR[0]: GPIO alternate function low register,
                                   AFR[1]: GPIO alternate function high register,   Address offset: 0x20-24 */
}GPIO_RegDef_t;


/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
    __vo uint32_t CR;           /* RCC clock control register, Address offset: 0x00 */
    __vo uint32_t PLLCFGR;      /* RCC PLL configuration register, Address offset: 0x04 */
    __vo uint32_t CFGR;         /* RCC clock configuration register, Address offset: 0x08 */
    __vo uint32_t CIR;          /* RCC clock interrupt register, Address offset: 0x0C */

    __vo uint32_t AHB1RSTR;     /* RCC AHB1 peripheral reset register, Address offset: 0x10 */
    __vo uint32_t AHB2RSTR;     /* RCC AHB2 peripheral reset register, Address offset: 0x14 */

    uint32_t RESERVED0[2];      /* Reserved, Address offset: 0x18 - 0x1C */

    __vo uint32_t APB1RSTR;     /* RCC APB1 peripheral reset register, Address offset: 0x20 */
    __vo uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register, Address offset: 0x24 */

    uint32_t RESERVED1[2];      /* Reserved, Address offset: 0x28 - 0x2C */

    __vo uint32_t AHB1ENR;      /* RCC AHB1 peripheral clock enable register, Address offset: 0x30 */
    __vo uint32_t AHB2ENR;      /* RCC AHB2 peripheral clock enable register, Address offset: 0x34 */

    uint32_t RESERVED2[2];      /* Reserved, Address offset: 0x38 - 0x3C */

    __vo uint32_t APB1ENR;      /* RCC APB1 peripheral clock enable register, Address offset: 0x40 */
    __vo uint32_t APB2ENR;      /* RCC APB2 peripheral clock enable register, Address offset: 0x44 */

    uint32_t RESERVED9[2];      /* Reserved, Address offset: 0x48 - 0x4C */

    __vo uint32_t AHB1LPENR;    /* RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    __vo uint32_t AHB2LPENR;    /* RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */

    uint32_t RESERVED3[2];      /* Reserved, Address offset: 0x58 - 0x5C */

    __vo uint32_t APB1LPENR;    /* RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    __vo uint32_t APB2LPENR;    /* RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */

    uint32_t RESERVED4[2];      /* Reserved, Address offset: 0x68 - 0x6C */

    __vo uint32_t BDCR;         /* RCC Backup domain control register, Address offset: 0x70 */
    __vo uint32_t CSR;          /* RCC clock control & status register, Address offset: 0x74 */

    uint32_t RESERVED5[2];      /* Reserved, Address offset: 0x78 - 0x7C */

    __vo uint32_t SSCGR;        /* RCC spread spectrum clock generation register, Address offset: 0x80 */
    __vo uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register, Address offset: 0x84 */
    __vo uint32_t DCKCFGR;      /* RCC dedicated clocks configuration register, Address offset: 0x8C */

} RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
    __vo uint32_t IMR;      /* EXTI Interrupt mask register,            Address offset: 0x00 */
    __vo uint32_t EMR;      /* EXTI Event mask register,                Address offset: 0x04 */
    __vo uint32_t RTSR;     /* EXTI Risign trigger selection register,  Address offset: 0x08 */
    __vo uint32_t FTSR;     /* EXTI Falling trigger selection register, Address offset: 0x0C */
    __vo uint32_t SWIER;    /* EXTI Software interrupt event register,  Address offset: 0x10 */
    __vo uint32_t PR;       /* EXTI Pending register,                   Address offset: 0x14 */
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
    __vo uint32_t MEMRMP;       /* SYSCFG Memory remap register,                        Address offset: 0x00    */
    __vo uint32_t PMC;          /* SYSCFG Peripheral mode configuration register,       Address offset: 0x04    */
    __vo uint32_t EXTICR[4];    /* SYSCFG External interrupt configuration register,    Address offset: 0x08-14 */
    __vo uint32_t CMPCR;        /* SYSCFG Compensation cell control register,           Address offset: 0x20    */
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
    __vo uint32_t CR1;      /* SPI control register 1,          Address offset: 0x00 */
    __vo uint32_t CR2;      /* SPI control register 2,          Address offset: 0x04 */
    __vo uint32_t SR;       /* SPI status register,             Address offset: 0x08 */
    __vo uint32_t DR;       /* SPI data register,               Address offset: 0x0C */
    __vo uint32_t CRCPR;    /* SPI CRC polynomial register,     Address offset: 0x10 */
    __vo uint32_t RXCRCR;   /* SPI RX CRC register,             Address offset: 0x14 */
    __vo uint32_t TXCRCR;   /* SPI TX CRC register,             Address offset: 0x18 */
    __vo uint32_t I2SCFGR;  /* SPI_I2S configuration register,  Address offset: 0x1C */
    __vo uint32_t I2SPR;    /* SPI_I2S prescaler register,      Address offset: 0x20 */
}SPI_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
    __vo uint32_t CR1;      /* I2C control register 1,      Address offset: 0x00 */
    __vo uint32_t CR2;      /* I2C control register 2,      Address offset: 0x04 */
    __vo uint32_t OAR1;     /* I2C own address register 1,  Address offset: 0x08 */
    __vo uint32_t OAR2;     /* I2C own address register 2,  Address offset: 0x0C */
    __vo uint32_t DR;       /* I2C data register,           Address offset: 0x10 */
    __vo uint32_t SR1;      /* I2C status register 1,       Address offset: 0x14 */
    __vo uint32_t SR2;      /* I2C status register 2,       Address offset: 0x18 */
    __vo uint32_t CCR;      /* I2C clock control register,  Address offset: 0x1C */
    __vo uint32_t TRISE;    /* I2C TRISE register,          Address offset: 0x20 */
    __vo uint32_t FLTR;     /* I2C FLTR register,           Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * Peripheral register definition structure for USART
 */
typedef struct
{
    __vo uint32_t SR;   /* USART status register,                   Address offset: 0x00 */
    __vo uint32_t DR;   /* USART data register,                     Address offset: 0x04 */
    __vo uint32_t BRR;  /* USART baud rate register,                Address offset: 0x08 */
    __vo uint32_t CR1;  /* USART control register 1,                Address offset: 0x0C */
    __vo uint32_t CR2;  /* USART control register 2,                Address offset: 0x10 */
    __vo uint32_t CR3;  /* USART control register 3,                Address offset: 0x14 */
    __vo uint32_t GTPR; /* USART guard time and prescaler register, Address offset: 0x18 */
}USART_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t)
 */
#define GPIOA           ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB           ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC           ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD           ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE           ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF           ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG           ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH           ((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC             ((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI            ((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG          ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1            ((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2            ((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3            ((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4            ((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1            ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2            ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3            ((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1          ((USART_RegDef_t*) USART1_BASEADDR)
#define USART2          ((USART_RegDef_t*) USART2_BASEADDR)
#define USART3          ((USART_RegDef_t*) USART3_BASEADDR)
#define UART4           ((USART_RegDef_t*) UART4_BASEADDR)
#define UART5           ((USART_RegDef_t*) UART5_BASEADDR)
#define USART6          ((USART_RegDef_t*) USART6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 0) ) /* GPIOA peripheral clock enabled */
#define GPIOB_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 1) ) /* GPIOB peripheral clock enabled */
#define GPIOC_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 2) ) /* GPIOC peripheral clock enabled */
#define GPIOD_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 3) ) /* GPIOD peripheral clock enabled */
#define GPIOE_PCLK_EN()     ( RCC->AHB1ENR |= (1 << 4) ) /* GPIOE peripheral clock enabled */


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()      ( RCC->APB1ENR |= (1 << 21) ) /* I2C1 peripheral clock enabled */
#define I2C2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 22) ) /* I2C2 peripheral clock enabled */
#define I2C3_PCLK_EN()      ( RCC->APB1ENR |= (1 << 23) ) /* I2C3 peripheral clock enabled */


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()      ( RCC->APB2ENR |= (1 << 12) ) /* SPI1 peripheral clock enabled */
#define SPI2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 14) ) /* SPI2 peripheral clock enabled */
#define SPI3_PCLK_EN()      ( RCC->APB1ENR |= (1 << 15) ) /* SPI3 peripheral clock enabled */
#define SPI4_PCLK_EN()      ( RCC->APB2ENR |= (1 << 13) ) /* SPI4 peripheral clock enabled */


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()    ( RCC->APB2ENR |= (1 << 4) )  /* USART1 peripheral clock enabled */
#define USART2_PCLK_EN()    ( RCC->APB1ENR |= (1 << 17) ) /* USART2 peripheral clock enabled */
#define USART3_PCLK_EN()    ( RCC->APB1ENR |= (1 << 18) ) /* USART3 peripheral clock enabled */
#define UART4_PCLK_EN()     ( RCC->APB1ENR |= (1 << 19) ) /* USART4 peripheral clock enabled */
#define UART5_PCLK_EN()     ( RCC->APB1ENR |= (1 << 20) ) /* USART5 peripheral clock enabled */
#define USART6_PCLK_EN()    ( RCC->APB2ENR |= (1 << 5) )  /* USART6 peripheral clock enabled */

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()    ( RCC->APB2ENR |= (1 << 14) ) /* SYSCFG peripheral clock enabled */

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 0) ) /* GPIOA peripheral clock disabled */
#define GPIOB_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 1) ) /* GPIOB peripheral clock disabled */
#define GPIOC_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 2) ) /* GPIOC peripheral clock disabled */
#define GPIOD_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 3) ) /* GPIOD peripheral clock disabled */
#define GPIOE_PCLK_DI()     ( RCC->AHB1ENR &= ~(1 << 4) ) /* GPIOE peripheral clock disabled */



/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 21) ) /* I2C1 peripheral clock disabled */
#define I2C2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 22) ) /* I2C2 peripheral clock disabled */
#define I2C3_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 23) ) /* I2C3 peripheral clock disabled */


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 12) ) /* SPI1 peripheral clock disabled */
#define SPI2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 14) ) /* SPI2 peripheral clock disabled */
#define SPI3_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 15) ) /* SPI3 peripheral clock disabled */
#define SPI4_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 13) ) /* SPI4 peripheral clock disabled */


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 4) )  /* USART1 peripheral clock disabled */
#define USART2_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 17) ) /* USART2 peripheral clock disabled */
#define USART3_PCLK_DI()    ( RCC->APB1ENR &= ~(1 << 18) ) /* USART3 peripheral clock disabled */
#define UART4_PCLK_DI()     ( RCC->APB1ENR &= ~(1 << 19) ) /* USART4 peripheral clock disabled */
#define UART5_PCLK_DI()     ( RCC->APB1ENR &= ~(1 << 20) ) /* USART5 peripheral clock disabled */
#define USART6_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 5) )  /* USART6 peripheral clock disabled */

/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()    ( RCC->APB2ENR &= ~(1 << 14) )  /* SYSCFG peripheral clock disabled */

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()    do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()    do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()    do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)


/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()    do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()    do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()    do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()    do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)


/*
 * Macros to reset SPIx peripherals
 */
#define USART1_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 4));  (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 5));  (RCC->APB2RSTR &= ~(1 << 5)); }while(0)

/*
 * Generic Macros
 */
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET        SET
#define FLAG_RESET      RESET

/*
 * IQR number
 */
#define IRQ_NO_EXTI0       6
#define IRQ_NO_EXTI1       7
#define IRQ_NO_EXTI2       8
#define IRQ_NO_EXTI3       9
#define IRQ_NO_EXTI4       10
#define IRQ_NO_EXTI5_9     23
#define IRQ_NO_EXTI10_15   40
#define IRQ_NO_SPI1        35
#define IRQ_NO_SPI2        36
#define IRQ_NO_SPI3        51

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSB_FIRST   7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RX_ONLY     10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRC_NEXT    12
#define SPI_CR1_CRC_EN      13
#define SPI_CR1_BIDI_OE     14
#define SPI_CR1_BIDI_MODE   15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RX_NOT_EMPTY         0
#define SPI_SR_TX_EMPTY             1
#define SPI_SR_CHSIDE               2
#define SPI_SR_UDR                  3
#define SPI_SR_CRC_ERR              4
#define SPI_SR_MODF                 5
#define SPI_SR_OVR                  6
#define SPI_SR_BSY                  7
#define SPI_SR_FRE                  8

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE          0
#define I2C_CR1_SMBUS       1
#define I2C_CR1_SMBTYPE     3
#define I2C_CR1_ENARP       4
#define I2C_CR1_ENPEC       5
#define I2C_CR1_ENGC        6
#define I2C_CR1_NOSTRECH    7
#define I2C_CR1_START       8
#define I2C_CR1_STOP        9
#define I2C_CR1_ACK         10
#define I2C_CR1_POS         11
#define I2C_CR1_PEC         12
#define I2C_CR1_ALERT       13
#define I2C_CR1_RESET       15


/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ        0
#define I2C_CR2_ITERREN     8
#define I2C_CR2_ITEVTEN     9
#define I2C_CR2_ITBUFEN     10
#define I2C_CR2_DMAEN       11
#define I2C_CR2_LAST        12


/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0       0
#define I2C_OAR1_ADD7_1     1
#define I2C_OAR1_ADD9_8     8
#define I2C_OAR1_ADDMODE    15


/*
 * Bit position definitions I2C_OAR2
 */
#define I2C_OAR2_ENDUAL     0
#define I2C_OAR2_ADD2       1


/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB          0
#define I2C_SR1_ADDR        1
#define I2C_SR1_BTF         2
#define I2C_SR1_ADD10       3
#define I2C_SR1_STOPF       4
#define I2C_SR1_RXNE        6
#define I2C_SR1_TXE         7
#define I2C_SR1_BERR        8
#define I2C_SR1_ARLO        9
#define I2C_SR1_AF          10
#define I2C_SR1_OVR         11
#define I2C_SR1_PECERR      12
#define I2C_SR1_TIMEOUT     14
#define I2C_SR1_SMBALERT    15


/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL         0
#define I2C_SR2_BUSY        1
#define I2C_SR2_TRA         2
#define I2C_SR2_GENCALL     4
#define I2C_SR2_SMBDEFAULT  5
#define I2C_SR2_SMBHOST     6
#define I2C_SR2_DUALF       7
#define I2C_SR2_PEC         8





#endif /* INC_STM32F401XX_H_ */
