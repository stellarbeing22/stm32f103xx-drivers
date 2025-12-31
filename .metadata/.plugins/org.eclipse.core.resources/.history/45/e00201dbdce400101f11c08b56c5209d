/*
 * stm32f103xx.h
 *
 *  Created on: Sep 14, 2025
 *      Author: stellarbeing22
 *
 * ========================================================================================================================
 *                                           FILE INDEX : stm32f103xx.h
 * ========================================================================================================================

1.  Includes & Basic Macros
    1.1  Standard includes (stddef.h, stdint.h)
    1.2  Custom macros (__vo, __weak)

2.  Processor-Specific Details
    2.1  NVIC ISERx register addresses
    2.2  NVIC ICERx register addresses
    2.3  NVIC IPRx base address and priority bits definition

3.  Memory Map Base Addresses
    3.1  Memory sections (FLASH, SRAM, ROM)
    3.2  Peripheral base address
    3.3  Bus base addresses (APB1, APB2, AHB)

4.  Peripheral Base Addresses
    4.1  APB1 peripherals (I2C1/2, SPI2, USART2–5)
    4.2  APB2 peripherals (AFIO, EXTI, GPIOA–E, SPI1, USART1)
    4.3  AHB peripherals (RCC)

5.  Peripheral Register Definition Structures
    5.1  GPIO register definition (GPIO_RegDef_t)
    5.2  AFIO register definition (AFIO_RegDef_t)
    5.3  RCC register definition (RCC_RegDef_t)
    5.4  EXTI register definition (EXTI_RegDef_t)
    5.5  SPI register definition (SPI_RegDef_t)
    5.6  I2C register definition (I2C_RegDef_t)

6.  Peripheral Definitions
    6.1  Typecasted base addresses (GPIOA–E, RCC, AFIO, EXTI, SPI1/2, I2C1/2)

7.  Clock Control Macros
    7.1  Clock enable macros (AFIO, GPIOx, SPIx, I2Cx, USART/UARTx)
    7.2  Clock disable macros (same structure as above)

8.  Peripheral Reset Macros
    8.1  GPIOx register reset macros
    8.2  SPIx register reset macros

9.  Interrupt Request (IRQ) Definitions
    9.1  EXTI IRQ numbers (EXTI0–EXTI15_10)
    9.2  SPI IRQ numbers (SPI1, SPI2)
    9.3  IRQ priority levels (PRI0–PRI15)

10. Generic Macros
    10.1 Boolean and flag macros (ENABLE, DISABLE, SET, RESET)

11. SPI Bit Position Definitions
    11.1 SPI_CR1 bit positions
    11.2 SPI_CR2 bit positions
    11.3 SPI_SR bit positions

12. I2C Bit Position Definitions
    11.1 I2C_CR1 bit positions
    11.2 I2C_CR2 bit positions
    11.3 I2C_SR1 bit positions
    11.4 I2C_SR2 bit positions
    11.5 I2C_CCR bit positions

13. Driver Includes
    13.1 stm32f103xx_gpio_drivers.h
    13.2 stm32f103xx_spi_drivers.h
    13.3 stm32f103xx_i2c_drivers.h

========================================================================================================================
*/



#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

/* =========================================================================
 * 1.Includes & Basic Macros
 * =========================================================================*/

//1.1
//Standard includes
#include<stddef.h>
#include<stdint.h>

//1.2
// Custom macros
#define __vo volatile
#define __weak __attribute__((weak))


/* =========================================================================
 * 2.START: Processor Specific Details
 * =========================================================================*/

//2.1
//ARM CORTEX M3 processor NVIC ISERx Register Addresses
#define NVIC_ISER0                         ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1                         ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2                         ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3                         ((__vo uint32_t *)0xE000E10C)


/* ARM CORTEX M3 processor NVIC ICERx Register Addresses*/
//2.2
#define NVIC_ICER0                         ((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1                         ((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2                         ((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3                         ((__vo uint32_t *)0xE000E18C)


/*ARM CORTEX M3 processor NVIC IPRx Register Addresses*/

//2.3
#define NVIC_IPR_BASEADDR                  ((__vo uint32_t *)0xE000E400)
#define PRIORITY_BITS                      4                             //In ST's microcontrollers its usually 4,refer to
                                                                         // section 10.1 of Interrupt and events(RM0008)


/*
 * FLASH_BASE: execution starts here by default (if BOOT0=0).
 * ROM_BASE: execution starts here if BOOT0=1 (system bootloader).
 * SRAM_BASE: execution starts here if BOOT0=1 and BOOT1=1 (rarely used, for debugging).
 * PERIPH_BASE: the root; APB1, APB2, AHB bus peripherals are offset from this.
 */


/* =========================================================================
 * 3.STM32F103xx Memory Map Base Addresses
 * =========================================================================
 */

//3.1
/* Memory sections (FLASH, SRAM, ROM)------------------------------------ */

#define FLASH_BASEADDR                     0x08000000UL       /* Embedded Flash memory (main user program memory) */
#define SRAM_BASEADDR                      0x20000000UL       /* On-chip SRAM (20 KB in STM32F103C8, more in larger variants) */
#define ROM_BASEADDR                       0x1FFFF000UL       /* System memory (factory programmed bootloader, read-only) */

//3.2
/* Peripheral base address ---------------------------------------------- */
#define PERIPH_BASEADDR                    0x40000000UL       /* Base address of peripheral registers*/

//3.3
/*Bus base addresses (APB1, APB2, AHB) ------------------------------ */
#define APB1PERIPH_BASEADDR                PERIPH_BASEADDR   /* APB1: TIM2–TIM7, USART2–5, I2C1–2, CAN, PWR, BKP, etc.*/
#define APB2PERIPH_BASEADDR                0x40010000UL      /* APB2: GPIOA–E, AFIO, EXTI, USART1, ADC1–2, TIM1, SPI1 */
#define AHBPERIPH_BASEADDR                 0x40020000UL      /* AHB : RCC, DMA1, DMA2, CRC, Flash interface4*/





/* =========================================================================
 * 4.Peripheral Base Addresses (only for this course: GPIO, SPI, I2C, USART)
 * =========================================================================*/


//4.1
/* ----------------------- APB1 Bus Peripherals ----------------------------
 * Slower clock domain (up to 36 MHz).
 * Contains communication peripherals like USART2/3, SPI2, I2C1/2.
 */

#define I2C1_BASEADDR                  (APB1PERIPH_BASEADDR + 0X5400)  /* I2C1: Inter-Integrated Circuit interface */
#define I2C2_BASEADDR                  (APB1PERIPH_BASEADDR + 0X5800)  /* I2C2: Inter-Integrated Circuit interface */

#define SPI2_BASEADDR                  (APB1PERIPH_BASEADDR + 0X3800)  /* SPI2: Serial Peripheral Interface (slave/master) */

#define USART2_BASEADDR                (APB1PERIPH_BASEADDR + 0X4400)  /* USART2: Universal Synchronous/Asynchronous Receiver/Transmitter */
#define USART3_BASEADDR                (APB1PERIPH_BASEADDR + 0X4800)  /* USART3: Universal Synchronous/Asynchronous Receiver/Transmitter */
#define UART4_BASEADDR                 (APB1PERIPH_BASEADDR + 0X4C00)  /* UART4: Universal Asynchronous Receiver/Transmitter */
#define UART5_BASEADDR                 (APB1PERIPH_BASEADDR + 0X5000)  /* UART5: Universal Asynchronous Receiver/Transmitter */


//4.2
/* ----------------------- APB2 Bus Peripherals ----------------------------
 * Faster clock domain (up to 72 MHz).
 * Holds GPIO ports, AFIO, EXTI, SPI1, USART1.
 */

#define AFIO_BASEADDR                  (APB2PERIPH_BASEADDR + 0x0000)  /* AFIO: Alternate Function I/O (pin remapping, debug configuration) */
#define EXTI_BASEADDR                  (APB2PERIPH_BASEADDR + 0x0400)  /* EXTI: External Interrupt/Event Controller */

#define SPI1_BASEADDR                  (APB2PERIPH_BASEADDR + 0x3000)  /* SPI1: High-speed SPI peripheral */
#define USART1_BASEADDR                (APB2PERIPH_BASEADDR + 0x3800)  /* USART1: Universal Synchronous/Asynchronous Receiver/Transmitter */


#define GPIOA_BASEADDR                 (APB2PERIPH_BASEADDR + 0x0800)  /* GPIOA: General Purpose I/O Port A */
#define GPIOB_BASEADDR                 (APB2PERIPH_BASEADDR + 0x0C00)  /* GPIOB: General Purpose I/O Port B */
#define GPIOC_BASEADDR                 (APB2PERIPH_BASEADDR + 0x1000)  /* GPIOC: General Purpose I/O Port C */
#define GPIOD_BASEADDR                 (APB2PERIPH_BASEADDR + 0x1400)  /* GPIOD: General Purpose I/O Port D */
#define GPIOE_BASEADDR                 (APB2PERIPH_BASEADDR + 0x1800)  /* GPIOE: General Purpose I/O Port E */


//4.3
/* ----------------------- AHB Bus Peripherals -----------------------------
 * High-speed bus (up to 72 MHz).
 * Contains system-level peripherals like RCC.
 */
#define RCC_BASEADDR                   (AHBPERIPH_BASEADDR + 0x1000)   /*Reset and Clock Control*/




/* =========================================================================
 * 5.Peripheral Register Definition Structures
 * =========================================================================*/

// 5.1
//GPIO
typedef struct                    /*------------GPIO Register Definition---------------- */
{
	__vo uint32_t CRL;            /*Port configuration register low         offset = 0x00*/
	__vo uint32_t CRH;            /*Port configuration register high        offset = 0x04*/
	__vo uint32_t IDR;            /*Port input data register                offset = 0x08*/
    __vo uint32_t ODR;            /*Port output data register               offset = 0x0C*/
	__vo uint32_t BSRR;           /*Port bit set/reset register             offset = 0x10*/
	__vo uint32_t BRR;            /*Port bit reset register                 offset = 0x14*/
	__vo uint32_t LCKR;           /*Port configuration lock register        offset = 0x18*/
}GPIO_RegDef_t;

// 5.2
//AFIO
typedef struct                    /*--------------AFIO REgister Definition---------------*/
{
	__vo uint32_t EVCR;           /*Event control register                  offset = 0x00*/
	__vo uint32_t MAPR;           /*AF remap register                       offset = 0x04*/
	__vo uint32_t EXTICR[4];     /*External interrupt config reg 1-4        offset = 0x08-0x14*/
	uint32_t RESERVED0;           /* Reserved                               offset = 0x18*/
	__vo uint32_t MAPR2;          /*AF remap register 2                     offset = 0x1C*/

}AFIO_RegDef_t;


// 5.3
//RCC
typedef struct                    /*--------------RCC Register Definition----------------*/
{
	__vo uint32_t CR;             /*Clock control register                  offset = 0x00*/
	__vo uint32_t CFGR;           /*Clock configuration register            offset = 0x04*/
	__vo uint32_t CIR;            /*Clock interrupt register                offset = 0x08*/
	__vo uint32_t APB2RSTR;       /*APB2 peripheral reset register          offset = 0x0C*/
	__vo uint32_t APB1RSTR;       /*APB1 peripheral reset register          offset = 0x10*/
	__vo uint32_t AHBENR;         /*AHB Peripheral Clock enable register    offset = 0x14*/
	__vo uint32_t APB2ENR;        /*APB2 peripheral clock enable register   offset = 0x18*/
	__vo uint32_t APB1ENR;        /*APB1 peripheral clock enable register   offset = 0x1C*/
	__vo uint32_t BDCR;           /*Backup domain control register          offset = 0x20*/
	__vo uint32_t CSR;            /*Control/status register                 offset = 0x24*/
	__vo uint32_t AHBSTR;         /*AHB peripheral clock reset register     offset = 0x28*/
	__vo uint32_t CFGR2;          /*Clock configuration register2           offset = 0x2C*/
}RCC_RegDef_t;


// 5.4
//EXTI
typedef struct                    /*-------------EXTI Register Definition----------------*/
{
	__vo uint32_t IMR;            /*Interrupt mask register                 offset = 0x00*/
	__vo uint32_t EMR;            /*Event mask register                     offset = 0x04*/
	__vo uint32_t RTSR;           /*Rising trigger selection register       offset = 0x08*/
	__vo uint32_t FTSR;           /*Falling trigger selection register      offset = 0x0C*/
	__vo uint32_t SWIER;          /*Software interrupt event register       offset = 0x10*/
	__vo uint32_t PR;             /*Pending register                        offset = 0x14*/
}EXTI_RegDef_t;


// 5.5
//SPI
typedef struct                    /*------------SPI Register Definition-----------------*/
{
	__vo uint32_t CR1;            /*SPI control register 1                 offset = 0x00*/
	__vo uint32_t CR2;            /*SPI control register 2                 offset = 0x04*/
	__vo uint32_t SR;             /*SPI status register                    offset = 0x08*/
	__vo uint32_t DR;             /*SPI data register                      offset = 0x0C*/
	__vo uint32_t CRCPR;          /*SPI CRC polynomial register            offset = 0x10*/
	__vo uint32_t RXCRCR;         /*SPI RX CRC register                    offset = 0x14*/
	__vo uint32_t TXCRCR;         /*SPI TX CRC register                    offset = 0x18*/
	__vo uint32_t I2SCFGR;        /*SPI_I2S configuration register         offset = 0x1C*/
	__vo uint32_t I2SPR;          /*SPI_I2S prescaler register             offset = 0x20*/
}SPI_RegDef_t;

// 5.6
//I2C
typedef struct                    /*------------SPI Register Definition-----------------*/
{
	__vo uint32_t CR1;            /*I2C control register 1                 offset = 0x00*/
	__vo uint32_t CR2;            /*I2C control register 1                 offset = 0x04*/
	__vo uint32_t OAR1;           /*I2C control register 1                 offset = 0x08*/
	__vo uint32_t OAR2;           /*I2C control register 1                 offset = 0x0C*/
	__vo uint32_t DR;             /*I2C control register 1                 offset = 0x10*/
	__vo uint32_t SR1;            /*I2C control register 1                 offset = 0x14*/
	__vo uint32_t SR2;            /*I2C control register 1                 offset = 0x18*/
	__vo uint32_t CCR;            /*I2C control register 1                 offset = 0x1C*/
	__vo uint32_t TRISE;          /*I2C control register 1                 offset = 0x20*/
}I2C_RegDef_t;


/* =========================================================================
 * 6.Peripheral Definitions
 * =========================================================================*/


//6.1
/*  Typecasted peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)*/
#define GPIOA                          ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                          ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                          ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                          ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                          ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define RCC                            ((RCC_RegDef_t*)   RCC_BASEADDR)
#define AFIO                           ((AFIO_RegDef_t*) AFIO_BASEADDR)
#define EXTI                           ((EXTI_RegDef_t*) EXTI_BASEADDR)

//SPI1/2
#define SPI1                           ((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2                           ((SPI_RegDef_t *) SPI2_BASEADDR)

//I2C1/2
#define I2C1                           ((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2                           ((I2C_RegDef_t *) I2C2_BASEADDR)

/* =========================================================================
 * 7.Clock Control Macros
 * =========================================================================*/


// 7.1
/* Clock Enable Macros*/

/* GPIOx */
#define AFIO_PCLK_EN                 (RCC->APB2ENR |= (1<<0))
#define GPIOA_PCLK_EN                (RCC->APB2ENR |= (1<<2))
#define GPIOB_PCLK_EN                (RCC->APB2ENR |= (1<<3))
#define GPIOC_PCLK_EN                (RCC->APB2ENR |= (1<<4))
#define GPIOD_PCLK_EN                (RCC->APB2ENR |= (1<<5))
#define GPIOE_PCLK_EN                (RCC->APB2ENR |= (1<<6))

/* SPIx */
#define SPI1_PCLK_EN                 (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN                 (RCC->APB1ENR |= (1<<14))

/* I2C */
#define I2C1_PCLK_EN                 (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN                 (RCC->APB1ENR |= (1<<22))

/* USART/UART */
#define USART1_PCLK_EN               (RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN               (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN               (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN                (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN                (RCC->APB1ENR |= (1<<20))


// 7.2
/*Clock Disable Macros */

/* GPIOx */
#define AFIO_PCLK_DI                 (RCC->APB2ENR &= ~(1<<0))
#define GPIOA_PCLK_DI                (RCC->APB2ENR &= ~(1<<2))
#define GPIOB_PCLK_DI                (RCC->APB2ENR &= ~(1<<3))
#define GPIOC_PCLK_DI                (RCC->APB2ENR &= ~(1<<4))
#define GPIOD_PCLK_DI                (RCC->APB2ENR &= ~(1<<5))
#define GPIOE_PCLK_DI                (RCC->APB2ENR &= ~(1<<6))

/* SPI */
#define SPI1_PCLK_DI                 (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI                 (RCC->APB1ENR &= ~(1<<14))

/* I2C */
#define I2C1_PCLK_DI                 (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI                 (RCC->APB1ENR &= ~(1<<22))

/* USART/UART */
#define USART1_PCLK_DI               (RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI               (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI               (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI                (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI                (RCC->APB1ENR &= ~(1<<20))




/* =========================================================================
 * 8.Peripheral Reset Macros
 * =========================================================================*/


//8.1
//GPIOx Register Reset

#define GPIOA_REG_RESET             do{ (RCC->APB2RSTR |= (1<<2)); (RCC->APB2RSTR &= ~(1<<2));} while(0)  //set and reset the RegReset bit of PORTA
#define GPIOB_REG_RESET             do{ (RCC->APB2RSTR |= (1<<3)); (RCC->APB2RSTR &= ~(1<<3));} while(0)  //set and reset the RegReset bit of PORTB
#define GPIOC_REG_RESET             do{ (RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4));} while(0)  //set and reset the RegReset bit of PORTC
#define GPIOD_REG_RESET             do{ (RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5));} while(0)  //set and reset the RegReset bit of PORTD
#define GPIOE_REG_RESET             do{ (RCC->APB2RSTR |= (1<<6)); (RCC->APB2RSTR &= ~(1<<6));} while(0)  //set and reset the RegReset bit of PORTE

//8.2
//SPIx Register Reset

#define SPI1_REG_RESET             do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));} while(0)  //set and reset the RegReset bit of SPI1
#define SPI2_REG_RESET             do{ (RCC->APB2RSTR |= (1<<14)); (RCC->APB2RSTR &= ~(1<<14));} while(0)  //set and reset the RegReset bit of SPI2



/* =========================================================================
 * 9.Interrupt Request (IRQ) Definitions (stm32f103xx-MCU)
 * =========================================================================*/

/*Don't confuse IRQ number with priority*/


//9.1
//EXTI IRQ numbers (EXTI0–EXTI15_10)

#define IRQ_EXTI0                   6
#define IRQ_EXTI1                   7
#define IRQ_EXTI2                   8
#define IRQ_EXTI3                   9
#define IRQ_EXTI4                   10
#define IRQ_EXTI9_5                 23
#define IRQ_EXTI15_10               40

//9.2
//SPI IRQ numbers (SPI1, SPI2)
#define IRQ_SPI1_Global             35
#define IRQ_SPI2_Global             36

//9.3
//IRQ priority levels (PRI0–PRI15)
#define IRQ_EXTI_PRI0               0
#define IRQ_EXTI_PRI1               1
#define IRQ_EXTI_PRI2               2
#define IRQ_EXTI_PRI3               3
#define IRQ_EXTI_PRI4               4
#define IRQ_EXTI_PRI5               5
#define IRQ_EXTI_PRI6               6
#define IRQ_EXTI_PRI7               7
#define IRQ_EXTI_PRI8               8
#define IRQ_EXTI_PRI9               9
#define IRQ_EXTI_PRI10              10
#define IRQ_EXTI_PRI11              11
#define IRQ_EXTI_PRI12              12
#define IRQ_EXTI_PRI13              13
#define IRQ_EXTI_PRI14              14
#define IRQ_EXTI_PRI15              15



/* =========================================================================
 * 10. Generic Macros
 * =========================================================================*/

//10.1
//Boolean and flag macros (ENABLE, DISABLE, SET, RESET)
#define ENABLE 	 	     			 1
#define DISABLE                      0
#define SET                          ENABLE
#define RESET                        DISABLE
#define GPIO_PIN_SET                 SET
#define GPIO_PIN_RESET               RESET
#define FLAG_SET             	     SET
#define FLAG_RESET                   RESET


/* =========================================================================
 * 11. SPI Bit Position Definitions
 * =========================================================================*/


//11.1
// SPI_CR1 bit positions
#define  SPI_CR1_CPHA              0
#define  SPI_CR1_CPOL              1
#define  SPI_CR1_MSTR              2
#define  SPI_CR1_BD                3
#define  SPI_CR1_SPE               6
#define  SPI_CR1_LSF_FIRST         7
#define  SPI_CR1_SSI               8
#define  SPI_CR1_SSM               9
#define  SPI_CR1_RXONLY            10
#define  SPI_CR1_DFF               11
#define  SPI_CR1_CRCNEXT           12
#define  SPI_CR1_CRCEN             13
#define  SPI_CR1_BIDIOE            14
#define  SPI_CR1_BIDIMODE          15

//11.2
//SPI_CR2 bit positions
#define  SPI_CR2_RXDMAEN           0
#define  SPI_CR2_TXDMAEN           1
#define  SPI_CR2_SSOE              2
#define  SPI_CR2_ERRIE             5
#define  SPI_CR2_RXNEIE            6
#define  SPI_CR2_TXEIE             7

//11.3
//SPI_SR bit positions
#define  SPI_SR_RXNE               0
#define  SPI_SR_TXE                1
#define  SPI_SR_CHSIDE             2
#define  SPI_SR_UDR                3
#define  SPI_SR_CRCERR             4
#define  SPI_SR_MODF               5
#define  SPI_SR_OVR                6
#define  SPI_SR_BSY                7


/* =========================================================================
 * 12. I2C Bit Position Definitions
 * =========================================================================*/

//12.1
//I2C_CR1 bit positions
#define I2C_CR1_PE                 0
#define I2C_CR1_SMBUS              1
#define I2C_CR1_SMB_TYPE           3
#define I2C_CR1_ENARP              4
#define I2C_CR1_ENPEC              5
#define I2C_CR1_ENGC               6
#define I2C_CR1_NO_STRETCH         7
#define I2C_CR1_START              8
#define I2C_CR1_STOP               9
#define I2C_CR1_ACK                10
#define I2C_CR1_POS                11
#define I2C_CR1_PEC                12
#define I2C_CR1_ALERT              13
#define I2C_CR1_SWRST              15

//12.2
//I2C_CR2 bit position
#define I2C_CR2_FREQ               0  //5:0
#define I2C_CR2_ITERREN            8
#define I2C_CR2_ITEVTEN            9
#define I2C_CR2_ITBUFEN            10
#define I2C_CR2_DMAEN              11
#define I2C_CR2_LAST               12


//12.3
//I2C_SR1 bit position
#define I2C_SR1_SB                 0
#define I2C_SR1_ADDR               1
#define I2C_SR1_BTF                2
#define I2C_SR1_ADD10              3
#define I2C_SR1_STOPF              4
#define I2C_SR1_RxNE               6
#define I2C_SR1_TxE                7
#define I2C_SR1_BERR               8
#define I2C_SR1_ARLO               9
#define I2C_SR1_AF                 10
#define I2C_SR1_OVR                11
#define I2C_SR1_PECERR             12
#define I2C_SR1_TIMEOUT            14
#define I2C_SR1_SMBALERT           15


//12.4
//I2C_SR2 bit position
#define I2C_SR2_MSL                0
#define I2C_SR2_BUSY               1
#define I2C_SR2_TRA                2
#define I2C_SR2_GENCALL            4
#define I2C_SR2_SMBDEFAULT         5
#define I2C_SR2_SMBHOST            6
#define I2C_SR2_DUALF              7
#define I2C_SR2_PEC                8  //15:8

//12.5
//I2C_CCR bit position
#define I2C_CCR_CCR                0  //11:0
#define I2C_CCR_DUTY               14
#define I2C_CCR_FS                 15








/* =========================================================================
 * 13. Driver Includes
 * =========================================================================*/


#include "stm32f103xx_gpio_drivers.h"
#include "stm32f103xx_spi_drivers.h"
#include "stm32f103xx_i2c_drivers.h"

#endif /* INC_STM32F103XX_H_ */
